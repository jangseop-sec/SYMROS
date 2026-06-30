import re
import os
import sys
import subprocess
import sqlite3
import glob
from queue import PriorityQueue
import threading
import json
import yaml
import queue
import signal

import type
from type import Topic, Service, Callback, CB_TYPE, LC_TYPE
from db import TOPICS, SERVICES, CALLBACKS, CLASS

from preprocess import find_all_msgs, find_all_srvs, find_all_acts, gen_symros_pkg, gen_conc_pkg
from exec_gen import construct_callback_info_yaml, symbolize_message_by_msg_dict, symbolize_parameter_by_param_dict, construct_test
from solver import *
from grater import extract_fp_only, grater, grater_init, check_is_fp_only
from symros_logging import *
from context import GlobalContext
from log_manager import log_manager
from execute_manager import execute_manager
from solve_manager import solve_manager
from recover_manager import recover_manager
from state_manager import state_manager
from negative_test_manager import negative_test_manager
from execute import *

from z3 import *
from datetime import datetime
import csv
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET
from multiprocessing import Process, Queue, JoinableQueue, Lock, Value, shared_memory, Array, Manager

import time
import traceback

import argparse
import random

# schdeuling setting
TIMEOUT = None
EID = -1
GRATER = None

# execution setting
NO_BUILD = False
RUN_MODE = ""

# shared queue
NUM_PROCESS = 16
NUM_PROCESS_PER_STATE = 1
NUM_NEGATIVE_TESTER = 4
MAX_CONSTRAINT_LEN = 300

# logging(visibility) setting
CAPTURE_CONCRETE_OUTPUT = None
CAPTURE_SYMBOLIC_OUTPUT = None

# sym mode execution  
def execute(exec_id: int, target_offset: int, sym_ws_dir: str, conc_ws_dir: str, check_ws_dir: str, result_dir: str) -> list[Callback]:

  start_num = 0
  start_location = "init"
  
  if not NO_BUILD:
    symros_log_info("[symros] colcon build...")
    subprocess.run(["./scripts/sym_build.bash", sym_ws_dir], text=True,  capture_output=True)
    symros_log_info("[symros] symbolic build is completed!")
    subprocess.run(["./scripts/conc_build.bash", conc_ws_dir], text=True, capture_output=True)
    symros_log_info("[symros] concrete build is completed!")
    subprocess.run(["./scripts/sym_build.bash", check_ws_dir], text=True, capture_output=True)
    symros_log_info("[symros] check build is completed!")

  # initialize setting for target callback concolic execution         
  
  # worker init
  workers = []
  solvers = []
  recovers = []
  negative_testers = []
  STRIDE = 64   # for cache-lined lock

  # input id sequence
  input_id_seq = Value('i', start_num)
  
  # global bitmap cache
  path_bitmap = shared_memory.SharedMemory(name="path_bitmap", create=True, size=64 * 1024).buf
  location_bitmap = shared_memory.SharedMemory(name="location_bitmap", create=True, size=64*1024).buf
  checker_bitmap = shared_memory.SharedMemory(name="checker_bitmap", create=True, size=64*1024).buf
  bug_bitmap = shared_memory.SharedMemory(name="bug_bitmap", create=True, size=64*1024).buf
  
  # locks for bitmap access
  path_bitmap_locks = [Lock() for _ in range(64 * 1024 // STRIDE)]
  location_bitmap_locks = [Lock() for _ in range(64 * 1024 // STRIDE)]
  checker_bitmap_locks = [Lock() for _ in range(64 * 1024 // STRIDE)]
  bug_bitmap_locks = [Lock() for _ in range(64 * 1024 // STRIDE)]
  bitmap_locks = [path_bitmap_locks, location_bitmap_locks, checker_bitmap_locks, bug_bitmap_locks]
  
  # global context
  ctx = GlobalContext(EID, GRATER, NUM_PROCESS, NUM_PROCESS_PER_STATE, target_offset=target_offset, timeout=TIMEOUT)
  
  # input queue initialize
  # add into message queue
  ctx.input_queues[1].put({
    "curr_input_id": start_num,
    "source_location": start_location,
    "fp_solve": False,
    "solve_time": 0  
  })   # add default input in ready queue
  # add into state queue
  ctx.input_queues[0].put({
    "curr_input_id": start_num,
    "source_location": start_location,
    "fp_solve": False,
    "solve_time": 0
  })
  
  with open(f"{result_dir}/cov_report.txt", "a") as f:
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    line = (
      f"[{now}] "
      f"input_id={start_num:<10}"
      f"src=init\n"
    )
    f.write(line)
  
  symros_log_info("put first ready input")
       
  # Grater init
  N_GRATER = 16
  grater_init(NUM_PROCESS)
  
  # process creation
  
  # genearte execute manager process
  for i in range(NUM_PROCESS):
    p = Process(target=execute_manager, args=(ctx, i, exec_id, sym_ws_dir, result_dir, input_id_seq, bitmap_locks))
    p.start()
    workers.append(p)
   
  # generate solve manager process
  for i in range(NUM_PROCESS):
    s = Process(target=solve_manager, args=(ctx, i, input_id_seq, exec_id, sym_ws_dir, result_dir))
    s.start()
    solvers.append(s)
    
  # generate recover manger process
  for i in range(NUM_PROCESS):
    r = Process(target=recover_manager, args=(ctx, i, check_ws_dir, exec_id, result_dir, bitmap_locks))
    r.start()
    recovers.append(r)
  
  # generate state maneger process
  g_m = Process(target=state_manager, args=(ctx, sym_ws_dir, result_dir, exec_id))
  g_m.start()
  
  # generate log manager process
  l_m = Process(target=log_manager, args=(ctx,))
  l_m.start()
  
  for s in solvers:
    s.join()
    symros_log_info(s.pid, s.exitcode)
  for w in workers:
    w.join()
    symros_log_info(w.pid, w.exitcode)
  for b in recovers:
    b.join()
    symros_log_info(b.pid, b.exitcode)
  for n in negative_testers:
    n.join()
    symros_log_info(n.pid, n.exitcode)
  
  symros_log_info("finish the callback testing!")


# test mode : construct test code and execute ----------------------------------------------------------------------

def test_symros(sym_ws_dir: str, conc_ws_dir: str, check_ws_dir: str, result_dir: str, target_callbacks: list[dict], test_id: int, n_target_callbacks: int, run_mode: str) -> list[Callback]:
  
  #construct
  default_input, target_offset, target_size, setup, body = construct_test(result_dir, target_callbacks, n_target_callbacks, test_id, True)
  
  # store default input
  margin = 1000
  default_input += b"\x00" * margin
  exec_folder = f'{result_dir}/test_{test_id}'
  if not os.path.exists(exec_folder):
    os.makedirs(exec_folder)
  
  with open(f'{exec_folder}/input_0', 'wb') as file:  # store default input
    file.write(default_input)

  if run_mode == "construct":
    # store symbolic executor
    with open(f'{sym_ws_dir}/src/symros_exec/src/main.cpp', 'r') as file:   # read main.cpp
      main_cpp = file.read()
    
    splitted_main_cpp = main_cpp.split('// SYMROS executor')                # inject executor in main.cpp
    splitted_main_cpp[1] = body    
    main_cpp = "// SYMROS executor".join(splitted_main_cpp) 
    
    splitted_main_cpp = main_cpp.split('// SYMROS setup')
    splitted_main_cpp[1] = setup
    main_cpp = "// SYMROS setup".join(splitted_main_cpp)
    
    with open(f'{sym_ws_dir}/src/symros_exec/src/main.cpp', 'w') as file:   # store main.cpp
      file.write(main_cpp)
    
    # store concrete executor
    with open(f'{conc_ws_dir}/src/symros_exec/src/main.cpp', 'r') as file:  # read main.cpp
      main_cpp = file.read()
    
    splitted_main_cpp = main_cpp.split('// SYMROS executor')                # inject executor in main.cpp
    splitted_main_cpp[1] = body
    main_cpp = "// SYMROS executor".join(splitted_main_cpp) 
    
    splitted_main_cpp = main_cpp.split('// SYMROS setup')
    splitted_main_cpp[1] = setup
    main_cpp = "// SYMROS setup".join(splitted_main_cpp)
    
    with open(f'{conc_ws_dir}/src/symros_exec/src/main.cpp', 'w') as file:  # store main.cpp
      file.write(main_cpp)
    
    # store check(sym) executor
    with open(f'{check_ws_dir}/src/symros_exec/src/main.cpp', 'r') as file:   # read main.cpp
      main_cpp = file.read()
    
    splitted_main_cpp = main_cpp.split('// SYMROS executor')                # inject executor in main.cpp
    splitted_main_cpp[1] = body
    main_cpp = "// SYMROS executor".join(splitted_main_cpp) 
    
    splitted_main_cpp = main_cpp.split('// SYMROS setup')
    splitted_main_cpp[1] = setup
    main_cpp = "// SYMROS setup".join(splitted_main_cpp)

    with open(f'{check_ws_dir}/src/symros_exec/src/main.cpp', 'w') as file:  # store main.cpp
      file.write(main_cpp)

  if run_mode == "symex":
    # execute
    return execute(test_id, target_offset, sym_ws_dir, conc_ws_dir, check_ws_dir, result_dir)
  
# TODO negative test mode
# add all input to input_queue for negative test
# add inputs into workers uniformly 
# start managers

def check_symros(exec_id: str, check_ws_dir: str, conc_ws_dir: str, result_dir: str, check_csv: str, target_callbacks, test_id: int, n_target_callbacks: int):
  new_callbacks = []
  start_num = 0
  start_location = "init"
  
  # initialize setting for target callback concolic execution         
  
  # construct
  default_input, target_offset, target_size, setup, body = construct_test(result_dir, target_callbacks, n_target_callbacks, test_id, True)
  
  # worker init
  negative_testers = []
  STRIDE = 64   # for cache-lined lock

  # global bitmap cache
  path_bitmap = shared_memory.SharedMemory(name="path_bitmap", create=True, size=64 * 1024).buf
  location_bitmap = shared_memory.SharedMemory(name="location_bitmap", create=True, size=64*1024).buf
  checker_bitmap = shared_memory.SharedMemory(name="checker_bitmap", create=True, size=64*1024).buf
  bug_bitmap = shared_memory.SharedMemory(name="bug_bitmap", create=True, size=64*1024).buf
  
  # locks for bitmap access
  path_bitmap_locks = [Lock() for _ in range(64 * 1024 // STRIDE)]
  location_bitmap_locks = [Lock() for _ in range(64 * 1024 // STRIDE)]
  checker_bitmap_locks = [Lock() for _ in range(64 * 1024 // STRIDE)]
  bug_bitmap_locks = [Lock() for _ in range(64 * 1024 // STRIDE)]
  bitmap_locks = [path_bitmap_locks, location_bitmap_locks, checker_bitmap_locks, bug_bitmap_locks]
  
  # global context
  ctx = GlobalContext(EID, GRATER, NUM_PROCESS, target_offset=target_offset)
  

  # Grater init
  N_GRATER = 16
  grater_init(NUM_PROCESS)
  
  job = {
    "input_id": 0,
  }
  ctx.input_queues[0].put(job)
  # with open(check_csv, newline='', encoding='utf-8') as f:
  #   reader = csv.DictReader(f)
  #   curr_worker_id = 0
  #   for row in reader:
  #     input_id = int(row['input_id'])
  #     if input_id > 1000000 and input_id < 2000000:
  #       if input_id // 1000000 == 3:
  #         input_id += 1000000
          
  #       job = {
  #         "input_id": input_id,
  #         'parent_id': int(row['parent_id']),
  #         "base_location": '.'.join(row['source_branch'].split('.')[3:])
  #       }
  #       ctx.input_queues[0].put(job)
  #       curr_worker_id = (curr_worker_id + 1) % NUM_PROCESS

  
  for file_name in os.listdir(f"{result_dir}/test_{exec_id}"):
    if file_name.startswith("input_"):
      if len(file_name.split('_')) > 2: continue
      input_id = int(file_name.split('_')[1])
      
      if input_id > 1000000:
        job = {
          "input_id": input_id,
        }
        ctx.input_queues[0].put(job)
  
  # genearte negative teseter process
  for i in range(NUM_PROCESS):
    p = Process(target=negative_test_manager, args=(ctx, i, exec_id, conc_ws_dir, check_ws_dir, result_dir, bitmap_locks))
    p.start()
    negative_testers.append(p)
  
  l_m = Process(target=log_manager, args=(ctx,))
  l_m.start()
  
  # clean
  for n in negative_testers:
    n.join()
    symros_log_info(n.pid, n.exitcode)
  
  l_m.join()
    
# config mode : construct test code and execute ----------------------------------------------------------------------

def config_symros(proj_dir: str, sym_ws_dir: str, conc_ws_dir: str, result_dir: str, entry_code: str):
    
  new_callback = []
  new_params = []
  
  os.makedirs(f"{result_dir}/test_0", exist_ok=True)
  os.makedirs(f"{result_dir}/config", exist_ok=True)
  
  # empty default input  
  with open(f'{result_dir}/test_0/input_0', 'wb') as file:  # store default input
    file.write(b"")
  
  # if not NO_BUILD:
  with open(f'{conc_ws_dir}/src/symros_exec/src/main.cpp', 'w') as file:   # store main.cpp
    file.write(entry_code)
  with open(f"{result_dir}/config/entry_code.cpp", 'w') as file:
    file.write(entry_code)
  
  # build 
  if not NO_BUILD:
    symros_log_info("[symros] colcon build...")
    subprocess.run(["./scripts/conc_build.bash", conc_ws_dir], text=True, capture_output=True)
    symros_log_info("[symros] concrete build is completed!")
  
  # execute
  symros_log_info(f"[symros] concrete execution - config")
  new_callbacks = []
  try:
    ret_status, new_callbacks, new_params = conc_execute(0, 0, conc_ws_dir, result_dir)
  except FileNotFoundError as e:
    symros_log_error(f"[symros:error] executor colcon build failed: {e}")
  except RuntimeError as e:
    symros_log_error(f"[symros:error] {e}")
    
  # parameter identification
  new_param_info = {
    "parameters": []
  }
  
  for param in new_params:
    if param['default_value'] is None:
      symros_log_warn(f"please set default value for parameter {param['name']} ({param['type']}) for configuration")
      # with open(f"{result_dir}/config/entry_code.cpp", 'r') as f:
      #   entry_code = f.read()
      # splitted_entry_code = entry_code.split('// lifecycle node transition: configuration')
      # declare_parameter_code = f"declare_parameter_if_not_declared(node, \"{param['name']}\", rclcpp::ParameterValue(/* SET DEFAULT VALUE (type={param['type']}) */));\n"
      # entry_code = splitted_entry_code[0] + declare_parameter_code + "\n// lifecycle node transition: configuration" + splitted_entry_code[1]
      # with open(f"{result_dir}/config/entry_code.cpp", 'w') as f:
      #   f.write(entry_code)
    # if param['type'].startswith('std::vector'):
    #   # TODO do not support vector parameter
    #   print(f"vector parameter is currently not supported: {param['name']} ({param['type']}): {param['default_value']}")
    #   continue 
    new_param_info['parameters'].append({
      "type": param['type'],
      "name": param['name'],
      "default_value": param['default_value'],
      "value": None
    })
  
  with open(f"{result_dir}/config/param_info.yaml", "w", encoding='utf-8') as f:
    yaml.safe_dump(
      new_param_info,
      f,
      allow_unicode=True,
      sort_keys=False)
  
  # callback identification
  new_callback_infos = []
  new_test_infos = []
  for new_callback in new_callbacks:
    target_snippet = construct_callback_info_yaml(new_callback)
    
    # callback metadata
    new_callback_info = {
      "callback_id": new_callback.executor_id,
      "name": new_callback.topic_name,
      "cb_type": new_callback.cb_type.value,
      "callee_idx": new_callback.callee_idx,
      "msg_info": target_snippet
    }
    new_callback_infos.append(new_callback_info)

  with open(f"{result_dir}/config/callback_info.yaml", "w", encoding='utf-8') as f:
    yaml.safe_dump(
      new_callback_infos, 
      f, 
      allow_unicode=True, 
      sort_keys=False)
  
# TODO goto util
def dict_update_by_dict(dst, src):
  if src is None: return
  
  for key, value in src.items():
    splitted = key.split('.')
    curr = dst

    for s in splitted[:-1]:
      if '[' in s:
        base = s.split('[')[0]
        idx = int(s.split('[')[1].split(']')[0])
        if base not in curr:
          break
        curr = curr[base]['value'][idx]['value']
      else:
        if s not in curr or not isinstance(curr[s], dict):
          break
        curr = curr[s]['value']
    curr[splitted[-1]]['value'] = value


def fill_default_input(schema_node, data):
  """
  Recursively fill a type-annotated schema node with actual data values.
  """
  if not isinstance(schema_node, dict) or "type" not in schema_node:
    return schema_node

  result = dict(schema_node)
  value = schema_node.get("value")

  if isinstance(value, dict):
    # Nested message type — recurse field by field
    if isinstance(data, dict):
      new_value = {}
      for field, sub_schema in value.items():
        if field in data:
          new_value[field] = fill_default_input(sub_schema, data[field])
        else:
          new_value[field] = sub_schema
      result["value"] = new_value
    else:
      result["value"] = data

  elif isinstance(value, list):
    # Array type — the template for each element is value[0]
    if isinstance(data, list):
      if value and isinstance(value[0], dict) and "type" in value[0]:
        # Complex element type (e.g. vector[geometry_msgs/TransformStamped])
        element_template = value[0]
        result["value"] = [
          fill_default_input(element_template, item) for item in data
        ]
      else:
        # Primitive element type (e.g. vector[int8]) — assign directly
        result["value"] = data
    else:
      result["value"] = value

  else:
    # Primitive scalar type
    result["value"] = data

  return result

# symros start point ------------------------------------------------------------------------------------------------------------------

def symros(run_mode, sym_ws_dir, conc_ws_dir, recover_ws_dir, proj_dir, result_dir, node_name=None, is_lifecycle=False, check_csv=None, test_id=None):
  # extract all node constructor -> database
  global CALLBACKS
  
  # preprocess
  find_all_msgs(f"{conc_ws_dir}/src", proj_dir)      # target project directory
  find_all_msgs(f"./ros/sym/humble/src", proj_dir)  # ROS2 library

  find_all_srvs(f"{conc_ws_dir}/src", proj_dir)      # target project directory
  find_all_srvs("./ros/sym/humble/src", proj_dir)   # ROS2 Library
  
  find_all_acts(f"{conc_ws_dir}/src", proj_dir)      # target project directory
  find_all_acts("./ros/sym/humble/src", proj_dir)   # ROS2 Library
  
  # make 'symros_exec' package in sym_ws, conc_ws, recover_ws

  # setup entry point
  if run_mode == "config":
    print("config symros...")
    if not NO_BUILD:
      gen_symros_pkg(sym_ws_dir, proj_dir)
      gen_conc_pkg(conc_ws_dir, proj_dir)
      gen_symros_pkg(recover_ws_dir, proj_dir)
        
    # generate entry code from node name and lifecycle transition
    entry_code = ""
    if node_name:
      entry_code += f"\n\nauto node = std::make_shared<{node_name}>(options);\n"
    if is_lifecycle:
      # TODO make user-write(?) entry code with concrete input?
      entry_code += """
// lifecycle node transition: configuration
node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
""" 
    if entry_code != "":
      with open(f'{conc_ws_dir}/src/symros_exec/src/main.cpp', 'r') as file:   # read main.cpp
        main_cpp = file.read()
        splitted_main_cpp = main_cpp.split('// SYMROS executor')                # inject executor in main.cpp
        splitted_main_cpp[1] = "\nSYMROS__INIT(true);\nrclcpp::NodeOptions options;\n" + entry_code
      entry_code = "// SYMROS executor".join(splitted_main_cpp)
    else:
      with open(f'{conc_ws_dir}/src/symros_exec/src/main.cpp', 'r') as file:   # read main.cpp
        main_cpp = file.read()
      with open(f"{result_dir}/config/entry_code.cpp", 'r') as file:
        entry_code_config = file.read()
        
      splitted_main_cpp = main_cpp.split('// SYMROS setup')
      splitted_main_cpp[1] = entry_code_config.split('// SYMROS setup')[1] 
      main_cpp = "// SYMROS setup".join(splitted_main_cpp)
      
      splitted_main_cpp = main_cpp.split('// SYMROS executor')
      splitted_main_cpp[1] = "\nSYMROS__INIT(true);" + entry_code_config.split('// SYMROS executor')[1]
      main_cpp = "// SYMROS executor".join(splitted_main_cpp)
      entry_code = main_cpp
    
    os.makedirs(f"{result_dir}/config", exist_ok=True)
    if entry_code != "":
      with open(f"{result_dir}/config/entry_code.cpp", 'w') as file:
        file.write(entry_code)
    else:
      with open(f"{result_dir}/config/entry_code.cpp", 'r') as file:
        entry_code = file.read()
    config_symros(proj_dir, sym_ws_dir, conc_ws_dir, result_dir, entry_code)
    
  elif (run_mode == "symex" or run_mode == "negtest" or run_mode == "construct") and test_id != 0:
    target_callbacks = []
    
    # generate target callback
    with open(f"{result_dir}/test_{test_id}/default_input.yaml", 'r', encoding="utf-8") as f:
      test_info = yaml.safe_load(f)
    
    # pre callback
    for callback_info in test_info['pre_callbacks']:
      
      if callback_info['callback_id'] == 0:   # activation function
        target_callbacks.append(callback_info)
        continue
      
      # TODO use default yaml file to generate default input
      with open(f"{result_dir}/config/callback_info.yaml", 'r', encoding='utf-8') as f:
        all_calback_info = yaml.safe_load(f)
        # target_callback_info = yaml.safe_load(f)
      default_callback_info = next(
        (curr_callback_info for curr_callback_info in all_calback_info if curr_callback_info.get("callback_id") == callback_info['callback_id']),
        None
      )
      
      # message_input = default_callback_info['msg_info']['value']
      per_test_message_input = callback_info['msg']
      # dict_update_by_dict(message_input, per_test_message_input)
      
      message_input = fill_default_input(default_callback_info['msg_info'], per_test_message_input)['value']
      
      callback_info['default'] = {}
      callback_info['default']['value'] = message_input  
      callback_info['default']['type'] = default_callback_info['msg_info']['type']
      target_callbacks.append(callback_info)    
    
    # target callbacks
    n_target_callbacks = 0
    for callback_info in test_info['target_callbacks']:
      n_target_callbacks += 1
      
      if callback_info['callback_id'] == 0:   # activation function
        target_callbacks.append(callback_info)
        continue
      
      # TODO use default yaml file to generate default input
      with open(f"{result_dir}/config/callback_info.yaml", 'r', encoding='utf-8') as f:
        all_calback_info = yaml.safe_load(f)
        # target_callback_info = yaml.safe_load(f)
      default_callback_info = next(
        (curr_callback_info for curr_callback_info in all_calback_info if curr_callback_info.get("callback_id") == callback_info['callback_id']),
        None
      )
      
      # message_input = default_callback_info['msg_info']['value']
      per_test_message_input = callback_info['msg']
      # dict_update_by_dict(message_input, per_test_message_input)
      
      message_input = fill_default_input(default_callback_info['msg_info'], per_test_message_input)['value']
      
      callback_info['default'] = {}
      callback_info['default']['value'] = message_input  
      callback_info['default']['type'] = default_callback_info['msg_info']['type']
      target_callbacks.append(callback_info) 
    
    if run_mode == "symex" or run_mode == "construct":
      test_symros(sym_ws_dir, conc_ws_dir, recover_ws_dir, result_dir, target_callbacks, test_id, n_target_callbacks, run_mode)
    else:
      check_symros(test_id, recover_ws_dir, conc_ws_dir, result_dir, check_csv, target_callbacks, test_id, n_target_callbacks)

  else:
    symros_log_error("[symros error] invalid configruation parameter")

# main function --------------------------------------------------------------------------------------------------------

if __name__ == "__main__":
  parser = argparse.ArgumentParser()
  
  # basic setting
  parser.add_argument("--project", help="target project for test")
  parser.add_argument("--node", help="target node name(wiht namespace) for test")
  parser.add_argument("--lifecycle", action="store_true", help="mark if target node is lifecylde node")
  parser.add_argument("--sym-ws", default="/home/ubuntu/symros/symros_ws_1", help="workspace for symbolic execution")
  parser.add_argument("--conc-ws", default="/home/ubuntu/symros/concrete_ws_1", help="workspace for concrete(general) execution")
  parser.add_argument("--recover-ws", default="/home/ubuntu/symros/symros_checker_ws", help="workspace for symbolic execution for recovery")
  # parser.add_argument("--check-csv", default="/home/ubuntu/symros/SYMROS/tmp/no.csv", help="workspace for symbolic execution for recovery")
  parser.add_argument("--result", default="./results", help="result folder to store generated inputs and executor snippets")
  
  # run mode
  parser.add_argument("--mode", type=str, default="config", help="run mode: config | construct | symex | check")    # TODO implement check mode(negative test)
  parser.add_argument("--test-id", type=int, default=0, help="callback id for testing")
  parser.add_argument("--no-build", action="store_true", help="no build executor for fast re-execute")
  
  # symbolic execution strategy
  parser.add_argument("--grater", action="store_true", help="active grater solver")
  parser.add_argument("--timeout", type=int, default=3600, help="timeout")
  
  # visibility
  parser.add_argument("--capture-concrete-output", action="store_true", help="print concrete execution log")
  parser.add_argument("--capture-symbolic-output", action="store_true", help="print symbolic execution log")
  
  args = parser.parse_args()
  
  sym_ws_dir = args.sym_ws
  conc_ws_dir = args.conc_ws
  recover_ws_dir = args.recover_ws
  check_csv = ""
  proj_dir = args.project
  node_name = args.node
  is_lifecycle = args.lifecycle
  result_dir = args.result
  
  run_mode = args.mode
  test_id = args.test_id
  exec_id = args.test_id    # TODO is needed?
  NO_BUILD = args.no_build
  RUN_MODE = args.mode
  
  # TODO local variable(=function parameter)
  GRATER = args.grater
  TIMEOUT = args.timeout
  EID = datetime.now().strftime("%y%m%d_%H%M%S")
  
  # logging setting
  CAPTURE_CONCRETE_OUTPUT = args.capture_concrete_output
  CAPTURE_SYMBOLIC_OUTPUT = args.capture_symbolic_output


  print(f"""
=======================================================================================================
      ___                         ___           ___           ___           ___     
     /  /\          __           /  /\         /  /\         /  /\         /  /\    
    /  /::\        |  |\        /  /::|       /  /::\       /  /::\       /  /::\   
   /__/:/\:\       |  |:|      /  /:|:|      /  /:/\:\     /  /:/\:\     /__/:/\:\  
  _\_ \:\ \:\      |  |:|     /  /:/|:|__   /  /::\ \:\   /  /:/  \:\   _\_ \:\ \:\ 
 /__/\ \:\ \:\     |__|:|__  /__/:/_|::::\ /__/:/\:\_\:\ /__/:/ \__\:\ /__/\ \:\ \:\\
 \  \:\ \:\_\/     /  /::::\ \__\/  /~~/:/ \__\/~|::\/:/ \  \:\ /  /:/ \  \:\ \:\_\/
  \  \:\_\:\      /  /:/~~~~       /  /:/     |  |:|::/   \  \:\  /:/   \  \:\_\:\  
   \  \:\/:/     /__/:/           /  /:/      |  |:|\/     \  \:\/:/     \  \:\/:/  
    \  \::/      \__\/           /__/:/       |__|:|~       \  \::/       \  \::/   
     \__\/                       \__\/         \__\|         \__\/         \__\/       
-------------------------------------------------------------------------------------------------------
Symbolic Execution Engine for ROS2 based Application.
SYMROS automatically identifys all callbacks in your application and test them!
* Target Applicaltion: {proj_dir}
* Run Mode
  - RUN_MODE: {run_mode}
  - TEST CALLBACK ID: {test_id}
  - BUILD NEW: {not NO_BUILD}
* Symbolic Execution Optimization:
  - GRATER: {GRATER}
* # timeout: {TIMEOUT}
* logging setting
  - CAPTURE_CONCRETE_OUTPUT: {CAPTURE_CONCRETE_OUTPUT}
  - CAPTURE_SYMBOLIC_OUTPUT: {CAPTURE_SYMBOLIC_OUTPUT}
=======================================================================================================
        """)

  # clean previous bitmap
  subprocess.run(["rm", "-rf", "/dev/shm/path_bitmap"])
  subprocess.run(["rm", "-rf", "/dev/shm/location_bitmap"])
  subprocess.run(["rm", "-rf", "/dev/shm/checker_bitmap"])
  subprocess.run(["rm", "-rf", "/dev/shm/bug_bitmap"])
  
  # run symros
  symros(run_mode, sym_ws_dir, conc_ws_dir, recover_ws_dir, proj_dir, result_dir, node_name, is_lifecycle, check_csv, test_id)
  
