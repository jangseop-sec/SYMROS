import json
from multiprocessing import shared_memory
from context import GlobalContext
from execute import parse_symcc_result   
from solver import *
from symros_logging import *
from grater import check_is_fp_only, grater
from util import *

import time
import subprocess
import queue
import os

def recover_once(ctx: GlobalContext, job, ws_dir: str, result_dir: str, exec_id: int, worker_id: int, is_state_mode: bool, bitmap_locks):
  
  # bitmap locks
  checker_bitmap_locks = bitmap_locks[2]
  bug_bitmap_locks = bitmap_locks[3]
  
  # replay buggy execution and get log
  bug_input_id = job['curr_input_id']
  base_location = job['source_location']
  input_file = f"{result_dir}/test_{exec_id}/input_{bug_input_id}"
  state_mode = "1" if is_state_mode else "0"
  checker_shm = shared_memory.SharedMemory(name="checker_bitmap", create=False, size=64*1024)
  bug_shm = shared_memory.SharedMemory(name="bug_bitmap", create=False, size=64*1024)
  all_checkers = checker_shm.buf
  all_bugs = bug_shm.buf
  
  # state update
  ctx.recover_state[worker_id]['input_id'] = bug_input_id
  ctx.recover_state[worker_id]['bug_location'] = base_location
  ctx.recover_state[worker_id]['status'] = "preprocess"
  
  try:
    symros_log_info("try to rerun")
    res = subprocess.run(
      ['./scripts/sym_install.bash', ws_dir, input_file, state_mode],
      check=False,
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      timeout=300,
      text=True
    )
    symros_log_info("complete rerun successfully")
    symcc_result = res.stdout
  except subprocess.TimeoutExpired as e:
    symros_log_warn("error is occured")
    symcc_result = e.stdout.decode('utf-8', errors="ignore")
  
  constraints = parse_symcc_result(symcc_result, input_file)
  
  # TODO negate taken abnormal operation branch
  context_constraints = []
  
  # build normal constraints
  anormal_check_constraints = queue.PriorityQueue()
  checker_locations = set()
  is_done = False
  is_target = False

  cnt = 0
  constraints = trim_pattern_loops(constraints, max_repeat=2, key="location")
  
  for curr_constraint_info in constraints:
    curr_constraint = curr_constraint_info['smt']
    curr_location = curr_constraint_info['location']
    
    if curr_location == "start target callback":
      is_target = True
      continue
    
    if curr_location == "None": continue
    else:
      curr_branch_info = {
        "id": int(curr_location.split('.')[0]),
        "data": int(curr_location.split('.')[1]),
        "taken": int(curr_location.split('.')[2])
      }
    
      if "c++/v1" in curr_location and "hash_table" in curr_location:
        continue
      if "rcl" in curr_location:
        continue
    
    if is_target and curr_branch_info['id'] >= 1000 and curr_branch_info['taken'] == 1:
      checker_location = '.'.join(curr_location.split('.')[3:])
      abnormal_op = curr_branch_info['id'] // 1000
      if not f"{abnormal_op}.{checker_location}" in checker_locations:
        priority = 1
        if abnormal_op == 6 or abnormal_op == 5 or abnormal_op == 2 or abnormal_op == 1:
          priority = 0
          
        checker_locations.add(f"{abnormal_op}.{checker_location}")
        anormal_check_constraints.put((priority, -cnt, curr_constraint_info))
        cnt += 1
      continue
    elif curr_branch_info['id'] >= 1000:
      continue
    elif is_done:
      continue
    
    # negate current constraint
    _, context_constraints = build_smt({
      "location": curr_location,
      "smt": curr_constraint
    }, context_constraints)
    if '.'.join(curr_location.split('.')[3:]) == '.'.join(base_location.split('.')[3:]):
      is_done = True
  
  
  # negate each anormal check constraint
  is_found_valid = False
  taken_checker_locations = set()
  
  recovery_begin = time.perf_counter()
  is_timeout = False
  
  while not anormal_check_constraints.empty():
    # timout
    if time.perf_counter() - recovery_begin > 300:
      is_timeout = True
      break
    
    _, _, curr_constraint_info = anormal_check_constraints.get()
    
    curr_constraint = curr_constraint_info['smt']
    curr_location = curr_constraint_info['location']
    
    curr_branch_info = {
      "id": int(curr_location.split('.')[0]),
      "data": int(curr_location.split('.')[1]),
      "taken": int(curr_location.split('.')[2])
    }
    
    checker_location = '.'.join(curr_location.split('.')[3:])
    taken_checker_locations.add(checker_location)
    if is_found_valid: continue
    
    # identify anormal operation
    digit = curr_branch_info['id'] // 1000
    abnormal_op = ""
    if digit == 1:
      abnormal_op += "integer overflow"
    elif digit == 2:
      abnormal_op += "integer divided by zero"
    elif digit == 3:
      abnormal_op += "exceptional signed integer"
    elif digit == 4:
      abnormal_op += "exceptional unsigned integer"
    elif digit == 5:
      abnormal_op += "floating points overflow"
    elif digit == 6:
      abnormal_op += "floating points divided by zero"
    elif digit == 7:
      abnormal_op += "exceptional floating points"
    elif digit == 8:
      abnormal_op += "integer exceptional minus value"
    elif digit == 9:
      abnormal_op += "integer maximum value"
    elif digit == 10:
      abnormal_op += "integer minimum value"
    elif digit == 11:
      abnormal_op += "integer signed maximum value"
    elif digit == 12:
      abnormal_op += "integer signed minimum value"
    elif digit == 13:
      abnormal_op += "floating points exceptional minus value"
    elif digit == 14:
      abnormal_op += "floating points maximum value"
    elif digit == 15:
      abnormal_op += "floating points minimum value"
    
    symros_log_info(f"try to recover {abnormal_op}: {checker_location}")
    ctx.recover_state[worker_id]['bug_location'] = curr_location
    ctx.recover_state[worker_id]['status'] = f"recover {abnormal_op}"
    
    # build smt
    curr_constraints, _ = build_smt({
      "location": curr_location,
      "smt": curr_constraint
    }, context_constraints)
    
    # caculate solving weights
    solving_weight = 0
    for c in curr_constraints:
      solving_weight += len(c['smt'].split('(assert')[1].split('\n'))   
    
    # store path constraints
    pc_info = {
      "input_id": 0,
      "parent_input_id": 0,
      "location": curr_location,
      "solving_weight": solving_weight,
      "constraint": curr_constraints
    }
    
    with open(f'{result_dir}/test_{exec_id}/pc_0.json', 'w') as f:
      json.dump(pc_info, f)
    
    # solve
    is_grater_enabled = ctx.grater_enable and solving_weight < 500
    
    generated_input = []
    fp_solve = False
    
    try:
      ctx.recover_state[worker_id]['status'] = f"recover {abnormal_op}: check fp"
      if is_grater_enabled and check_is_fp_only(curr_constraints):
        fp_solve = True
        ctx.recover_state[worker_id]['status'] = f"recover {abnormal_op}: solve with grater"
        _, generated_input = grater(exec_id, 0, ctx.num_process+worker_id, ctx.recover_state[worker_id], result_dir)
      elif is_timeout:
        fp_solve = True
    except Exception as e:
      fp_solve = False
      generated_input = []
    
    if len(generated_input) == 0:
      ctx.recover_state[worker_id]['status'] = f"recover {abnormal_op}: solve with Z3"
      generated_input = solve_smt(curr_constraints)
    if len(generated_input) == 0:     # unsat
      continue
    
    with open(f"{result_dir}/test_{exec_id}/input_{bug_input_id}", 'rb') as f:
      default_input = f.read()
    
    current_input = bytearray(default_input)
    target_offset = ctx.target_offset if worker_id != 0 else 0
    for i in range(len(generated_input)):
      if generated_input[i] != -1:
        current_input[i] = generated_input[i]
    
    abnormal_op_str = '_'.join(abnormal_op.split(' '))
    with open(f"{result_dir}/test_{exec_id}/input_{bug_input_id}_{abnormal_op_str}_recover", 'wb') as f:
      f.write(bytes(current_input))
    
    # execute
    try:
      ctx.recover_state[worker_id]['status'] = f"recover {abnormal_op}: execute"
      res = subprocess.run(
        ['./scripts/sym_install.bash', ws_dir, f"{result_dir}/test_{exec_id}/input_{bug_input_id}_{abnormal_op_str}_recover", state_mode],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=180,
        text=True
      )
    except subprocess.TimeoutExpired as e:
      continue  # still timeout
    
    if res.returncode != 0:   # still error
      continue
    
    # successfully find out root cause
    ctx.recover_state[worker_id]['status'] = f"recover {abnormal_op}: found"
    
    symbolic_vars = set()
    for expr in parse_smt2_string(curr_constraint):
      symbolic_vars |= get_vars(expr)
      
    err_desc = f"{abnormal_op} at {'.'.join(curr_location.split('.')[3:])}\n"
    err_desc += "dependent inputs:\n"
    for var in symbolic_vars:
      var_idx = int(var.split('_')[-1])
      err_desc += f"\t{var}: {current_input[var_idx]}\n"
      
    os.makedirs(f"{result_dir}/err/desc/", exist_ok=True)
    
    with open(f"{result_dir}/err/desc/err_{exec_id}_{bug_input_id}_{'_'.join(abnormal_op.split(' '))}.txt", 'w') as f:
      f.write(err_desc)

    # store generated input as cache
    
    # push valid input to input_ready_queue
    valid_input_id = bug_input_id % 1000000 + 2000000   # grant second highest-priority (2)
    with open(f"{result_dir}/test_{exec_id}/input_{valid_input_id}", 'wb') as f:
      f.write(bytes(current_input))
    
    valid_input_job = {
      "curr_input_id": valid_input_id,
      "source_location": job['source_location'],
      "fp_solve": fp_solve,
      "solve_time": 0
    }
    
    ctx.input_queues[ctx.queue_index[worker_id]].put(valid_input_job)
    symros_log_info(f"put recovered input: {bug_input_id} -> {valid_input_id}")
    
    is_found_valid = True
    
    location_hash = update_path_hash(0, checker_location)
    current_lock_index = (location_hash >> 3) % 64
    with bug_bitmap_locks[current_lock_index]:
      set_bit(all_bugs, location_hash)
    
  if not is_found_valid or is_timeout:
    err_desc = f"unkown bug for input {bug_input_id}"
    with open(f"{result_dir}/err/desc/err_{exec_id}_{bug_input_id}_unknown.txt", 'w') as f:
      f.write(err_desc)
    
  for checker_location in list(taken_checker_locations):
    location_hash = update_path_hash(0, checker_location)
    current_lock_index = (location_hash >> 3) % 64
    with checker_bitmap_locks[current_lock_index]:
      set_bit(all_checkers, location_hash)

def recover_manager(ctx: GlobalContext, worker_id: int, ws_dir: str, exec_id: int, result_dir: str, bitmap_locks):
  
  # state initialize
  ctx.recover_state[worker_id]['input_id'] = -1
  ctx.recover_state[worker_id]['bug_location'] = ""
  ctx.recover_state[worker_id]['status'] = "wait"
  ctx.recover_state[worker_id]['remain_input'] = 0
  
  while True:
    curr_queue_index = ctx.queue_index[worker_id]
    job = ctx.bug_input_queues[curr_queue_index].get()
    if job is None:
      break
    try:
      recover_once(ctx, job, ws_dir, result_dir, exec_id, worker_id, ctx.is_state_mode, bitmap_locks)
      time.sleep(0.1)   # preemption
    finally:
      ctx.recover_state[worker_id]['input_id'] = -1
      ctx.recover_state[worker_id]['bug_location'] = ""
      ctx.recover_state[worker_id]['status'] = "wait"
      ctx.recover_state[worker_id]['remain_input'] = ctx.bug_input_queues[curr_queue_index].qsize()
      ctx.bug_input_queues[curr_queue_index].task_done()