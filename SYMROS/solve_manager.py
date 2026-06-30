from context import GlobalContext
from symros_logging import *
from grater import check_is_fp_only, grater
from solver import solve_smt_str, trim_smt, solve_smt
from execute import sym_execute
import os
import traceback
import time
import csv
import threading
import queue
import json

from datetime import datetime


def solve_once(ctx: GlobalContext, worker_id: int, exec_id: int, curr_input_id: int, queue_idx: int, sym_ws_dir: str, result_dir: str):
  time_solve = -1
  fp_solve = False
  base_location = ""
  is_new_state = False
  
  # state update
  curr_state = ctx.solver_state[worker_id]
  curr_state['input_id'] = curr_input_id

  # solving curr input constraints
  with open(f'{result_dir}/test_{exec_id}/pc_{curr_input_id}.json', 'r') as f:
    data_ = f.read()

    data_dict = json.loads(data_)
    base_constraint = data_dict["constraint"]
    base_location = data_dict["location"]
    default_input_id = int(data_dict["parent_input_id"])
    solving_weight = data_dict["solving_weight"]

  MAX_CONSTRAINT_LEN = 500  # TODO user-defiend max constraint len
  
  # priority 3 is new state
  if curr_input_id > 3000000:   
    is_new_state = True
  
  start_solve = time.perf_counter()
  symros_log_info(f"try to solve path constraints:input id={curr_input_id}")
  
  # grater
  generated_input = []
  fp_solve = False
  if ctx.grater_enable:
    try:
      if solving_weight < MAX_CONSTRAINT_LEN and check_is_fp_only(base_constraint):
        fp_solve = True  
        curr_state['status'] = "wait"
        curr_state['method'] = "grater"
        
        # grater solve
        _, generated_input = grater(exec_id, curr_input_id, worker_id, curr_state, result_dir)
          
    except Exception as e:
      symros_log_warn(f"Grater encountered an error. Treating it as UNSAT: {e}" )
      fp_solve = False
      traceback.print_exc()
      with open(f"{result_dir}/error_report.txt", "w") as f:
        f.write(f"{exec_id},{curr_input_id},GRATER_ERROR")
      generated_input = []

  if len(generated_input) == 0:
    fp_solve = False
    curr_state['status'] = "solve"
    curr_state['method'] = "z3"
    symros_log_info("Z3 try to solve...")
    # adaptive solving timeout
    time_gap_hours = int((time.perf_counter() - ctx.recent_new_found.value) // 3600)
    timeout = 300000
    if time_gap_hours >= 1:
      timeout = 600000
    if time_gap_hours >= 2:
      timeout = 900000
    if time_gap_hours >= 3:
      timeout = 1200000
    generated_input = solve_smt(base_constraint, timeout)
    
  if len(generated_input) == 0: 
    symros_log_info("path constraints is unsatisfiable :( skip!")
    os.remove(f"{result_dir}/test_{exec_id}/pc_{curr_input_id}.json")
    return    # unsat
  
  curr_state['status'] = "construct"
  curr_state['method'] = ""
  
  symros_log_info("smt solver finds solution! Let's try to execute!")
          
  with open(f'{result_dir}/test_{exec_id}/input_{default_input_id}', 'rb') as f:
    default_input = f.read()
  
  # combine solving input with default input and generate current_input
  current_input = bytearray(default_input)
  target_offset = ctx.target_offset if worker_id != 0 else 0
  for i in range(len(generated_input)):
    if generated_input[i] != -1:
      current_input[i] = generated_input[i]
  
  with open(f'{result_dir}/test_{exec_id}/input_{curr_input_id}', 'wb') as f:
    f.write(bytes(current_input))
  time_solve = time.perf_counter() - start_solve
  
  new_job = {
    "curr_input_id": curr_input_id,
    "source_location": base_location,
    "fp_solve": fp_solve,
    "solve_time": time_solve
  }
  
  # logging
  filename = f"tmp/result_{exec_id}_{ctx.eid}.csv"
  file_exists = os.path.exists(filename)
  with open(filename, "a", newline="", encoding="utf-8") as f:
    writer = csv.DictWriter(
        f,
        fieldnames=[
            "current_time",
            "input_id",
            "solving_time",
            "fp_solve",
            "parent_id",
            "source_branch"
        ]
    )

    # add header at first
    if not file_exists:
        writer.writeheader()

    if curr_input_id > 1000000:
      with open(f"{result_dir}/cov_report.txt", "a") as f:
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        line = (
          f"[{now}] "
          f"input_id={curr_input_id:<10}"
          f"src={'.'.join(new_job['source_location'].split('.')[3:]) if len(new_job['source_location'].split('.')) > 3 else ''}\n"
        )
        f.write(line)      
    
    writer.writerow({
        "current_time": time.perf_counter(),
        "input_id": new_job['curr_input_id'],
        "solving_time": new_job["solve_time"],
        "fp_solve": new_job["fp_solve"],
        "parent_id": default_input_id,
        "source_branch": new_job['source_location']
    })

  if is_new_state:
    ctx.new_state_queue.put(new_job)
    symros_log_info(f"put new state input: {curr_input_id}")
    
  else:
    ctx.input_queues[queue_idx].put(new_job)
    symros_log_info(f"put new ready input: {curr_input_id}")
  
  os.remove(f"{result_dir}/test_{exec_id}/pc_{curr_input_id}.json")


# worker ------------------------------------------------------------------------------------------


# per process constraint queue
constraints_queue = queue.PriorityQueue()

def solve_worker(ctx: GlobalContext, worker_id: int, exec_id: int, sym_ws_dir: str, result_dir: str):
  
  # state intialization
  ctx.solver_state[worker_id]['input_id'] = -1
  ctx.solver_state[worker_id]['method'] = ""
  ctx.solver_state[worker_id]['status'] = "wait"
  ctx.solver_state[worker_id]['remain_constraints'] = 0
  
  while True:
    _, _, _, curr_queue_index, curr_input_id = constraints_queue.get()
    if curr_input_id == None:
      break
    try:
      solve_once(ctx, worker_id, exec_id, curr_input_id, curr_queue_index, sym_ws_dir, result_dir)
      time.sleep(0.1)   # preemption
    finally:
      ctx.solver_state[worker_id]['input_id'] = -1
      ctx.solver_state[worker_id]['method'] = ""
      ctx.solver_state[worker_id]['status'] = "wait"
      ctx.solver_state[worker_id]['remain_constraints'] = constraints_queue.qsize()
      constraints_queue.task_done() 
    # TODO some event if queue index is changed

# manager ------------------------------------------------------------------------------------------

def solve_manager(ctx: GlobalContext, worker_id: int, input_id_seq, exec_id: int, sym_ws_dir: str,result_dir: str):
  s = threading.Thread(target=solve_worker, args=(ctx, worker_id, exec_id, sym_ws_dir, result_dir), daemon=True)
  # fs = threading.Thread(target=fp_solve_worker, args=(worker_id, exec_id), daemon=True)
  s.start()
  # fs.start()
  
  while True:
    try:
      # pick current input
      curr_queue_index = ctx.queue_index[worker_id]
      job = ctx.constraint_queues[curr_queue_index].get()
      curr_input_id = job['id']
      solving_weight = job['solving_weight']
      symros_log_warn(f"get curr input id = {curr_input_id}", worker_id)
      if curr_input_id is None: break       # current input is not existed, terminate
      if curr_input_id == -1: 
        symros_log_error(f"error is occured, stop", worker_id)
        break
      bonus = 1000000
      priority = (curr_input_id // bonus)
      constraints_queue.put((-priority, solving_weight, curr_input_id, curr_queue_index, curr_input_id))
        
    finally:
      ctx.constraint_queues[curr_queue_index].task_done()
      time.sleep(0.1) # preemption