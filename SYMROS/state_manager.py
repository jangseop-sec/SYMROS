from context import GlobalContext
from symros_logging import *
from execute import sym_execute

import subprocess
import os

def check_new_state(ctx: GlobalContext, job, ws_dir: str, result_dir: str, exec_id: int, curr_idx: int, n_state: int):

  new_state_id = job['curr_input_id']
  
  # check valid state
  exec_result, constraints = sym_execute(exec_id, new_state_id, ws_dir, result_dir, False)
  
  if exec_result == "ERR" or exec_result == "INFINITE_LOOP":
    return
  
  if len(constraints) == 0:
    return 
  
  if constraints[-1]["location"] == "start target callback":
    return
  

  # make new state to the highest priority in queue
  os.rename(f"{result_dir}/test_{exec_id}/input_{new_state_id}", f"{result_dir}/test_{exec_id}/input_{new_state_id+1000000}")
  job['curr_input_id'] += 1000000  
  
  ctx.input_queues[curr_idx].put(job) 
  ctx.state_source[curr_idx] = f"{job['source_location']} - {job['curr_input_id']}"
  symros_log_info(f"[global state manager] new state is found!: id={new_state_id}")
  

def state_manager(ctx: GlobalContext, ws_dir: str, result_dir: str, exec_id: int):
  curr_idx = 2
  n_state = 1
  
  while True:
    try:
      new_job = ctx.new_state_queue.get()
      if new_job is None: 
        symros_log_error("error is triggered, global state manager is stopped")
        break
      check_new_state(ctx, new_job, ws_dir, result_dir, exec_id, curr_idx, n_state)
      curr_idx = (curr_idx) % ((ctx.num_process - 1) // ctx.num_process_per_state) + 1
      if n_state < ctx.num_process:
        n_state += 1
    finally:
      ctx.new_state_queue.task_done()