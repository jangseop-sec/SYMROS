import queue
from context import GlobalContext
from multiprocessing import shared_memory
from symros_logging import *
from execute import sym_execute
from util import *
from solver import *
import threading

import time
import json
from datetime import datetime

# ----------------------------------------------------------------------------------------------------------------------------

def execute_once(ctx: GlobalContext, worker_id, job, exec_id, sym_ws_dir, result_dir, input_id_seq, bitmap_locks, curr_queue_index):
  
  path_bitmap_locks = bitmap_locks[0]
  location_bitmap_locks = bitmap_locks[1]
  
  # parse input params
  curr_input_id = job['curr_input_id']
  source_location = job['source_location']
  
  # state update
  ctx.executor_state[worker_id]['input_id'] = curr_input_id
  ctx.executor_state[worker_id]['source_location'] = ':'.join(source_location.split('.')[3:])
  ctx.executor_state[worker_id]['status'] = "execute"
  
  path_shm = shared_memory.SharedMemory(name="path_bitmap", create=False, size=64*1024)
  location_shm = shared_memory.SharedMemory(name="location_bitmap", create=False, size=64*1024)
  all_pathes = path_shm.buf
  all_locations = location_shm.buf

  # symbolic execution
  if worker_id != 0:
      
    symros_log_info(f"symbolic execution - execution id: {exec_id} / input id: {curr_input_id}", worker_id)
    sym_exec_result, curr_constraints = sym_execute(exec_id, curr_input_id, sym_ws_dir, result_dir, False)
    symros_log_info(f"symbolic execution is finished (id={curr_input_id})", worker_id) 
    
    if sym_exec_result == "INFINITE_LOOP" or sym_exec_result == "ERR":
      ctx.bug_input_queues[curr_queue_index].put(job)
      return

    n_generated_new_input = 0
    path_hash = 0
    context_constraints = []

    # filter generate constraints 
    new_inputs = []
    is_start_callback = False
    
    ctx.executor_state[worker_id]['status'] = "generate constraints"
    
    visited_locations = {}
    n_constraints = len(curr_constraints)
    cnt_constraints = 0
    for curr_constraint_info in curr_constraints:
      cnt_constraints += 1
      ctx.executor_state[worker_id]['source_location'] = ':'.join(source_location.split('.')[3:]) + f"({cnt_constraints} / {n_constraints})"
      curr_constraint = curr_constraint_info["smt"]
      curr_location = curr_constraint_info["location"]
      
      if curr_location in visited_locations and visited_locations[curr_location] >= 100:   # if the same location is visited more than 3 times, skip it (potential infinite loop)
        continue
      else:
        visited_locations[curr_location] = visited_locations.get(curr_location, 0) + 1
      
      if curr_location == "start target callback":
        is_start_callback = True
        continue
      
      # parse source location information
      if curr_location != "None":
        curr_branch_info = {
          "id": int(curr_location.split('.')[0]),
          "data": int(curr_location.split('.')[1]),
          "taken": int(curr_location.split('.')[2])
        }
      else:
        curr_branch_info = {
          "id": -1,
          "data": -1,
          "taken": -1
        }
        
      # if checker branch, skip (in symex mode, it is not existed actually)
      if curr_branch_info['id'] >=1000:
        continue

      # duplicated path removed
      is_duplicated = False
      branch_id = f"{curr_branch_info['id']}.{curr_branch_info['data']}.{curr_branch_info['taken']}"
      path_hash = update_path_hash(path_hash, curr_location)
      symros_log_debug(f"{curr_location}: {path_hash}", worker_id)

      current_lock_index = (path_hash >> 3) % 64
      with path_bitmap_locks[current_lock_index]:
        if test_bit(all_pathes, path_hash):
          symros_log_debug(f"prune duplicated path in bitmap", worker_id)
          # mark as "duplicated"
          is_duplicated = True
        else:
          set_bit(all_pathes, path_hash)
      
      # prioritize new location
      location_id = f"{curr_branch_info['id']}.{'.'.join(curr_location.split('.')[3:])}"
      location_hash = update_path_hash(0, location_id)
      is_new_location = 0
      
      current_lock_index = (location_hash >> 3) % 64
      with location_bitmap_locks[current_lock_index]:
        if not test_bit(all_locations, location_hash):
          symros_log_debug(f"new location is found: {curr_location}", worker_id)
          set_bit(all_locations, location_hash)
          # mark as "new location"
          is_new_location = 1    
          ctx.recent_new_found.value = time.perf_counter()  # update recent new found time
      
      # generate current input path constraint
      splitted = curr_location.split('.')
      splitted[2] = '0' if splitted[2] == '1' else '1'  # flip taken value for constraint negation
      flipped_location = '.'.join(splitted)
      curr_constraints, context_constraints = build_smt({
        "smt": curr_constraint,
        "location": flipped_location,
        "dependencies": []
      }, context_constraints)
      
      symros_log_info(f"curr_constraints: {curr_constraints}", worker_id)
      
      # remove path constraints for pre-callbacks
      if not is_start_callback:
        symros_log_info("skip path constraints for pre-callbacks")
        continue

      # early checking logic
      if len(curr_constraints) == 0:    # if duplicated constraints
        symros_log_debug(f"duplicated constraints: {curr_location}", worker_id)
        is_duplicated = True
      elif "symros_math" in curr_location:
        is_duplicated = True
      elif "main.cpp" in curr_location:
        is_duplicated = True
      
      if is_duplicated:
        symros_log_debug(f"prune duplicated constraint", worker_id)
        continue
      
      # new callback id
      with input_id_seq.get_lock():
        input_id_seq.value += 1
        new_input_id = input_id_seq.value
        
      # priority policy
      # new state: 3
      # new location: 1
        
      # check it reach new location
      priority_bonus = 1000000
      if is_new_location: 
        new_input_id += priority_bonus
      
      # calculate solving weight
      solving_weight = 0
      for c in curr_constraints:
        solving_weight += len(c['smt'].split("(assert")[1].splitlines())
        
      # store input file
      new_input_file = f'./{result_dir}/test_{exec_id}/pc_{new_input_id}.json'
      pc_info = {
        "input_id": new_input_id,
        "parent_input_id": curr_input_id,
        "location": flipped_location,
        "solving_weight": solving_weight,
        "constraint": curr_constraints
      }
      with open(new_input_file, 'w') as f:
        json.dump(pc_info, f, indent=4)

      new_inputs.append({
        "input_id": new_input_id, 
        "path_hash": path_hash,
        "location_hash": location_hash
      })
      n_generated_new_input += 1

      ctx.constraint_queues[curr_queue_index].put({
        "id": new_input_id,
        "solving_weight": solving_weight
      })
      
      symros_log_info(f"add new input constraints at {curr_location} + {new_input_file}", worker_id)
    
    # new state explore
    if curr_input_id >= 1000000 or curr_input_id == 0:
      ctx.input_queues[0].put(job)
  else:
    _, curr_state_constraints = sym_execute(exec_id, curr_input_id, sym_ws_dir, result_dir, True)  # explore new state
    symros_log_info(f"new state exploration is finished (id={curr_input_id})", worker_id)
    
    new_inputs = []
    n_generated_new_input = 0
    is_start_callback = False
    context_constraints = []
    visited_locations = {}
    ctx.executor_state[worker_id]['status'] = "generate constraints"
    
    n_constraints = len(curr_state_constraints)
    cnt_constraints = 0
    for curr_constraint_info in curr_state_constraints:
      cnt_constraints += 1
      ctx.executor_state[worker_id]['source_location'] = ':'.join(source_location.split('.')[3:]) + f"({cnt_constraints} / {n_constraints})"
      curr_constraint = curr_constraint_info["smt"]
      curr_location = curr_constraint_info["location"]
      
      if curr_location in visited_locations and visited_locations[curr_location] >= 100:   # if the same location is visited more than 3 times, skip it (potential infinite loop)
        continue
      else:
        visited_locations[curr_location] = visited_locations.get(curr_location, 0) + 1
      
      if curr_location == "start target callback":
        is_start_callback = True
        continue
      
      # parse source location information
      if curr_location != "None":
        curr_branch_info = {
          "id": int(curr_location.split('.')[0]),
          "data": int(curr_location.split('.')[1]),
          "taken": int(curr_location.split('.')[2])
        }
      else:
        curr_branch_info = {
          "id": -1,
          "data": -1,
          "taken": -1
        }
      
      # if checker branch, skip (in symex mode, it is not existed actually)
      if curr_branch_info['id'] >=1000:
        continue
      
      # duplicated location removed
      is_duplicated = False
      location_id = f"{curr_branch_info['id']}.{'.'.join(curr_location.split('.')[3:])}"
      location_hash = update_path_hash(1, location_id)  # differ hash value from target message contraints
      is_new_location = 0
      current_lock_index = (location_hash >> 3) % 64
      with location_bitmap_locks[current_lock_index]:
        if test_bit(all_locations, location_hash):
          symros_log_debug(f"duplicated location: {curr_location}", worker_id)
          is_duplicated = True
        else:
          set_bit(all_locations, location_hash)
      
      # generate current input path contraint
      splitted = curr_location.split('.')
      splitted[2] = '0' if splitted[2] == '1' else '1'  # flip taken value for constraint negation
      flipped_location = '.'.join(splitted)
      curr_constraints, context_constraints = build_smt({
        "smt": curr_constraint,
        "location": flipped_location,
      }, context_constraints) 
      
      # early checking logic  
      if len(curr_constraints) == 0:    # if duplicated constraints
        symros_log_debug(f"duplicated constraints", worker_id)
        is_duplicated = True
      elif "symros_math" in curr_constraints:
        is_duplicated = True

      if is_duplicated:
        symros_log_debug(f"prune duplicated constraint", worker_id)
        continue
      
      # new callback id
      with input_id_seq.get_lock():
        input_id_seq.value += 1
        new_input_id = input_id_seq.value
        
      # priority policy
      # new state: 3
      # new location: 1
        
      # check it reach new location
      priority_bonus = 1000000
      new_input_id += 3 * priority_bonus
      
      # calculate solving weight
      solving_weight = 0
      for c in curr_constraints:
        solving_weight += len(c['smt'].split("(assert")[1].splitlines())
        
      # store input file
      pc_info = {
        "input_id": new_input_id,
        "parent_input_id": curr_input_id,
        "location": flipped_location,
        "solving_weight": solving_weight,
        "constraint": curr_constraints
      }
      
      new_input_file = f'./{result_dir}/test_{exec_id}/pc_{new_input_id}.json'
      
      # new_input = curr_constraint + "\n===\n" + curr_location + "\n===\n" + str(curr_input_id)
      with open(new_input_file, 'w') as f:
        json.dump(pc_info, f, indent=4)

      n_generated_new_input += 1
      ctx.constraint_queues[curr_queue_index].put({
        "id": new_input_id,
        "solving_weight": solving_weight
      })
      symros_log_info(f"add new state input constraints at {curr_location} + {new_input_file}", worker_id)

# ----------------------------------------------------------------------------------------------------------------------------

# per process input queue
sym_input_queue = queue.PriorityQueue()

# worker
def execute_once_worker(ctx: GlobalContext, worker_id: int, exec_id: int, sym_ws_dir: str, result_dir: str, input_id_seq, bitmap_locks: list):
  # state initalize
  ctx.executor_state[worker_id]['input_id'] = -1
  ctx.executor_state[worker_id]['source_location'] = ""
  ctx.executor_state[worker_id]['status'] = ""
  ctx.executor_state[worker_id]['remain_input'] = 0
  
  while True:
    symros_log_info(f"try to symbolic execution with new input", worker_id)
    _, curr_queue_index, job = sym_input_queue.get()
    if job is None:
      break
    try:
      execute_once(ctx, worker_id, job, exec_id, sym_ws_dir, result_dir, input_id_seq, bitmap_locks, curr_queue_index)
      time.sleep(0.1) # preemption
    finally:
      sym_input_queue.task_done()
      ctx.executor_state[worker_id]['input_id'] = -1
      ctx.executor_state[worker_id]['source_location'] = ""
      ctx.executor_state[worker_id]['status'] = "wait"
      ctx.executor_state[worker_id]['remain_input'] = sym_input_queue.qsize()
# ----------------------------------------------------------------------------------------------------------------------------

# manager
def execute_manager(ctx: GlobalContext, worker_id: int, exec_id: int, sym_ws_dir: str, result_dir: str, input_id_seq, bitmap_locks: list):
  test_worker = threading.Thread(target=execute_once_worker, args=(ctx, worker_id, exec_id, sym_ws_dir, result_dir, input_id_seq, bitmap_locks,), daemon=True)
  test_worker.start()
  
  while True:
    symros_log_info(f"try to get new ready input from solver", worker_id)
    curr_queue_index = ctx.queue_index[worker_id]
    job = ctx.input_queues[curr_queue_index].get()
    if job is None:
      symros_log_error(f"error is occured, stop", worker_id)
      break
    try:
      bonus = 1000000
      priority = (job['curr_input_id'] // bonus) * bonus
      sym_input_queue.put((job['curr_input_id'] - 2 * priority, curr_queue_index, job))
    finally:
      ctx.input_queues[curr_queue_index].task_done()
      time.sleep(0.1) # preemption
  
