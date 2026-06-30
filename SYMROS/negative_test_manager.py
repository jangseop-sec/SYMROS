# TODO negative test manager
from multiprocessing import shared_memory
from context import GlobalContext
from symros_logging import *
from grater import check_is_fp_only, grater
from solver import *
from execute import sym_execute
from util import *
from datetime import datetime

import os
import time
import csv
import queue
import subprocess
import json

def negative_test(ctx: GlobalContext, job, exec_id: int, check_ws_dir: str, conc_ws_dir: str, result_dir: str, worker_id: int, bitmap_locks):
  
  bug_shm = shared_memory.SharedMemory(name="bug_bitmap", create=False, size=64*1024)
  all_checkers = bug_shm.buf
  bug_bitmap_locks = bitmap_locks[3]
  
  input_id = job['input_id']
  
  ctx.recover_state[worker_id]['input_id'] = input_id
  ctx.recover_state[worker_id]['bug_location'] = ""
  ctx.recover_state[worker_id]['status'] = "check execute"
  
  exec_result, constraints = sym_execute(exec_id, input_id, check_ws_dir, result_dir, False, timeout=1200)
  
  if exec_result == "INFINITE_LOOP" or exec_result == "ERR":
    symros_log_info(f"already find error with base input, skip negative test: input id={input_id}")
    return

  ctx.recover_state[worker_id]['status'] = "checker identify"
  
  # build normal constraints
  context_constraints = []
  
  is_target = False
  
  cnt = 1
  skip_current_bound_checking = False
  total_checkers = 0
  
  # counte total checking branches
  for curr_constraint_info in constraints:
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
    
    if is_target and curr_branch_info['id'] >= 1000 and curr_branch_info['taken'] ==0:
      total_checkers += 1
    
  is_target = False
  
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
  
    if is_target and curr_branch_info['id'] >= 1000 and curr_branch_info['taken'] ==0:
      # identify anormal operation
      abnormal_op = ""
      
      digit = (curr_branch_info['id'] // 1000)
      if digit == 1:
        abnormal_op += "integer overflow"
      elif digit == 2:
        abnormal_op += "integer divided by zero"
      elif digit == 3:
        abnormal_op += "floating points overflow"
      elif digit == 4:
        abnormal_op += "floating points divided by zero"
      elif digit == 5:
        abnormal_op += "integer unsigned boundary"
      elif digit == 6:
        abnormal_op += "integer unsigned boundary"
      elif digit == 7:
        abnormal_op += "integer signed boundary"
      elif digit == 8:
        abnormal_op += "integer signed boundary"
      elif digit == 9:
        abnormal_op += "floating points boundary"
      elif digit == 10:
        abnormal_op += "floating points boundary"
      elif digit == 11:
        abnormal_op += "floating points epsilon boundary"
      elif digit == 12:
        abnormal_op += "floating points epsilon boundary"
      
      if digit == 5 or digit == 7 or digit == 9 or digit == 11:
        if skip_current_bound_checking:
          cnt += 1
          continue
      if digit == 6 or digit == 8 or digit == 10 or digit == 12:
        skip_current_bound_checking = False
        
      checker_hash = update_path_hash(worker_id, curr_location)
      current_lock_index = (checker_hash >> 3) % 64
      with bug_bitmap_locks[current_lock_index]:
        if test_bit(all_checkers, checker_hash):
          symros_log_debug("duplicated checker", worker_id)
          cnt += 1
          continue
        set_bit(all_checkers, checker_hash)
      curr_constraints, _ = build_smt({
        "location": curr_location,
        "smt": curr_constraint
      }, context_constraints)

      if len(curr_constraints) == 0:
        symros_log_info(f"constraint is empty after build smt, skip solving: {curr_location}")
        cnt += 1
        continue

      ctx.recover_state[worker_id]['bug_location'] = f"{curr_location} ({cnt}/{total_checkers})"
      ctx.recover_state[worker_id]['status'] = f"recover {abnormal_op}"
      cnt += 1
      
      exceptional_input_id = input_id + (10000000 * curr_branch_info['id'])   # TODO need better input id management
      
      # caculate solving weights
      solving_weight = 0
      for c in curr_constraints:
        solving_weight += len(c['smt'].split('(assert')[1].split('\n'))   
      
      # store path constraints
      pc_info = {
        "input_id": exceptional_input_id,
        "parent_input_id": input_id,
        "location": curr_location,
        "solving_weight": solving_weight,
        "constraint": curr_constraints
      }
      
      is_grater_enabled = ctx.grater_enable and solving_weight < 500
      
      with open(f'{result_dir}/test_{exec_id}/neg_test/pc_{exceptional_input_id}.json', 'w') as f:
        json.dump(pc_info, f, indent=4)
      
      # solve
      generated_input = []
      fp_solve = False
      
      try:
        ctx.recover_state[worker_id]['status'] = f"solve {abnormal_op}: check fp"
        if is_grater_enabled and check_is_fp_only(curr_constraints):
          fp_solve = True
          ctx.recover_state[worker_id]['status'] = f"solve {abnormal_op}: solve with grater"
          _, generated_input = grater(exec_id, exceptional_input_id, 1, ctx.recover_state[worker_id], result_dir, "neg_test/")  
      except Exception as e:
        symros_log_info(f"grater error!: {e}")
        fp_solve = False 
        generated_input = []
      
      if len(generated_input) == 0:
        ctx.recover_state[worker_id]['status'] = f"solve {abnormal_op}: solve with Z3"
        generated_input = solve_smt(curr_constraints)
        
      if len(generated_input) == 0:     # unsat
        symros_log_info("unsat")
        filename = f"{result_dir}/test_{exec_id}/neg_test/solving_result.csv"
        file_exists = os.path.isfile(filename)
        with open(filename, 'a', newline='') as csvfile:
          fieldnames = ['input_id', 'checker_id', "count", 'abnormal_op', 'location', 'method', 'time']
          writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
          if not file_exists:
            writer.writeheader()
          writer.writerow({
            'input_id': input_id,
            'checker_id': curr_branch_info['id'],
            'count': f"{cnt}/{total_checkers}",
            'abnormal_op': abnormal_op,
            'location': curr_location,
            'method': "grater" if fp_solve else "z3",
            'time': 'unsat'  
          })
        continue
      
      # if solve current bound checking, skip foward checking
      if digit >= 5:
        skip_current_bound_checking = True
      
      with open(f"{result_dir}/test_{exec_id}/input_{input_id}", 'rb') as f:
        default_input = f.read()
      
      current_input = bytearray(default_input)
      for i in range(len(generated_input)):
        if generated_input[i] != -1:
          current_input[i] = generated_input[i]
      
      bug_op_str = '_'.join(abnormal_op.split(' '))
      with open(f"{result_dir}/test_{exec_id}/neg_test/input_{input_id}_{curr_branch_info['id']}_{bug_op_str}", 'wb') as f:
        f.write(bytes(current_input))

      # record solving result
      filename = f"{result_dir}/test_{exec_id}/neg_test/solving_result.csv"
      file_exists = os.path.isfile(filename)
      with open(filename, 'a', newline='') as csvfile:
        fieldnames = ['solved_at', 'input_id', 'checker_id', "count", 'abnormal_op', 'location', 'method', 'time']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        if not file_exists:
          writer.writeheader()
        writer.writerow({
          'solved_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'), 
          'input_id': input_id,
          'checker_id': curr_branch_info['id'],
          'count': f"{cnt}/{total_checkers}",
          'abnormal_op': abnormal_op,
          'location': curr_location,
          'method': "grater" if fp_solve else "z3",
          'time': time.time()   
        })
      
      # execute
      is_err = False
      try:
        ctx.recover_state[worker_id]['status'] = f"solve {abnormal_op}: execute"
        res = subprocess.run(
          ['./scripts/conc_install.bash', conc_ws_dir, f"{result_dir}/test_{exec_id}/neg_test/input_{input_id}_{curr_branch_info['id']}_{bug_op_str}"],
          check=False,
          stdout=subprocess.PIPE,
          stderr=subprocess.STDOUT,
          timeout=180
        )
        is_err = res.returncode != 0
      except subprocess.TimeoutExpired as e:
        symros_log_info(f"negative test find new error cause by {abnormal_op} wiht input {input_id}")
        err_desc = f"execution timeout caused by {abnormal_op} at {curr_location}\n"
        err_desc += "dependent inputs:\n"
        
        symbolic_vars = set()
        for expr in parse_smt2_string(curr_constraint):
          symbolic_vars |= get_vars(expr)
        
        for var in symbolic_vars:
          var_idx = int(var.split('_')[-1])
          err_desc += f"\t{var}: {current_input[var_idx]}\n"
        with open(f"{result_dir}/err/desc/err_{exec_id}_{input_id}_{'_'.join(abnormal_op.split(' '))}.txt", "w") as f:
          f.write(err_desc)
        checker_hash = update_path_hash(digit, curr_location)
        set_bit(all_checkers, checker_hash)
      
      if is_err:   # still error
        symros_log_info(f"negative test find new error cause by {abnormal_op} wiht input_{input_id}_{curr_branch_info['id']}_{bug_op_str}")
        err_desc = f"system crash caused by {abnormal_op} at {curr_location}\n"
        err_desc += "dependent inputs:\n"
        
        symbolic_vars = set()
        for expr in parse_smt2_string(curr_constraint):
          symbolic_vars |= get_vars(expr)
        
        for var in symbolic_vars:
          var_idx = int(var.split('_')[-1])
          err_desc += f"\t{var}: {current_input[var_idx]}\n"
        with open(f"{result_dir}/err/desc/err_{exec_id}_{exceptional_input_id}_{'_'.join(abnormal_op.split(' '))}.txt", "w") as f:
          f.write(err_desc)
          
        with open(f"{result_dir}/test_{exec_id}/neg_test/errlog_input_{input_id}_{curr_branch_info['id']}_{bug_op_str}.txt", 'w') as f:
          f.write(res.stdout.decode('utf-8', errors='replace'))
        
        checker_hash = update_path_hash(digit, curr_location)
        set_bit(all_checkers, checker_hash)
        _, context_constraints = build_smt({
          "location": curr_location,
          "smt": curr_constraint
        }, context_constraints)

    elif curr_branch_info['id'] >= 1000:
      continue
    else:
      # negate current constraint
      _, context_constraints = build_smt({
       "location": curr_location,
       "smt": curr_constraint
      }, context_constraints)

  symros_log_info(f"test complete with input {input_id}")
    

# per-process queue for negative testing
input_queue = queue.PriorityQueue()

def negative_test_worker(ctx: GlobalContext, exec_id: int, check_ws_dir: str, conc_ws_dir: str, worker_id: int, result_dir: str, bitmap_locks):

  ctx.solver_state[worker_id]['input_id'] = -1
  ctx.solver_state[worker_id]['method'] = ""
  ctx.solver_state[worker_id]['status'] = "wait"
  ctx.solver_state[worker_id]['remain_constraints'] = 0
  
  ctx.executor_state[worker_id]['input_id'] = -1
  ctx.executor_state[worker_id]['source_location'] = ""
  ctx.executor_state[worker_id]['status'] = ""
  ctx.executor_state[worker_id]['remain_input'] = 0
  
  ctx.recover_state[worker_id]['input_id'] = -1
  ctx.recover_state[worker_id]['bug_location'] = ""
  ctx.recover_state[worker_id]['status'] = "wait"
  ctx.recover_state[worker_id]['remain_input'] = input_queue.qsize()
  
  while True:
    _, _, job = input_queue.get()
    if job is None:
      break
    try:
      negative_test(ctx, job, exec_id, check_ws_dir, conc_ws_dir, result_dir, worker_id, bitmap_locks)
    finally:
      ctx.recover_state[worker_id]['input_id'] = -1
      ctx.recover_state[worker_id]['bug_location'] = ""
      ctx.recover_state[worker_id]['status'] = "wait"
      ctx.recover_state[worker_id]['remain_input'] = input_queue.qsize()
      input_queue.task_done()

def negative_test_manager(ctx, worker_id, exec_id, conc_ws_dir, check_ws_dir, result_dir, bitmap_locks):
  
  ctx.solver_state[worker_id]['input_id'] = -1
  ctx.solver_state[worker_id]['method'] = ""
  ctx.solver_state[worker_id]['status'] = "wait"
  ctx.solver_state[worker_id]['remain_constraints'] = 0
  
  ctx.executor_state[worker_id]['input_id'] = -1
  ctx.executor_state[worker_id]['source_location'] = ""
  ctx.executor_state[worker_id]['status'] = ""
  ctx.executor_state[worker_id]['remain_input'] = 0
  
  ctx.recover_state[worker_id]['input_id'] = -1
  ctx.recover_state[worker_id]['bug_location'] = ""
  ctx.recover_state[worker_id]['status'] = "wait"
  ctx.recover_state[worker_id]['remain_input'] = input_queue.qsize()
  
  while True:
    job = ctx.input_queues[0].get()
    if job is None:
      break
    try:
      negative_test(ctx, job, exec_id, check_ws_dir, conc_ws_dir, result_dir, worker_id, bitmap_locks)
    finally:
      ctx.recover_state[worker_id]['input_id'] = -1
      ctx.recover_state[worker_id]['bug_location'] = ""
      ctx.recover_state[worker_id]['status'] = "wait"
      ctx.recover_state[worker_id]['remain_input'] = ctx.input_queues[0].qsize()
      ctx.input_queues[0].task_done()
  