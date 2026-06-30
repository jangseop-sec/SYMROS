from z3 import *

from parse import parse
from solve_with_check import solve_with_check

import threading
from symros_logging import *

import os
import glob
import random
import time
import json
# global
declare_vars = {}

def is_calc(expr) -> Bool:  
  # `grater` cannot handle 
  cannot_handle_keywords = [
    "fpRoundToIntegral",
    "fpSqrt",
    "fpIsNaN",
    "fpIsInf",
    "fpIsZero",
    "fpIsNormal",
    "fpIsSubnormal",
    "Distinct",
    "Implies",
    "If",
    '+oo',
    '-oo',
    'NaN',
    'fp.isNaN',
    'fp.isInfinite',
    'fp.to_sbv',
    'fp.to_ubv',   
    'fp.to_real',  
    'distinct',
    '=>',
    'ite'
  ]

  if (not "fpToFP" in str(expr)) and (not "fp.to_ieee_bv" in str(expr)): return False
  if any(keyword in str(expr) for keyword in cannot_handle_keywords): return False
  return True

def promote_to_double(expr):
  if is_fp(expr) and expr.sort() == FPSort(8,24):
    return fpToFP(RNE(), expr, FPSort(11,53))
  if is_bv(expr):
    return fpToFP(RNE(), expr, FPSort(11,53))
  return expr

def unwrap_fp_cast(expr):
  curr_expr = expr
  global declare_vars
  
  # recursive search concat expresstion
  while curr_expr.decl().kind() != Z3_OP_CONCAT:
    if curr_expr.decl().name() == "to_fp" or curr_expr.decl().name() == "fp.to_ieee_bv":
      if curr_expr.arg(0).decl().name() == "roundNearestTiesToEven":
        curr_expr = curr_expr.arg(1)
      else:
        curr_expr = curr_expr.arg(0)
      
    else:
      return curr_expr
      
  # index parsing, declare new floating point variable
  var_list = []
  for i in range(curr_expr.num_args()):
     var_list.append(curr_expr.arg(i).decl().name())
  
  try: 
    start_index = int(str(curr_expr.arg(curr_expr.num_args() - 1)).split('_')[-1])
    end_index   = int(str(curr_expr.arg(0)).split('_')[-1])
  except Exception as e:
    symros_log_error(f"Cannot parse type: {e}")
    return None
  nbytes = curr_expr.num_args()

  new_var_name = f"fp_{start_index}_{end_index}"

  if nbytes == 4:
    if not new_var_name in declare_vars.keys(): declare_vars[new_var_name] = var_list
    return promote_to_double(FP(new_var_name, FPSort(8, 24)))
  elif nbytes == 8:
    if not new_var_name in declare_vars.keys(): declare_vars[new_var_name] = var_list
    return FP(new_var_name, FPSort(11, 53))
  else:
    raise ValueError(f"unsupported fp byte width: {nbytes}")
cnt = 0
cnt_per_expr = 0

def rewrite(expr):
  if expr is None: return None
  
  global cnt, cnt_per_expr
  cnt_th = 300000
  cnt_per_expr_th = 30000
  if cnt == cnt_th:
    return None
  if cnt_per_expr == cnt_per_expr_th:
    return None
  cnt_per_expr += 1
  cnt += 1
  # leaf node
  if expr.num_args() == 0:
    return expr

  # visit child nodes recursively
  new_args = []
  for i in range(expr.num_args()):
    # if node is fp-casting, it converts to pure floating point variable
    if expr.arg(i).decl().name() == "to_fp" or expr.arg(i).decl().name() == "fp.to_ieee_bv":
      rewrite_expr = rewrite(unwrap_fp_cast(expr.arg(i)))
    else:
      rewrite_expr = rewrite(expr.arg(i))
    if rewrite_expr is None: return None
    new_args.append(rewrite_expr)
  try:
    return expr.decl()(*new_args)
  except Z3Exception as e:
    with open("./tmp/smt_error.log", "w") as f:
      symros_log_error(f"Error rewriting expression: {expr}\nError: {e}")
      f.write(f"Error rewriting expression: {expr}\nError: {e}")
      return None

def check_is_fp_only(contstrains: list) -> bool:
  exprs = []
  for c in contstrains:
    exprs.append(parse_smt2_string(c['smt'])[0])
    
  is_fp = False
  
  # if flipped constraints is not fp_only, skip
  if not is_calc(exprs[-1]):
    return False
  
  for e in exprs:
    if not is_calc(e):
      continue
    is_fp = True
  return is_fp

def extract_fp_only(exprs: list[ExprRef]) -> (str, bool):
  global cnt
  cnt = 0
  s = Solver()
  is_no_fp = True
  is_timeout = True
  for e in exprs[::-1]:
    if not is_calc(e): continue
    is_no_fp = False
    global cnt_per_expr
    cnt_per_expr = 0
    rewrite_expr = rewrite(simplify(e))
    if rewrite_expr is not None:
      is_timeout = False
      s.add(rewrite_expr)
  
  if is_no_fp and is_timeout:
    is_timeout = False
    
  return s.to_smt2(), is_no_fp, is_timeout

def grater_solve(exprs: list[ExprRef], first_try: bool, slot_id: int):
  global declare_vars
  
  if first_try:
    declare_vars.clear()
    fp_only_smt, is_no_fp, is_timeout = extract_fp_only(exprs)
    if is_no_fp:
      return {}, False
    if is_timeout:
      return {}, True
  
    parsed_smt = "(set-logic QF_FP)\n" + fp_only_smt + "\n(exit)"
    symros_log_info(f"parsed smt:\n{parsed_smt}")
      
    with open(f"./smts/slot_{slot_id}/smt.smt2", 'w') as file:
      file.write(parsed_smt)
    parse(f"./output/slot_{slot_id}", f"./smts/slot_{slot_id}")
  else:
    symros_log_info("grater retry to solve, skip parsing phase!")
 
  solved, bv64, bv32 = solve_with_check(f"./output/slot_{slot_id}")
  
  var_names = list(declare_vars.keys())
  answer = {}
  
  if not solved:
    return {}, True
  
  for x64, x32 in zip(bv64, bv32):
    current_vars = declare_vars[var_names.pop()]
    if len(current_vars) == 8:
      bs = int(x64).to_bytes(8, byteorder="big")
      idx = 0
      for b in bs:
        answer[current_vars[idx]] = b
        idx += 1
    else:
      bs = int(x32).to_bytes(4, byteorder="big")
      idx = 0
      for b in bs:
        answer[current_vars[idx]] = b
        idx += 1
  return answer, True

def grater_check(exprs: list[ExprRef], answer: dict, slot: int):
  s = Solver()
  bv_vars = {}
 
  def get_vars(e):
    vars = set()
    if is_const(e) and e.decl().kind() == Z3_OP_UNINTERPRETED:
      vars.add(str(e))
    for child in e.children():
      vars |= get_vars(child)
    return vars
  
  for name, val in answer.items():
    bv = BitVec(name, 8)
    bv_vars[name] = bv
    s.add(bv == BitVecVal(val, 8))
  
  # exprs = parse_smt2_string(smt2)
  generated_inputs = []
  
  s.add(exprs)
  s.set(timeout=10000)
  if s.check() == sat:
    m = s.model()
    for d in m.decls():
      val = m[d]
      if hasattr(val, 'as_long'):
        idx = int(d.name().split('_')[-1])
        
        if len(generated_inputs) <= idx:
          for _ in range(idx - len(generated_inputs) + 1):
            generated_inputs.append(-1)
        generated_inputs[idx] = val.as_long()
    return generated_inputs
  else:
    return []

import os
from multiprocessing import JoinableQueue

slots = [JoinableQueue(), JoinableQueue()]



def grater_init(n_slot: int):
  for i in range(0, n_slot+1):
    slots[0].put(i)
    slots[1].put(n_slot+i)
  
    # smt folder
    os.makedirs(f"./smts/slot_{i}", exist_ok=True)
    os.makedirs(f"./output/slot_{i}", exist_ok=True)
    os.makedirs(f"./smts/slot_{n_slot+i}", exist_ok=True)
    os.makedirs(f"./output/slot_{n_slot+i}", exist_ok=True)

def grater(exec_id: int, curr_input_id: int, slot_id: int, curr_state, result_dir: str, sub_dir: str = "") -> (str, bool):
  symros_log_info(f"{exec_id}/{curr_input_id}")
  with open(f'{result_dir}/test_{exec_id}/{sub_dir}pc_{curr_input_id}.json', 'r') as f:
    base_constraints = json.load(f)
  
  exprs = []
  for c in base_constraints['constraint']:
    exprs.append(parse_smt2_string(c['smt'])[0])
    
  curr_state['status'] = "solve"
  
  # grater
  fp_answers, is_fp_smt = grater_solve(exprs, True, slot_id)

  if fp_answers and is_fp_smt:
    generated_input = grater_check(exprs, fp_answers, slot_id) 
  else:
    generated_input = []
  return is_fp_smt, generated_input
  
  
