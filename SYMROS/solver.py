from z3 import *
import re
from symros_logging import *
import copy
from collections import deque

def get_vars(e):
    vars = set()
    s = e.sexpr()
    vars = set(re.findall(r'\b[a-zA-Z_][a-zA-Z0-9_]*__\d+\b', s))
    
    return vars
  
def parse_declare_funs(smt_text):
    decls = set()
    for line in smt_text.splitlines():
        line = line.strip()
        if line.startswith("(declare-fun"):
            # (declare-fun name () (_ BitVec 8))
            name = line.split()[1]
            decls.add(name)
    return decls

def count_assertion(smt2: str) -> int:
  count = 0
  for line in smt2.split('\n'):
    line = line.strip()
    if line.startswith("(assert "):
      count += 1
  return count

def remove_duplicated_assertions(smt: str):
  splitted = smt.split("(assert ")

  s = []

  for assertion in splitted:
    if assertion not in s:
      s.append(assertion)

  return "(assert ".join(s)

def trim_smt(smt: str):
  splitted = smt.split("(assert")
  # count lines of SMT
  loc = 0
  for i in range(1, len(splitted) - 1):
    loc += len(splitted[i].splitlines())
    
  # remove heavy assertion
  while loc > 500:
    max_loc = 0
    max_idx = -1
    
    for i in range(1, len(splitted) - 1):
      if max_loc < len(splitted[i].splitlines()):
        max_idx = i
        max_loc = len(splitted[i].splitlines())
    
    splitted.pop(max_idx)
    loc -= max_loc
  
  
  # reconstruct SMT
  return "(assert".join(splitted)

def build_smt(cond, preconds):
  cond = copy.deepcopy(cond)
  preconds = copy.deepcopy(preconds)

  cond_expr = parse_smt2_string(cond['smt'])[0]
  target_vars_ = get_vars(cond_expr)
  target_vars = []
  
  for var in target_vars_:
    if "_string_size" in var:
      continue
    target_vars.append(var)
    
  target_conds = []  
  queue = deque()
  remaining = []

  for precond in preconds:
    precond_expr = parse_smt2_string(precond['smt'])[0]
    curr_vars = set(get_vars(precond_expr))
    
    if precond_expr.eq(cond_expr):
      return [], preconds
    
    queue.append((precond, precond_expr, curr_vars))

  remaining_queue = deque()
  target_vars_set = set(target_vars)

  while queue:
    precond, precond_expr, curr_vars = queue.popleft()
    
    if target_vars_set & curr_vars:
      target_conds.append(precond)
      new_vars = curr_vars - target_vars_set
      target_vars_set |= curr_vars
      
      if new_vars:
        queue.extend(remaining_queue)
        remaining_queue.clear()
    else:
      remaining_queue.append((precond, precond_expr, curr_vars))

  remaining = [p for p, _, _ in remaining_queue]

  s = Solver()
  s.add(Not(cond_expr))
  
  target_conds.append({
    'location': cond['location'],
    'smt': s.to_smt2()
  })
  preconds.append(cond)
  
  return target_conds, preconds

def solve_smt(constraints: list, timeout: int = 300000) -> list[int]:
  s = Solver()
  for cond in constraints:
    s.add(parse_smt2_string(cond['smt']))
  generated_inputs = []
  s.set(timeout=timeout)

  if s.check() == sat:
    m = s.model()

    for d in m.decls():
      val = m[d]
      if hasattr(val, "as_long"): # if bitvec
        idx = int(d.name().split('__')[-1])
        
        if len(generated_inputs) <= idx:
          for _ in range(idx - len(generated_inputs) + 1): 
            generated_inputs.append(-1)
        generated_inputs[idx] = val.as_long()

  return generated_inputs

def solve_smt_str(smt: str, timeout: int = 300000) -> list[int]:
  F = parse_smt2_string(smt)
  s = Solver()
  s.add(F)
  
  generated_inputs = []
  s.set(timeout=timeout)
  
  if s.check() == sat:
    m = s.model()

    for d in m.decls():
      val = m[d]
      if hasattr(val, "as_long"): # if bitvec
        idx = int(d.name().split('_')[-1])
  
        if len(generated_inputs) <= idx:
          for _ in range(idx - len(generated_inputs) + 1): 
            generated_inputs.append(-1)
        generated_inputs[idx] = val.as_long()
