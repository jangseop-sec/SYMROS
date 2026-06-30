import subprocess
from solver import *
from type import Callback, CB_TYPE
import type
import ast
from symros_logging import *

def parse_symcc_result(symcc_result: str, default_input_file: str) -> list[dict]:
  
  constraints = []
  
  sybmolic_barrier_start = "==== SYMBOLIC BARRIER START ===="
  sybmolic_barrier_stop = "==== SYMBOLIC BARRIER STOP ===="
  start_callback_test = "===== START CALLBACK TEST ====="
  
  n_barrier = len(symcc_result.split(sybmolic_barrier_stop))
  _symcc_result = ""
  
  for i in range(n_barrier):
    if i == 0: continue
    _symcc_result += symcc_result.split(sybmolic_barrier_stop)[i].split(sybmolic_barrier_start)[0]
  symcc_result = _symcc_result

  # parse symcc_result
  splitted_results = symcc_result.split('Trying to solve:')
  pattern = r"stdin(\d+) -> #x([0-9a-fA-F]{2})"
  
  if start_callback_test in splitted_results[0]:
    constraints.append({
      "location": "start target callback",
      "smt": None
    })
  
  for result in splitted_results[1:]:
    
    splitted_result = result.split("====end of smt====")
    
    constraint_info_str = splitted_result[0]
    
    unhandled_location = [
      # RCL library
      "rmw",
      "rcutil",
      "rcl",
      "rclcpp",
      "symros_graph",
      # C++ library
      "c++/v1",
    ]
    
    if any(loc in constraint_info_str for loc in unhandled_location):
      if start_callback_test in result:
        constraints.append({
          "location": "start target callback",
          "smt": None
        })
      continue
    
    if "Location:" in constraint_info_str:
      constraint_info = {
        "location": constraint_info_str.split("Location:")[1].split('\n')[0],
        "smt": constraint_info_str.split('SMT:')[1]
      }
    else:
      constraint_info = {
        "location": "None",
        "smt": constraint_info_str
      }
    constraints.append(constraint_info)
    
    if start_callback_test in result:
        constraints.append({
          "location": "start target callback",
          "smt": None
        })
    
  return constraints

def sym_execute(exec_id: int, input_id: int, ws_dir: str, result_dir: str, is_state_mode: bool, timeout: int =300) -> tuple[str, list[dict]]:
  
  input_file = f'{result_dir}/test_{exec_id}/input_{input_id}'
  executable = f'./scripts/sym_install.bash'
  state_mode = "1" if is_state_mode else "0"
  exec_result = "SUCCESS"
  
  # runner install/execute
  try:
    res = subprocess.run(
      [executable, ws_dir, input_file, state_mode],
      check=False,
      stdout=subprocess.PIPE,
      stderr=subprocess.STDOUT,
      timeout=timeout,
      # text=True
    )

    symcc_result = res.stdout.decode("utf-8", errors="replace")
    if res.returncode != 0 and "callback.dispatch" in symcc_result:
      with open(f"{result_dir}/error_report.txt", "w") as f:
        f.write(symcc_result)
      return "ERR", []
     
  except subprocess.TimeoutExpired as e:
    with open(f"{result_dir}/error_report.txt", 'a') as f:
      f.write(f"{exec_id},{input_id},SYMEX_TIMEOUT\n")
    
    partial_output = e.stdout
    if not partial_output:
      return "INFINITE_LOOP", []
    
    try:
      symcc_result = partial_output.decode("utf-8", errors="replace")
      constraints = parse_symcc_result(symcc_result, input_file)
      return "INFINITE_LOOP", constraints
    except Exception:
      return "ERR", []

  constraints = parse_symcc_result(symcc_result, input_file)
  
  return exec_result, constraints

def conc_execute(exec_id: int, input_id: int, ws_dir: str, result_dir: str) -> (str, list[Callback], int, int, int, int):
  ret_status = "SUCCESS"

  # run executor with input_{input_id}.txt
  try: 
    input_file = f'{result_dir}/test_{exec_id}/input_{input_id}'
    env = os.environ.copy()
    env['LLVM_PROFILE_FILE'] = f'{ws_dir}/symros-result/run_{input_id}.profraw'

    res = subprocess.run(
      ['./scripts/conc_install.bash', ws_dir, input_file],
      timeout=120,
      env=env,
      check=False,
      # text=True,
      start_new_session=True,
      capture_output=True
    )
    
    if res.returncode != 0:
      symros_log_error(f"concrete execution failed: return code (exit {res.returncode})")
      ret_status = "ERR"
      with open(f"{result_dir}/error_report.txt", 'a') as f:
        f.write(f"{exec_id},{input_id},EXEC_FAIL\n")

  except UnicodeDecodeError as e:
    symros_log_error("error is occured because stdout includes non-printable character :( ", e)
    return "EXEC_TIMEOUT", [], []
  except subprocess.TimeoutExpired as e:
    symros_log_error("timeout for unknown reason ", e)
    with open(f"{result_dir}/error_report.txt", 'a') as f:
      f.write(f"{exec_id},{input_id},SYMEX_TIMEOUT")
    # if concrete execution is timeout, kill all concrete execution
    subprocess.run(
      ["pkill", "-9", "-f", "concrete_ws_1/build/symros_exec/exec"],
      check=False
    )
    return "EXEC_TIMEOUT", [], []
  
  
  run_result = res.stdout.decode("utf-8", errors="replace")
  
  # check sanitzier
  if "ERROR: AddressSanitizer" in res.stderr.decode("utf-8", errors="replace"):
    with open(f"{result_dir}/error_report.txt", "a") as f:
      f.write(f"{exec_id},{input_id},ASAN_DETECT\n")  
    return "ASAN_DETECT", [], []
      
  # parse result to store new visited callback
  # TODO only config mode
  new_callbacks = []
  new_params = []
      
  # disable collect param-related message
  param_msg = [
    'GetParameters', 'GetParameterTypes', 'SetParameters', 'SetParametersAtomically', 'ListParameters', 'DescribeParameters', 'ParameterEvent',
    'get_parameters', 'get_parameter_types', 'set_parameters', 'set_parameters_atomically', 'list_parameters', 'describe_parameters', 'parameter_events'
  ]
  
  for result in run_result.split('\n'):
    if result.startswith('symros_result$$'):
      splitted_result = result.split('$$')
      res_type = splitted_result[1]
      
      if res_type == "new_param":
        param_name = splitted_result[2]
        param_type = splitted_result[3]
        if len(splitted_result) >= 5:
          if splitted_result[4] == "NOT_SET": 
            default_value = None
          else:
            default_value_str = splitted_result[4]
            if param_type == "bool":
              default_value = True if default_value_str == "true" else False
            elif param_type == "std::string":
              default_value = default_value_str
            else:
              try:
                default_value = ast.literal_eval(default_value_str)
              except Exception:
                if default_value_str.lower() == "nan":
                  default_value = float("nan")
                elif default_value_str.lower() == "inf":
                  default_value = float("inf")
                elif default_value_str.lower() == "-inf":
                  default_value = float("-inf")
                else:
                  default_value = default_value_str
                  
        else: default_value = None
        if not param_name.startswith("qos_overrides."):
          new_params.append({
            "type": param_type,
            "name": param_name,
            "default_value": default_value
          })
      else:
        cb_idx = int(splitted_result[2])
        topic_name = splitted_result[3]
        msg_type_splitted = splitted_result[4].split('::')
        msg_type = msg_type_splitted[0] + '/' + msg_type_splitted[2]
        
        # filter out parameter-related message
        if msg_type.split('/')[-1] in param_msg:
          continue

        symros_log_info(f"[symros] new callback is detected: [{cb_idx}] {res_type}: ({topic_name})({msg_type})")
        
        # TODO filter messages to in-pkg/ex-pkg/roslib
        if res_type == 'new_sub':
          new_callback = Callback(type.EXEC_ID_SEQ, CB_TYPE.SUB)
          type.EXEC_ID_SEQ += 1
        
        elif res_type == "new_srv":
          new_callback = Callback(type.EXEC_ID_SEQ, CB_TYPE.SRV)
          type.EXEC_ID_SEQ += 1
        
        elif res_type == "new_action_srv":
          new_callback = Callback(type.EXEC_ID_SEQ, CB_TYPE.ACT)
          type.EXEC_ID_SEQ += 1
          
        new_callback.init_sub_srv(cb_idx, msg_type, topic_name)
        new_callbacks.append(new_callback)
          
  return ret_status, new_callbacks, new_params