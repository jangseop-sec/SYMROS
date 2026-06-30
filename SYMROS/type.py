from enum import Enum

COVERAGE = 0

# TODO : gloval sequence convert to sequence class (e.g. if call get() auto-increase)
CB_ID_SEQ = 0
EXEC_ID_SEQ = 1

class CB_TYPE(Enum):
  ENTRY = 0
  SUB = 1
  SRV = 2
  LC_CB = 3
  ACT = 4
  
class LC_TYPE(Enum):
  CONFIGURE = 1,
  CLEANUP = 2,
  ACTIVATE = 3,
  DEACTIVATE = 4,
  SHUTDOWN = 5

class Callback:
  callback_id: int
  executor_id: int
  cb_type: CB_TYPE
  entyr_code: str # only for ENTRY
  topic_name: str # only for SUB/SRV
  callee_idx: int  # only for SUB/SRV
  msg_name: str  # only for SUB/SRV
  caller_id: int  # only for SUB/SRV
  caller_type: CB_TYPE  # only for SUB/SRV
  node_var: str # only for LC_CB
  lc_type: LC_TYPE # only for LC_CB
  flipped: bool
  path_branches: list[str]
  
  def __init__(self, executor_id, cb_type, callback_id=None):
    global CB_ID_SEQ
    if callback_id is None:
      self.callback_id = CB_ID_SEQ
      CB_ID_SEQ += 1
    else:
      self.callback_id = callback_id
      CB_ID_SEQ = callback_id + 1
      
    self.executor_id = executor_id
    self.cb_type = cb_type
    self.flipped = False
    self.path_branches = []
  
  def init_entry(self, entry_code):
    self.entry_code = entry_code
  
  def init_sub_srv(self, callee_idx, msg_name, topic_name):
    self.callee_idx = callee_idx
    self.msg_name = msg_name
    self.topic_name = topic_name

  def init_lc_callback(self, caller_id, caller_type, lc_type):
    self.caller_id = caller_id
    self.caller_type = caller_type
    self.lc_type = lc_type
  
  def print_info(self):
    if self.cb_type == CB_TYPE.ENTRY:
      print(f"[{self.callback_id}/{self.executor_id}] ENTRY")
    elif self.cb_type == CB_TYPE.LC_CB:
      print(f"[{self.callback_id}/{self.executor_id}] LC_CB: {self.lc_type}: caller_id={self.caller_id}, caller_type={self.caller_type.name}")
    else:
      print(f"[{self.callback_id}/{self.executor_id}] {self.cb_type.name}: caller_id={self.caller_id}, caller_type={self.caller_type.name}, callee_idx={self.callee_idx}, msg_name={self.msg_name}")

class Class:
  name: str
  fields: list[tuple[str, str]]
  filename: str
  
  def __init__(self, name, fields, filename):
    self.name = name
    self.fields = fields
    self.filename = filename

  def print_info(self):
    print(f"Class: {self.name}")
    for ftype, fname in self.fields:
      print(f"  - {ftype} {fname}")
    
class Topic:
  type: str
  fields: list[tuple[str, str]]  # list of (type, name)
  filename: str
  
  def __init__(self, type, fields, filename):
    self.type = type
    self.fields = fields
    self.filename = filename
  
  def print_info(self):
    print(f"Topic: {self.type}")
    for ftype, fname in self.fields:
      print(f"  - {ftype} {fname}")

class Service:
  type: str
  req_fields: list[tuple[str, str]]
  res_fields: list[tuple[str, str]]
  filename: str
  
  def __init__(self, type, req_fields, res_fields, filename):
    self.type = type
    self.req_fields = req_fields
    self.res_fields = res_fields
    self.filename = filename
      
  def print_info(self):
    print(f"Service: {self.type}")
    print("  Request:")
    for ftype, fname in self.req_fields:
      print(f"    - {ftype} {fname}")
    print("  Response:")
    for ftype, fname in self.res_fields:
      print(f"    - {ftype} {fname}")
      

class Action:
  type: str
  goal_fields: list[tuple[str, str]]
  result_fields: list[tuple[str, str]]
  feedback_fields: list[tuple[str, str]]
  filename: str
  
  def __init__(self, type, goal_fields, result_fields, feedback_fields, filename):
    self.type = type
    self.goal_fields = goal_fields
    self.result_fields = result_fields
    self.feedback_fields = feedback_fields
    self.filename = filename
      