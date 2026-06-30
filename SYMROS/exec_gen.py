from pathlib import Path
import sys
import os
import re
import struct
import yaml
from type import Callback, CB_TYPE, LC_TYPE
from db import CALLBACKS, TOPICS, SERVICES, CLASS, ACTIONS

def construct_message_yaml(msg_type: str) -> dict:
  
  ros_to_cpp_type_map = {
    "bool": "bool",
    "byte": "uint8_t",
    "char": "int8_t",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "float32": "float",
    "float64": "double",
    "string": "std::string",
    "wstring": "std::wstring",
  }
  
  splitted = msg_type.split('/')
  msg_namespace = f"{splitted[0]}::msg::{splitted[1]}"
  
  msg_info = None
  for msg in TOPICS:
    if msg.type == msg_namespace:
      msg_info = msg
      break
  if msg_info is None:
    raise ValueError(f"Message {msg_namespace} not found in database")
  msg_dict = {}
  
  for field_type, field_name in msg_info.fields:
    
    array_match = re.match(r"^(.+?)\[(<=)?(\d*)\]$", field_type)
    
    if "=" in field_name:   # ignore constant
      continue
    
    if array_match:   # if array
      base_type = array_match.group(1)
      size_constraint = array_match.group(2)
      size = array_match.group(3)
      
      is_variable_length = False
      if size == "":
        is_variable_length = True
        size = "1"   # TODO what fixed-size is appropriate?
      if size_constraint is not None:
        is_variable_length = True
      size = int(size)
      if base_type in ros_to_cpp_type_map:
        if is_variable_length:
          msg_dict[field_name] = {
            "type": f"vector[{base_type}]",
            "value": []
          }
        else:
          msg_dict[field_name] = {
            "type": f"list[{base_type}]",
            "value": []
          }
        for i in range(size):          
          msg_dict[field_name]["value"].append(None)
          
      elif '/' in base_type:   # out-of-package message type
        if is_variable_length:
          msg_dict[field_name] = {
            "type": f"vector[{base_type}]",
            "value": []
          }
        else:
          msg_dict[field_name] = {
            "type": f"list[{base_type}]",
            "value": []
          }
        for i in range(size):
          sub_msg_dict = construct_message_yaml(base_type)
          msg_dict[field_name]["value"].append(sub_msg_dict)
      else: # in-package message type
        compact_field_type = msg_type.split('/')[0] + "/" + base_type
        if is_variable_length:
          msg_dict[field_name] = {
            "type": f"vector[{base_type}]",
            "value": []
          }
        else:
          msg_dict[field_name] = {
            "type": f"list[{base_type}]",
            "value": []
          }
        for i in range(size):
          sub_msg_dict = construct_message_yaml(compact_field_type)
          msg_dict[field_name]["value"].append(sub_msg_dict)
    
    else:   # not array    
      if field_type in ros_to_cpp_type_map:
        msg_dict[field_name] = {
          "type": field_type,
          "value": None
        }
        
      elif '/' in field_type:   # out-of-package message type
        sub_msg_dict = construct_message_yaml(field_type)
        msg_dict[field_name] = sub_msg_dict
      else:   # in-package message type
        compact_field_type = msg_type.split('/')[0] + "/" + field_type
        sub_msg_dict = construct_message_yaml(compact_field_type)
        msg_dict[field_name] = sub_msg_dict
        
  return {"type": msg_type, "value": msg_dict }

def construct_action_server_yaml(act_type: str) -> dict:
  
  ros_to_cpp_type_map = {
    "bool": "bool",
    "byte": "uint8_t",
    "char": "int8_t",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "float32": "float",
    "float64": "double",
    "string": "std::string",
    "wstring": "std::wstring",
  }
  
  splitted = act_type.split('/')
  act_namespace = f"{splitted[0]}::action::{splitted[1]}"
  
  act_srv_dict = {}
  
  act_info = None
  for act in ACTIONS:
    if act.type == act_namespace:
      act_info = act
      break
  if act_info is None:
    raise ValueError(f"Message {act_namespace} not found in database")
  
  for field_type, field_name in act_info.goal_fields:
    array_match = re.match(r"^(.+?)\[(<=)?(\d*)\]$", field_type)
    
    if "=" in field_name:   # ignore constant
      continue
    
    if array_match:   # if array
      base_type = array_match.group(1)
      size_constraint = array_match.group(2)
      size = array_match.group(3)
      is_variable_length = False
      if size == "":
        is_variable_length = True
        size = "1"   # TODO what fixed-size is appropriate?
      if size_constraint is not None:
        is_variable_length = True
      size = int(size)
      if base_type in ros_to_cpp_type_map:
        if is_variable_length:
          act_srv_dict[field_name] = {
            "type": f"vector[{base_type}]",
            "value": []
          }
        else:
          act_srv_dict[field_name] = {
            "type": f"list[{base_type}]",
            "value": []
          }
        for i in range(size):
          act_srv_dict[field_name]["value"].append(None)
      elif '/' in base_type:   # out-of-package message type
        if is_variable_length:
          act_srv_dict[field_name] = {
            "type": f"vector[{base_type}]",
            "value": []
          }
        else:
          act_srv_dict[field_name] = {
            "type": f"list[{base_type}]",
            "value": []
          }
        for i in range(size):
          sub_msg_dict = construct_message_yaml(base_type)
          act_srv_dict[field_name]["value"].append(sub_msg_dict)
      else: # in-package message type
        for i in range(size):
          compact_field_type = act_type.split('/')[0] + "/" + base_type
          act_srv_dict[field_name] = {
            "type": f"list[{compact_field_type}]",
            "value": []
          }
          for i in range(size):
            sub_msg_dict = construct_message_yaml(compact_field_type)
            act_srv_dict[field_name]["value"].append(sub_msg_dict)
    
    else:   # not array
      
      if field_type in ros_to_cpp_type_map:
        act_srv_dict[field_name] = {
          "type": field_type,
          "value": None
        }
      elif '/' in field_type:   # out-of-package message type
        sub_msg_dict = construct_message_yaml(field_type)
        act_srv_dict[field_name] = sub_msg_dict
      else:   # in-package message type
        compact_field_type = act_type.split('/')[0] + "/" + field_type
        sub_msg_dict = construct_message_yaml(compact_field_type)
        act_srv_dict[field_name] = sub_msg_dict

  return {"type": act_type, "value": act_srv_dict}

  
def construct_request_yaml(req_type: str) -> str:
  
  ros_to_cpp_type_map = {
    "bool": "bool",
    "byte": "uint8_t",
    "char": "int8_t",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "float32": "float",
    "float64": "double",
    "string": "std::string",
    "wstring": "std::wstring",
  }
  
  splitted = req_type.split('/')
  req_namespace = f"{splitted[0]}::srv::{splitted[1]}"
  
  req_info = None
  for req in SERVICES:
    if req.type == req_namespace:
      req_info = req
      break
  if req_info is None:
    raise ValueError(f"Message {req_namespace} not found in database")
  
  req_msg_dict = {}
  
  for field_type, field_name in req_info.req_fields:
    array_match = re.match(r"^(.+?)\[(<=)?(\d*)\]$", field_type)
    
    if "=" in field_name:   # ignore constant
      continue
    
    if array_match:   # if array
      base_type = array_match.group(1)
      size_constraint = array_match.group(2)
      size = array_match.group(3)
      is_variable_length = False
      if size == "":
        is_variable_length = True
        size = "10"   # TODO what fixed-size is appropriate?
      if size_constraint is not None:
        is_variable_length = True
      size = int(size)
      if base_type in ros_to_cpp_type_map:
        if is_variable_length:
          req_msg_dict[field_name] = {
            "type": f"vector[{base_type}]",
            "value": []
          }
        else:
          req_msg_dict[field_name] = {
            "type": f"list[{base_type}]",
            "value": []
          }
        for i in range(size):
          req_msg_dict[field_name]["value"].append(None)
      elif '/' in base_type:   # out-of-package message type
        if is_variable_length:
          req_msg_dict[field_name] = {
            "type": f"vector[{base_type}]",
            "value": []
          }
        else:
          req_msg_dict[field_name] = {
            "type": f"list[{base_type}]",
            "value": []
          }
        for i in range(size):
          sub_msg_dict = construct_message_yaml(base_type)
          req_msg_dict[field_name]["value"].append(sub_msg_dict)
      else: # in-package message type
        for i in range(size):
          compact_field_type = req_type.split('/')[0] + "/" + base_type
          req_msg_dict[field_name] = {
            "type": f"list[{compact_field_type}]",
            "value": []
          }
          for i in range(size):
            sub_msg_dict = construct_message_yaml(compact_field_type)
            req_msg_dict[field_name]["value"].append(sub_msg_dict)
    
    else:   # not array
      
      if field_type in ros_to_cpp_type_map:
        req_msg_dict[field_name] = {
          "type": field_type,
          "value": None
        }
      elif '/' in field_type:   # out-of-package message type
        sub_msg_dict = construct_message_yaml(field_type)
        req_msg_dict[field_name] = sub_msg_dict
      else:   # in-package message type
        compact_field_type = req_type.split('/')[0] + "/" + field_type
        sub_msg_dict = construct_message_yaml(compact_field_type)
        req_msg_dict[field_name] = sub_msg_dict
  
  return {"type": req_type, "value": req_msg_dict}

import struct

import struct

def pack_default_input(type, value=None) -> bytes:
  if value is None:
    value = 0

  # --- bool ---
  if type == 'bool':
    return struct.pack("<?", bool(value))

  # --- 8 bit ---
  elif type in ["int8_t", "int8", "char"]:
    return struct.pack("<b", int(value))

  elif type in ["uint8_t", "uint8", "byte"]:
    return struct.pack("<B", int(value))

  # --- 16 bit ---
  elif type in ["int16_t", "int16"]:
    return struct.pack("<h", int(value))

  elif type in ["uint16_t", "uint16"]:
    return struct.pack("<H", int(value))

  # --- 32 bit ---
  elif type in ["int32_t", "int", "int32"]:
    return struct.pack("<i", int(value))

  elif type in ["uint32_t", "unsigned int", "uint32"]:
    return struct.pack("<I", int(value))

  # --- 64 bit ---
  elif type in ["int64_t", "long long", "int64"]:
    return struct.pack("<q", int(value))

  elif type in ["uint64_t", "unsigned long long", "uint64"]:
    return struct.pack("<Q", int(value))

  # --- floating ---
  elif type == "float":
    return struct.pack("<f", float(value))

  elif type == "double":
    return struct.pack("<d", float(value))

  # --- fallback ---
  else:
    return b''

def symbolize_parameter_by_param_dict(param_dict) -> str:
  
  sym_param = ""
  default_input = b''
  param_list = param_dict['parameters']
  param_setup = ""
  param_add = ""
  param_set = set()
  
  for param_info in param_list:
    param_type = param_info['type']
    param_name = param_info['name']
    param_default_value = param_info['default_value']
    param_value = param_info['value']
    
    if param_name in param_set: continue
    if "=" in param_name: continue
    param_set.add(param_name)
    
    
    safe_name = re.sub(r'[^A-Za-z0-9_.]', '_', param_name)
    safe_name = re.sub(r'\.', '__', safe_name)
    var_name = f"param__{safe_name}"
    
    array_match = re.match(r"^(.+?)\[(<=)?(\d*)\]$", param_type)
    
    if param_type.startswith("std::vector"):
      match = re.search(r'std::vector<(.+)>', param_type)
      if not match: continue
      base_type = match.group(1)
      
      sym_param += f"{param_type} {var_name};\n"
      
      if base_type == "std::string":
        if param_value is not None:
          for item in param_value:
            sym_param += f"{var_name}.push_back(\"{item}\");\n\n"
        elif param_default_value is not None:
          for i, item in enumerate(param_default_value):
            sym_param += f"uint8_t {var_name}_{i}_string_size;\n"
            sym_param += f"symros_make_symbolic_with_prefix(&{var_name}_{i}_string_size, sizeof(uint8_t), fd, \"{var_name}_{i}_string_size\", 2, \"uint8_t\");\n"
            sym_param += f"char {var_name}_{i}_string_data[MAX_LEN];\n"
            sym_param += f"symros_make_symbolic_with_prefix(&{var_name}_{i}_string_data, MAX_LEN, fd, \"{var_name}_{i}_string_data\", 2, \"char *\");\n"
            sym_param += f"{var_name}.push_back(std::string({var_name}_{i}_string_data, {var_name}_{i}_string_size));\n\n"
            default_input += struct.pack("B", len(item))
            default_input += struct.pack('64s', item.encode('utf-8'))
      else:
        if param_value is not None:
          for item in param_value:
            if base_type == "bool":
              item = "true" if item else "false"
            sym_param += f"{var_name}.push_back({item});\n\n"
        elif param_default_value is not None:
          for i, item in enumerate(param_default_value):
            sym_param += f"{base_type} {var_name}_{i};\n"
            sym_param += f"symros_make_symbolic_with_prefix(&{var_name}_{i}, sizeof({base_type}), fd, \"{var_name}_{i}\", 2, \"{base_type}\");\n"
            sym_param += f"{var_name}.push_back({var_name}_{i});\n\n"
            default_input += pack_default_input(base_type, item)
      
    else:
      if param_type == "std::string":
        if param_value is not None:
          sym_param += f"std::string {var_name} = \"{param_value}\";\n\n"
        else:
          sym_param += f"uint8_t {var_name}_string_size;\n"
          sym_param += f"symros_make_symbolic_with_prefix(&{var_name}_string_size, sizeof(uint8_t), fd, \"{var_name}_string_size\", 2, \"uint8_t\");\n"
          sym_param += f"char {var_name}_string_data[MAX_LEN];\n"
          sym_param += f"symros_make_symbolic_with_prefix(&{var_name}_string_data, MAX_LEN, fd, \"{var_name}_string_data\", 2, \"char *\");\n"
          sym_param += f"std::string {var_name}({var_name}_string_data, {var_name}_string_size);\n\n"
          # default input
          if param_default_value is not None:
            string_size = len(param_default_value)
            default_input += struct.pack("B", string_size)
            default_input += struct.pack('64s', param_default_value.encode('utf-8'))
          else:
            default_input += struct.pack("B", 0)
            default_input += struct.pack('64s', b'')
      else:
        if param_value is not None:
          if param_type == "bool":
            param_value = "true" if param_value else "false"
          sym_param += f"{param_type} {var_name} = {param_value};\n\n"
        else:
          if param_type == "int": param_type = "int64_t"
          sym_param += f"{param_type} {var_name};\n"
          sym_param += f"symros_make_symbolic_with_prefix(&{var_name}, sizeof({param_type}), fd, \"{var_name}\", 2, \"{param_type}\");\n\n"
          if param_default_value is not None:
            default_input += pack_default_input(param_type, param_default_value)
          else:
            default_input += pack_default_input(param_type, None)
    
    param_setup += f"rclcpp::Parameter(\"{param_name}\", {var_name}),\n"
    param_add += f"symros::SymROSManager::get_instance().symros_add_param(\"{param_name}\", {var_name});\n"
  
  # return f"{sym_param}\n" + f"\noptions.parameter_overrides({{\n{param_setup}}});\n\n", default_input
  return f"{sym_param}\n" + f"\n{param_add}\n\n", default_input

        
def symbolize_message_by_msg_dict(msg_dict, msg_name, is_request, is_action, is_target) -> str:
  ros_to_cpp_type_map = {
    "bool": "bool",
    "byte": "uint8_t",
    "char": "int8_t",
    "int8": "int8_t",
    "uint8": "uint8_t",
    "int16": "int16_t",
    "uint16": "uint16_t",
    "int32": "int32_t",
    "uint32": "uint32_t",
    "int64": "int64_t",
    "uint64": "uint64_t",
    "float32": "float",
    "float64": "double",
    "string": "std::string",
    "wstring": "std::wstring",
  }

  is_target_str = "1" if is_target else "0"
  msg_type = msg_dict['type']
  default_input = b''
  push_options = []
  if is_action:
    splitted = msg_type.split('/')
    act_srv_namespace = f"{splitted[0]}::action::{splitted[1]}"
    sym_msg_code = f"auto {msg_name} = std::make_shared<{act_srv_namespace}::Goal>();\n"
  
  elif is_request:
    splitted = msg_type.split('/')
    req_namespace = f"{splitted[0]}::srv::{splitted[1]}"
    sym_msg_code = f"auto {msg_name} = std::make_shared<{req_namespace}::Request>();\n"
  else:
    splitted = msg_type.split('/')
    msg_namespace = f"{splitted[0]}::msg::{splitted[1]}"
    sym_msg_code = f"auto {msg_name} = std::make_shared<{msg_namespace}>();\n"
  
  for field_name, field_info in msg_dict['value'].items():
    if "=" in field_name:
      continue
    tmp_field_name = f"{msg_name}_{field_name}"
    # TODO inject from default input
    
    if field_info['type'].startswith("list") or field_info['type'].startswith("vector"):
      base_type = field_info['type'].split('[')[1].split(']')[0]
      default_values = field_info['value']
      
      if base_type in ros_to_cpp_type_map:
        cpp_type = ros_to_cpp_type_map[base_type]
        i = 0
        for default_value in default_values:
          if cpp_type == "std::string":
            sym_msg_code += f"uint8_t {tmp_field_name}_{i}_string_size;\n"
            sym_msg_code += f"symros_make_symbolic_with_prefix(&{tmp_field_name}_{i}_string_size, sizeof(uint8_t), fd, \"{tmp_field_name}_{i}_string_size\", {is_target_str}, \"uint8_t\");\n"
            sym_msg_code += f"char {tmp_field_name}_{i}_string_data[MAX_LEN];\n"
            sym_msg_code += f"symros_make_symbolic_with_prefix(&{tmp_field_name}_{i}_string_data, MAX_LEN, fd, \"{tmp_field_name}_{i}_string_data\", {is_target_str}, \"char *\");\n"
            sym_msg_code += f"std::string {tmp_field_name}_{i}({tmp_field_name}_{i}_string_data, {tmp_field_name}_{i}_string_size);\n"

            if default_value == None:              
              default_input += struct.pack("B", 0)
              default_input += struct.pack('64s', b'')
            else:
              default_input += struct.pack("B", len(default_value))
              default_input += struct.pack("64s", default_value.encode('utf-8'))
          else:
            sym_msg_code += f"{cpp_type} {tmp_field_name}_{i};\n"
            sym_msg_code += f"symros_make_symbolic_with_prefix(&{tmp_field_name}_{i}, sizeof({cpp_type}), fd, \"{tmp_field_name}_{i}\", {is_target_str}, \"{cpp_type}\");\n"
            default_input += pack_default_input(cpp_type, default_value)
          i += 1 
        
        if field_info['type'].startswith("vector"):
          sym_msg_code += f"bool empty_{tmp_field_name};\n"
          sym_msg_code += f"symros_make_symbolic_with_prefix(&empty_{tmp_field_name}, sizeof(bool), fd, \"empty_{tmp_field_name}\", 2, \"bool\");\n"
          default_input += pack_default_input("bool", False)
          sym_msg_code += f"\nif (!empty_{tmp_field_name}) {{\n\n"
          push_options.append(f"empty_{tmp_field_name}")
        
        for i in range(len(default_values)):
          if field_info['type'].startswith("vector"):
            # TODO symbolic push option value
            sym_msg_code += f"{msg_name}->{field_name}.push_back({tmp_field_name}_{i});\n\n"
          else:
            sym_msg_code += f"{msg_name}->{field_name}[{i}] = {tmp_field_name}_{i};\n\n"
        
        if field_info['type'].startswith("vector"):
          sym_msg_code += f"\n}}\n\n"
        
      else:
        i = 0
        for default_value in default_values:
          sub_sym_msg_code, sub_default_input, sub_push_options= symbolize_message_by_msg_dict(default_value, f"{tmp_field_name}_{i}", False, False, is_target)
          sym_msg_code += sub_sym_msg_code
          default_input += sub_default_input
          push_options.extend(sub_push_options)
          i += 1
          
        if field_info['type'].startswith("vector"):
          sym_msg_code += f"bool empty_{tmp_field_name};\n"
          sym_msg_code += f"symros_make_symbolic_with_prefix(&empty_{tmp_field_name}, sizeof(bool), fd, \"empty_{tmp_field_name}\", 2, \"bool\");\n"
          default_input += pack_default_input("bool", False)
          sym_msg_code += f"\nif (!empty_{tmp_field_name}) {{\n\n"
          push_options.append(f"empty_{tmp_field_name}")
          
        for i in range(len(default_values)):
          if field_info['type'].startswith("vector"):
            # TODO symbolic push option value
            sym_msg_code += f"{msg_name}->{field_name}.push_back(*{tmp_field_name}_{i});\n\n"
          else:
            sym_msg_code += f"{msg_name}->{field_name}[{i}] = *{tmp_field_name}_{i};\n\n"
        
        if field_info['type'].startswith("vector"):
          sym_msg_code += f"\n}}\n\n"
    else:
      if field_info['type'] in ros_to_cpp_type_map:
        cpp_type = ros_to_cpp_type_map[field_info['type']]
        if cpp_type == "std::string":
          sym_msg_code += f"uint8_t {tmp_field_name}_string_size;\n"
          sym_msg_code += f"symros_make_symbolic_with_prefix(&{tmp_field_name}_string_size, sizeof(uint8_t), fd, \"{tmp_field_name}_string_size\", {is_target_str}, \"uint8_t\");\n"
          sym_msg_code += f"char {tmp_field_name}_string_data[MAX_LEN];\n"
          sym_msg_code += f"symros_make_symbolic_with_prefix(&{tmp_field_name}_string_data, MAX_LEN, fd, \"{tmp_field_name}_string_data\", {is_target_str}, \"char *\");\n"
          sym_msg_code += f"std::string {tmp_field_name}({tmp_field_name}_string_data, {tmp_field_name}_string_size);\n"
          if field_info['value'] == None:
            default_input += struct.pack("B", 0)
            default_input += struct.pack('64s', b'')
          else:
            default_input += struct.pack("B", len(field_info['value']))
            default_input += struct.pack('64s', field_info['value'].encode('utf-8'))
        else:
          sym_msg_code += f"{cpp_type} {tmp_field_name};\n"
          sym_msg_code += f"symros_make_symbolic_with_prefix(&{tmp_field_name}, sizeof({cpp_type}), fd, \"{tmp_field_name}\", {is_target_str}, \"{cpp_type}\");\n"
          default_input += pack_default_input(cpp_type, field_info['value'])
        sym_msg_code += f"{msg_name}->{field_name} = {tmp_field_name};\n\n"
      else:
        sub_sym_msg_code, sub_default_input, sub_push_options = symbolize_message_by_msg_dict(field_info, tmp_field_name, False, False, is_target)
        sym_msg_code += sub_sym_msg_code
        default_input += sub_default_input
        push_options.extend(sub_push_options)
        sym_msg_code += f"{msg_name}->{field_name} = *{tmp_field_name};\n\n"

  return sym_msg_code, default_input, push_options

def construct_callback_info_yaml(callback: Callback):
  if callback.cb_type == CB_TYPE.SUB:
    return construct_message_yaml(callback.msg_name)
  if callback.cb_type == CB_TYPE.SRV:
    return construct_request_yaml(callback.msg_name)
  if callback.cb_type == CB_TYPE.ACT:
    return construct_action_server_yaml(callback.msg_name)

def construct_test(result_dir, target_callbacks, n_target_callbacks, test_id, is_state_mode):
  
  # generate executor based on callstack
  default_input = b''
  symbolic_barrier_start = "\nprintf(\"\\n==== SYMBOLIC BARRIER START ====\\n\");fflush(stdout);\n"
  symbolic_barrier_stop = "\nprintf(\"\\n==== SYMBOLIC BARRIER STOP ====\\n\");fflush(stdout);\n"
 
  # generate executor for target callback (config + target callback)
  open_file = "\nint fd = open(argv[1], O_RDONLY);\n\n"
  set_mode = "STATE_MODE = argv[2][0] == \'1\';\n\n" if is_state_mode else ""
  
  # parameter symbolization
  if os.path.exists(f"{result_dir}/test_{test_id}/param_info.yaml"):
    with open(f"{result_dir}/test_{test_id}/param_info.yaml", "r", encoding="utf-8") as f:
      param_dict = yaml.safe_load(f)
  else:
    with open(f"{result_dir}/config/param_info.yaml", "r", encoding="utf-8") as f:
      param_dict = yaml.safe_load(f)

  sym_param, param_default_input = symbolize_parameter_by_param_dict(param_dict)
  
  default_input += param_default_input
  
  # config code injection
  symros_init = "\nSYMROS__INIT(false);\n\n"

  with open(f"{result_dir}/config/entry_code.cpp", 'r') as file:
    snippet = file.read()

  entry_code_executor = snippet.split('// SYMROS executor')[1]
  entry_code_setup = snippet.split('// SYMROS setup')[1]
  node_setup = f"{open_file}{set_mode}{symros_init}{symbolic_barrier_start}{sym_param}{symbolic_barrier_stop}{entry_code_executor}"
  
  declare_sub = ""
  sym_and_call = ""
  
  # callback code
  cnt = 0
  target_offset = 0
  n_closed = 0
  is_target = False
  for idx, target_callback in enumerate(target_callbacks):
    if len(target_callbacks) - idx == n_target_callbacks:   # start point of target
      target_offset = len(default_input)
      is_target = True
      start_target = "\n\nprintf(\"\\n===== START CALLBACK TEST =====\\n\");fflush(stdout);\n\n"
    else:
      start_target = ""
    
    # activation function
    if target_callback['callback_id'] == 0:
      sym_message_ = ""
      call_callback_ = "node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);\n"
    else: 
      with open(f"{result_dir}/config/callback_info.yaml", 'r', encoding="utf-8") as f:
        all_callback_info = yaml.safe_load(f)
        # target_callback_info = yaml.safe_load(f)
      target_callback_info = next(
        (callback_info for callback_info in all_callback_info if callback_info.get("callback_id") == target_callback['callback_id']),
        None
      ) 
      if target_callback_info['cb_type'] == 1:
        splitted = target_callback_info['msg_info']['type'].split('/')
        message_namespace_ = f"{splitted[0]}::msg::{splitted[1]}"
        message_name_ = f"msg_{cnt}"
        subscription_name_ = f"sub_{cnt}"
        declare_sub += f"SYMROS__DECLARE_SUBSCRIPTION({message_namespace_}, {subscription_name_}, {target_callback_info['callee_idx']});\n"
        call_callback_ = f"SYMROS__CALL_SUBSCRIPTION_CALLBACK({subscription_name_}, {message_name_}, {message_name_}_info_);\n"
        sym_message_, msg_default_input, push_options = symbolize_message_by_msg_dict(target_callback['default'], message_name_, False, False, is_target)
        default_input += msg_default_input
      elif target_callback_info['cb_type'] == 2:
        splitted = target_callback_info['msg_info']['type'].split('/')
        service_namespace_ = f"{splitted[0]}::srv::{splitted[1]}"
        request_name_ = f"req_{cnt}"
        service_name_ = f"srv_{cnt}"
        declare_sub += f"SYMROS__DECLARE_SERVICE({service_namespace_}, {service_name_}, {target_callback_info['callee_idx']});\n"
        call_callback_ = f"SYMROS__CALL_SERVICE_CALLBACK({service_name_}, {request_name_});\n"
        sym_message_, req_default_input, push_options = symbolize_message_by_msg_dict(target_callback['default'], request_name_, True, False, is_target)
        default_input += req_default_input
      else:
        splitted = target_callback_info['msg_info']['type'].split('/')
        action_server_namespace_ = f"{splitted[0]}::action::{splitted[1]}"
        action_request_name_ = f"act_req_{cnt}"
        action_server_name = f"act_srv_{cnt}"
        declare_sub += f"SYMROS__DECLARE_ACTION_SERVER({action_server_namespace_}, {action_server_name}, {target_callback_info['callee_idx']});\n"
        call_callback_ = f"SYMROS_CallActionServerCallback<{action_server_namespace_}>({action_server_name}, {action_request_name_});\n"
        sym_message_, req_default_input, push_options = symbolize_message_by_msg_dict(target_callback['default'], action_request_name_, False, True, is_target)
        default_input += req_default_input
        
      # if target_callback['condition'] is not None:
      #   sym_and_call += f"if ( {target_callback['condition']} ) {{\n"
      #   n_closed += 1

      if target_callback['optional']:
        sym_and_call += f"bool call_option_{target_callback['callback_id']};\n"
        sym_and_call += f""
        check_option_begin = f"symros_make_symbolic_with_prefix(&call_option_{target_callback['callback_id']}, sizeof(bool), fd, \"call_option_{target_callback['callback_id']}\", 2, \"bool\");\nif (call_option_{target_callback['callback_id']}) {{\n\n"
        check_option_end = "}\n\n"
        default_input += b"\x00"
      else:
        check_option_begin = ""
        check_option_end = ""
      
      push_options_ = ""
      for push_option in push_options:
        push_options_ += f"\nif ({push_option}) {{" + "}\n"


    sym_and_call += f"{start_target}{symbolic_barrier_start}{sym_message_}{symbolic_barrier_stop}\n\n{push_options_}\n\n{check_option_begin}{call_callback_}{check_option_end}\n\n"

    cnt += 1
  
  sym_and_call += "}" * n_closed
  
  symros_init = "\nSYMROS__INIT(true);\n\n"
  close_file = "\nclose(fd);\n\n"
  callback_start = "printf(\"[symros] callback test start\\n\");\n"
  callback_finish = "printf(\"[symros] callback test finish\\n\");\n"
  
  if "node->trigger_transition" in sym_and_call:
    cleanup_node= "\nnode->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);\nnode->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);\nnode->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_DESTROY);\n"
  else:
    cleanup_node= ""
  target_size = len(default_input) - target_offset
  
  return default_input, target_offset, target_size, entry_code_setup, f"{node_setup}{symros_init}{declare_sub}{sym_and_call}{callback_finish}{close_file}{cleanup_node}"
  