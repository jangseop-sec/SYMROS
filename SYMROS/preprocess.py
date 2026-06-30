import os
import subprocess
import sys
import re
import shlex

import type
from type import Topic, Service, Callback, Class, CB_TYPE, Action
from db import CALLBACKS, TOPICS, SERVICES, CLASS, ACTIONS

from pathlib import Path
import xml.etree.ElementTree as ET

message_packages = [
  "unique_identifier_msgs", "map_msgs", "nav_msgs", "actionlib_msgs",
  "visualization_msgs", "statistics_msgs", "action_msgs", "rmw_dds_common",
  "trajectory_msgs", "example_interfaces", "diagnostic_msgs",
  "tf2_msgs", "stereo_msgs", "shape_msgs", "std_msgs",
  "nav_2d_msgs", "geometry_msgs", "rosgraph_msgs", "rosbag2_interfaces",
  "rcl_interfaces", "bond", "turtlesim", "builtin_interfaces",
  "dwb_msgs", "lifecycle_msgs", "nav2_msgs", "sensor_msgs", 
  "octomap_msgs"
]

def pascal_to_snake(name: str) -> str:
    s = re.sub(r'(?<=[A-Z])(?=[A-Z][a-z])', '_', name)
    s = re.sub(r'(?<=[a-z0-9])(?=[A-Z])', '_', s)
    return s.lower()
  
def get_deps(package_xml: str) -> (str, list[str], list[str], list[str]):
  tree = ET.parse(package_xml)
  root = tree.getroot()
  
  depends = [dep.text.strip() for dep in root.findall("depend")]
  build_depends = [dep.text.strip() for dep in root.findall("build_depend")]
  exec_depends = [dep.text.strip() for dep in root.findall("exec_depend")]
  
  name = [dep.text.strip() for dep in root.findall("name")]
  
  return name[0], depends, build_depends, exec_depends

def get_find_packages(target_dir: str) -> list[str]:
    """
    """
    with open(f'{target_dir}/CMakeLists.txt', 'r') as file:
      cmake_text = file.read()
    
    pattern = re.compile(r'(?is)find_package\s*\((.*?)\)')
    deps = []
    for m in pattern.finditer(cmake_text):
        inside = m.group(1)

        inside_nocom = re.sub(r'(?m)#.*$', '', inside)

        tokens = shlex.split(inside_nocom, posix=True)
        if not tokens:
            continue

        pkg = tokens[0]

        if any(tok.upper() == 'REQUIRED' for tok in tokens[1:]):
            deps.append(pkg)

    seen, out = set(), []
    for d in deps:
        if d not in seen:
            seen.add(d)
            out.append(d)
    return out

def parse_msg_file(file_path: str) -> Topic:
  with open(file_path, 'r') as f:
    lines = f.readlines()
  
  msg_name = '::'.join(file_path.split('/')[-3:])
  msg_name = msg_name.replace('.msg', '')
  fields = []
  
  for line in lines:
    line = line.strip()
    if line and not line.startswith('#'):
      parts = line.split()
      if len(parts) >= 2:
        field_type = parts[0]
        field_name = parts[1]
        fields.append((field_type, field_name))
  
  splitted_msg_name = msg_name.split('::')
  splitted_msg_name[-1] = pascal_to_snake(splitted_msg_name[-1]) + ".hpp"
  filename = "/".join(splitted_msg_name)
  
  return Topic(msg_name, fields, filename)


def parse_srv_file(file_path: str) -> Service:
  with open(file_path, 'r') as f:
    content = f.read()
  
  srv_name = '::'.join(file_path.split('/')[-3:])
  srv_name = srv_name.replace('.srv', '')
  req_fields = []
  res_fields = []
  
  if '---' in content:
    req_part, res_part = content.split('---', 1)
    req_lines = req_part.strip().splitlines()
    res_lines = res_part.strip().splitlines()
    
    for line in req_lines:
      line = line.strip()
      if line and not line.startswith('#'):
        parts = line.split()
        if len(parts) >= 2:
          field_type = parts[0]
          field_name = parts[1]
          req_fields.append((field_type, field_name))
    
    for line in res_lines:
      line = line.strip()
      if line and not line.startswith('#'):
        parts = line.split()
        if len(parts) >= 2:
          field_type = parts[0]
          field_name = parts[1]
          res_fields.append((field_type, field_name))
  
  splitted_srv_name = srv_name.split('::')
  splitted_srv_name[-1] = pascal_to_snake(splitted_srv_name[-1]) + ".hpp"
  filename = '/'.join(splitted_srv_name)
  
  return Service(srv_name, req_fields, res_fields, filename)

def parse_act_file(file_path: str) -> Action:
  with open(file_path, 'r') as f:
    content = f.read()
  
  act_name = '::'.join(file_path.split('/')[-3:])
  act_name = act_name.replace('.action', '')
  goal_fields = []
  result_fields = []
  feedback_fields = []
  
  if '---' in content:
    goal_part = content.split('---')[0].splitlines()
    result_part = content.split('---')[1].splitlines()
    feedback_part = content.split('---')[2].splitlines()
    
    for line in goal_part:
      line = line.strip()
      if line and not line.startswith('#') and not '=' in line:
        parts = line.split()
        if len(parts) >= 2:
          field_type = parts[0]
          field_name = parts[1]
          goal_fields.append((field_type, field_name))
    
    for line in result_part:
      line = line.strip()
      if line and not line.startswith('#') and not '=' in line:
        parts = line.split()
        if len(parts) >= 2:
          field_type = parts[0]
          field_name = parts[1]
          result_fields.append((field_type, field_name))
    
    for line in feedback_part:
      line = line.strip()
      if line and not line.startswith('#') and not '=' in line:
        parts = line.split()
        if len(parts) >= 2:
          field_type = parts[0]
          field_name = parts[1]
          feedback_fields.append((field_type, field_name))
  splitted_act_name = act_name.split('::')
  splitted_act_name[-1] = pascal_to_snake(splitted_act_name[-1]) + ".hpp"
  filename = '/'.join(splitted_act_name)
  
  return Action(act_name, goal_fields, result_fields, feedback_fields, filename)

def find_all_msgs(root_dir: str, target_pkg: str) -> list[Topic]:
  
  find_topics =[]
  
  pkg_name, deps, build_deps, exec_deps = get_deps(f'{target_pkg}/package.xml')
  deps = deps + build_deps + exec_deps
  
  message_packages = [
    "unique_identifier_msgs", "map_msgs", "nav_msgs", "actionlib_msgs",
    "visualization_msgs", "statistics_msgs", "action_msgs", "rmw_dds_common",
    "trajectory_msgs", "example_interfaces", "diagnostic_msgs",
    "tf2_msgs", "stereo_msgs", "shape_msgs", "std_msgs",
    "nav_2d_msgs", "geometry_msgs", "rosgraph_msgs", "rosbag2_interfaces",
    "rcl_interfaces", "bond", "turtlesim", "builtin_interfaces",
    "dwb_msgs", "lifecycle_msgs", "nav2_msgs", "sensor_msgs", 
    "octomap_msgs"
  ]
  
  for dirpath, dirnames, filenames in os.walk(root_dir):
    for filename in filenames:
      if filename.endswith('.msg'):
        full_path = os.path.join(dirpath, filename)
        curr_topic = parse_msg_file(full_path)
        
        if curr_topic.type.split('::')[0] in message_packages:        
          find_topics.append(curr_topic)
          global TOPICS
          TOPICS.append(curr_topic)
  
  for dirpath, dirnames, filenames in os.walk(target_pkg):
    for filename in filenames:
      if filename.endswith('.msg'):
        full_path = os.path.join(dirpath, filename)
        curr_topic = parse_msg_file(full_path)
        
        if curr_topic.type.split('::')[0] == pkg_name:        
          find_topics.append(curr_topic)
          TOPICS.append(curr_topic)
          
  return find_topics

def find_all_srvs(root_dir: str, target_pkg: str) -> list[Service]:
  find_services = []
  pkg_name, deps, build_deps, exec_deps = get_deps(f'{target_pkg}/package.xml')
  deps = deps + build_deps + exec_deps
  
  message_packages = [
    "unique_identifier_msgs", "map_msgs", "nav_msgs", "actionlib_msgs",
    "visualization_msgs", "statistics_msgs", "action_msgs", "rmw_dds_common",
    "trajectory_msgs", "example_interfaces", "diagnostic_msgs",
    "tf2_msgs", "stereo_msgs", "shape_msgs", "std_msgs",
    "nav_2d_msgs", "geometry_msgs", "rosgraph_msgs", "rosbag2_interfaces",
    "rcl_interfaces", "bond", "turtlesim", "builtin_interfaces",
    "dwb_msgs", "lifecycle_msgs", "nav2_msgs", "sensor_msgs", "std_srvs", 
    
  ]
  
  for dirpath, dirnames, filenames in os.walk(root_dir):
    for filename in filenames:
      if filename.endswith('.srv'):
        full_path = os.path.join(dirpath, filename)
        curr_service = parse_srv_file(full_path)
        
        if curr_service.type.split('::')[0] in message_packages:
          find_services.append(curr_service)  
          global SERVICES
          SERVICES.append(curr_service)

  for dirpath, dirnames, filenames in os.walk(target_pkg):
    for filename in filenames:
      if filename.endswith('.srv'):
        full_path = os.path.join(dirpath, filename)
        curr_service = parse_srv_file(full_path)
        
        if curr_service.type.split('::')[0] == pkg_name:
          find_services.append(curr_service)  
          SERVICES.append(curr_service)
  
  return find_services

def find_all_acts(root_dir: str, target_pkg: str) -> list[Action]:
  find_actions = []
  pkg_name, deps, build_deps, exec_deps = get_deps(f'{target_pkg}/package.xml')
  deps = deps + build_deps + exec_deps
  
  message_packages = [
    "unique_identifier_msgs", "map_msgs", "nav_msgs", "actionlib_msgs",
    "visualization_msgs", "statistics_msgs", "action_msgs", "rmw_dds_common",
    "trajectory_msgs", "example_interfaces", "diagnostic_msgs",
    "tf2_msgs", "stereo_msgs", "shape_msgs", "std_msgs",
    "nav_2d_msgs", "geometry_msgs", "rosgraph_msgs", "rosbag2_interfaces",
    "rcl_interfaces", "bond", "turtlesim", "builtin_interfaces",
    "dwb_msgs", "lifecycle_msgs", "nav2_msgs", "sensor_msgs", "std_srvs", 
    
  ]
  
  for dirpath, dirnames, filenames in os.walk(root_dir):
    for filename in filenames:
      if filename.endswith('.action'):
        full_path = os.path.join(dirpath, filename)
        curr_action = parse_act_file(full_path)
        
        if curr_action.type.split('::')[0] in message_packages:
          find_actions.append(curr_action)  
          global ACTIONS
          ACTIONS.append(curr_action)

  for dirpath, dirnames, filenames in os.walk(target_pkg):
    for filename in filenames:
      if filename.endswith('.action'):
        full_path = os.path.join(dirpath, filename)
        curr_action = parse_act_file(full_path)
        
        if curr_action.type.split('::')[0] == pkg_name:
          find_actions.append(curr_action)  
          ACTIONS.append(curr_action)
          
  return find_actions

def gen_symros_pkg(ws_dir: str, target_dir: str):
  """
  - create executable pkg: symros_exec in target workspace
    - package.xml
    - CMakeLists.txt
    - src/main.cpp
    - include/
    - package.xml
    - license
  """
  
  symros_exec = f'{ws_dir}/src/symros_exec'
  
  with open('./res/LICENSE.tm', 'r') as file:
    LICENSE = file.read()
  
  with open('./res/CMakeLists.sym.txt', 'r') as file:
    CMakeLists_txt = file.read()
  
  with open('./res/package.tm.xml', 'r') as file:
    package_xml = file.read()  
  
  with open('./res/main.sym.cpp', 'r') as file:
    main_cpp = file.read()
  
  os.makedirs(f'{symros_exec}/src', exist_ok=True)       # source directory
  os.makedirs(f'{symros_exec}/include', exist_ok=True)   # include directory

  target_name, dependencies, build_deps, exec_deps = get_deps(f'{target_dir}/package.xml')
  
  # builtin message type in ROS2 library
  message_packages = [
    "unique_identifier_msgs", "map_msgs", "nav_msgs", "actionlib_msgs",
    "visualization_msgs", "statistics_msgs", "action_msgs", "rmw_dds_common",
    "trajectory_msgs", "example_interfaces", "diagnostic_msgs",
    "tf2_msgs", "stereo_msgs", "shape_msgs", "std_msgs",
    "nav_2d_msgs", "geometry_msgs", "rosgraph_msgs", "rosbag2_interfaces",
    "rcl_interfaces", "bond", "turtlesim", "builtin_interfaces",
    "dwb_msgs", "lifecycle_msgs", "nav2_msgs", "sensor_msgs", "std_srvs", 
  ]
  
  # inject dependencies into package.xml
  depends_in_xml = ""
  for dep in build_deps:
    depends_in_xml += f"<build_depend>{dep}</build_depend>\n"
  depends_in_xml += '\n'
  for dep in dependencies:
    depends_in_xml += f"<depend>{dep}</depend>\n"
  for dep in message_packages:
    if dep not in dependencies and dep not in build_deps and dep not in exec_deps:
      depends_in_xml += f"<depend>{dep}</depend>\n"
  depends_in_xml += f"<depend>{target_name}</depend>\n\n"
  for dep in exec_deps:
    depends_in_xml += f"<exec_depend>{dep}</exec_depend>\n"
    
  splitted_package_xml = package_xml.split('<!-- SYMROS dependencies -->')
  splitted_package_xml[1] = '\n' + depends_in_xml + '\n'
  package_xml = '<!-- SYMROS dependencies -->'.join(splitted_package_xml)

  # inject dependencies into CMakeLitst.txt
  splitted_CMakeLists_txt = CMakeLists_txt.split('# SYMROS find target application')
  splitted_CMakeLists_txt[1] = f'\nfind_package({target_name} REQUIRED)\n'
  CMakeLists_txt = '# SYMROS find target application'.join(splitted_CMakeLists_txt)
  
  splitted_CMakeLists_txt = CMakeLists_txt.split("# SYMROS find target's dependencies")
  deps_in_cmake = ""
  find_packages = get_find_packages(target_dir)
  
  for msg_pkg in message_packages:
    if msg_pkg not in find_packages:
      find_packages.append(msg_pkg)
    
  for dep in find_packages:
    deps_in_cmake += f"find_package({dep} REQUIRED)\n"
  
  splitted_CMakeLists_txt[1] = f"\n{deps_in_cmake}\n"
  CMakeLists_txt = "# SYMROS find target's dependencies".join(splitted_CMakeLists_txt)
  
  splitted_CMakeLists_txt = CMakeLists_txt.split("# SYMROS set dependencies")
  deps_in_cmake = '\n' + '\n'.join(find_packages) + '\n' + target_name + '\n'
  splitted_CMakeLists_txt[1] = deps_in_cmake
  CMakeLists_txt = "# SYMROS set dependencies".join(splitted_CMakeLists_txt)
  
  # ineject header information into main.cpp
  
  # message header 
  msg_headers_in_main_cpp = "\n"
  for topic in TOPICS:
    msg_headers_in_main_cpp += f"#include \"{topic.filename}\"\n"
  srv_headers_in_main_cpp = "\n"
  for service in SERVICES:
    srv_headers_in_main_cpp += f"#include \"{service.filename}\"\n"
  act_headers_in_main_cpp = "\n"
  for action in ACTIONS:
    act_headers_in_main_cpp += f"#include \"{action.filename}\"\n"
  splitted_main_cpp = main_cpp.split("// SYMROS message headers")
  splitted_main_cpp[1] = msg_headers_in_main_cpp + srv_headers_in_main_cpp + act_headers_in_main_cpp
  main_cpp = "// SYMROS message headers".join(splitted_main_cpp)
  
  # target include header
  target_header_in_main_cpp = "\n"
  include_path = f"{target_dir}/include"
  if os.path.isdir(include_path): 
    for dirpath, _, filenames in os.walk(include_path):
      for f in filenames:
        if f.endswith((".h", ".hpp")):
          rel_path = os.path.relpath(os.path.join(dirpath, f), include_path)
          target_header_in_main_cpp += f"#include \"{rel_path}\"\n"
  splitted_main_cpp = main_cpp.split("// SYMROS target include headers")
  splitted_main_cpp[1] = target_header_in_main_cpp
  main_cpp = "// SYMROS target include headers".join(splitted_main_cpp)
  
  # write code in file
  with open(f'{symros_exec}/LICENSE', 'w') as file:
    file.write(LICENSE)
  
  with open(f'{symros_exec}/CMakeLists.txt', 'w') as file:
    file.write(CMakeLists_txt)
  
  with open(f'{symros_exec}/package.xml', 'w') as file:
    file.write(package_xml)
  
  with open(f'{symros_exec}/src/main.cpp', 'w') as file:
    file.write(main_cpp)

def gen_conc_pkg(ws_dir: str, target_dir: str):
  """
  - create executable pkg: symros_exec in target workspace
    - package.xml
    - CMakeLists.txt
    - src/main.cpp
    - include/
    - package.xml
    - license
  """
  
  symros_exec = f'{ws_dir}/src/symros_exec'
  
  with open('./res/LICENSE.tm', 'r') as file:
    LICENSE = file.read()
  
  with open('./res/CMakeLists.conc.txt', 'r') as file:
    CMakeLists_txt = file.read()
  
  with open('./res/package.tm.xml', 'r') as file:
    package_xml = file.read()  
  
  with open('./res/main.conc.cpp', 'r') as file:
    main_cpp = file.read()
  
  os.makedirs(f'{symros_exec}/src', exist_ok=True)       # source directory
  os.makedirs(f'{symros_exec}/include', exist_ok=True)   # include directory
  
  target_name, dependencies, build_deps, exec_deps = get_deps(f'{target_dir}/package.xml')
  
  # builtin message type in ROS2 library
  message_packages = [
    "unique_identifier_msgs", "map_msgs", "nav_msgs", "actionlib_msgs",
    "visualization_msgs", "statistics_msgs", "action_msgs", "rmw_dds_common",
    "trajectory_msgs", "example_interfaces", "diagnostic_msgs",
    "tf2_msgs", "stereo_msgs", "shape_msgs", "std_msgs",
    "nav_2d_msgs", "geometry_msgs", "rosgraph_msgs", "rosbag2_interfaces",
    "rcl_interfaces", "bond", "turtlesim", "builtin_interfaces",
    "dwb_msgs", "lifecycle_msgs", "nav2_msgs", "sensor_msgs", "std_srvs", 
  ]
  
  # inject dependencies into package.xml
  depends_in_xml = ""
  for dep in build_deps:
    depends_in_xml += f"\t<build_depend>{dep}</build_depend>\n"
  depends_in_xml += '\n'
  for dep in dependencies:
    depends_in_xml += f"\t<depend>{dep}</depend>\n"
  depends_in_xml += '\n'
  for dep in message_packages:
    if dep not in dependencies and dep not in build_deps and dep not in exec_deps:
      depends_in_xml += f"\t<depend>{dep}</depend>\n"
  depends_in_xml += '\n'
    
  depends_in_xml += f"\t<depend>{target_name}</depend>\n\n"
  for dep in exec_deps:
    depends_in_xml += f"\t<exec_depend>{dep}</exec_depend>\n"
  
  splitted_package_xml = package_xml.split('<!-- SYMROS dependencies -->')
  splitted_package_xml[1] = '\n' + depends_in_xml + '\n'
  package_xml = '<!-- SYMROS dependencies -->'.join(splitted_package_xml)

  # inject dependencies into CMakeLitst.txt
  splitted_CMakeLists_txt = CMakeLists_txt.split('# SYMROS find target application')
  splitted_CMakeLists_txt[1] = f'\nfind_package({target_name} REQUIRED)\n'
  CMakeLists_txt = '# SYMROS find target application'.join(splitted_CMakeLists_txt)
  
  splitted_CMakeLists_txt = CMakeLists_txt.split("# SYMROS find target's dependencies")
  deps_in_cmake = ""
  find_packages = get_find_packages(target_dir)
  
  for msg_pkg in message_packages:
    if msg_pkg not in find_packages:
      find_packages.append(msg_pkg)
  
  for dep in find_packages:
    deps_in_cmake += f"find_package({dep} REQUIRED)\n"
  splitted_CMakeLists_txt[1] = f"\n{deps_in_cmake}\n"
  CMakeLists_txt = "# SYMROS find target's dependencies".join(splitted_CMakeLists_txt)
  
  splitted_CMakeLists_txt = CMakeLists_txt.split("# SYMROS set dependencies")
  print(splitted_CMakeLists_txt[1])
  deps_in_cmake = '\n' + '\n'.join(find_packages) + '\n' + target_name + '\n'
  splitted_CMakeLists_txt[1] = deps_in_cmake
  CMakeLists_txt = "# SYMROS set dependencies".join(splitted_CMakeLists_txt)
  
  # ineject header information into main.cpp
  
  # message header 
  msg_headers_in_main_cpp = "\n"
  for topic in TOPICS:
    msg_headers_in_main_cpp += f"#include \"{topic.filename}\"\n"
  srv_headers_in_main_cpp = "\n"
  for service in SERVICES:
    srv_headers_in_main_cpp += f"#include \"{service.filename}\"\n"
  act_headers_in_main_cpp = "\n"
  for action in ACTIONS:
    act_headers_in_main_cpp += f"#include \"{action.filename}\"\n"
  splitted_main_cpp = main_cpp.split("// SYMROS message headers")
  splitted_main_cpp[1] = msg_headers_in_main_cpp + srv_headers_in_main_cpp + act_headers_in_main_cpp
  main_cpp = "// SYMROS message headers".join(splitted_main_cpp)
  
  # target include header
  target_header_in_main_cpp = "\n"
  include_path = f"{target_dir}/include"
  if os.path.isdir(include_path): 
    for dirpath, _, filenames in os.walk(include_path):
      for f in filenames:
        if f.endswith((".h", ".hpp")):
          rel_path = os.path.relpath(os.path.join(dirpath, f), include_path)
          target_header_in_main_cpp += f"#include \"{rel_path}\"\n"
  splitted_main_cpp = main_cpp.split("// SYMROS target include headers")
  splitted_main_cpp[1] = target_header_in_main_cpp
  main_cpp = "// SYMROS target include headers".join(splitted_main_cpp)
  
  with open(f'{symros_exec}/LICENSE', 'w') as file:
    file.write(LICENSE)
  
  with open(f'{symros_exec}/CMakeLists.txt', 'w') as file:
    file.write(CMakeLists_txt)
  
  with open(f'{symros_exec}/package.xml', 'w') as file:
    file.write(package_xml)
  
  with open(f'{symros_exec}/src/main.cpp', 'w') as file:
    file.write(main_cpp)
