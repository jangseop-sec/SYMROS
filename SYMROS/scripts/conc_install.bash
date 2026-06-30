#!/bin/bash

SYMROS_PROJ=/home/ubuntu/symros/SYMROS
WS=$1
INPUT_FILE=$2

# install concrete symros executor
source $WS/install/setup.bash

# install concrete ros2 client library
source $SYMROS_PROJ/ros/conc/humble/install/setup.bash


# disable symbolic ros2-nodds library path
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
  eval "export $var=\"\$(echo \${$var} | tr ':' '\n' | grep -v '/home/ubuntu/symros/SYMROS/ros-nodds/build-sym' | paste -sd: -)\""
done

# disable symbolic ros2 client library paths
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
  eval "export $var=\"\$(echo \${$var} | tr ':' '\n' | grep -v '/ros/sym' | paste -sd: -)\""
done

# disable symbolic target workspace paths
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
  eval "export $var=\"\$(echo \${$var} | tr ':' '\n' | grep -v 'symros_ws' | paste -sd: -)\""
done

export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "/home/ubuntu/symros/SYMROS/ros-nodds/build-sym" | paste -sd:)
export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libatomic.so.1:/home/ubuntu/symros/SYMROS/ros-nodds/build-conc/libros2_no_dds.so"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SYMROS_PROJ/ros2-nodds/build-conc
export ASAN_OPTIONS=detect_leaks=0:detect_odr_violation=0
echo $ASAN_OPTIONS

# prohibit core dump
ulimit -c 0


# execute
$WS/build/symros_exec/exec "$INPUT_FILE" "0"
# ros2 run symros_exec exec "$INPUT_FILE" "0"
