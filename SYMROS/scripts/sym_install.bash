#!/bin/bash

SYMROS_PROJ=/home/ubuntu/symros/SYMROS
WS=$1
INPUT_FILE=$2
STATE_MODE=$3

unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH
# install concrete symros executor
source $WS/install/setup.bash

# install symbolic ros2 client library (libc++ base; must match sym_build.bash)
source $SYMROS_PROJ/ros/sym/humble/install/setup.bash

# disable concrete ros2-nodds library path
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
  eval "export $var=\"\$(echo \${$var} | tr ':' '\n' | grep -v '/home/ubuntu/symros/SYMROS/ros-nodds/build-conc' | paste -sd: -)\""
done

# disable concrete ros2 client library paths
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
  eval "export $var=\"\$(echo \${$var} | tr ':' '\n' | grep -v '/ros/conc' | paste -sd: -)\""
done

# disable concrete target workspace paths
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
  eval "export $var=\"\$(echo \${$var} | tr ':' '\n' | grep -v 'concrete_ws' | paste -sd: -)\""
done

export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v "/home/ubuntu/symros/SYMROS/ros-nodds/build-conc" | paste -sd:)
export LD_PRELOAD="/usr/lib/x86_64-linux-gnu/libatomic.so.1:/home/ubuntu/symros/SYMROS/ros-nodds/build-sym/libros2_no_dds.so:/home/ubuntu/symros/SYMROS/symcc/build/SymCCRuntime-prefix/src/SymCCRuntime-build/libsymcc-rt.so:/home/ubuntu/symros/SYMROS/symros-libcxx/build/libsymros_sym_oracle.so"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$SYMROS_PROJ/ros-nodds/build-sym
export SYMCC_LIBCXX_PATH=/home/ubuntu/symros/libcxx_symros_install/
export SYMCC_MEMORY_INPUT=yes

# prohibit core dump
ulimit -c 0

export LTTNG_UST_REGISTER_TIMEOUT=0   
# execute
OMP_NUM_THREADS=1 $WS/build/symros_exec/exec "$INPUT_FILE" "$STATE_MODE"