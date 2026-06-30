#!/bin/bash

SYMROS_PROJ=/home/ubuntu/symros/SYMROS
WS=$1
cd $WS

TARGET=symros_exec

# /home/ubuntu/symros/SYMROS/ros/sym/humble/install/setup.bash:
source $SYMROS_PROJ/ros/sym/humble/install/setup.bash 
source ./install/setup.bash

# export SYMCC_LIBCXX_PATH=/home/ubuntu/symros/libcxx_clang_install/

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

# build
colcon build --packages-up-to symros_exec --cmake-args \
  -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_VERBOSE_MAKEFILE=ON \
  -DCMAKE_C_COMPILER=/home/ubuntu/symros/SYMROS/symcc/build-check-bound/symcc \
  -DCMAKE_CXX_COMPILER=/home/ubuntu/symros/SYMROS/symcc/build-check-bound/sym++ \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_C_FLAGS="-g -O0 -ffp-contract=off -Wno-error" \
  -DCMAKE_CXX_FLAGS="-g -O0 -fno-vectorize -fno-slp-vectorize -DHAVE_DRAND48 -DEIGEN_DONT_VECTORIZE -DXTENSOR_DISABLE_SIMD -DSYMROS_MODE -ffp-contract=off -Wno-unknown-warning-option -Wno-error" 

# install
source install/setup.bash

