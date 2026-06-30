#!/bin/bash

SYMROS_PROJ=/home/ubuntu/symros/SYMROS
WS=$1
TARGET=$2
BUILD_TYPE=$3
cd $WS

if [ -z "$TARGET" ]; then
  TARGET=symros_exec
fi

BUILD_CMD=""
if [[ "$BUILD_TYPE" = "select" ]]; then
  BUILD_CMD="--packages-select $TARGET"
elif [[ "$BUILD_TYPE" = "up-to" ]]; then
  BUILD_CMD="--packages-up-to $TARGET"
elif [[ "$TARGET" = "all" ]]; then
  BUILD_CMD=""
fi

# install symbolic ROS client library
source $SYMROS_PROJ/ros/conc/humble/install/setup.bash
source ./install/setup.bash

echo "Build command option: $BUILD_CMD"

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
  eval "export $var=\"\$(echo \${$var} | tr ':' '\n' | grep -v 'sym_ws' | paste -sd: -)\""
done

export CC=clang-12
export CXX=clang++-12

# build
colcon build $BUILD_CMD --cmake-args \
	-DBUILD_TESTING=OFF \
	-DCMAKE_BUILD_TYPE=Release \
	-DCMAKE_C_COMPILER=clang-12 \
	-DCMAKE_CXX_COMPILER=clang++-12 \
	-DCMAKE_CXX_STANDARD=17 \
	-DCMAKE_C_FLAGS="-O0 -g -fprofile-instr-generate -fcoverage-mapping -DHAVE_DRAND48" \
	-DCMAKE_CXX_FLAGS="-O0 -g -fprofile-instr-generate -fcoverage-mapping -DHAVE_DRAND48"

# install
source ./install/setup.bash
