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
elif [[ "$BUILD_TARGET" = "all" ]]; then
  BUILD_CMD=""
fi
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH

if [ -n "$NO_LIBCXX" ]; then
  source /opt/ros/humble/local_setup.bash
  source $SYMROS_PROJ/ros/sym/humble-libstdc++/install/local_setup.bash
  unset SYMCC_LIBCXX_PATH
  export SYMCC_REGULAR_LIBCXX=1
else
  source $SYMROS_PROJ/ros/sym/humble/install/local_setup.bash
  source ./install/local_setup.bash
  unset SYMCC_REGULAR_LIBCXX
  export SYMCC_LIBCXX_PATH=/home/ubuntu/symros/libcxx_symros_install/
fi

# echo libcxx and build cmd option
if [ -n "$NO_LIBCXX" ]; then
  echo "Using regular libc++"
else
  echo "Using symbolic libc++ at $SYMCC_LIBCXX_PATH"
fi
echo "Build command option: $BUILD_CMD"

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
  eval "export $var=\"\$(echo \${$var} | tr ':' '\n' | grep -v 'conc_ws' | paste -sd: -)\""
done

# build
colcon build $BUILD_CMD --cmake-args \
  -DBUILD_TESTING=OFF \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_VERBOSE_MAKEFILE=ON \
  -DCMAKE_C_COMPILER=/home/ubuntu/symros/SYMROS/symcc/build/symcc \
  -DCMAKE_CXX_COMPILER=/home/ubuntu/symros/SYMROS/symcc/build/sym++ \
  -DCMAKE_CXX_STANDARD=17 \
  -DCMAKE_C_FLAGS="-g -O0 -ffp-contract=off -Wno-error" \
  -DCMAKE_CXX_FLAGS="-g -O0 -fno-vectorize -fno-slp-vectorize -DHAVE_DRAND48 -DEIGEN_DONT_VECTORIZE -DXTENSOR_DISABLE_SIMD -DSYMROS_MODE -ffp-contract=off -Wno-unknown-warning-option -Wno-error" 


# install
source install/setup.bash
