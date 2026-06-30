# SYMROS Environment Setup

Prerequisite environment setup for building and running SYMROS. All paths below
assume the layout under `/home/ubuntu/symros`.

## 1. symcc compiler

Build two variants of the symcc compiler.

```bash
# build for symbolic execution
cd symcc
mkdir -p build && cd build
cmake -G Ninja -DSYMCC_RT_BACKEND=simple -DZ3_TRUST_SYSTEM_VERSION=on ..
ninja

# build for bug-guided input space exploration
cd symcc
mkdir -p build_check && cd build_check
cmake -G Ninja -DSYMCC_RT_BACKEND=simple -DZ3_TRUST_SYSTEM_VERSION=on -DOVERFLOW_CHECK=ON -DBOUND_CHECK=ON ..
ninja
```

## 2. libc++

Build libc++ with clang-12 and export its location.

```bash
git clone https://github.com/llvm/llvm-project.git -b llvmorg-12.0.1

cd llvm-project
mkdir -p build && cd build
cmake -G Ninja ../../llvm-project/llvm \
  -DLLVM_ENABLE_PROJECTS="clang;libcxx;libcxxabi" \
  -DLLVM_TARGETS_TO_BUILD="X86" \
  -DLLVM_DISTRIBUTION_COMPONENTS="clang;cxx;cxxabi;cxx-headers" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/ubuntu/symros/libcxx_clang_install \
  -DCMAKE_C_COMPILER=clang-12 \
  -DCMAKE_CXX_COMPILER=clang++-12
ninja distribution
ninja install-distribution

export SYMCC_LIBCXX_PATH=/home/ubuntu/symros/libcxx_clang_install
```

## 3. ros-nodds

```bash
./build.sh sym
./build.sh conc
```

## 4. empy downgrade

```bash
pip install "empy==3.3.4"
```

## 5. Required system / Python packages

```bash
sudo apt install libtinyxml2-dev
sudo apt install libssl-dev
sudo apt install libeigen3-dev
sudo apt install python3-numpy
sudo apt install python3-lark
sudo apt install libasio-dev
sudo apt install qtbase5-dev
sudo apt install uuid-dev
sudo apt install libgtest-dev
sudo apt install libcurl4-openssl-dev
sudo apt install libbullet-dev

sudo apt install python3-orocos-kdl
sudo apt install libopencv-dev
sudo apt install libboost-python-dev

sudo apt install libboost-program-options-dev

sudo apt install libgraphicsmagick1-dev
sudo apt install libxtensor-dev
sudo apt install libomp-dev
sudo apt install libompl-dev

sudo apt-get install -y libgraphicsmagick++1-dev libgraphicsmagick1-dev
sudo apt-get install -y libboost-all-dev
sudo apt-get install -y libsuitesparse-dev
sudo apt-get install -y libceres-dev
sudo apt-get install -y libnanoflann-dev
sudo apt install liborocos-kdl-dev

sudo apt-get install -y libgeographic-dev geographiclib-tools
pip install z3
pip install pandas
pip install black
pip install scipy
pip install autograd func_timeout bitstring
```

## 6. Full build of the three workspaces

Run from the `SYMROS` project root before testing any package.

```bash
./scripts/conc_build.bash ../workspace/conc_ws
./scripts/sym_build.bash ../workspace/sym_ws
./scripts/chk_build.bash ../workspace/chk_ws
```

After the full build, see each package's test procedure under
`samples/<package>/README.md`.
