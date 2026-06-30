cmake_minimum_required(VERSION 3.10)
project(GenSymROSExec)

set(LLVM_DIR  "/usr/local/llvm/lib/cmake/llvm")
set(Clang_DIR "/usr/local/llvm/lib/cmake/clang")

find_package(LLVM REQUIRED CONFIG)
find_package(Clang REQUIRED CONFIG)

include_directories(${LLVM_INCLUDE_DIRS})
include_directories(${CLANG_INCLUDE_DIRS})
add_definitions(${LLVM_DEFINITIONS})

set(CMAKE_CXX_STANDARD 17)
set(CAME_C_COMPILER "/usr/local/llvm/bin/clang")
set(CAME_CXX_COMPILER "/usr/local/llvm/bin/clang++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -frtti -fPIC")


add_executable(ctor-identify ctor-identify.cpp)
add_executable(disable-main disable-main.cpp)
add_executable(node-field-identify node-field-identify.cpp)

target_link_libraries(ctor-identify
  PRIVATE
  clangAST
  clangASTMatchers
  clangRewrite
  clangFrontend
  clangTooling
  clangToolingCore
  clangAnalysis
  clangEdit
  clangLex
  clangBasic
  clangSerialization
  clangDriver
  clangParse
)

target_link_libraries(disable-main
  PRIVATE
  clangAST
  clangASTMatchers
  clangRewrite
  clangFrontend
  clangTooling
  clangBasic
  clangSerialization
  clangDriver
  clangParse
)

target_link_libraries(node-field-identify
  PRIVATE
  clangAST
  clangASTMatchers
  clangRewrite
  clangFrontend
  clangTooling
  clangBasic
  clangSerialization
  clangDriver
  clangParse
)

link_directories(/home/ubuntu/llvm-project/build/lib)