if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-gnu-folding-constant)
endif()

add_library(pf_lib SHARED
  pf.c
  pf_kdtree.c
  pf_pdf.c
  pf_vector.c
  eig3.c
  pf_draw.c
)

target_include_directories(pf_lib PRIVATE ../include)
if(HAVE_DRAND48)
  target_compile_definitions(pf_lib PRIVATE "HAVE_DRAND48")
endif()
target_link_libraries(pf_lib m)

install(TARGETS
  pf_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)



# ros2-nodds dependency
add_compile_options(-w)
set(ros_nodds /home/ubuntu/symros/SYMROS/ros-nodds)
include_directories(${ros_nodds}/include)
link_directories(${ros_nodds}/build-conc)
link_libraries(ros2_no_dds)
# ros2-nodds dependency
