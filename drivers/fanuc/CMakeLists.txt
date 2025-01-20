cmake_minimum_required(VERSION 3.16)


project(jacobi-fanuc-driver-examples)


option(BUILD_FIND_JACOBI "Find globally installed Jacobi libraries." ON)


if(BUILD_FIND_JACOBI)
  find_package(jacobi-motion REQUIRED)
  find_package(jacobi-fanuc-driver REQUIRED)
endif()


set(EXAMPLES_LIST
  control_io
  move
  read_state
  trajectory
)

foreach(example IN LISTS EXAMPLES_LIST)
  add_executable(fanuc-${example} ${example}.cpp)
  target_compile_features(fanuc-${example} PRIVATE cxx_std_17)
  target_link_libraries(fanuc-${example} PRIVATE jacobi::motion jacobi::fanuc-driver)
endforeach()
