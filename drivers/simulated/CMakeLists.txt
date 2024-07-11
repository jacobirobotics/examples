cmake_minimum_required(VERSION 3.11)


project(jacobi-abb-driver-examples)


option(BUILD_FIND_JACOBI "Find globally installed Jacobi libraries." ON)


if(BUILD_FIND_JACOBI)
  find_package(jacobi-motion REQUIRED)
  find_package(jacobi-simulated-driver REQUIRED)
endif()


set(EXAMPLES_LIST
  control_io
  move
  read_state
  trajectory
)

foreach(example IN LISTS EXAMPLES_LIST)
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE jacobi::motion jacobi::simulated-driver)
endforeach()
