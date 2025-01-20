cmake_minimum_required(VERSION 3.16)


project(jacobi-yaskawa-driver-examples)


option(BUILD_FIND_JACOBI "Find globally installed Jacobi libraries." ON)


if(BUILD_FIND_JACOBI)
  find_package(jacobi-motion REQUIRED)
  find_package(jacobi-yaskawa-driver REQUIRED)
endif()


set(EXAMPLES_LIST
  control_io
  move
  read_state
  trajectory
)

foreach(example IN LISTS EXAMPLES_LIST)
  add_executable(yaskawa-${example} ${example}.cpp)
  target_compile_features(yaskawa-${example} PRIVATE cxx_std_17)
  target_link_libraries(yaskawa-${example} PRIVATE jacobi::motion jacobi::yaskawa-driver)
endforeach()
