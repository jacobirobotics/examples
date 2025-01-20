cmake_minimum_required(VERSION 3.16)


project(jacobi-universal-driver-examples)


option(BUILD_FIND_JACOBI "Find globally installed Jacobi libraries." ON)


if(BUILD_FIND_JACOBI)
  find_package(jacobi-motion REQUIRED)
  find_package(jacobi-universal-driver REQUIRED)
endif()


set(EXAMPLES_LIST
  control_io
  move
  read_state
  trajectory
)

foreach(example IN LISTS EXAMPLES_LIST)
  add_executable(universal-${example} ${example}.cpp)
  target_compile_features(universal-${example} PRIVATE cxx_std_17)
  target_link_libraries(universal-${example} PRIVATE jacobi::motion jacobi::universal-driver)
endforeach()
