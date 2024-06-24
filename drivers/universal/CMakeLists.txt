# This is an CMakeLists.txt example file for using the UR driver in your CMake project
cmake_minimum_required(VERSION 3.11)


project(using-jacobi)


find_package(jacobi-motion REQUIRED)
find_package(jacobi-universal-driver REQUIRED)


add_executable(move move.cpp)
target_link_libraries(run PRIVATE jacobi::motion jacobi::universal-driver)
