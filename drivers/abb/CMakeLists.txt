# This is an CMakeLists.txt example file for using the ABB driver in your CMake project
cmake_minimum_required(VERSION 3.11)


project(using-jacobi)


find_package(jacobi-motion REQUIRED)
find_package(jacobi-abb-driver REQUIRED)


add_executable(run run.cpp)
target_link_libraries(run PRIVATE jacobi::motion jacobi::abb-driver)
