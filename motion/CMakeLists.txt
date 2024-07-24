cmake_minimum_required(VERSION 3.16)


project(jacobi-motion-examples)


option(BUILD_FIND_JACOBI "Find globally installed Jacobi libraries." ON)


if(BUILD_FIND_JACOBI)
  find_package(jacobi-motion REQUIRED)
endif()


set(EXAMPLES_LIST
  01_quick_start_guide
  02_real_time_motion
  03_motion_interface
  04_robot_kinematics
  05_along_waypoints
  06_plan_in_studio
  07_dual_arm
  08_linear_motion
)

foreach(example IN LISTS EXAMPLES_LIST)
  add_executable(${example} ${example}.cpp)
  target_link_libraries(${example} PRIVATE jacobi::motion)
endforeach()
