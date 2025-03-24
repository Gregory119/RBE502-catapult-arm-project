cmake_minimum_required(VERSION 3.5)
project(catapult)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(throw src/throw.cpp)
target_include_directories(throw PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(throw PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  throw
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS throw
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake)
  find_package(ament_cmake_pytest REQUIRED)
  add_launch_test(test/test_gazebo.py
    TIMEOUT
      180
  )
endif()

ament_package()
