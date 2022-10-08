cmake_minimum_required(VERSION 3.8)
project(riptide_async_autonomy)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(riptide_msgs2 REQUIRED)

# autonomy library
add_library(async_autonomy SHARED
  src/plugins.cpp
)
target_include_directories(async_autonomy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(async_autonomy
  rclcpp
  behaviortree_cpp_v3
  riptide_msgs2
)

target_compile_definitions(async_autonomy PRIVATE BT_PLUGIN_EXPORT)


# build an executable for running trees
# Tree execution executable
add_executable(executor src/executor.cpp)
target_include_directories(executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(executor PUBLIC AUTONOMY_PKG_NAME="${PROJECT_NAME}")
ament_target_dependencies(executor
  ament_index_cpp
  rclcpp
  behaviortree_cpp_v3
  rclcpp_action
  riptide_msgs2
)


#install library and node
install(TARGETS
  async_autonomy executor
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY trees
  DESTINATION share/${PROJECT_NAME})


ament_package()
