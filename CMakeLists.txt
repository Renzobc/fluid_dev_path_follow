cmake_minimum_required(VERSION 3.8)
project(path_follow)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(Robot_link SHARED src/Robot_link.cpp)

add_library(path_publish SHARED src/path_publish.cpp)

# NOTHINIG IS INSIDE THE INCLUDE FOLDER
target_include_directories(Robot_link PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
${Eigen_INCLUDE_DIRS}
$<INSTALL_INTERFACE:include>)

target_include_directories(path_publish PRIVATE
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)


ament_target_dependencies(Robot_link 
"rclcpp" 
"std_msgs" 
"rclcpp_components" 
"geometry_msgs" 
"tf2_ros" 
"tf2"
"nav_msgs"
"Eigen3")

ament_target_dependencies(path_publish 
"rclcpp" 
"std_msgs" 
"nav_msgs"
"rclcpp_components" 
"geometry_msgs" 
"tf2_ros" 
"tf2")

rclcpp_components_register_node(Robot_link PLUGIN "path_follow::RobotLink" EXECUTABLE Robot_link_exec)
rclcpp_components_register_node(Robot_link PLUGIN "path_follow::RobotControl" EXECUTABLE RobotControl_exec)
rclcpp_components_register_node(Robot_link PLUGIN "path_follow::InitialPoseBroadcast" EXECUTABLE RobotInitial_exec)

rclcpp_components_register_node(path_publish PLUGIN "path_follow::publish_path" EXECUTABLE path_publish_exec)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(TARGETS 
Robot_link
path_publish
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(unit_test tests/unit_test.cpp)
  target_include_directories(unit_test PUBLIC src/)
  target_link_libraries(unit_test Robot_link)
endif()

set(DCMAKE_BUILD_TYPE RelWithDebInfo)

ament_package()


