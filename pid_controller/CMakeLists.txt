cmake_minimum_required(VERSION 3.8)
project(pid_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(pid_controller_msgs REQUIRED)



add_executable(PCN src/pid_controller_node.cpp)
add_executable(RIN src/reference_input_node.cpp)

ament_target_dependencies(PCN rclcpp std_msgs rcl_interfaces pid_controller_msgs)
ament_target_dependencies(RIN rclcpp std_msgs rcl_interfaces pid_controller_msgs)

target_include_directories(PCN PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(PCN PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_include_directories(RIN PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(RIN PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS 
  PCN 
  RIN
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
