cmake_minimum_required(VERSION 3.8)
project(csv_collector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(clearpath_api REQUIRED)
find_package(diagnostic_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(csv_collector "src/collector.cpp")
ament_target_dependencies(
  csv_collector 
  rclcpp 
  std_msgs 
  diagnostic_msgs
  )

add_executable(read_diagnostic "src/read_diagnostic.cpp")
ament_target_dependencies(
  read_diagnostic
  rclcpp 
  std_msgs 
  diagnostic_msgs
  )

install(TARGETS csv_collector read_diagnostic DESTINATION lib/${PROJECT_NAME})
ament_package()
