cmake_minimum_required(VERSION 3.8)
project(battery_state_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

add_executable(
  battery_state_publisher 
  "src/battery_state_publisher.cpp"
  "src/pzem_sensor.cpp"
  )
ament_target_dependencies(
  battery_state_publisher
  rclcpp 
  std_msgs 
  sensor_msgs 
  )

target_include_directories(battery_state_publisher PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIBMODBUS_INCLUDE_DIRS}
)

# Link libmodbus
target_link_libraries(battery_state_publisher
  ${LIBMODBUS_LIBRARIES}
)

# Add compiler flags for libmodbus
target_compile_options(battery_state_publisher PRIVATE
  ${LIBMODBUS_CFLAGS_OTHER}
)

install(TARGETS battery_state_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
