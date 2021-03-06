cmake_minimum_required(VERSION 3.5)
project(tutorial_lifecycle)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${std_msgs_INCLUDE_DIRS}
  ${lifecycle_msgs_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)

add_executable(lifecycle_talker src/lifecycle_talker.cpp)
target_link_libraries(lifecycle_talker ${rclcpp_lifecycle_LIBRARIES} ${std_msgs_LIBRARIES})

add_executable(lifecycle_listener src/lifecycle_listener.cpp)
target_link_libraries(lifecycle_listener ${rclcpp_lifecycle_LIBRARIES}  ${std_msgs_LIBRARIES})

add_executable(lifecycle_service_client src/lifecycle_service_client.cpp)
target_link_libraries(lifecycle_service_client ${rclcpp_lifecycle_LIBRARIES}  ${std_msgs_LIBRARIES})

install(TARGETS
  lifecycle_talker
  lifecycle_listener
  lifecycle_service_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
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
endif()

ament_package()
