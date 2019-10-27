cmake_minimum_required(VERSION 3.5)
project(tutorial_composition)

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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

## create tutorial_composition as shared library
# library name definition and source link
add_library(tutorial_composition SHARED
  src/publisher_node.cpp
  src/subscriber_node.cpp
)
# compile setting
target_compile_definitions(tutorial_composition PRIVATE "TUTORIAL_COMPOSITION_DLL")
target_compile_options(tutorial_composition PUBLIC -Wall)
ament_target_dependencies(tutorial_composition rclcpp rclcpp_components std_msgs)
# this macro allow the other package to include this package header.
target_include_directories(tutorial_composition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
  )
endif()

# export cmake target
ament_export_interfaces(export_tutorial_composition HAS_LIBRARY_TARGET)
# necessary package for creating library
ament_export_dependencies(rclcpp std_msgs)
# export library name
ament_export_libraries(tutorial_composition)
# library include directory
ament_export_include_directories(include)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS tutorial_composition
  EXPORT export_tutorial_composition
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
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
