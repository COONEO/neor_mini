cmake_minimum_required(VERSION 3.5)
project(neor_mini_foxy)

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
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(neor_mini_foxy_node src/neor_mini_foxy_node.cpp)
target_include_directories(neor_mini_foxy_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS neor_mini_foxy_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  # add neor_mini
  install(
  DIRECTORY launch meshes models rviz src urdf worlds
  DESTINATION share/${PROJECT_NAME}
)
endif()

install(
  PROGRAMS
    scripts/spawn_bot.py
  DESTINATION
    lib/${PROJECT_NAME}/
)

install(
  DIRECTORY
    launch
    urdf
    worlds
    scripts
  DESTINATION
    share/${PROJECT_NAME}/
)
ament_package()
