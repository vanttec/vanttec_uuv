cmake_minimum_required(VERSION 3.8)
project(uuv_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Boost 1.45.0 COMPONENTS filesystem)

set(DEPENDENCIES
ament_cmake
rclcpp
std_msgs
geometry_msgs
nav_msgs
tf2_geometry_msgs
visualization_msgs
sensor_msgs
Eigen3
tf2_ros
)

add_subdirectory(libs/uuv_libs)

add_executable(dynamic_model_uuv src/dynamic_model_uuv.cpp)
target_include_directories(dynamic_model_uuv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs)
target_compile_features(dynamic_model_uuv PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(dynamic_model_uuv ${DEPENDENCIES})
target_link_libraries(dynamic_model_uuv uuv_libs)

add_executable(pid_node src/pid_node.cpp)
target_include_directories(pid_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs)
target_compile_features(pid_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pid_node ${DEPENDENCIES})
target_link_libraries(pid_node uuv_libs)

add_executable(path_publisher_node src/path_publisher_node.cpp)
target_include_directories(path_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs)
target_compile_features(path_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(path_publisher_node ${DEPENDENCIES})
target_link_libraries(path_publisher_node uuv_libs)

add_executable(guidance_node src/guidance_node.cpp)
target_include_directories(guidance_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs)
target_compile_features(guidance_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(guidance_node ${DEPENDENCIES})
target_link_libraries(guidance_node uuv_libs)



install(TARGETS
  dynamic_model_uuv
  pid_node
  path_publisher_node
  guidance_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  # config
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/teleop_node_forces.py
  DESTINATION lib/${PROJECT_NAME}
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