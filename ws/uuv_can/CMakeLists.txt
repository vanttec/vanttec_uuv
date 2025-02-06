# Original code from: https://github.com/DemianMArin/my_vanttec_uuv.git
# Modified by: Abraham de Jesus Maldonado Mata
# Date: 06/02/2025

cmake_minimum_required(VERSION 3.8)
project(uuv_can)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(Boost REQUIRED COMPONENTS system PATHS /usr/local/include)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME})

# Executables
add_executable(uuv_can_node src/can_node_uuv.cpp src/can_node_base.cpp)
# add_executable(teleop_node src/uuv_teleop.cpp)
# Adding CAN libraries
target_include_directories(uuv_can_node PUBLIC
  ${Boost_INCLUDE_DIRS}
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_CANLib>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libs/vanttec_CANLib/platform_specific/vanttec_canlib_linux>
  $<INSTALL_INTERFACE:include>)

# Adding C requirements
target_compile_features(uuv_can_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# Where to find CAN libraries
add_subdirectory(libs/vanttec_CANLib)
target_link_libraries(uuv_can_node Vanttec_CANLibLIB Boost::system)

# Adding dependencies
ament_target_dependencies(uuv_can_node rclcpp std_msgs geometry_msgs std_srvs Eigen3)


install(
  TARGETS
  uuv_can_node
  DESTINATION lib/${PROJECT_NAME}/
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
