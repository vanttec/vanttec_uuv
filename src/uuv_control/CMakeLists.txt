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
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(tf_transformations REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(teleop_twist_joy REQUIRED)
find_package(Boost 1.45.0 COMPONENTS filesystem)

# ament_python_install_package(${PROJECT_NAME})


add_subdirectory(libs/uuv_libs)

#dynamic_model_uuv node
add_executable(dynamic_model_uuv src/dynamic_model_uuv.cpp)
target_include_directories(dynamic_model_uuv PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs/src)
target_compile_features(dynamic_model_uuv PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(dynamic_model_uuv rclcpp std_msgs Eigen3 geometry_msgs nav_msgs tf2_ros)
target_link_libraries(dynamic_model_uuv uuv_libs)

#pid6dof node
add_executable(pid6dof_node src/pid6dof_node.cpp)
target_include_directories(pid6dof_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs/src)
target_compile_features(pid6dof_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(pid6dof_node rclcpp std_msgs Eigen3 geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(pid6dof_node uuv_libs)

#objetivo_cuadrado node
add_executable(objetivo_cuadrado src/objetivo_cuadrado.cpp)
target_include_directories(objetivo_cuadrado PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs/src)
target_compile_features(objetivo_cuadrado PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(objetivo_cuadrado rclcpp std_msgs Eigen3 geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(objetivo_cuadrado uuv_libs)

#objetivo_hundir node
add_executable(objetivo_hundir src/objetivo_hundir.cpp)
target_include_directories(objetivo_hundir PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs/src)
target_compile_features(objetivo_hundir PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(objetivo_hundir rclcpp std_msgs Eigen3 geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(objetivo_hundir uuv_libs)

#estados node
add_executable(estados src/estados.cpp)
target_include_directories(estados PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs/src)
target_compile_features(estados PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(estados rclcpp std_msgs Eigen3 geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(estados uuv_libs)

#matrizlocacion node
add_executable(matriz_locacion src/matriz_locacion.cpp)
target_include_directories(matriz_locacion PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include> libs/uuv_libs/src)
target_compile_features(matriz_locacion PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(matriz_locacion rclcpp std_msgs Eigen3 geometry_msgs tf2 tf2_ros tf2_geometry_msgs)
target_link_libraries(matriz_locacion uuv_libs)



install(TARGETS
  dynamic_model_uuv
  # pid6dof_node
  objetivo_cuadrado
  estados
  objetivo_hundir
  matriz_locacion
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  # config
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/teleop_node_forces.py
  src/visualizador.py
  scripts/pid.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/pid.py
  src/visualizador.py
  src/uuv_mesh.py
  src/joystick.py
  DESTINATION lib/${PROJECT_NAME})

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
