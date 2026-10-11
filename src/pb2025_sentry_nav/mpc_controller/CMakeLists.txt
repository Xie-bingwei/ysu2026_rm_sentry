cmake_minimum_required(VERSION 3.10)
project(mpc_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp_vendor REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/mpc_controller.cpp
  src/mpc_model.cpp
  src/mpc_solver.cpp
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen osqp::osqp)

pluginlib_export_plugin_description_file(nav2_core mpc_controller.xml)

# add_executable(plugin_probe src/plugin_probe.cpp)
# target_link_libraries(plugin_probe ${PROJECT_NAME})
# ament_target_dependencies(plugin_probe rclcpp rclcpp_lifecycle nav2_core nav2_costmap_2d tf2_ros)
# install(TARGETS plugin_probe DESTINATION lib/${PROJECT_NAME})

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)