cmake_minimum_required(VERSION 3.8)
project(ur5e_csv_controller LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  Eigen3
  ur_msgs
  generate_parameter_library
)

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  ur5e_csv_controller_parameters
  src/ur5e_csv_controller_parameters.yaml
)

## COMPILE
add_library(ur5e_csv_controller SHARED src/ur5e_csv_controller.cpp)
target_compile_features(ur5e_csv_controller PUBLIC cxx_std_17)
target_include_directories(ur5e_csv_controller PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/ur5e_csv_controller>
  ${Eigen3_INCLUDE_DIR}
)
target_link_libraries(ur5e_csv_controller PUBLIC
  ur5e_csv_controller_parameters
)
target_link_libraries(ur5e_csv_controller PUBLIC Eigen3::Eigen)
ament_target_dependencies(
  ur5e_csv_controller PUBLIC 
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# export controller plugins
pluginlib_export_plugin_description_file(controller_interface plugin_description.xml)

install(
  DIRECTORY 
  include
  config
  launch
  csv_files
  DESTINATION share/${PROJECT_NAME}/
)

install(
  TARGETS ur5e_csv_controller ur5e_csv_controller_parameters
  EXPORT export_ur5e_csv_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

ament_export_targets(export_ur5e_csv_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
