cmake_minimum_required(VERSION 3.16)
project(omni_drive_arduino LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  omni_drive_arduino
  SHARED
  hardware/omni_drive_arduino.cpp
)
target_compile_features(omni_drive_arduino PUBLIC cxx_std_17)
target_include_directories(omni_drive_arduino PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/omni_drive_arduino>
)
ament_target_dependencies(
  omni_drive_arduino PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(omni_drive_arduino PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "omni_drive_arduino_BUILDING_DLL")
pluginlib_export_plugin_description_file(hardware_interface omni_drive_arduino.xml)






# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/omni_drive_arduino
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf description/rviz description/meshes
  DESTINATION share/omni_drive_arduino
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/omni_drive_arduino
)
install(TARGETS omni_drive_arduino
  EXPORT export_omni_drive_arduino
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


## TESTING
# if(BUILD_TESTING)
#   find_package(ament_cmake_gtest REQUIRED)
# endif()

## EXPORTS
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_targets(export_omni_drive_arduino HAS_LIBRARY_TARGET)
ament_package()

