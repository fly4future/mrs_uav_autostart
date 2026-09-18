cmake_minimum_required(VERSION 3.28)
project(mrs_uav_autostart)

# set the correct standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1 == use rclcpp::Timer implementation
# 0 == use MRS's thread timer implementation
set(USE_ROS_TIMER 0)

if(${USE_ROS_TIMER})
  MESSAGE(WARNING "[mrs_uav_autostart]: Compiling with ROS Timers. This can cause high CPU load in runtime.")
endif()

# set the compile options to show code warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options("-Wall" "-Wextra" "-Wpedantic")
endif()

if(ENABLE_COVERAGE)
  message(WARNING "building for coverage, the performance might be limited")

  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} --coverage -fprofile-arcs")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage -fprofile-arcs")

  add_definitions("-O1")
  add_definitions("-g")
endif()

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  mrs_msgs
  mrs_lib
  std_msgs
  std_srvs
  mrs_uav_testing
  backward_ros
)

foreach(DEP IN LISTS DEPENDENCIES)
  find_package(${DEP} REQUIRED)
endforeach()

set(LIBRARIES
  MrsUavAutostart_AutomaticStart
)

## --------------------------------------------------------------
## |                           Compile                          |
## --------------------------------------------------------------

# Automatic start

add_library(MrsUavAutostart_AutomaticStart SHARED
  src/automatic_start.cpp
)

target_link_libraries(MrsUavAutostart_AutomaticStart
  PUBLIC
    rclcpp::rclcpp
    rclcpp_components::component
    mrs_lib::mrs_lib
    ${mrs_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
)

rclcpp_components_register_nodes(MrsUavAutostart_AutomaticStart
  "mrs_uav_autostart::AutomaticStart"
)

target_compile_definitions(MrsUavAutostart_AutomaticStart PRIVATE USE_ROS_TIMER=${USE_ROS_TIMER})

## --------------------------------------------------------------
## |                           testing                          |
## --------------------------------------------------------------

if(ENABLE_TESTS)

  add_subdirectory(test)

  if(NOT DEFINED ENV{IN_NIX_SHELL})
    find_package(ament_cmake_clang_format REQUIRED)
    ament_clang_format(CONFIG_FILE "${PROJECT_SOURCE_DIR}/.clang-format")
    find_package(ament_cmake_xmllint REQUIRED)
    ament_xmllint()
  endif()

endif()

## --------------------------------------------------------------
## |                           install                          |
## --------------------------------------------------------------

ament_export_libraries(
  ${LIBRARIES}
  )

install(
  TARGETS ${LIBRARIES}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY scripts
  USE_SOURCE_PERMISSIONS
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(
  export_${PROJECT_NAME} HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  ${DEPENDENCIES}
)

ament_package()
