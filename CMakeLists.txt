cmake_minimum_required(VERSION 3.5)
project(microstrain_3dmgx2_imu)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_library(LOG4CXX_LIBRARY log4cxx)
if(NOT LOG4CXX_LIBRARY)
  message(FATAL_ERROR "Couldn't find log4cxx library")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(class_loader REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(self_test REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)

## Declare a cpp library
add_library(3dmgx2 SHARED
  src/3dmgx2.cc
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/AddOffset.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(get_id
  src/get_id.cpp
)

target_link_libraries(get_id
  3dmgx2
)

ament_target_dependencies(get_id
  rclcpp
)

add_library(microstrain_3dmgx2_node SHARED
  src/imu_node.cc
)

rclcpp_components_register_nodes(microstrain_3dmgx2_node "microstrain_3dmgx2_imu::ImuNode")

target_link_libraries(microstrain_3dmgx2_node
  3dmgx2
)

ament_target_dependencies(microstrain_3dmgx2_node
  rclcpp
  class_loader
  builtin_interfaces
  rosidl_default_generators
  sensor_msgs
  self_test
  diagnostic_updater
  tf2
  std_srvs
  std_msgs
)

get_default_rmw_implementation(rmw_implementation)
find_package("${rmw_implementation}" REQUIRED)
get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

foreach(typesupport_impl ${typesupport_impls})
  rosidl_target_interfaces(microstrain_3dmgx2_node
    ${PROJECT_NAME} ${typesupport_impl}
  )
endforeach()

target_include_directories(microstrain_3dmgx2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS 3dmgx2 microstrain_3dmgx2_node get_id
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
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
