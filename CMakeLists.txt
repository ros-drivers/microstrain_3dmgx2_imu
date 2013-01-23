# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(microstrain_3dmgx2_imu)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS message_generation roscpp sensor_msgs self_test diagnostic_updater tf std_srvs)

find_library(LOG4CXX_LIBRARY log4cxx)

add_service_files(
  FILES AddOffset.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(3dmgx2 src/3dmgx2.cc)

add_executable(imu_node src/imu_node.cc)
target_link_libraries(imu_node 3dmgx2 ${catkin_LIBRARIES})

# Utility to recover ID from IMU
add_executable(get_id src/get_id.cpp)
target_link_libraries(get_id 3dmgx2 ${catkin_LIBRARIES})
target_link_libraries(get_id ${LOG4CXX_LIBRARY})

catkin_package(
    DEPENDS 
    CATKIN-DEPENDS roscpp sensor_msgs self_test diagnostic_updater tf std_srvs
    INCLUDE_DIRS include
    LIBRARIES 3dmgx2 ${LOG4CXX_LIBRARY}
)

# Install targets
install(TARGETS 3dmgx2 DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS imu_node get_id
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)