cmake_minimum_required(VERSION 2.8.3)
project(imu_100d2)


add_definitions(-std=c++11)
find_package(catkin REQUIRED roscpp sensor_msgs tf cmake_modules)

find_package(catkin REQUIRED COMPONENTS)


include_directories(${catkin_INCLUDE_DIRS})

find_package(Eigen REQUIRED)

catkin_package(
)

add_executable(imu_100d2
  src/imu_100d2.cc
)

target_link_libraries(imu_100d2
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

install(
  DIRECTORY /usr/local/apollo/jsoncpp/lib
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}/..
  FILES_MATCHING PATTERN "*.so*"
)

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params
  FILES_MATCHING PATTERN "*.yaml"
)

install(FILES
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)
