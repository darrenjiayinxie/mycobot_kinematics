cmake_minimum_required(VERSION 2.8.3)
project(mycobot_kinematics)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  moveit_core
  moveit_kinematics
  pluginlib
  roscpp
)
find_package(Boost REQUIRED COMPONENTS filesystem)
#find_package (Eigen3 3.3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mycobot_kin mycobot_kinematics_plugin
  CATKIN_DEPENDS geometry_msgs moveit_core moveit_kinematics pluginlib roscpp 
  DEPENDS Boost
)

###########
## Build ##
###########


include_directories(
  include ${catkin_INCLUDE_DIRS}
  SYSTEM ${Boost_INCLUDE_DIR}
)

add_library(mycobot_kin src/mycobot_kin.cpp)
#target_link_libraries(mycobot_kin)
add_library(mycobot_kinematics_plugin src/mycobot_kinematics_plugin.cpp)
target_link_libraries(mycobot_kinematics_plugin
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  mycobot_kin
)


#add_library(ur_kin src/ur_kin.cpp)
add_executable(main src/main.cpp)
target_link_libraries(main mycobot_kin)

#############
## Install ##
#############

install(TARGETS mycobot_kin mycobot_kinematics_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# install moveit plugin description file
install(FILES ur_moveit_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mycobot_kinematics.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
