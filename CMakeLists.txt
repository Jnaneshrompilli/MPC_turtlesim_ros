cmake_minimum_required(VERSION 3.0.2)
project(intermediate_level)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

#FIND_LIBRARY(IPOPT_LIBRARIES ipopt /usr/local/lib/)

find_package(PkgConfig REQUIRED)
pkg_check_modules(CPPAD REQUIRED cppad)


catkin_package(
   INCLUDE_DIRS include
   LIBRARIES intermediate_level
   CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
  /home/jnanneswar/catkin_ws/src/intermediate_level/include/intermediate_level
  )

#add_executable(test_mpc src/test_mpc.cpp)
add_executable(turtlesim_mpc_controller src/turtlesim_mpc_controller.cpp)

#add_executable(mpc_controller src/mpc_controller.cpp)
#add_executable(ros_path_planner src/ros_path_planner.cpp)

add_executable(mpc_controller src/ros_path_planner.cpp src/mpc_controller.cpp)

#target_link_libraries(test_mpc
# ${catkin_LIBRARIES}
#ipopt
#)
target_link_libraries(turtlesim_mpc_controller 
  ${catkin_LIBRARIES}
  ipopt)
target_link_libraries(mpc_controller 
  ${catkin_LIBRARIES}
  ipopt)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
#catkin_install_python(PROGRAMS
 #scripts/turtlesim_mpc_controller.py
 #DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_intermediate_level.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
