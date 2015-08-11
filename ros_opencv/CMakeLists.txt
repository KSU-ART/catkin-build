cmake_minimum_required(VERSION 2.8.3)
project(ros_opencv)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cv_bridge image_transport roscpp std_msgs message_generation)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/groovy/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Diffmessage.msg
   LightToLasermessage.msg
   Depthmessage.msg
   TrackingPoint.msg
   ObstacleDetected.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   ros_opencv
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_opencv
  CATKIN_DEPENDS cv_bridge image_transport message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(ros_opencv
#   src/${PROJECT_NAME}/ros_opencv.cpp
# )

## Declare a cpp executable
# add_executable(ros_opencv_node src/ros_opencv_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ros_opencv_node ros_opencv_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ros_opencv_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/groovy/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ros_opencv ros_opencv_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_opencv.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})
#add_executable(bluelight src/Light.cpp)
#target_link_libraries(bluelight ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
#add_executable(lightTalker src/LightTalker.cpp)
#add_dependencies(lightTalker ${catkin_EXPORTED_TARGETS})
#target_link_libraries(lightTalker ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
#add_executable(lightToHokuyo src/LightToLaser.cpp)
#add_dependencies(lightToHokuyo ${catkin_EXPORTED_TARGETS})
#target_link_libraries(lightToHokuyo ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
add_executable(xtionTrackColor src/XtionTrackColor.cpp)
add_dependencies(xtionTrackColor ${catkin_EXPORTED_TARGETS} ros_opencv_generate_messages_cpp)
target_link_libraries(xtionTrackColor ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
#add_executable(testXtion src/TestXtion.cpp)
#add_dependencies(testXtion ${catkin_EXPORTED_TARGETS})
#target_link_libraries(testXtion ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
add_executable(processLines src/ProcessLines.cpp)
add_dependencies(processLines ${catkin_EXPORTED_TARGETS})
target_link_libraries(processLines ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
add_executable(trackIRobot src/TrackIRobot.cpp)
add_dependencies(trackIRobot ${catkin_EXPORTED_TARGETS} ros_opencv_generate_messages_cpp)
target_link_libraries(trackIRobot ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
add_executable(object_avoid src/object_avoid.cpp)
add_dependencies(object_avoid ${catkin_EXPORTED_TARGETS} ros_opencv_generate_messages_cpp)
target_link_libraries(object_avoid ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
add_executable(trackBlobs src/TrackBlobs.cpp)
add_dependencies(trackBlobs ${catkin_EXPORTED_TARGETS} ros_opencv_generate_messages_cpp)
target_link_libraries(trackBlobs ${catkin_LIBRARIES} ${OPENCV_LIBRARIES})
