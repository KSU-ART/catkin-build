cmake_minimum_required(VERSION 2.8.3)
project(roomba_iarc)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(OpenCV REQUIRED)
catkin_package(DEPENDS system_lib)
add_executable(blob_detect blob_detect.cpp)
target_link_libraries(blob_detect ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

