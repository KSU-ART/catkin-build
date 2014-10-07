cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

#rosbuild_add_link_flags()
rosbuild_add_executable(advanced_servo src/advanced_servo.cpp)
rosbuild_add_executable(advanced_servo_client src/advanced_servo_client.cpp)
rosbuild_add_executable(high_speed_encoder src/high_speed_encoder.cpp)
rosbuild_add_executable(high_speed_encoder_client src/high_speed_encoder_client.cpp)
rosbuild_add_executable(motor_control_hc src/motor_control_hc.cpp)
rosbuild_add_executable(motor_control_hc_client src/motor_control_hc_client.cpp)
rosbuild_add_executable(interface_kit src/interface_kit.cpp)
rosbuild_add_executable(interface_kit_client src/interface_kit_client.cpp)
rosbuild_add_executable(accelerometer src/accelerometer.cpp)
rosbuild_add_executable(accelerometer_client src/accelerometer_client.cpp)
rosbuild_add_executable(manager src/manager.cpp)
rosbuild_add_executable(manager_client src/manager_client.cpp)
rosbuild_add_executable(stepper src/stepper.cpp)
rosbuild_add_executable(stepper_client src/stepper_client.cpp)
rosbuild_add_executable(odometry src/odometry.cpp)
rosbuild_add_executable(joystick src/joystick.cpp)
rosbuild_add_executable(lidar src/lidar.cpp)
rosbuild_add_executable(lidar_client src/lidar_client.cpp)
rosbuild_add_executable(joystick_motor_control src/joystick_motor_control.cpp)
rosbuild_add_executable(joystick_servo_control src/joystick_servo_control.cpp)
rosbuild_add_executable(servo_glimpse src/servo_glimpse.cpp)
rosbuild_add_executable(rfid src/rfid.cpp)
rosbuild_add_executable(rfid_client src/rfid_client.cpp)
rosbuild_add_executable(textlcd src/textlcd.cpp)
rosbuild_add_executable(textlcd_client src/textlcd_client.cpp)
rosbuild_add_executable(phsensor src/phsensor.cpp)
rosbuild_add_executable(phsensor_client src/phsensor_client.cpp)
rosbuild_add_executable(spatial src/spatial.cpp)
rosbuild_add_executable(spatial_client src/spatial_client.cpp)
rosbuild_add_executable(led src/led.cpp)
rosbuild_add_executable(led_client src/led_client.cpp)
rosbuild_add_executable(ir src/ir.cpp)
rosbuild_add_executable(ir_client src/ir_client.cpp)

target_link_libraries(advanced_servo phidget21)
target_link_libraries(high_speed_encoder phidget21)
target_link_libraries(motor_control_hc phidget21)
target_link_libraries(interface_kit phidget21)
target_link_libraries(accelerometer phidget21)
target_link_libraries(manager phidget21)
target_link_libraries(stepper phidget21)
target_link_libraries(odometry phidget21)
target_link_libraries(joystick phidget21)
target_link_libraries(lidar phidget21)
target_link_libraries(joystick_motor_control phidget21)
target_link_libraries(joystick_servo_control phidget21)
target_link_libraries(servo_glimpse phidget21)
target_link_libraries(rfid phidget21)
target_link_libraries(textlcd phidget21)
target_link_libraries(phsensor phidget21)
target_link_libraries(spatial phidget21)
target_link_libraries(led phidget21)
target_link_libraries(ir phidget21)
