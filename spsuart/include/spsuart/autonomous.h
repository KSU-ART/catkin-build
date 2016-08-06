#ifndef AUTONOMOUS_H
#define AUTONOMOUS_H

#include "ros/ros.h"

#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/VFR_HUD.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include "mavlink/v1.0/ardupilotmega/mavlink.h"

#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//#include "px_comm/OpticalFlow.h"
#include "ros_opencv/TrackingPoint.h"
#include <PIDController.h>
 
#define ROLL_CHANNEL 0
#define PITCH_CHANNEL 1
#define THROTTLE_CHANNEL 2
#define YAW_CHANNEL 3
#define MODE_CHANNEL 4
#define RETRACT_CHANNEL 6
#define GIMBAL_PITCH_CHANNEL 5
#define GIMBAL_YAW_CHANNEL 7
#define MANUAL_CONTROL 8

#define LOW_PWM 1000
#define MID_PWM 1500
#define HIGH_PWM 2000

#define LAND_MODE 1000
#define STABILIZE_MODE 1500
#define ALT_HOLD_MODE 2000

#define ROLL_TRIM 1513
#define PITCH_TRIM 1510

#define GIMBAL_ROLL_MIN 1117  
#define GIMBAL_ROLL_MAX 1853
#define GIMBAL_ROLL_TRIM 1521
#define GIMBAL_TILT_MIN 1464
#define GIMBAL_TILT_MAX 1962
#define GIMBAL_TILT_TRIM 1464
/* Copying the values from roll since Hank 2.0 replaces roll with tilt, We probably should update these later */
#define GIMBAL_PITCH_MIN 1117  
#define GIMBAL_PITCH_MAX 1853
#define GIMBAL_PITCH_TRIM 1521


// in meters
#define MIN_HEIGHT 0.0
#define MAX_HEIGHT 3.0

#endif /* AUTONOMOUS_H */
