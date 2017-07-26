// Holds the Global Variables for the drone
#ifndef FLIGHT_DEFINITIONS_H
#define FLIGHT_DEFINITIONS_H

///remember: {0-8} is 9 channels.
#define ROLL_CHANNEL 0
#define PITCH_CHANNEL 1
#define THROTTLE_CHANNEL 2
#define YAW_CHANNEL 3
#define MODE_CHANNEL 4
#define MANUAL_CONTROL 8

///PWM valuse can range from 1100 to 1900
#define LOW_PWM 1100
#define MID_PWM 1500
#define HIGH_PWM 1900

#define LAND_MODE 1100
#define STABILIZE_MODE 1500
#define ALT_HOLD_MODE 1900

///Slow descents rate needs to be tuned for landing 
#define SLOW_DESCENT 1500

///Trim values will be added onto the mavros message outside of PID loop:
#define ROLL_TRIM 0
#define PITCH_TRIM 0
#define YAW_TRIM 0

#define MAX_HEIGHT 2.0


#endif
