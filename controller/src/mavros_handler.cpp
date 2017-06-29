// All documentation are found in the *.h corresponding files in the /include file

#include "mavros_handler.h"

void mavros_handler::release_msg_channels(){
    RC_MSG.channels[MODE_CHANNEL] = RC_MSG.CHAN_RELEASE;

    RC_MSG.channels[ROLL_CHANNEL] = RC_MSG.CHAN_RELEASE;
    RC_MSG.channels[PITCH_CHANNEL] = RC_MSG.CHAN_RELEASE;
    RC_MSG.channels[THROTTLE_CHANNEL] = RC_MSG.CHAN_RELEASE;
    RC_MSG.channels[YAW_CHANNEL] = RC_MSG.CHAN_RELEASE;
}

void mavros_handler::land_msg_channels(){
    RC_MSG.channels[MODE_CHANNEL] = LAND_MODE;

    RC_MSG.channels[ROLL_CHANNEL] = RC_MSG.CHAN_RELEASE;
    RC_MSG.channels[PITCH_CHANNEL] = RC_MSG.CHAN_RELEASE;
    RC_MSG.channels[YAW_CHANNEL] = MID_PWM;
    RC_MSG.channels[THROTTLE_CHANNEL] = MID_PWM;
}

void mavros_handler::ai_msg_channels(int roll, int pitch, int yaw, int throttle){
    RC_MSG.channels[MODE_CHANNEL] = mode;

    RC_MSG.channels[ROLL_CHANNEL] = roll + ROLL_TRIM;
    RC_MSG.channels[PITCH_CHANNEL] = pitch + PITCH_TRIM;
    RC_MSG.channels[YAW_CHANNEL] = yaw + YAW_TRIM;
    RC_MSG.channels[THROTTLE_CHANNEL] = throttle;
}

int mavros_handler::getNonBlocking(){
    struct termios initial_settings, new_settings;
    int n;

    unsigned char key;

    tcgetattr(0, &initial_settings);

    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 0;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(0, TCSANOW, &new_settings);
    n = getchar();
    key = n;
    tcsetattr(0, TCSANOW, &initial_settings);

    return key;
}

/// update loop for mavros handler
void mavros_handler::update_loop(int roll, int pitch, int yaw, int throttle){
    // get land command
    landChar = getNonBlocking();

    // one time only toggle to land mode
    if (landChar == ' ' && !LAND_TOGGLE){
        LAND_TOGGLE = true;
        flight_mode = land;
        std::cout << "LAND MODE ENGAGED" << std::endl;
    }

    switch(flight_mode){
    case ai:
        ai_msg_channels(roll, pitch, yaw, throttle);
        break;
    case land:
        land_msg_channels();
        break;
    default:
        release_msg_channels();
        break;
    }

    if (DEBUG){
        std::cout 
            << "mode: " 	<< RC_MSG.channels[MODE_CHANNEL] << std::endl
            << "roll: "		<< RC_MSG.channels[ROLL_CHANNEL] << std::endl
            << "pitch: "	<< RC_MSG.channels[PITCH_CHANNEL] << std::endl
            << "yaw: " 		<< RC_MSG.channels[YAW_CHANNEL] << std::endl
            << "throttle: " << RC_MSG.channels[THROTTLE_CHANNEL] << std::endl;
    }

    // pubishes the mavros messages to the pixhawk
    rc_pub.publish(RC_MSG);
}

/// releases all channels
void mavros_handler::disable_rc_overide(){
    release_msg_channels();
    rc_pub.publish(RC_MSG);
}

void mavros_handler::RCIn_callback(const mavros_msgs::RCIn& msg)
{
    if (msg.channels[MANUAL_CONTROL] >= MID_PWM){
        if(!LAND_TOGGLE){
            std::cout << "manual_mode\n";
            flight_mode = manual;
        }
    }
    else{
        // Reset ai to TakeOff state
        std_msgs::Bool reset_signal;
        reset_signal.data = true;
        ai_reset_pub.publish(reset_signal);

        flight_mode = ai;
    }
}