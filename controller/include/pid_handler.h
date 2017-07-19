/*****************************************************************************************
 * Handles which PIDs are currently activated
 * Also manages PID callibration and holding the calibration values.
 *
 *****************************************************************************************/
#ifndef PID_HANDLER_H
#define PID_HANDLER_H

#include <iostream>
#include <PIDController.h>
#include <fstream>
#include <string>
#include <vector>

class pid_handler{
private:
	PIDController* altitudePID;

	PIDController* yawYoloPID;
	PIDController* pitchYoloPID;
	
	PIDController* pitchDownCamPID;
	PIDController* rollDownCamPID;

	PIDController* pitchObstaclePID;
	PIDController* rollObstaclePID;

	PIDController* yawRTraversalPID;

	PIDController* pitchEdgeDetectPID;
	PIDController* rollEdgeDetectPID;

	enum mode_enum{
		DownCam 	    = 0,
		Obstacle 	    = 1,
		Yolo 		    = 2,
        EdgeDetect      = 3,
        RandomTraversal = 4
	};

	mode_enum pitch_mode = Yolo;
	mode_enum roll_mode = DownCam;
    mode_enum yaw_mode = Yolo;

    bool DEBUG = true;

public:
    //constuctors
    pid_handler(){
        // default constructor
        // please do NOT use these values at competition, use the tested calibration files only
        altitudePID         = new PIDController(100, 0, 0, -250, 250);
        yawYoloPID          = new PIDController(100, 0, 0, -250, 250);
        pitchYoloPID        = new PIDController(100, 0, 0, -250, 250);
        pitchDownCamPID     = new PIDController(100, 0, 0, -250, 250);
        rollDownCamPID      = new PIDController(100, 0, 0, -250, 250);
        pitchObstaclePID    = new PIDController(100, 0, 0, -250, 250);
        rollObstaclePID     = new PIDController(100, 0, 0, -250, 250);
        yawRTraversalPID    = new PIDController(100, 0, 0, -250, 250);
        pitchEdgeDetectPID  = new PIDController(100, 0, 0, -250, 250);
        rollEdgeDetectPID   = new PIDController(100, 0, 0, -250, 250);
    }

    pid_handler(std::string calibration_file){
        load_PID_calibrations(calibration_file);
    }

    void initialize_zero_target();

    void reset_all();

    // when mode is set, the pids are turn on.
    void set_pitch_mode(std::string value);

    std::string get_pitch_mode();

    void set_roll_mode(std::string value);

    std::string get_roll_mode();

    void set_yaw_mode(std::string value);

    std::string get_yaw_mode();

    // returns the current pid bsed on the state mode
	PIDController& getThrottlePID();

	PIDController& getPitchPID();

	PIDController& getRollPID();

	PIDController& getYawPID();

    // string parser for reading in the calibration file
    std::vector<std::string> parse_calibrations(std::string input, std::string delimiter);

    // loads calibration values from file to new pid controllers
    void load_PID_calibrations(std::string file);
};

#endif
