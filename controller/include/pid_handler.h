/********************
 * Handles which PIDs are currently activated
 * Also manages PID callibration and holding the calibration values.
 *
 ********************/

#ifndef ROBOT_INCLUDES_H
#define ROBOT_INCLUDES_H
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

	enum mode_enum{
		DownCam 	= 0,
		Obstacle 	= 1,
		Yolo 		= 2
	};

	mode_enum pitch_mode = Yolo;
	mode_enum roll_mode = DownCam;

public:
    //constuctor
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
    }

    pid_handler(std::string calibration_file){
        load_PID_calibrations(calibration_file);
    }

    // when mode is set, the pids are turn on.
    void set_pitch_mode(std::string value){
        if (value.compare("DownCam") == 0){
            pitch_mode = DownCam;
            pitchDownCamPID->on();
            pitchObstaclePID->off();
            yawYoloPID->off();
        }
        else if (value.compare("Obstacle") == 0){
            pitch_mode = Obstacle;
            pitchDownCamPID->off();
            pitchObstaclePID->on();
            yawYoloPID->off();
        }
        else if (value.compare("Yolo") == 0){
            pitch_mode = Yolo;
            pitchDownCamPID->off();
            pitchObstaclePID->off();
            yawYoloPID->on();
        }
        else{
            std::cout << "pitch_mode: " << value << " does not exist" << std::endl;
        }
    }
    std::string get_pitch_mode(){
        switch (pitch_mode){
        case DownCam:
            return "DownCam";
        case Obstacle:
            return "Obstacle";
        case Yolo:
            return "Yolo";
        default:
            return "Undefined";
        }
    }

    void set_roll_mode(std::string value){
        if (value.compare("DownCam") == 0){
            roll_mode = DownCam;
            rollDownCamPID->on();
            rollObstaclePID->off();
        }
        else if (value.compare("Obstacle") == 0){
            roll_mode = Obstacle;
            rollDownCamPID->off();
            rollObstaclePID->on();
        }
        else{
            std::cout << "roll_mode: " << value << " does not exist" << std::endl;
        }
    }
    std::string get_roll_mode(){
        switch (roll_mode){
        case DownCam:
            return "DownCam";
        case Obstacle:
            return "Obstacle";
        default:
            return "Undefined";
        }
    }

	PIDController& getThrorrlePID(){
		return altitudePID;
	}

	PIDController& getPitchPID(){
		switch(pitch_mode){
		case DownCam:
			return pitchDownCamPID;
		case Obstacle:
			return pitchObstaclePID;
		case Yolo:
			return pitchYoloPID;
		default:
			return new PIDController;
		}
	}

	PIDController& getRollPID(){
		switch(roll_mode){
		case DownCam:
			return rollDownCamPID;
		case Obstacle:
			return rollObstaclePID;
		default:
			return new PIDController;
		}
	}

	PIDController& getYawPID(){
		return yawYoloPID;
	}

    void save_PID_calibrations(){

    }

    std::vector<std::string> parse_calibrations(std::string input, std::string delimiter){
        size_t pos = 0;
        std::string token;
        std::vector<std::string> params;
        while ((pos = input.find(delimiter)) != std::string::npos) {
            token = input.substr(0, pos);
            params.push_back(token);
            input.erase(0, pos + delimiter.length());
        }
        return params
    }

    void load_PID_calibrations(std::string file){
        std::string line;
        std::ifstream calibration_file(file);
        if(calibration_file.is_open()){
            while(std::getline(calibration_file, line)){
                std::vector<std::string> params;
                params = parse_calibrations(line, ',');
                switch(params[0]){
                case "altitudePID":
                    altitudePID     = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
                    break;
                case "yawYoloPID":
                    yawYoloPID      = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
                    break;
                case "pitchYoloPID":
                    pitchYoloPID    = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
                    break;
                case "pitchDownCamPID":
                    pitchDownCamPID = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
                    break;
                case "rollDownCamPID":
                    rollDownCamPID  = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
                    break;
                case "pitchObstaclePID":
                    pitchObstaclePID= new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
                    break;
                case "rollObstaclePID":
                    rollObstaclePID = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
                    break;
                }
            }
            calibration_file.close();
        }
        else{
            std::cout << "ERROR: NO PID CALIBRATION FILE FOUND" << std::end;
        }
    }
}