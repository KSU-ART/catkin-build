// More documentation are found in the *.h corresponding files in the /include file

#include "pid_handler.h"

void pid_handler::initialize_zero_target(){
    altitudePID->targetSetpoint(0);
    yawYoloPID->targetSetpoint(0);
    pitchYoloPID->targetSetpoint(0);
    pitchDownCamPID->targetSetpoint(0);
    rollDownCamPID->targetSetpoint(0);
    pitchObstaclePID->targetSetpoint(0);
    rollObstaclePID->targetSetpoint(0);
}

void pid_handler::reset_all(){
    altitudePID->on();
    yawYoloPID->on();
    pitchYoloPID->on();
    pitchDownCamPID->off();
    rollDownCamPID->off();
    pitchObstaclePID->off();
    rollObstaclePID->off();
}

// when mode is set, the pids are turn on.
void pid_handler::set_pitch_mode(std::string value){
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

std::string pid_handler::get_pitch_mode(){
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

void pid_handler::set_roll_mode(std::string value){
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

std::string pid_handler::get_roll_mode(){
    switch (roll_mode){
    case DownCam:
        return "DownCam";
    case Obstacle:
        return "Obstacle";
    default:
        return "Undefined";
    }
}

PIDController& pid_handler::getThrorrlePID(){
    return *altitudePID;
}

PIDController& pid_handler::getPitchPID(){
    switch(pitch_mode){
    case DownCam:
        return *pitchDownCamPID;
    case Obstacle:
        return *pitchObstaclePID;
    case Yolo:
        return *pitchYoloPID;
    default:
        return *(new PIDController);
    }
}

PIDController& pid_handler::getRollPID(){
    switch(roll_mode){
    case DownCam:
        return *rollDownCamPID;
    case Obstacle:
        return *rollObstaclePID;
    default:
        return *(new PIDController);
    }
}

PIDController& pid_handler::getYawPID(){
    return *yawYoloPID;
}

std::vector<std::string> pid_handler::parse_calibrations(std::string input, std::string delimiter){
    size_t pos = 0;
    std::string token;
    std::vector<std::string> params;
    while ((pos = input.find(delimiter)) != std::string::npos) {
        token = input.substr(0, pos);
        params.push_back(token);
        input.erase(0, pos + delimiter.length());
    }
    return params;
}

void pid_handler::load_PID_calibrations(std::string file){
    std::string line;
    std::ifstream calibration_file(file);
    if(calibration_file.is_open()){
        while(std::getline(calibration_file, line)){
            std::vector<std::string> params;
            params = parse_calibrations(line, ",");
            if(params[0] == "altitudePID"){
                altitudePID     = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
            }
            else if(params[0] == "yawYoloPID"){
                yawYoloPID      = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
            }
            else if(params[0] == "pitchYoloPID"){
                pitchYoloPID    = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
            }
            else if(params[0] == "pitchDownCamPID"){
                pitchDownCamPID = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
            }
            else if(params[0] == "rollDownCamPID"){
                rollDownCamPID  = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
            }
            else if(params[0] == "pitchObstaclePID"){
                pitchObstaclePID= new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
            }
            else if(params[0] == "rollObstaclePID"){
                rollObstaclePID = new PIDController(std::stoi(params[1]), std::stoi(params[2]), std::stoi(params[3]), std::stoi(params[4]), std::stoi(params[5]));
            }
        }
        calibration_file.close();
    }
    else{
        std::cout << "ERROR: NO PID CALIBRATION FILE FOUND" << std::endl;
    }
}