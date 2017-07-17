// More documentation are found in the *.h corresponding files in the /include file

#include "pid_handler.h"

void pid_handler::initialize_zero_target(){
    reset_all();
    altitudePID->targetSetpoint(0);
    yawYoloPID->targetSetpoint(0);
    pitchYoloPID->targetSetpoint(0);
    pitchDownCamPID->targetSetpoint(0);
    rollDownCamPID->targetSetpoint(0);
    pitchObstaclePID->targetSetpoint(0);
    rollObstaclePID->targetSetpoint(0);
    yawRTraversalPID->targetSetpoint(0);
    pitchEdgeDetectPID->targetSetpoint(0);
    rollEdgeDetectPID->targetSetpoint(0);
}

void pid_handler::reset_all(){
    altitudePID->on();
    yawYoloPID->on();
    pitchYoloPID->on();
    pitchDownCamPID->off();
    rollDownCamPID->off();
    pitchObstaclePID->off();
    rollObstaclePID->off();
    yawRTraversalPID->off();
    pitchEdgeDetectPID->off();
    rollEdgeDetectPID->off();
}

// when mode is set, the pids are turn on.
void pid_handler::set_pitch_mode(std::string value){
    if (value.compare("DownCam") == 0){
        pitch_mode = DownCam;
        pitchDownCamPID->on();
        pitchObstaclePID->off();
        pitchYoloPID->off();
        pitchEdgeDetectPID->off();
    }
    else if (value.compare("Obstacle") == 0){
        pitch_mode = Obstacle;
        pitchDownCamPID->off();
        pitchObstaclePID->on();
        pitchYoloPID->off();
        pitchEdgeDetectPID->off();
    }
    else if (value.compare("Yolo") == 0){
        pitch_mode = Yolo;
        pitchDownCamPID->off();
        pitchObstaclePID->off();
        pitchYoloPID->on();
        pitchEdgeDetectPID->off();
    }
    else if (value.compare("EdgeDetect") == 0){
        pitch_mode = EdgeDetect;
        pitchDownCamPID->off();
        pitchObstaclePID->off();
        pitchYoloPID->off();
        pitchEdgeDetectPID->on();
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
    case EdgeDetect:
        return "EdgeDetect";
    default:
        return "Undefined";
    }
}

void pid_handler::set_roll_mode(std::string value){
    if (value.compare("DownCam") == 0){
        roll_mode = DownCam;
        rollDownCamPID->on();
        rollObstaclePID->off();
        rollEdgeDetectPID->off();
    }
    else if (value.compare("Obstacle") == 0){
        roll_mode = Obstacle;
        rollDownCamPID->off();
        rollObstaclePID->on();
        rollEdgeDetectPID->off();
    }
    else if (value.compare("EdgeDetect") == 0){
        roll_mode = EdgeDetect;
        rollDownCamPID->off();
        rollObstaclePID->off();
        rollEdgeDetectPID->on();
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
    case EdgeDetect:
        return "EdgeDetect";
    default:
        return "Undefined";
    }
}


void pid_handler::set_yaw_mode(std::string value){
    if (value.compare("Yolo") == 0){
        yaw_mode = Yolo;
        yawYoloPID->on();
        yawRTraversalPID->off();
    }
    else if (value.compare("RandomTraversal") == 0){
        yaw_mode = RandomTraversal;
        yawYoloPID->off();
        yawRTraversalPID->on();
    }
    else{
        std::cout << "yaw_mode: " << value << " does not exist" << std::endl;
    }
}

std::string pid_handler::get_yaw_mode(){
    switch (yaw_mode){
    case Yolo:
        return "Yolo";
    case RandomTraversal:
        return "RandomTraversal";
    default:
        return "Undefined";
    }
}

PIDController& pid_handler::getThrottlePID(){
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
    switch(yaw_mode){
    case Yolo:
        return *yawYoloPID;
    case RandomTraversal:
        return *yawRTraversalPID;
    default:
        return *(new PIDController);
    }
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
    params.push_back(input);
    return params;
}

void pid_handler::load_PID_calibrations(std::string file){
    std::string line;
    std::ifstream calibration_file(file);
    if(calibration_file.is_open()){
        while(std::getline(calibration_file, line)){
            std::vector<std::string> params;
            params = parse_calibrations(line, ",");
            std::string key = params[0];
            if(DEBUG){
                std::cout << params[0] << " " << params.size() << std::endl;
                std::cout << params[1] << " " << params[2] << " " << params[3] << " " << params[4] << " " << params[5] << std::endl;            
            }
            if(key == "altitudePID"){
                altitudePID     = new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "yawYoloPID"){
                yawYoloPID      = new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "pitchYoloPID"){
                pitchYoloPID    = new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "pitchDownCamPID"){
                pitchDownCamPID = new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "rollDownCamPID"){
                rollDownCamPID  = new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "pitchObstaclePID"){
                pitchObstaclePID= new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "rollObstaclePID"){
                rollObstaclePID = new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "yawRTraversalPID"){
                yawRTraversalPID  = new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "pitchEdgeDetectPID"){
                pitchEdgeDetectPID= new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
            else if(key == "rollEdgeDetectPID"){
                rollEdgeDetectPID = new PIDController(std::stod(params[1]), std::stod(params[2]), std::stod(params[3]), std::stod(params[4]), std::stod(params[5]));
            }
        }
        calibration_file.close();
    }
    else{
        std::cout << "ERROR: NO PID CALIBRATION FILE FOUND" << std::endl;
    }
}