/* 
 * File:   PIDController.cpp
 * Author: User
 * 
 * Created on April 22, 2014, 6:28 PM
 */

#include "PIDController.h"
#include <iostream>
#include <cmath>

using namespace std;

PIDController::PIDController(double kp, double ki, double kd) {
    this->setGains(kp, ki, kd);
    this->setConstraints(-1, -1);
    this->init();
}

PIDController::PIDController(double newKp, double newKi, double newKd, double lowerConstraint, double upperConstraint) {
    this->setGains(kp, ki, kd);
    this->setConstraints(lowerConstraint, upperConstraint);
    this->init();
}

PIDController::PIDController() {
    this->setGains(0, 0, 0);
    this->setConstraints(-100,100);
    this->init();
}

PIDController::PIDController(const PIDController& orig) {
    this->setGains(orig.kp, orig.ki, orig.kd);
    this->setConstraints(orig.lowerConstraint, orig.upperConstraint);
    integrator = orig.integrator;
    lastError = orig.lastError;
    lastTime = orig.lastTime;
}

PIDController::~PIDController() {
}
        
void PIDController::targetSetpoint(double setpoint) {
    this->setpoint = setpoint;
}

void PIDController::setGains(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setConstraints(double lowerConstraint, double upperConstraint) {
    this->lowerConstraint = lowerConstraint;
    this->upperConstraint = upperConstraint;
}

double PIDController::getSetpoint() {
    return setpoint;
}

double PIDController::getKp() {
    return kp;
}

double PIDController::getKi() {
    return ki;
}

double PIDController::getKd() {
    return kd;
}

void PIDController::init() {
    setpoint = 0;
    integrator = 0;
    lastError = 0;
    lastTime = 0;    
}

bool PIDController::isSettled() {

	cout << "Setpoint: " << setpoint << endl;
	cout << "Lasterror: " << lastError << endl;

	double percent = (1-(setpoint + lastError))/setpoint;

	cout<<"Percent: "<<percent<<endl;	

	if(abs(percent) < 0.02)
	{
		return true;
	}
	else
	{
		return false;
	}
}

double PIDController::calc(double processVariable, double nowTime) {
    double error = setpoint - processVariable;
    cout << "Error: " << error << endl;
    cout << "nowTime: " << nowTime << endl;
    cout << "lastTime: " << lastTime << endl;
    double samplingTime = nowTime - lastTime;
    double differentiator = (error - lastError)/samplingTime;
    cout << "Diff: " << differentiator << endl;
    integrator += (error * samplingTime);
    cout << "Int: " << integrator << endl;
    double controlVariable = kp * error + ki * integrator + kd * differentiator;
    cout << "CV (before constraint): " << controlVariable << endl;
    if(controlVariable < lowerConstraint) {
        controlVariable = lowerConstraint;
    }
    else if(controlVariable > upperConstraint) {
        controlVariable = upperConstraint;
    }
    lastError = error;
    lastTime = nowTime;
    
    return controlVariable;
}

