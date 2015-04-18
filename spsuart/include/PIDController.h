/* 
 * File:   PIDController.h
 * Author: User
 *
 * Created on April 22, 2014, 6:28 PM
 */

#ifndef PIDCONTROLLER_H
#define	PIDCONTROLLER_H

class PIDController {
    public:
        PIDController();
        PIDController(const PIDController& orig);
        PIDController(double newKp, double newKi, double newKd);
        PIDController(double newKp, double newKi, double newKd, double lowerConstraint, double upperConstraint);
        virtual ~PIDController();
        
        void targetSetpoint(double setpoint);
        void setGains(double kp, double ki, double kd);
        void setConstraints(double lowerConstraint, double upperConstraint);
        
        double getSetpoint();
        double getKp();
        double getKi();
        double getKd();
        
        void init();
        bool isSettled();
        double calc(double feedback, double nowTime);

        
    private:
        double setpoint; 
        double kp, ki, kd;
        double lowerConstraint, upperConstraint;
        double lastError;
        double lastTime;
        double integrator;
        
};

#endif	/* PIDCONTROLLER_H */

