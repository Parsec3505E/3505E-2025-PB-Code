#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"

class Drivetrain
{
    public: 
        //Drivetrain setup
        Drivetrain(std::vector<std::int8_t> leftMotorPorts, std::vector<std::int8_t> rightMotorPorts, int gyroPort);
        ~Drivetrain();

        //variables
        bool controllerReversed = false;

        //functions for driver control
        void updateDrivetrain(pros::Controller &master);

        //functions for autonomus routines

        //miscellaneous functions
        void setDriveSpeed(float leftSpeed, float rightSpeed); 
        void setGyroAngle(float angle); 
        float getGyroAngle(); 

        //internal classes (motor groups, sensors, etc.)
        pros::MotorGroup* leftSide;
        pros::MotorGroup* rightSide;
        pros::Imu* gyroSensor;


};

#endif