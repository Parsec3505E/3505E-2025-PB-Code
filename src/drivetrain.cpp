#ifndef DRIVETRAIN_HPP
#define DRIVETRAIN_HPP

#include "api.h"

class Drivetrain
{
public:
    // Drivetrain setup
    Drivetrain(std::vector<std::int8_t> leftMotorPorts, std::vector<std::int8_t> rightMotorPorts, int gyroPort);
    ~Drivetrain();

    // variables
    bool controllerReversed = false;

    unsigned long int defaultStoppingTime = 5000;

    float defaultGyroMargin = 2.0; 
    float deflaultMargin = 0.1;

    float wheelDiam = 3.25;

    float defaultAcc = 0.8;
    float defaultDecc = 19;
    float defaultGyroAcc = 0.8;
    float defaultGyroDecc = 3;
    float defaultGyrokP = 0.005;
    float defaultGyrokD = 0;
    float defaultMinspeed = 50;
    float defaultMaxspeed = 600;



    enum MoveStates
    {
        IN,
        TIME
    };


    const float NOTHING = 999999; 

    enum motorDef 
    {
        RIGHT, 
        LEFT, 
        NONE
    };


    float leftRelativeBase = 0;
    float rightRelativeBase = 0;

     // functions for driver control
    void updateDrivetrain(pros::Controller &master);
   

    // functions for autonomus routines
    float motionProfiling(float minSpeed, float maxSpeed, float accRate, float deccRate, float targetDistance, float currentDistance, float time, bool accelTime, bool shouldStop, float margin);
    void turnGyroRelative(float speed, float angle, float accRate = NOTHING, float decRate = NOTHING, float startSpeed = NOTHING, float endSpeed = NOTHING, bool shouldStop = true, unsigned long int stoppingTime = NOTHING, float margin = NOTHING);
    void turnGyroAbsolute(float speed, float angle, float accRate = NOTHING, float decRate = NOTHING, float startSpeed = NOTHING, float endSpeed = NOTHING, bool shouldStop = true, unsigned long int stoppingTime = NOTHING, float margin = NOTHING);

    // miscellaneous functions
    void setDriveSpeed(float leftSpeed, float rightSpeed);
    void setGyroAngle(float angle);
    float getGyroAngle();
    float sgn(float num);
    float angleDiff(float angle1, float angle2);
    void resetMotors();
    float min2(float num1, float num2);
    float getRelativeIN(int side);
    

    // internal classes (motor groups, sensors, etc.)
    pros::MotorGroup *leftSide;
    pros::MotorGroup *rightSide;
    pros::Imu *gyroSensor;
};

#endif
