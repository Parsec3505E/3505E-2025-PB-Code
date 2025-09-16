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
    float defaultMinspeed = 50;
    unsigned long int defaultStoppingTime = 5000;
    float wheelDiam = 3.25;
    float wheelDist = 13.5;

    float defaultAcc = 0.8;
    float defaultDecc = 19;
    float defaultGyroAcc = 0.8;
    float defaultGyroDecc = 3;
    float defaultGyrokP = 0.005;
    float defaultGyrokD = 0;
    float defaultMinspeed = 50;

    enum MoveStates
    {
        IN,
        TIME
    };

    enum defaultNums 
    {
        NOTHING = -999999; // used for default parameters
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

    // internal classes (motor groups, sensors, etc.)
    pros::MotorGroup *leftSide;
    pros::MotorGroup *rightSide;
    pros::Imu *gyroSensor;
};

#endif
