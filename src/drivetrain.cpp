#include "drivetrain.hpp"
#include <cmath> //move later
//this is a test please work
//Drivetrain constructor to define all ports
Drivetrain::Drivetrain(std::vector<std::int8_t>leftMotorPorts, std::vector<std::int8_t> rightMotorPorts, int gyroPort)
{
    //initialize motor groups for left side 
    this->leftSide = new pros::MotorGroup(leftMotorPorts);
    this->leftSide->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    this->leftSide->set_gearing_all(pros::E_MOTOR_GEAR_BLUE);

    //initialize motor groups for right side
    this->rightSide = new pros::MotorGroup(rightMotorPorts);
    this->rightSide->set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    this->rightSide->set_gearing_all(pros::E_MOTOR_GEAR_BLUE);
   
    //create inertial sensor object
    this->gyroSensor = new pros::Imu(gyroPort);
}

Drivetrain::~Drivetrain()
{
    delete leftSide;
    delete rightSide;
    delete gyroSensor;
}

//drivetrain control function
void Drivetrain::updateDrivetrain(pros::Controller &master)
{
    //get forward input 
    float speed = master.get_analog((controllerReversed ? pros::E_CONTROLLER_ANALOG_RIGHT_Y : pros::E_CONTROLLER_ANALOG_LEFT_Y));
    speed *= fabs(speed) > 5;
    speed = speed*fabs(speed)/127.0;
    
    //get turning input
    float turn = master.get_analog((controllerReversed ? pros::E_CONTROLLER_ANALOG_LEFT_X : pros::E_CONTROLLER_ANALOG_RIGHT_X));
    turn *= fabs(turn) > 5;
    turn = turn*fabs(turn)/127.0;

    //set drive speeds
    this->setDriveSpeed((speed + turn)*600.0/127.0, (speed - turn)*600.0/127.0);
}

//AUTONOMOUS FUNCTIONS

//function to move the robot a certain distance
/* void Drivtrain::move(float maxSpeed, float distance, float accelRate, float decelRate, float kP, float targetAngle, float minSpeed)
{
    
} */

//MISCELLANEOUS FUNCTIONS

//function to set drivetrain speed
void Drivetrain::setDriveSpeed(float leftSpeed, float rightSpeed)
{
    this->leftSide->move_velocity(leftSpeed);
    this->rightSide->move_velocity(rightSpeed);
}

// Function to set the current gyro angle of the robot
void Drivetrain::setGyroAngle(float angle)
{
    gyroSensor->set_heading(angle);
}
// Function to get the current gyro angle of the robot
float Drivetrain::getGyroAngle()
{
    return gyroSensor->get_heading();
}

//this is a test comment