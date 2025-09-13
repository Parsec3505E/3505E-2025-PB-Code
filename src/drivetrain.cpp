#include "drivetrain.hpp"
#include <cmath> //move later

//Drivetrain constructor to define all ports
Drivetrain::Drivetrain(std::vector<std::int8_t>leftMotorPorts, std::vector<std::int8_t> rightMotorPorts, int gyroPort)
{
    //initialize motor groups for left side 
    this->leftSide = new pros::MotorGroup(leftMotorPorts);
    this->leftSide->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    this->leftSide->set_gearing_all(pros::E_MOTOR_GEAR_BLUE);

    //initialize motor groups for right side
    this->rightSide = new pros::MotorGroup(rightMotorPorts);
    this->rightSide->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
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
//the below function calculates difference between two angles, returning a value between -180 and 180 in order to find the shortest path
float Drivetrain::angleDiff(float angle1, float angle2)
{
    if (angle2-angle1 >= 180)
    {
        return -360+angle2-angle1;
    }
    else if (angle2-angle1 <=-180)
    {
        return 360+angle2-angle1;
    }
    return angle2-angle1;
}
// this function moves the robot 



void Drivetrain::turnGyroRelative(float speed, float angle, float accRate, float decRate, float startSpeed, float endSpeed, bool shouldStop, unsigned long int stoppingTime, float margin){
    // Default parameters
    startSpeed = (startSpeed == NOTHING ? defaultMinspeed : startSpeed);
    endSpeed = (endSpeed == NOTHING ? defaultMinspeed : endSpeed);
    accRate = (accRate == NOTHING ? defaultGyroAcc : accRate);
    deccRate = (deccRate == NOTHING ? defaultGyroDecc : deccRate);
    stoppingTime = (stoppingTime == NOTHING ? defaultStoppingTime : stoppingTime);
    margin = (margin == NOTHING ? defaultGyroMargin : margin);


    // Changing values so they work
    startSpeed = fabs(startSpeed);
    endSpeed = fabs(endSpeed)
    speed = fabs(speed);


    //variables used in loop
    float leftSpeed;
    float rightSpeed;

    unsigned long int time = pros::millis();

    float startAngle = getGyroAngle();

    //the loop
    while(true){
        //breaking conditions

        
        //if we take too long, break
        if (pros::millis-time>=stoppingTime){
            break;
        }
        // this checks if the angle we turned is within the margin of error of the target angle and if we should stop immediately
        if(fabs(angleDiff(angle, angleDiff(currentangle, startAngle)))<=margin && shouldStop){
            break;
        }
        //this checks if we have turned past the target angle and if we should not stop immediately
        else if (fabs(angle) <= fabs(angleDiff(getGyroAngle(), gyroBaseAngle)) && !shouldStop)
        {
            break;
        }
       
       
        //calculate left and right speeds in proportion to the angle we have turned -- note to reader understand this you must understand the motion profiling function
        leftSpeed = motionProfile(startSpeed, sgn(angle)*speed, endSpeed, accRate, deccRate, angle, angleDiff(gyroBaseAngle, getGyroAngle()), pros::millis()-baseTime, true, shouldStop, margin);
        rightSpeed = -leftSpeed;


        setDriveSpeed(leftSpeed, rightSpeed);
    }


    //resets the angle of the gyro to 0
    setGyroAngle(0); //actual function should be resetPositionRelative(); but I don't know how it works
    
    //checks if the motor stops moving or cruises
    if(shouldStop)
    {
        setDriveSpeed(0,0);
    }
    else
    {
        setDriveSpeed(sgn(angle)*endSpeed,-sgn(angle)*endSpeed)
    }
}

void Drivetrain::turnGyroAbsolute(float speed, float angle, float accRate, float decRate, float startSpeed, float endSpeed, bool shouldStop, unsigned long int stoppingTime, float margin)
{
    //this function just calls the Gyro relative function
    turnGyroRelative(speed, angleDiff(getGyroAngle(), angle), accRate, decRate, startSpeed, endSpeed, shouldStop, stoppingTime, margin);
}
//AUTONOMOUS FUNCTIONS

//function to move the robot a certain distance
/* void Drivtrain::move(float maxSpeed, float distance, float accelRate, float decelRate, float kP, float targetAngle, float minSpeed)
{
    
} */

//MISCELLANEOUS FUNCTIONS

//function to set drivetrain speed
//special note: "this ->" means thats the drive train uses leftside which will in turn use move velocity
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

float Drivetrain::sgn(num){
   return((num > 0) ? 1: (num < 0) ? -1:0);
}
    
