#include "drivetrain.hpp"

// Drivetrain constructor to define all ports
Drivetrain::Drivetrain(std::vector<std::int8_t> leftMotorPorts, std::vector<std::int8_t> rightMotorPorts, int gyroPort)
{
    // initialize motor groups for left side
    this->leftSide = new pros::MotorGroup(leftMotorPorts);
    this->leftSide->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    this->leftSide->set_gearing_all(pros::E_MOTOR_GEAR_BLUE);

    // initialize motor groups for right side
    this->rightSide = new pros::MotorGroup(rightMotorPorts);
    this->rightSide->set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    this->rightSide->set_gearing_all(pros::E_MOTOR_GEAR_BLUE);

    // create inertial sensor object
    this->gyroSensor = new pros::Imu(gyroPort);
}

Drivetrain::~Drivetrain()
{
    delete leftSide;
    delete rightSide;
    delete gyroSensor;
}

// drivetrain control function
void Drivetrain::updateDrivetrain(pros::Controller &master)
{
    // get forward input
    float speed = master.get_analog((controllerReversed ? pros::E_CONTROLLER_ANALOG_RIGHT_Y : pros::E_CONTROLLER_ANALOG_LEFT_Y));
    speed *= fabs(speed) > 5;
    speed = speed * fabs(speed) / 127.0;

    // get turning input
    float turn = master.get_analog((controllerReversed ? pros::E_CONTROLLER_ANALOG_LEFT_X : pros::E_CONTROLLER_ANALOG_RIGHT_X));
    turn *= fabs(turn) > 5;
    turn = turn * fabs(turn) / 127.0;

    // set drive speeds
    this->setDriveSpeed((speed + turn) * 600.0 / 127.0, (speed - turn) * 600.0 / 127.0);
}
// the below function calculates difference between two angles, returning a value between -180 and 180 in order to find the shortest path
float Drivetrain::angleDiff(float angle1, float angle2)
{
    if (angle2 - angle1 >= 180)
    {
        return -360 + angle2 - angle1;
    }
    else if (angle2 - angle1 <= -180)
    {
        return 360 + angle2 - angle1;
    }
    return angle2 - angle1;
}
// this function resets encoders
void Drivetrain::resetMotors()
{
    leftSide->tare_position();
    rightSide->tare_position();
}

// this function turns the robot relative to the gyro
void Drivetrain::turnGyroRelative(float minSpeed, float maxSpeed, float angle, float accRate, float deccRate, bool shouldStop, unsigned long int stoppingTime, float margin)
{
    // Default parameters
    minSpeed = (minSpeed == NOTHING ? defaultMinspeed : minSpeed);
    minSpeed = (maxSpeed == NOTHING ? defaultMaxspeed : maxSpeed);
    accRate = (accRate == NOTHING ? defaultGyroAcc : accRate);
    deccRate = (deccRate == NOTHING ? defaultGyroDecc : deccRate);
    stoppingTime = (stoppingTime == NOTHING ? defaultStoppingTime : stoppingTime);
    margin = (margin == NOTHING ? defaultGyroMargin : margin);

    // Changing values so they work
    maxSpeed = fabs(maxSpeed);
    minSpeed = fabs(minSpeed)

        // variables used in loop
        float leftSpeed;
        float rightSpeed;

    unsigned long int time = pros::millis();

    float startAngle = getGyroAngle();

    // the loop
    while (true)
    {
        // breaking conditions

        // if we take too long, break
        if (pros::millis - time >= stoppingTime)
        {
            break;
        }
        // this checks if the angle we turned is within the margin of error of the target angle and if we should stop immediately
        if (fabs(angleDiff(angle, angleDiff(getGyroAngle(), startAngle))) <= margin && shouldStop)
        {
            break;
        }
        // this checks if we have turned past the target angle and if we should not stop immediately
        else if (fabs(angle) <= fabs(angleDiff(getGyroAngle(), gyroBaseAngle)) && !shouldStop)
        {
            break;
        }

        // calculate left and right speeds in proportion to the angle we have turned -- note to reader understand this you must understand the motion profiling function
        leftSpeed = motionProfiling(minSpeed, maxSpeed, accRate, deccRate, angle, angleDiff(gyroBaseAngle, getGyroAngle()), pros::millis() - baseTime, true, shouldStop, margin);
        rightSpeed = -leftSpeed;

        setDriveSpeed(leftSpeed, rightSpeed);
    }

    // resets the angle of the gyro to 0
    setGyroAngle(0); // actual function should be resetPositionRelative(); but I don't know how it works

    // checks if the motor stops moving or cruises
    if (shouldStop)
    {
        setDriveSpeed(0, 0);
    }
    else
    {
        setDriveSpeed(sgn(angle)*minSpeed, -sgn(angle)*minSpeed);
    }
}

void Drivetrain::turnGyroAbsolute(float minSpeed, float maxSpeed, float angle, float accRate, float decRate, bool shouldStop, unsigned long int stoppingTime, float margin)
{
    // this function just calls the Gyro relative function
    turnGyroRelative(minSpeed, maxSpeed, angleDiff(getGyroAngle(), angle), accRate, decRate, shouldStop, stoppingTime, margin);
}
// AUTONOMOUS FUNCTIONS

// function to move the robot a certain distance
/* void Drivtrain::move(float maxSpeed, float distance, float accelRate, float decelRate, float kP, float targetAngle, float minSpeed)
{

} */

float Drivetrain::motionProfiling(float minSpeed, float maxSpeed, float accRate, float deccRate, float targetDistance, float currentDistance, float time, bool accelTime, bool shouldStop, float margin)
{
    // Makes sure all the variables are valid
    minSpeed = fabs(minSpeed);
    maxSpeed = fabs(maxSpeed);
    targetDistance = fabs(targetDistance);

    // Sets the acceleration rate to a high number if zero
    if (accRate == 0)
    {
        accRate = 99999999;
    }

    // Sets the deceleration rate to a high number if zero
    if (deccRate == 0)
    {
        deccRate = 99999999;
    }

    // Checks if the robot should be moving backwards
    bool movingBackwards = (targetDistance < 0);
    targetDistance = fabs(targetDistance);
    currentDistance *= (movingBackwards ? -1 : 1);

    // Checks the direction to accelerate
    accRate = fabs(accRate);
    deccRate = -fabs(deccRate);

    // Starts setting the speed based on where it is
    float newSpeed = 0;

    // If the robot is behind, set the speed to the starting speed
    if (currentDistance <= 0)
    {
        newSpeed = minSpeed;
    }

    // Setting the speed for between the start and target for only degrees or only time
    if (!accelTime)
    {
        // Find intersection point of accel line and deccel line
        float accelIntersection = (maxSpeed - minSpeed) / accRate;

        float deccelIntersection = (targetDistance - margin) - (maxSpeed - minSpeed) / fabs(deccRate);

        // If the intersection points are valid, get the speed in a certain way
        if (accelIntersection <= deccelIntersection)
        {
            if (currentDistance < accelIntersection && currentDistance > 0)
            {
                newSpeed = minSpeed + accRate * currentDistance;
            }
            else if (currentDistance > deccelIntersection && currentDistance < targetDistance - margin)
            {
                newSpeed = deccRate * (currentDistance - targetDistance + margin) + minSpeed;
            }
            else
            {
                newSpeed = maxSpeed;
            }
        }
        else
        {
            // If there will be no or infinite intersections, speed will go directly from start to end
            if (fabs(accRate) == fabs(deccRate))
            {
                newSpeed = currentDistance * (maxSpeed - minSpeed) / (targetDistance - margin) + minSpeed;
            }
            else
            {
                // Finds intersection point of the accel and deccel lines
                float accDeccIntersection = ((maxSpeed - minSpeed) / deccRate + targetDistance - margin) / (1 - accRate / deccRate);

                // If the current distance is before the intersection point, use accRate to speed up
                if (currentDistance < accDeccIntersection)
                {
                    newSpeed = accRate * currentDistance + minSpeed;
                }
                // If the current distance is past the intersection point use, deccRate to slow down
                else
                {
                    newSpeed = deccRate * (currentDistance - targetDistance + margin) + minSpeed;
                }
            }
        }
    }
    else
    {
        // Setting the speed for between the start and target for accel time and decel degrees
        float accelSpeed = accRate * time + minSpeed;
        float decelSpeed = deccRate * (currentDistance - targetDistance + margin) + minSpeed;

        // Ensure accelSpeed never exceeds maxSpeed
        accelSpeed = min2(accelSpeed, maxSpeed);

        // The robot should always use the slower speed (so it doesn't overshoot)
        newSpeed = min2(accelSpeed, decelSpeed);
    }

    // Different overshoot behaviour depending on whether the motor should stop or not
    if (shouldStop)
    {
        // If the motor should stop, set the speed to the ending speed with the right direction
        if (currentDistance >= targetDistance - margin)
        {
            newSpeed = minSpeed * sgn(targetDistance - currentDistance);
        }
    }
    else
    {
        // If the motor should not stop, set the speed to the ending speed
        if (currentDistance >= targetDistance - margin)
        {
            newSpeed = minSpeed;
        }
    }

    // Stops the motor in the acceptable range if the motor should stop
    if (fabs(targetDistance - currentDistance) < margin && shouldStop)
    {
        newSpeed = 0;
    }

    return newSpeed * (movingBackwards ? -1 : 1);
}

// Function to move with the gyro to straighten the robot
void Drivetrain::moveGyro(float minSpeed, float maxSpeed, float distance, float targetAngle, MoveStates state, bool shouldStop, float accRate, float deccRate, float kP, float kD, unsigned long int stoppingTime)
{
    // Default parameters if applicable
    float margin = 0;
    minSpeed = (minSpeed == NOTHING ? defaultMinspeed : minSpeed);
    maxSpeed = (maxSpeed == NOTHING ? defaultMaxspeed : maxSpeed);
    accRate = (accRate == NOTHING ? defaultAcc : accRate);
    deccRate = (deccRate == NOTHING ? defaultDecc : deccRate);
    kP = (kP == NOTHING ? defaultGyrokP : kP);
    kD = (kD == NOTHING ? defaultGyrokD : kD);
    stoppingTime = (stoppingTime == NOTHING ? defaultStoppingTime : stoppingTime);

    // Sets variables for the main loop
    float prevTurnErr = 0;

    if (state == TIME)
    {
        deccRate = accRate;
    }

    kP = fabs(kP)*sgn(maxSpeed);
    kD = -fabs(kD)*sgn(maxSpeed);

    minSpeed = fabs(minSpeed);

    distance = distance*sgn(maxSpeed);

    // Variables for the main loop
    float leftSpeed;
    float rightSpeed;
    float turnErr;
    float steer;
    unsigned long int prevTime = pros::millis();

    unsigned long int baseTime = pros::millis();

    while (true)
    {
        // Exit conditions
        if (pros::millis()-baseTime>=stoppingTime)
        {
            break;
        }
        if (fabs(getRelativeIN(LEFT)+getRelativeIN(RIGHT)) >= 2.0*fabs(distance))
        {
            break;
        }
        if (state == TIME && pros::millis()-baseTime>=fabs(distance))
        {
            break;
        }

        // Gets the speed based on where the robot is
        if (state == IN)
        {
            leftSpeed = motionProfiling(minSpeed, maxSpeed, accRate, deccRate, distance, (getRelativeIN(LEFT)+getRelativeIN(RIGHT))/2.0, pros::millis()-baseTime, true, false, margin);
        }
        else
        {
            leftSpeed = motionProfiling(minSpeed, maxSpeed, accRate, deccRate, distance, pros::millis()-baseTime, pros::millis()-baseTime, true, false, margin);
        }
        rightSpeed = leftSpeed;

        // Does the PD based on the gyro error
        turnErr = angleDiff(getGyroAngle(), targetAngle);
        turnErr *= leftSpeed;

        steer = turnErr*kP+(turnErr-prevTurnErr)/(pros::millis()-prevTime)*kD;

        leftSpeed += steer;
        rightSpeed -= steer;

        prevTurnErr = turnErr;
        prevTime = pros::millis();

        setDriveSpeed(leftSpeed, rightSpeed);
        pros::delay(20);
    }
    if (shouldStop)
    {
        setDriveSpeed(0, 0);
    }
    else
    {
        setDriveSpeed(*minSpeed, minSpeed);
    }
}


// MISCELLANEOUS FUNCTIONS

// function to set drivetrain speed
// special note: "this ->" means thats the drive train uses leftside which will in turn use move velocity
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

float Drivetrain::sgn(float num)
{
    return ((num > 0) ? 1 : (num < 0) ? -1: 0);
}

float Drivetrain::getRelativeIN(int side)
{
    float ticks;
    float gearRatio = 48.0/72.0; // gear ratio
    if (side == LEFT){
        ticks = leftSide->get_position(1);
    }
    else{
        ticks = rightSide->get_position(1);
    }
    float absoluteIN = gearRatio*wheelDiam*M_PI*ticks/300.0;
    return absoluteIN - (side == LEFT ? leftRelativeBase : rightRelativeBase);
}
