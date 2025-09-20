#ifndef AUTON
#define AUTON

#include "drivetrain.hpp"

void skills(){
   pros::delay(200);
   intake.setSpeed(200);
   drivetrain.moveGyro(30, 10, Drivetrain::IN, false); 
} 

#endif