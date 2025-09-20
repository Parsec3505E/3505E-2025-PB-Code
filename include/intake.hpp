#ifndef INTAKE_HPP
#define INTAKE_HPP

#include "api.h"

class Intake
{
    public:
        // Variables
        bool controllerReversed = false;
        bool isRunning = false;
        
        // Intake Setup
        Intake(int intakePort;
        ~Intake();

        // Functions for driver control
        void updateIntake(pros::Controller &master);

        // Miscellaneous functions
        void setSpeed(float speed);

        // Internal classes (i.e. motor groups, sensors)
        pros::Motor* intakeMotor;
};

#endif
