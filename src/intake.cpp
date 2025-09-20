#include "intake.hpp"

// Intake constructor to define all ports
Intake::Intake(int intakePort)
{
    // Creates the intake motor object
    this->intakeMotor = new pros::Motor(intakePort);
}

// Intake destructor to delete all pointers
Intake::~Intake()
{
    delete this->intakeMotor;
}

// Intake driver control code
void Intake::updateIntake(pros::Controller &master)
{
    // Sets the intake speed (forward or backwards) based on whether the button is pressed & held
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
		this->setSpeed(-200);
	}
	else if (this->isRunning)
    {
        this->setSpeed(200);
    }
    else
    {
        this->setSpeed(0);
    }

    // Switches the state (on or off) if the button is pressed
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1))
    {
        this->isRunning = !this->isRunning;
    }
}


// MISCELLANEOUS FUNCTIONS

// Function to set the speed of the intake
void Intake::setSpeed(float speed)
{
    this->intakeMotor->move_velocity(speed);
}


