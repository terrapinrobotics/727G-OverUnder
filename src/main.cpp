#include "main.h"
#include "reauto/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

auto chassis =
	reauto::ChassisBuilder<>()
		.motors({14, -19, -18}, {16, 15, -13}, pros::Motor_Gears::blue)
		.controller(master)
		.build();

// the catapult is free spinning, so no need to add any functionality to reauto.
pros::Motor cata(12);
pros::Motor intake(11);

// walls
pros::adi::Pneumatics walls('A', false);

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	chassis->setDriveExponent(3);
 	chassis->setControllerDeadband(12);

	while (true) {
		chassis->tank();

		if (master.get_digital(DIGITAL_R1)) {
			intake.move_voltage(-12000);
		}

		else if (master.get_digital(DIGITAL_R2)) {
			intake.move_voltage(12000);
		}

		else {
			intake.brake();
		}

		if (master.get_digital(DIGITAL_L1)) {
			cata.move_voltage(12000);
		}

		else {
			cata.move_voltage(0);
		}

		if (master.get_digital_new_press(DIGITAL_L2)) {
			walls.toggle();
		}

		pros::delay(10);
	}
}
