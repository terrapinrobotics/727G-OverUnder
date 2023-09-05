#include "main.h"
#include "reauto/api.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);

auto chassis =
	reauto::ChassisBuilder<>()
		.motors({-20, 19, 18}, {-16, 15, 13}, pros::Motor_Gears::blue)
		.controller(master)
		.build();

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	chassis->setDriveExponent(3);
 	chassis->setControllerDeadband(12);

	while (true) {
		chassis->tank();
		pros::delay(10);
	}
}
