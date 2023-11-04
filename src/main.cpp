#include "main.h"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "reauto/api.hpp"
#include "reauto/motion/purepursuit/PurePursuit.hpp"
#include "reauto/odom/Odometry.hpp"
#include <fstream>
#include <memory>
#include <streambuf>
#include <string>

// A = close HWP, B = far, C = skills
#define AUTO 'B'
// A for ARCADE, T for TANK
#define DRIVETYPE 'A'

pros::Controller master(pros::E_CONTROLLER_MASTER);

auto chassis =
	reauto::ChassisBuilder<>()
		.motors({14, -19, -18}, {16, 15, -13}, pros::MotorGears::blue)
		.controller(master)
		.setChassisConstants(10.5_in, 3.25_in, 360)
		.imu(21)
		.build();

// the catapult is free spinning, so no need to add any functionality to reauto.
pros::Motor cata(12);
pros::Motor intake(11, pros::MotorGears::green);

// walls
pros::adi::Pneumatics walls('A', false);
pros::adi::Pneumatics doinker('B', false);
pros::adi::Pneumatics climb('E', false);

#define LAT_MOVE 12
#define ANG_MOVE 45

// set up PID controller for lateral movement
std::vector<IPIDConstants> latConstants = {
	{ 24, 0, 0, 0},
	{ 12, 0, 0.4, 6},
	{ 12, 0, 0.4, 12},
	{ 12, 0, 0.6, 18},
	{ 12, 0, 0.7, 24},
	{12, 0, 0.9, 60}
};

// angular movement
std::vector<IPIDConstants> angConstants = {
	{3.8, 0, 0.21, 45},
	{3.6, 0, 0.24, 70},
	{3.6, 0, 0.24, 90},
	{3.4, 0, 0.24, 120}
};

PIDExits latExits = {
	0.1,
	0.5,
	50,
	140,
	250
};

PIDExits angExits = {
	0.5,
	2,
	60,
	150,
	250
};

auto latPID = std::make_shared<reauto::controller::PIDController>(latConstants, latExits, 0, 5);
auto angPID = std::make_shared<reauto::controller::PIDController>(angConstants, angExits, 5);
auto controller = std::make_shared<reauto::MotionController>(chassis, latPID.get(), angPID.get());

auto purePursuit = std::make_shared<reauto::motion::PurePursuit>(chassis.get());

void initialize() {
	chassis->init();
}

void disabled() {}

void competition_initialize() {}

void shootLoop(int time_ms) {
	// shoot for ~45 seconds
	cata.move_voltage(10800);
	pros::delay(time_ms);
	cata.move_voltage(0);
}

void skillsAuto() {
	controller->turn(-37_deg, 100_pct, false, 900);

	pros::delay(1000);

	doinker.toggle();
	shootLoop(36000);
	doinker.toggle();

	// delay for safety
	pros::delay(1500);

	controller->drive({0, 82}, 80_pct, false, 1950);

	// turn around the corner
	controller->drive({-20, 82}, 100_pct, true, 1200);
	controller->drive({-22, 62}, 100_pct, true, 820);
	
	controller->drive({-54, 61}, 80_pct, true, 1000);

	controller->turn(-180_deg, 80_pct, false, 750);
	controller->drive(4_in, 100_pct, 500);
	pros::delay(500); // TODO: figure out why this is needed

	// walls out
	walls.toggle();

	controller->drive({-54, 95}, 100_pct, true, 2000);

	// back up a bit
	controller->drive(10_in, 100_pct, 1500);

	// walls in
	walls.toggle();

	controller->drive({-70, 68}, 100_pct, true, 2000);

	// walls out
	walls.toggle();

	controller->drive({-54, 102}, 100_pct, true, 2000);

	// turn to push under
	controller->turn(-180_deg, 100_pct, false, 600);

	controller->drive(14_in);

	// walls in
	walls.toggle();
}

void matchAutoFar() {
	controller->drive({0, 50}, 90_pct, false, 1800);
	controller->turn(92_deg, 100_pct, false, 1000);
	controller->drive(5_in, 100_pct, 900);
	
	// score triball
	intake.move_relative(720, 600);
	pros::delay(500);
	controller->drive(6_in, 100_pct, 650);

	// back up for other triball
	controller->drive({-15, 47}, 100_pct, false, 1800);
	controller->turn(-90_deg, 100_pct, false, 1000);
	walls.toggle();
	pros::delay(200);
	controller->drive({8, 50}, 100_pct, true, 1800);

	// go forwards and remove walls
	controller->drive(10_in, 100_pct, 1000);
	walls.toggle();

	// drive to triball
	intake.move(-127);
	controller->drive({-18, 55}, 80_pct, false, 1800);

	// back up with triball
	pros::delay(1000);
	intake.brake();
	controller->drive(-12_in, 100_pct, 1500);
	controller->turn(92_deg, 100_pct, false, 1000);

	// spit out triball
	intake.move_relative(360, 150);
	pros::delay(750);
	controller->drive(-5_in, 100_pct, 750);

	// reverse to goal
	controller->turn(-90_deg, 100_pct, false, 900);
	walls.toggle();
	controller->drive({16, 50}, 100_pct, true); // should be scored!
}

void matchAutoClose() {
	controller->drive({0, 28}, 100_pct, false, 1200);
	controller->turn(47_deg, 100_pct, false, 900);
	controller->drive(-28_in, 100_pct, 1800);

	// grab match load
	doinker.toggle();
	pros::delay(500);

	// bring it fwd
	controller->turn(105_deg, 100_pct, false, 1000);
	controller->drive(8_in, 100_pct, 1000);
	doinker.toggle();

	// go score preload
	controller->drive({-5, 55}, 90_pct, false, 1500);

	// turn to face goal
	controller->turn(-90_deg, 100_pct, false, 1000);

	// score preload
	controller->drive(4_in, 100_pct, 800);
	intake.move_relative(720, 600);
	pros::delay(450);
	controller->drive(8_in, 100_pct, 800);

	// elevate
	controller->drive({-5, 8}, 100_pct, false, 1800);
	controller->drive({30, 8}, 80_pct, false, 2000);
}

void autonomous() {
	if (AUTO == 'A') {
		matchAutoClose();
	}

	else if (AUTO == 'B') {
		matchAutoFar();
	}

	else {
		skillsAuto();
	}
}

void opcontrol() {
	//controller->drive(LAT_MOVE);
	//controller->turn(ANG_MOVE);

	chassis->setDriveExponent(3);
 	chassis->setControllerDeadband(12);

	chassis->setArcadeDriveChannels(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_X);
	chassis->setSecondaryArcadeTurnChannel(pros::E_CONTROLLER_ANALOG_LEFT_X);

	//controller->drive({12, 12}, 127);

	while (true) {
		// step our tank drive loop, handled by reauto
		if (DRIVETYPE == 'T') chassis->tank();
		else chassis->arcade();

		// get pose
		Pose p = chassis->getPose();
		//std::cout << "X: " << p.x << ", Y: " << p.y << ", Angle: " << p.theta.value_or(0) << std::endl;

		// intake controls
		if (master.get_digital(DIGITAL_R1)) {
			intake.move_voltage(-12000);
		}

		else if (master.get_digital(DIGITAL_R2)) {
			intake.move_voltage(12000);
		}

		else {
			intake.brake();
		}

		// catapult controls
		if (master.get_digital(DIGITAL_L1)) {
			cata.move_voltage(10800);
		}

		else {
			cata.move_voltage(0);
		}
		
		// pneumatic walls
		if (master.get_digital_new_press(DIGITAL_L2)) {
			walls.toggle();
		}

		if (master.get_digital_new_press(DIGITAL_A)) {
			doinker.toggle();
		}

		// climb
		if (master.get_digital_new_press(DIGITAL_X)) {
			climb.toggle();
		}

		pros::delay(10);
	}
}