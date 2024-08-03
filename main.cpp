#include "main.h"
#include "logger.h"
#include <cmath>
#include <cstdio>

void initialize() {
	pros::lcd::initialize();
	
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
	while (true) {
		
		pros::delay(20);// Run for 20 ms then update
	}
}