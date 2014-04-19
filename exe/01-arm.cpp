/**
 * @file 01-arm.cpp
 * @author Can Erdogan
 * @date April 19, 2014
 * @brief Performs Jacobian on the right arm of Hubo as a practice for static walking. Using only 
 * six joints for now.
 */

#include "math.h"
#include <iostream>
#include <HuboCmd/Commander.hpp>
#include <HuboRT/Daemonizer.hpp>

int main(int argc, char* argv[]) {

	// Create the commander and redirect the kill signal
	HuboCmd::Commander cmd;
	HuboRT::Daemonizer rt;
	rt.redirect_signals();
	if(!cmd.initialized()) {
		std::cout << "Commander was not initialized successfully!" << std::endl;
		return 1;
	}

	// Get the joint indices
	std::string joint_names [] = {"RSP", "RSR", "RSY", "REP", "RWY", "RWP"};
	size_t joint_indices [6];
	for(size_t i = 0; i < 6; i++) joint_indices[i] = cmd.get_index(joint_names[i]);
	cmd.update();

	// Claim the right arm joints
	std::cout << "Claiming joint" << std::endl;
	for(size_t i = 0; i < 6; i++) cmd.claim_joint(joint_indices[i]);
	cmd.send_commands();
	cmd.update();

	// Set the command mode and reference positions
	double last_goal_values [6]; 
	for(size_t i = 0; i < 6; i++) {
		cmd.set_mode(joint_indices[i], HUBO_CMD_RIGID);
		cmd.set_position(joint_indices[i], cmd.joints[joint_indices[i]].position);
		last_goal_values[i] = cmd.joints[joint_indices[i]].position;
	}
	cmd.send_commands();

	// Set the goal joint values
	double k = 0.0;
	double goal_values [6] = {k, 0.0, 0.0, -2*k, 0.0, 0.0};

	// Update the commands
	int counter = 0;
	double max_step_size = 0.006;
	while(rt.good()) {

		// Update the state
		cmd.update();

		// Decide on the next reference values
		bool updated = false;
		for(size_t i = 0; i < 6; i++) {
			if(fabs(last_goal_values[i] - cmd.joints[joint_indices[i]].position) < 0.020) {
				updated = true;
				double goal = cmd.joints[joint_indices[i]].position;
				bool setPos = false;
				if((goal_values[i] - cmd.joints[joint_indices[i]].position) > 0.020) {
					goal = last_goal_values[i] + max_step_size;
					setPos = true;
				}
				else if((goal_values[i] - cmd.joints[joint_indices[i]].position) < -0.020) {
					goal = last_goal_values[i] - max_step_size;
					setPos = true;
				}
				if(setPos) {
					cmd.set_position(joint_indices[i], goal);
					last_goal_values[i] = goal;
				}
			}
		}
		if(updated) cmd.send_commands();

		// Print the current values
		if(counter > 50) {
			std::cout << "=========================================================" << std::endl;
			for(size_t i = 0; i < 6; i++) 
				std::cout << cmd.joints[joint_indices[i]] << std::endl;
			std::cout << "goal pos: {";
			for(size_t i = 0; i < 6; i++) 
				std::cout << last_goal_values[i] << "\t";
			std::cout << "}\n";
			counter = 0;
		}
		++counter;
	}

	return 0;
}
