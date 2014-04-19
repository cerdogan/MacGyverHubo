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
#include <Eigen/Dense>

typedef Eigen::Matrix<double,6,1> Vector6d;

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
	for(size_t i = 0; i < 6; i++) {
		cmd.set_mode(joint_indices[i], HUBO_CMD_RIGID);
		cmd.set_position(joint_indices[i], cmd.joints[joint_indices[i]].position);
	}
	cmd.send_commands();

	// Set the goal joint values
	Vector6d goal = Vector6d::Zero();
//	goal << -0.3, -0.1, 0.0, -1.0, 1.57, 0.0;

	// Update the commands
	int counter = 0;
	double max_step_size = 0.010;
	while(rt.good()) {

		// Update the state
		cmd.update();
		Vector6d state;
		for(size_t i = 0; i < 6; i++) 
			state(i) = cmd.joints[joint_indices[i]].position;

		// Decide on the next reference values
		double norm = (goal - state).norm();
		Vector6d next; 
		if(norm > 1e-4) next = state + max_step_size * (goal - state).normalized();
		else next = goal;
		for(size_t i = 0; i < 6; i++) 
			if(fabs(goal(i) - cmd.joints[joint_indices[i]].position) > 2*max_step_size)
				cmd.set_position(joint_indices[i], next(i));
		cmd.send_commands();

		// Print the current values
		if(counter > 50) {
			std::cout << "=========================================================" << std::endl;
			std::cout << "goal: " << goal.transpose() << std::endl;
			std::cout << "state: " << state.transpose() << std::endl;
			for(size_t i = 0; i < 6; i++) 
				std::cout << cmd.joints[joint_indices[i]] << std::endl;
			std::cout << "next: " << next.transpose() << std::endl;
			counter = 0;
		}
		++counter;
	}

	return 0;
}
