/**
 * @file 02-hips.cpp
 * @author Can Erdogan
 * @date May 12, 2014
 * @brief Twists the elbow joints up and swings the hip and ankle roll joints as if the hips sway.
 */

#include "math.h"
#include <iostream>
#include <HuboCmd/Commander.hpp>
#include <HuboRT/Daemonizer.hpp>
#include <Eigen/Dense>

#include <amino.h>

#define NUM_JOINTS 14

typedef Eigen::Matrix<double,14,1> Vector14d;
typedef Eigen::Matrix<double,6,1> Vector6d;

bool dbg = false;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
Vector6d dx = Vector6d::Zero();

/* ********************************************************************************************* */
void* kbhit (void* ) {

	static const double step_size = 0.02;
  char input;
  while(true){ 
    input=std::cin.get(); 
    pthread_mutex_lock(&mutex);
    if(input=='j') dx(1) += step_size;
    else if(input=='l') dx(1) -= step_size;
    else if(input=='i') dx(0) += step_size;
    else if(input=='k') dx(0) -= step_size;
    else if(input=='u') dx(2) += step_size;
    else if(input=='o') dx(2) -= step_size;
    else if(input=='r') dx = Vector6d::Zero();
    pthread_mutex_unlock(&mutex);

	}
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

	// Start the thread for reading keyboard inputs
  pthread_t kbhitThread;
  pthread_create(&kbhitThread, NULL, &kbhit, NULL);

	// Create the commander and redirect the kill signal
	HuboCmd::Commander cmd;
	HuboRT::Daemonizer rt;
	rt.redirect_signals();
	if(!cmd.initialized()) {
		std::cout << "Commander was not initialized successfully!" << std::endl;
		return 1;
	}

	// Get the joint indices
	std::string joint_names [NUM_JOINTS] = {"LHY", "LHR", "LHP", "LKP", "LAP", "LAR", 
																					"RHY", "RHR", "RHP", "RKP", "RAP", "RAR",
																					"LEP", "REP"};
	size_t joint_indices [NUM_JOINTS];
	for(size_t i = 0; i < NUM_JOINTS; i++) joint_indices[i] = cmd.get_index(joint_names[i]);
	cmd.update();

	// Claim the leg joints
	std::cout << "Claiming joint" << std::endl;
	for(size_t i = 0; i < NUM_JOINTS; i++) cmd.claim_joint(joint_indices[i]);
	cmd.send_commands();
	cmd.update();

	// Set the command mode and reference positions
	for(size_t i = 0; i < NUM_JOINTS; i++) {
		cmd.set_mode(joint_indices[i], HUBO_CMD_RIGID);
		cmd.set_position(joint_indices[i], cmd.joints[joint_indices[i]].position);
	}
	cmd.send_commands();

	// Set the goal joint values
	Vector14d goal = Vector14d::Zero();
	goal << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					-5*M_PI/6.0, -5*M_PI/6.0;

	// Update the commands
	int c_ = 0;
	double max_step_size = 0.010;
	bool reached_initial = false;
	while(rt.good()) {

		// Update the state
		pthread_mutex_lock(&mutex);
		dbg = ((c_++ % 50) == 0);
		if(dbg) std::cout << "=========================================================" << std::endl;
		cmd.update();
		Vector14d state;
		for(size_t i = 0; i < NUM_JOINTS; i++) 
			state(i) = cmd.joints[joint_indices[i]].position;

		// Set goal configuration using Jacobian if reached start configuration
		double err = (goal - state).norm();
		if(dbg) std::cout << "err: " << err << std::endl;
		
		// Compute the next joint command
		double norm = (goal - state).norm();
		Vector14d next; 
		if(norm > 1e-4) next = state + max_step_size * (goal - state).normalized();
		else {
			next = goal;
		}

		// Set command
		for(size_t i = 0; i < NUM_JOINTS; i++) 
			if(fabs(goal(i) - cmd.joints[joint_indices[i]].position) > 2*max_step_size) 
				cmd.set_position(joint_indices[i], next(i));
		cmd.send_commands();

		// Print stuff
		if(dbg) {
			std::cout << "goal: " << goal.transpose() << std::endl;
			std::cout << "state: " << state.transpose() << std::endl;
			std::cout << "next: " << next.transpose() << std::endl;
		}

		// Print the current values
		if(dbg) {
			for(size_t i = 0; i < NUM_JOINTS; i++) 
				std::cout << cmd.joints[joint_indices[i]] << std::endl;
		}
		pthread_mutex_unlock(&mutex);
	}

	return 0;
}
