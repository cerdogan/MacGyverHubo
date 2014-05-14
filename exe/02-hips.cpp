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
#include <vector>

#define NUM_JOINTS 16

using namespace std;

typedef Eigen::Matrix<double,16,1> Vector16d;
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
																					"LSP", "LEP", "RSP", "REP"};
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

	// Goal 1: Raise the arms 
	vector <Vector16d> goals;
	Vector16d goal1 = Vector16d::Zero();
	goal1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
					 0.8, -5*M_PI/6.0, 0.8, -5*M_PI/6.0;
	goals.push_back(goal1);

	// Goal 2: Bend the knees
	static const double bendAngle = (-25.0 / 180.0) * M_PI;
	Vector16d goal2 = Vector16d::Zero();
	goal2 << 0.0, 0.0, bendAngle, -2*bendAngle, bendAngle, 0.0,
					 0.0, 0.0, bendAngle, -2*bendAngle, bendAngle, 0.0,
					 0.8, -5*M_PI/6.0, 0.8, -5*M_PI/6.0;
	goals.push_back(goal2);

	// Goal 3: Sway the hips
	static const double swayAngle = (-11.0 / 180.0) * M_PI;
	Vector16d goal3 = Vector16d::Zero();
	goal3 << 0.0, swayAngle, bendAngle, -2*bendAngle, bendAngle, -swayAngle,
					 0.0, swayAngle, bendAngle, -2*bendAngle, bendAngle, -swayAngle,
					 0.8, -5*M_PI/6.0, 0.8, -5*M_PI/6.0;
	goals.push_back(goal3);

	// Reset the goals with zero configuration
	if((argc > 1) && (strcmp(argv[1], "-r") == 0)) {
		goals.clear();
		Vector16d goal = Vector16d::Zero();
		goals.push_back(goal);
	}

	// Reset only the hip goals
	else if((argc > 1) && (strcmp(argv[1], "-l") == 0)) {
		goals.clear();
		goals.push_back(goal1);
	}

	// Reset only the hip goals
	else if((argc > 1) && (strcmp(argv[1], "-s") == 0)) {
		goals.clear();
		goals.push_back(goal2);
	}

	// Reset only the hip goals
	else if((argc > 1) && (strcmp(argv[1], "-2") == 0)) {
		goals.clear();
		goals.push_back(goal1);
		goals.push_back(goal2);
	}

	// Go to the last goal
	else if((argc > 1) && (strcmp(argv[1], "-q") == 0)) {
		goals.clear();
		goals.push_back(goal3);
	}

	// Update the commands
	int c_ = 0;
	double max_step_size = 0.005;
	int goal_index = 0;
	Vector16d goal = goals[goal_index];
	int reached_goal_ctr = 0;
	Vector16d lastNext;
	while(rt.good()) {

		// Update the step size based on the goal
		if(goal_index >= 2) max_step_size = 0.002;

		// Update the state
		dbg = ((c_++ % 50) == 0);
		if(dbg) std::cout << "=========================================================" << std::endl;
		cmd.update();
		Vector16d state;
		for(size_t i = 0; i < NUM_JOINTS; i++) 
			state(i) = cmd.joints[joint_indices[i]].position;
		if(c_ == 1) lastNext = state;

		// Update the goal
		double err = (goal - state).norm();
		if(dbg) std::cout << "goal index: " << goal_index << ", err: " << err << std::endl;
		if(err < 0.05) {

			// Wait a bit
			reached_goal_ctr++;
			if(dbg) std::cout << "reached goal ctr: " << reached_goal_ctr << endl;

			// Also, update the goal configuration
			if(reached_goal_ctr > 50) {
				goal = goals[min((int)(goals.size()-1),goal_index+1)];
				goal_index++;
				reached_goal_ctr = 0;
				std::cout << "new goal: " << goal.transpose() << ", ready? " << std::endl;
			}
		}

		// Compute the next joint command using the current goal, state and step size
		Vector16d next = lastNext + max_step_size * (goal - lastNext).normalized();
		lastNext = next;

		// Set command
		for(size_t i = 0; i < NUM_JOINTS; i++) 
			if(fabs(goal(i) - cmd.joints[joint_indices[i]].position) > 2*max_step_size) 
				cmd.set_position(joint_indices[i], next(i));
		cmd.send_commands();

		// Update the command
		HuboCan::error_result_t result = cmd.update();
		if(result != HuboCan::OKAY) {   
			std::cout << "Update threw an error: " << result << std::endl;
			return 1;
		}   

		// Print stuff
		if(dbg) {
			std::cout << "goal index: " << goal_index << std::endl;
			std::cout << "goal: " << goal.transpose() << std::endl;
			std::cout << "state: " << state.transpose() << std::endl;
			std::cout << "next : " << next.transpose() << std::endl;
		}

		// Print the current values
		if(dbg) {
			for(size_t i = 0; i < NUM_JOINTS; i++) 
				std::cout << cmd.joints[joint_indices[i]] << std::endl;
		}
	}

	return 0;
}
