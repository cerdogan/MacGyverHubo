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

#include <amino.h>
#include <utils/urdf/DartLoader.h>
#include <simulation/World.h>
#include <dynamics/Skeleton.h>
#include <dynamics/BodyNode.h>

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

	// Load the scene
	dart::utils::DartLoader dl;
	dart::simulation::World* world = dl.parseWorld("../data/dart/scenes/01-World-Robot.urdf");
	dart::dynamics::Skeleton* hubo = world->getSkeleton("Hubo");
	std::cout << "Loaded world. Ready to start?" << std::endl;
	getchar();

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
	goal << -M_PI/6, 0.0, 0.0, -2*M_PI/3, 0.0, M_PI/3;

	// Set the right arm ids
	std::vector <int> rarm_ids;
	for(size_t i = 38; i < 44; i++) rarm_ids.push_back(i);

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
		Vector6d state;
		for(size_t i = 0; i < 6; i++) 
			state(i) = cmd.joints[joint_indices[i]].position;

		// -----------------------------------------------------------------------
		// Set goal configuration using Jacobian if reached start configuration
		double err = (goal - state).norm();
		if(dbg) std::cout << "err: " << err << std::endl;
		if(err < (5 * 1e-2)) {

			// Compute the Jacobian
			hubo->setConfig(rarm_ids, state);
			Eigen::MatrixXd J = hubo->getBodyNode("Body_RWP")->getWorldJacobian().bottomRightCorner<6,6>();
			Eigen::MatrixXd temp = J.topRightCorner<3,6>();
			J.topRightCorner<3,6>() = J.bottomRightCorner<3,6>();
			J.bottomRightCorner<3,6>() = temp;
			for(size_t i = 0; i < 6; i++) J(i,i) += 0.005;
			if(dbg) std::cout << "J: [\n" << J << "]\n";

			// Compute the inverse
			Eigen::Matrix6d JInv = J.inverse();
			//JInv = J;
			//aa_la_inv(6, JInv.data());

			// Compute the joint space velocity from workspace velocity
			if(dbg) std::cout << "dx: " << dx.transpose() << std::endl;
			Vector6d dq = (JInv * dx);
			double norm = dq.norm();
			if(norm > 1e-4) dq = max_step_size * (dq / norm);
			else dq = Vector6d::Zero();
			if(dbg) std::cout << "JInv: [\n" << JInv << "]\n";
			if(dbg) std::cout << "dq: " << dq.transpose() << std::endl;

			// Update the joint command
			Vector6d next = state + max_step_size * dq;
			if(dbg) std::cout << "next: " << next.transpose() << std::endl;
			for(size_t i = 0; i < 6; i++) 
				if(fabs(dq(i)) > 0.00001)
					cmd.set_position(joint_indices[i], next(i));
//			cmd.send_commands();
		}
		
		// -----------------------------------------------------------------------
		// Move to the initial configuration
		else {

			// Compute the next joint command
			double norm = (goal - state).norm();
			Vector6d next; 
			if(norm > 1e-4) next = state + max_step_size * (goal - state).normalized();
			else {
				next = goal;
			}

			// Set command
			for(size_t i = 0; i < 6; i++) 
				if(fabs(goal(i) - cmd.joints[joint_indices[i]].position) > 2*max_step_size) 
					cmd.set_position(joint_indices[i], next(i));
			cmd.send_commands();

			// Print stuff
			if(dbg) {
				std::cout << "goal: " << goal.transpose() << std::endl;
				std::cout << "state: " << state.transpose() << std::endl;
				std::cout << "next: " << next.transpose() << std::endl;
			}
		}

		// Print the current values
		if(dbg) {
			for(size_t i = 0; i < 6; i++) 
				std::cout << cmd.joints[joint_indices[i]] << std::endl;
		}
    pthread_mutex_unlock(&mutex);
	}

	return 0;
}
