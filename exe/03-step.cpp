/**
 * @file 03-step.cpp
 * @author Can Erdogan
 * @date April 19, 2014
 * @brief Assuming the robot's knees are bent, the executable raises the right foot, moves it
 * forward, takes the left foot down, places the right foot. 
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

#define NUM_JOINTS 14

using namespace std;

typedef Eigen::Matrix<double,14,1> Vector14d;
typedef Eigen::Matrix<double,6,1> Vector6d;

bool dbg = false;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

dart::dynamics::Skeleton* hubo;
double max_step_size = 0.005;

/* ********************************************************************************************* */
void moveFoot(const Eigen::VectorXd& dx, bool left, Vector6d& dq) {

	// Get the jacobian
	Eigen::MatrixXd J = hubo->getBodyNode(left ? "leftFoot" : "rightFoot")
		->getWorldJacobian().bottomRightCorner<6,6>();
	Eigen::MatrixXd temp = J.topRightCorner<3,6>();
	J.topRightCorner<3,6>() = J.bottomRightCorner<3,6>();
	J.bottomRightCorner<3,6>() = temp;
	for(size_t i = 0; i < 6; i++) J(i,i) += 0.005;
	if(dbg) std::cout << "J= [\n" << J << "];\n";

	// Compute the inverse
	Eigen::MatrixXd JInv;
	JInv = J;
	aa_la_inv(6, JInv.data());

	// Compute joint space velocity
	if(dbg) cout << "dxRightLeg: " << dx.transpose() << endl;
	dq = (JInv * dx);
	if(dq.norm() > max_step_size) dq = dq.normalized() * max_step_size;
	if(dbg) cout << "dqRightLeg: " << dq.transpose() << endl;
}

/* ********************************************************************************************* */
int main(int argc, char* argv[]) {

	// Load the scene
	dart::utils::DartLoader dl;
	dart::simulation::World* world = dl.parseWorld("../data/dart/scenes/01-World-Robot.urdf");
	hubo = world->getSkeleton("Hubo");
	std::cout << "Loaded world. Ready to start?" << std::endl;
	getchar();

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
	cmd.update();

	// Set the right arm ids
	std::vector <int> rightLegIds;
	for(size_t i = 12; i < 18; i++) rightLegIds.push_back(i);

	// Update the commands
	int c_ = 0;
	Vector14d lastNext;
	size_t goal_index = 1;
	while(rt.good()) {

		// Update the state
		dbg = ((c_++ % 50) == 0);
		if(dbg) std::cout << "=========================================================" << std::endl;
		cmd.update();
		Vector14d state;
		for(size_t i = 0; i < NUM_JOINTS; i++) 
			state(i) = cmd.joints[joint_indices[i]].position;
		if(c_ == 1) lastNext = state;
		Vector6d rightLegState = state.block<6,1>(6,0);
		hubo->setConfig(rightLegIds, rightLegState);

		// Set the workspace velocity
		Eigen::Vector6d dx;
		if(goal_index == 0) 
			dx << 0.0, 0.0, -1.0, 0.0, 0.0, 0.0;
		else if(goal_index == 1) 
			dx << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;

		// Get the jointspace velocity for the right leg
		Eigen::Vector6d dqRightLeg;
		moveFoot(dx, false, dqRightLeg);
		Vector14d dq = Vector14d::Zero();
		dq.block<6,1>(6,0) = dqRightLeg;
		if(dbg) cout << "dq: " << dq.transpose() << endl;

		// Compute the next joint command using the current goal, state and step size
		Vector14d next = lastNext + dq;
		lastNext = next;

		// Set command
		for(size_t i = 0; i < NUM_JOINTS; i++) 
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
