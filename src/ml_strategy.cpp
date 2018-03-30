#include <ros/ros.h>
#include <thread>
#include <Eigen/Dense>

// Defined libraries
#include "ml_strategy/team_strategy_class.h"
#include "mg_msgs/GameState.h"



void GameStateCallback(const mg_msgs::GameState::ConstPtr& msg) {

}

int main(int argc, char** argv){
	ros::init(argc, argv, "ml_strategy");
	ros::NodeHandle node("~");
  	ROS_INFO("Marcelino's ML Wizard strategy started!");

  	// Get team quads
	std::vector<std::string> quad_names;
	std::vector<double> init_pos;
	node.getParam("MyTeam", quad_names);
	node.getParam("InitialPosition", init_pos);

	if (float(quad_names.size()) > float(init_pos.size())/3.0) {
		ROS_ERROR("[ml_strategy]: Initial positions not well defined!");
		return 0;
	} else {
		// Add balloons
		for (uint i = 0; i < quad_names.size(); i++) {
			Eigen::Vector3d pos(init_pos[3*i], init_pos[3*i+1], init_pos[3*i+2]);
			// globals_.balloons.push_back(Balloon(color, pos));
		}
	}

  	ros::Subscriber game_state_sub = node.subscribe<mg_msgs::GameState>
  				("/mediation_layer/Game_State", 10, GameStateCallback);
  

	// ROS loop that starts callbacks/publishers
	ros::spin();

	return 0;

}