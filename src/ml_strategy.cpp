
#include "ml_strategy/globals.h"

// Global variables--------------------------
globalVariables globals_;
mutexStruct mutexes_;

void GameStateCallback(const mg_msgs::GameState::ConstPtr& msg) {
	uint n_quads = msg->GameState.size();
	pthread_mutex_lock(&mutexes_.m_team_strategy);
	for (uint i = 0; i < n_quads; i++) {
		globals_.obj_team_strategy.
			UpdateQuadOdom(msg->GameState[i].child_frame_id, 
				           msg->GameState[i]);
	}
	pthread_mutex_unlock(&mutexes_.m_team_strategy);
	// UpdateQuadOdom(const std::string &name, 
 //                                  const nav_msgs::Odometry &odom)
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

	// Set quad roles based on the number of quads
	std::vector<uint> roles;
	QuadRole role_struct;
	if(quad_names.size() == 2) {
		roles = {role_struct.GOALKEEPER, role_struct.CENTRAL};
	} else if(quad_names.size() == 3) {
		roles = {role_struct.GOALKEEPER, role_struct.RIGHT, role_struct.LEFT};
	} else {
		roles.resize(quad_names.size());
	}

	if (float(quad_names.size()) > float(init_pos.size())/3.0) {
		ROS_ERROR("[ml_strategy]: Initial positions not well defined!");
		return 0;
	} else {
		// Add team quads
		for (uint i = 0; i < quad_names.size(); i++) {
			Eigen::Vector3d pos(init_pos[3*i], init_pos[3*i+1], init_pos[3*i+2]);
			std::string output_topic = quad_names[i] + "/px4_control/PVA_Ref";
			globals_.obj_team_strategy.
				AddQuad(quad_names[i], roles[i], pos, output_topic, &node);
		}
	}

  	ros::Subscriber game_state_sub = node.subscribe<mg_msgs::GameState>
  				("/mediation_layer/Game_State", 10, GameStateCallback);
  

	// ROS loop that starts callbacks/publishers
	ros::spin();

	// Kill mutexes
	mutexes_.destroy();

	return 0;

}