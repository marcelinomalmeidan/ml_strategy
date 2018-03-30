#include "ml_strategy/team_strategy_class.h"

TeamStrategy::TeamStrategy() {
	n_quads_ = 0;
	n_enemies_ = 0;
}

// Methods
void TeamStrategy::PrintQuadNames() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		std::cout << it->name << std::endl;
	}
	std::cout << std::endl;
}

void TeamStrategy::PrintQuadReferences(const std::string &name) {
	std::set<QuadData>::iterator it;
	this->FindQuadIndex(name, &it);
	if (it != quads_.end()) {
		ROS_INFO("Quad %s references: %f\t%f\t%f", name.c_str(), 
			     it->reference.Pos.x, it->reference.Pos.y, it->reference.Pos.z);
	} else {
		ROS_INFO("[mediation layer] Couldn't print reference: quad name not found");
	}
}

void TeamStrategy::AddQuad(const std::string &quad_name,
    			 		   const uint &role,
    			 		   const Eigen::Vector3d &ref_pos,
					       const std::string &output_topic,
					       ros::NodeHandle *nh) {
	QuadData new_quad;
	new_quad.name = quad_name;

	// Check whether quad name already exists
	if(quads_.find(new_quad) != quads_.end()) {
		ROS_WARN("[mediation layer] Tried to add quad ""%s"": already exists!", quad_name.c_str());
	} else {
		mg_msgs::PVA emptyPVA = helper::GetEmptyPVA();
		new_quad.reference = emptyPVA;
		new_quad.reference.Pos = helper::Vec3d2point(ref_pos);
		new_quad.vehicle_odom = helper::GetZeroOdom();
		new_quad.role.State = role;
		// new_quad.error_integrator = rk4(k_, kd_, max_vel_, max_acc_);
		new_quad.nh = *nh;
		new_quad.pub_reference = new_quad.nh.advertise<mg_msgs::PVA>(output_topic, 1);
		quads_.insert(new_quad);
		n_quads_ = n_quads_ + 1;
	}
}

void TeamStrategy::AddEnemy(const std::string &enemy_name,
		      				const nav_msgs::Odometry &odom) {
	EnemyData new_enemy;
	new_enemy.name = enemy_name;

	// Check whether enemy name already exists
	if(enemies_.find(new_enemy) != enemies_.end()) {
		ROS_WARN("[mediation layer] Tried to add quad ""%s"": already exists!", enemy_name.c_str());
	} else {
		new_enemy.vehicle_odom = odom;
		enemies_.insert(new_enemy);
		n_enemies_ = n_enemies_ + 1;
	}
}

void TeamStrategy::UpdateQuadOdom(const std::string &name, 
                                  const nav_msgs::Odometry &odom) {
	std::set<QuadData>::iterator it1;
	this->FindQuadIndex(name, &it1);
	if (it1 != quads_.end()) {
		it1->vehicle_odom = odom;
	} else {
		std::set<EnemyData>::iterator it2;
		this->FindEnemyIndex(name, &it2);
		if (it2 != enemies_.end()) {
			it2->vehicle_odom = odom;
		} else {
			this->AddEnemy(name, odom);
		}
	}
}

void TeamStrategy::FindQuadIndex(const std::string &name,
               					 std::set<QuadData>::iterator *index){
	QuadData quad_with_name;
	quad_with_name.name = name;
	*index = quads_.find(quad_with_name);
}

void TeamStrategy::FindEnemyIndex(const std::string &name,
               					  std::set<EnemyData>::iterator *index){
	EnemyData quad_with_name;
	quad_with_name.name = name;
	*index = enemies_.find(quad_with_name);
}

void TeamStrategy::PublishReferences() {
	std::set<QuadData>::iterator it;
	for(it = quads_.begin(); it != quads_.end(); ++it) {
		it->pub_reference.publish(it->reference);	
	}
}