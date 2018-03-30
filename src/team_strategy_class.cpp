#include "ml_strategy/team_strategy_class.h"

TeamStrategy::TeamStrategy() {

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
		new_quad.vehicle_odom = helper::GetZeroOdom();
		new_quad.quad_id = n_quads_;
		// new_quad.error_integrator = rk4(k_, kd_, max_vel_, max_acc_);
		new_quad.nh = *nh;
		new_quad.pub_reference = new_quad.nh.advertise<mg_msgs::PVA>(output_topic, 1);
		quads_.insert(new_quad);
		n_quads_ = n_quads_ + 1;
	}
}

void TeamStrategy::FindQuadIndex(const std::string &name,
               std::set<QuadData>::iterator *index){
	QuadData quad_with_name;
	quad_with_name.name = name;
	*index = quads_.find(quad_with_name);
}

void TeamStrategy::PublishReferences() {

}