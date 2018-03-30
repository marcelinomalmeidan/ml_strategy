#ifndef STRATEGY_CLASS_H_
#define STRATEGY_CLASS_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "ml_strategy/helper.h"
#include "nav_msgs/Odometry.h"
#include "mg_msgs/PVA.h"

struct QuadData {
    std::string name;                            // Unique name for vehicle
    uint quad_id;								 // Unique Ids for the quads (id=0 is goalkeeper)
    mutable mg_msgs::PVA reference;			     // Output reference
    mutable nav_msgs::Odometry vehicle_odom;     // Measured odometry
    // mutable rk4 error_integrator;                // Runge-kutta dynamics integrator
    mutable ros::NodeHandle nh;                  // ROS Nodehandle
    mutable ros::Publisher pub_reference;  		 // Publishes the reference

    bool operator<(const QuadData& other) const {
        int compareResult = name.compare(other.name);
        return (compareResult < 0);
    }
};

struct EnemyData {
    std::string name;                            // Unique name for vehicle
    mutable nav_msgs::Odometry vehicle_odom;     // Measured odometry
    mutable uint danger;

    bool operator<(const QuadData& other) const {
        int compareResult = name.compare(other.name);
        return (compareResult < 0);
    }
};

class TeamStrategy {
 public:
 	std::set<QuadData> quads_;
 	std::string goalkeeper_;
 	uint n_quads_, n_enemies_;

 	// Constructors
 	TeamStrategy();

 	// Methods
    void PrintQuadNames();
    void PrintQuadReferences(const std::string &name);
    void AddQuad(const std::string &quad_name,
			     const std::string &output_topic,
			     ros::NodeHandle *nh);
    void FindQuadIndex(const std::string &name,
    	               std::set<QuadData>::iterator *index);  // Returns -1 if it can't find
    void PublishReferences();

};

#endif  // STRATEGY_CLASS_H_