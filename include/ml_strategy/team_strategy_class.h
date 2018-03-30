#ifndef STRATEGY_CLASS_H_
#define STRATEGY_CLASS_H_

#include <ros/ros.h>
#include <Eigen/Dense>
#include "ml_strategy/helper.h"
#include "nav_msgs/Odometry.h"
#include "mg_msgs/PVA.h"

// Quadcopter roles
struct QuadRole{
	uint GOALKEEPER = 0;
	uint RIGHT = 1;
	uint LEFT = 2;
	uint CENTRAL = 3;
	uint FREE = 4;

	int State = GOALKEEPER;
};

// Enemy danger assignment
struct EnemyDanger{
	uint NEUTRAL = 0;
	uint WARNING = 1;
	uint DANGER = 2;

	int State = NEUTRAL;
};

struct QuadData {
    std::string name;                            // Unique name for vehicle
    mutable QuadRole role;					     // Quad role in the team
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
    mutable EnemyDanger danger;

    bool operator<(const EnemyData& other) const {
        int compareResult = name.compare(other.name);
        return (compareResult < 0);
    }
};

class TeamStrategy {
 public:
 	std::set<QuadData> quads_;
 	std::set<EnemyData> enemies_;
 	std::string goalkeeper_;
 	uint n_quads_, n_enemies_;

 	// Constructors
 	TeamStrategy();

 	// Methods
    void PrintQuadNames();
    void PrintQuadReferences(const std::string &name);
    void AddQuad(const std::string &quad_name,
	 		     const uint &role,
	 		     const Eigen::Vector3d &ref_pos,
		         const std::string &output_topic,
		         ros::NodeHandle *nh);
	void AddEnemy(const std::string &enemy_name,
			      const nav_msgs::Odometry &odom);
    void FindQuadIndex(const std::string &name,
    	               std::set<QuadData>::iterator *index);  // Returns -1 if it can't find
	void FindEnemyIndex(const std::string &name,
                		std::set<EnemyData>::iterator *index);
    void UpdateQuadOdom(const std::string &name, 
                        const nav_msgs::Odometry &odom);
    void PublishReferences();

};

#endif  // STRATEGY_CLASS_H_