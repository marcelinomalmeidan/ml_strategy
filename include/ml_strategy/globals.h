#include <ros/ros.h>
#include <thread>
#include <Eigen/Dense>

// Defined libraries
// #include "ml_strategy/team_strategy_class.h"
#include "mg_msgs/GameState.h"

// Structs for global variables
#include "ml_strategy/structs.h"


// Declare global variables
extern globalVariables globals_;
extern mutexStruct mutexes_;