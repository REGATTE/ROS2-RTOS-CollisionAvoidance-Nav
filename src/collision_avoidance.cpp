
#include "navigation/collision_avoidance.hpp"

namespace navigation{

  CollisionAvoidance::CollisionAvoidance(std::string uav_name, std::string trajectory_topic, int priority, double offset_height, double horizontal_distance_threshold, double height_distance_threshold){

    this->_uav_name_ = uav_name;
    this->_trajectory_topic_ = trajectory_topic;
    this->_priority_ = priority;
    this->_offset_height_ = offset_height;
    this->_horizontal_distance_threshold_ = horizontal_distance_threshold;
    this->_height_distance_threshold_ = height_distance_threshold;

    /* other_uav_trajectory_subscriber_  = this->create_subscription<fog_msgs::msg::FutureTrajectory>("~/trajectories_in", 1, std::bind(&Navigation::otherUavTracjectoryCallback, this, _1)); */

  }



  /* void CollisionAvoidance::otherUavTracjectoryCallback(const fog_msgs::msg::FutureTrajectory::SharedPtr msg){ */

  /*   return; */
  /* } */

  /* bool CollisionAvoidance::checkCollisions(const Eigen::Vector4d& one, const fog_msgs::msg::Vector4Stamped& two){ */
  /*   if (sqrt(one.x() * two.x + one.y() * two.y) < this->_horizontal_distance_threshold_ || fabs(one.z() - two.z) < this->_height_distance_threshold_) */
  /*   { */
  /*       return true; */
  /*   } */
  /* } */
}
