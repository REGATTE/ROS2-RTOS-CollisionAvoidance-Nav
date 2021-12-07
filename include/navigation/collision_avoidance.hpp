#ifndef NAVIGATION_COLLISION_AVOIDANCE_HPP
#define NAVIGATION_COLLISION_AVOIDANCE_HPP

#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <mutex>
#include <Eigen>
#include <fog_msgs/FutureTrajectory.hpp>

namespace navigation
{

class CollisionAvoidance {

public:
  CollisionAvoidance(std::string uav_name, std::string trajectory_topic, int priority, double offset_height, double horizontal_distance_threshold, double height_distance_threshold);
  bool checkTrajectory(std::vector<Eigen::Vector4d>& waypoints);
  virtual ~CollisionAvoidance(){};
  
private:
  std::string _uav_name_;
  std::string _trajectory_topic_;
  int _priority_;
  double _offset_height_;
  double _horizontal_distance_threshold_;
  double _height_distance_threshold_;

  std::vector<std::string> avoidance_names_;
  std::mutex mutex_other_uav_trajectories;
  std::mutex mutex_other_uav_diagnostics;

  std::map<std::string, std::vector<fog_msgs::msg::Vector4Stamped>> other_uav_trajectory;
  std::map<std::string, fog_msgs::msg::NavigationDiagnostics> other_uav_diagnostics;
  rclcpp::Subscription<fog_msgs::msg::FutureTrajectory>::UniquePtr other_uav_trajectory_subscriber_;
  std::vector<rclcpp::Subscription<fog_msgs::msg::FutureTrajectory>::SharedPtr> other_uav_trajectory_subscribers_;
  std::vector<rclcpp::Subscription<fog_msgs::msg::NavigationDiagnostics>::SharedPtr> other_uav_diag_subscribers_;

  bool checkCollisions(const Eigen::Vector4d& point_one, const fog_msgs::msg::Vector4Stamped& point_two);
  void otherUavTracjectoryCallback(const fog_msgs::msg::FutureTrajectory::SharedPtr msg);
  void otherUavDiagnosticsCallback(const fog_msgs::msg::NavigationDiagnostics::SharedPtr msg);

};

}  // namespace navigation
#endif
