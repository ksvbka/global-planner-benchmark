#ifndef __PLANNER_BENCHMARK_H__
#define __PLANNER_BENCHMARK_H__

#include <vector>
#include <string>
#include <boost/pointer_cast.hpp>

#include <ros/ros.h>
#include <pluginlib/class_loader.hpp>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>


class PlannerBenchmark {
 public:
  PlannerBenchmark(tf2_ros::Buffer& tf);
  ~PlannerBenchmark();
 
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
  bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
  void setPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);
  bool getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap);

 private:
  tf2_ros::Buffer& tf_;
  std::string robot_base_frame_, global_frame_;
  geometry_msgs::PoseStamped global_pose_;

  ros::Subscriber goal_sub_;
  costmap_2d::Costmap2DROS* planner_costmap_ros_;

  pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
  boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;

  std::vector<geometry_msgs::PoseStamped>* planner_plan_;
};

#endif  //__PLANNER_BENCHMARK_H__
