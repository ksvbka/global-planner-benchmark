#ifndef __FAKE_STATE_NODE_H__
#define __FAKE_STATE_NODE_H__

#include <math.h>

#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>


class FakeState
{
 public:
  FakeState();
  ~FakeState();
  bool init();
  bool update();
  void setPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose);

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Topic Publishers
  ros::Publisher joint_states_pub_;
  ros::Publisher odom_pub_;

  // ROS Topic Subscribers
  ros::Subscriber pose_sub_;

  sensor_msgs::JointState joint_states_;
  nav_msgs::Odometry odom_;
  tf::TransformBroadcaster tf_broadcaster_;

};

#endif // __FAKE_STATE_NODE_H__
