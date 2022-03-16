#include "fake_state.h"

FakeState::FakeState()
: nh_priv_("~")
{
  bool init_result = init();
  ROS_ASSERT(init_result);
}

FakeState::~FakeState()
{
}


bool FakeState::init()
{
  nh_.param("joint_states_frame", joint_states_.header.frame_id, std::string("base_footprint"));
  nh_.param("odom_frame", odom_.header.frame_id, std::string("odom"));
  nh_.param("base_frame", odom_.child_frame_id, std::string("base_footprint"));

  // initialize publishers
  joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 100);
  odom_pub_         = nh_.advertise<nav_msgs::Odometry>("odom", 100);

  odom_.pose.pose.orientation.w = 1;
  
  // initialize subscribers
  pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&FakeState::setPose, this, _1));

  return true;
}


void FakeState::setPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_ptr)
{
  odom_.pose.pose = pose_ptr->pose.pose;
  ROS_INFO("Set Pose %4f, %4f", odom_.pose.pose.position.x, odom_.pose.pose.position.y);
}


bool FakeState::update()
{
  ros::Time time_now = ros::Time::now();

  // odom
  odom_.header.stamp = time_now;
  odom_pub_.publish(odom_);

  // joint_states
  joint_states_.header.stamp = time_now;
  joint_states_pub_.publish(joint_states_);

  // tf
  geometry_msgs::TransformStamped odom_tf;
  odom_tf.header = odom_.header;
  odom_tf.child_frame_id = odom_.child_frame_id;
  odom_tf.transform.translation.x = odom_.pose.pose.position.x;
  odom_tf.transform.translation.y = odom_.pose.pose.position.y;
  odom_tf.transform.translation.z = odom_.pose.pose.position.z;
  odom_tf.transform.rotation = odom_.pose.pose.orientation;
  tf_broadcaster_.sendTransform(odom_tf);

  return true;
}


int main(int argc, char* argv[])
{
  ros::init(argc, argv, "fake_node");
  FakeState fake_state;

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
    fake_state.update();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
