#include <chrono>

#include <planner_benchmark.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

PlannerBenchmark::PlannerBenchmark(tf2_ros::Buffer& tf):
      tf_(tf),
      planner_plan_(NULL),
      bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
{
  ros::NodeHandle private_nh("~");
  ros::NodeHandle nh;

  std::string global_planner;
  private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
  private_nh.param("global_costmap/robot_base_frame", robot_base_frame_, std::string("base_link"));
  private_nh.param("global_costmap/global_frame", global_frame_, std::string("map"));

  planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();

  // we'll provide a mechanism for some people to send goals as PoseStamped
  // messages over a topic they won't get any useful information back about its
  // status, but this is useful for tools like nav_view and rviz
  ros::NodeHandle simple_nh("move_base_simple");
  goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&PlannerBenchmark::goalCB, this, _1));

  // create the ros wrapper for the planner's costmap... and initializer a
  // pointer we'll use with the underlying map
  planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
  planner_costmap_ros_->pause();

  // initialize the global planner
  try {
    planner_ = bgp_loader_.createInstance(global_planner);
    planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_FATAL(
        "Failed to create the %s planner, are you sure it is properly "
        "registered and that the containing library is built? Exception: %s",
        global_planner.c_str(), ex.what());
    exit(1);
  }

  // Start actively updating costmaps based on sensor data
  planner_costmap_ros_->start();
}

PlannerBenchmark::~PlannerBenchmark() {
  if (planner_costmap_ros_ != NULL)
    delete planner_costmap_ros_;
  
  delete planner_plan_;
  planner_.reset();
}

void PlannerBenchmark::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal) {
  ros::Time start = ros::Time::now();
  makePlan(*goal, *planner_plan_);

  ros::Duration elapsed = ros::Time::now() - start;
  std::string node_name = ros::this_node::getName();
  ROS_INFO("[%s] \t - time: %.6fs - path size: %ld", node_name.c_str(), elapsed.toSec(), planner_plan_->size());
}

bool PlannerBenchmark::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
  boost::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(planner_costmap_ros_->getCostmap()->getMutex()));

  //make sure to set the plan to be empty initially
  plan.clear();

  //since this gets called on handle activate
  if(planner_costmap_ros_ == NULL) {
    ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
    return false;
  }

  //get the starting pose of the robot
  geometry_msgs::PoseStamped global_pose;
  if(!getRobotPose(global_pose, planner_costmap_ros_)) {
    ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
    return false;
  }

  const geometry_msgs::PoseStamped& start = global_pose;

  //if the planner fails or returns a zero length plan, planning failed
  if(!planner_->makePlan(start, goal, plan) || plan.empty()){
    ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
    return false;
  }

  return true;
}


bool PlannerBenchmark::getRobotPose(geometry_msgs::PoseStamped& global_pose, costmap_2d::Costmap2DROS* costmap)
{
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = robot_base_frame_;
  robot_pose.header.stamp = ros::Time(); // latest available
  ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

  // get robot pose on the given costmap frame
  try
  {
    tf_.transform(robot_pose, global_pose, costmap->getGlobalFrameID());
  }
  catch (tf2::LookupException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ConnectivityException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
    return false;
  }
  catch (tf2::ExtrapolationException& ex)
  {
    ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
    return false;
  }

  // check if global_pose time stamp is within costmap transform tolerance
  if (current_time.toSec() - global_pose.header.stamp.toSec() > costmap->getTransformTolerance())
  {
    ROS_WARN_THROTTLE(1.0, "Transform timeout for %s. " \
                      "Current time: %.4f, pose stamp: %.4f, tolerance: %.4f", costmap->getName().c_str(),
                      current_time.toSec(), global_pose.header.stamp.toSec(), costmap->getTransformTolerance());
    return false;
  }

  return true;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "planner_benchmark_node");
  tf2_ros::Buffer tf_buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(tf_buffer);

  PlannerBenchmark  planner_benchmark(tf_buffer);
  ros::spin();

  return 0;
}
