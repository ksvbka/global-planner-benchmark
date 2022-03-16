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
  // goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&MoveBase::goalCB, this, _1));

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

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "planner_benchmark_node");
  tf2_ros::Buffer tf_buffer(ros::Duration(10));
  tf2_ros::TransformListener tf(tf_buffer);

  PlannerBenchmark  planner_benchmark(tf_buffer);
  ros::spin();

  return 0;
}
