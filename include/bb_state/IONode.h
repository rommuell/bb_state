// io_node.h

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "EposManager/EPOSControl.h"
#include "bb_state/TwistWithID.h"

#include "time.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "ioboard/IOFromBoard.h"
#include "ioboard/IOToBoard.h"

// Threading for async IO
// #include <thread>

const float BB_WIDTH = 0.264;
const float center_of_rot_frontwheel = 0.31;

class IONode
{
private:
  // status vars for integration
  double old_pose_theta_read_by_escon_;
  double pose_x_read_by_escon_, pose_y_read_by_escon_;
  double pose_theta_read_by_escon_leapfrog_;
  double phi_front_wheel_;
  double abs_vel_read_by_escon;
  double ang_vel_read_by_escon;
  double pose_theta_read_by_escon;

  int oldangleinquants;
  int odom_count;

  ros::NodeHandle n;
  ros::Subscriber rake_io_sub;
  ros::Subscriber move_sub;
  ros::Subscriber from_escon_sub;
  ros::Publisher to_epos_pub;
  ros::Publisher odom_pub;
  ros::Publisher to_escon_pub;
  nav_msgs::Odometry odom;
  ioboard::IOToBoard io_to_board_message_;
  
  ros::Time last_time_right, last_time_left, last_publish_time;

  EposManager::EPOSControl Eposmsg;

  // Output Variables:
  uint64_t Output_Message;
  uint64_t Output_Status;

  // Input Variables:
  uint64_t Input_Message;

public:
  void SendTwist(const bb_state::TwistWithID velocitymessage);
  void IOBoardCallback(const ioboard::IOFromBoard from_escon_message);
  void PublishOdometry();
  IONode();
  ~IONode();
};
