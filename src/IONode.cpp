#include "time.h"

#include "ros/ros.h"

#include <nav_msgs/Odometry.h>

#include <bb_state/TwistWithID.h>

#include <EposManager/EPOSControl.h>
#include <tf/transform_broadcaster.h>
#include <ioboard/IOFromBoard.h>
#include <ioboard/IOToBoard.h>
// #include <callback_queue.h>
#include <bb_state/IONode.h>

// Threading for async IO
#include <thread>

void IONode::SendTwist(const bb_state::TwistWithID velocitymessage) {

  double abs_vel_input, ang_vel_input;
  char sign_right, sign_left;
  double vel_right_wheel, vel_left_wheel;
  int hexvel_left_wheel, hexvel_right_wheel;
  double phi_front_wheel;

  abs_vel_input = velocitymessage.twist.linear.x;
  ang_vel_input = velocitymessage.twist.angular.z;  
  //Calculation of Wheel velocities:
  vel_left_wheel = (abs_vel_input + ang_vel_input * BB_WIDTH);
  vel_right_wheel = (abs_vel_input - ang_vel_input * BB_WIDTH);
  
  // Velocity signs
  sign_left = vel_left_wheel < 0 ? -1 : 1;
  sign_right = vel_right_wheel < 0 ? -1 : 1;

  //Conversion to numbers that the escon takes:
  hexvel_left_wheel = round(32512 * vel_left_wheel + 32768);
  hexvel_right_wheel = round(32512 * vel_right_wheel + 32768);

  ROS_INFO("HV_LEFT: %d, HV_RIGHT: %d, BB_WIDTH: %f", hexvel_left_wheel, hexvel_right_wheel, BB_WIDTH);
  uint8_t rake = velocitymessage.rake;

  io_to_board_message_.motor_left = hexvel_left_wheel;
  io_to_board_message_.motor_right = hexvel_right_wheel;
  io_to_board_message_.status = 1;
  io_to_board_message_.rake_flags = rake;

  to_escon_pub.publish(io_to_board_message_);


  //Calculation of Frontwheel Steeringangle:
  if(abs_vel_input < 0.00001 && abs_vel_input > -0.00001) {
    if(ang_vel_input > 0.00002 || ang_vel_input < -0.00002) {
      if(ang_vel_input < 0) {
        phi_front_wheel = -M_PI/2;
        }
      else {
        phi_front_wheel = M_PI/2;
      }
    }
    else if(oldangleinquants == 0) {
      phi_front_wheel = 0;
    }
  }
  else {
    phi_front_wheel = -atan2((center_of_rot_frontwheel * ang_vel_input), 
                              abs_vel_input
                            );
  }

  // Conversion of Frontwheel angle into numbers 
  // corresponding to the quants of front wheel motor:
  int angleinquants = floor(phi_front_wheel/M_PI * 1464);
  int roundsturnedfrontwheelmotor = floor(oldangleinquants/1464);
  //Ensuring that the front wheel does sensible movements:
  angleinquants = roundsturnedfrontwheelmotor * 1464 + angleinquants;

  int test_angle = angleinquants - 2928;
  int temp = 333330;
  for(int i = 0; i < 5; test_angle += 1464, i++) {
    ROS_INFO("%d , %d", test_angle, i);
    if(abs(oldangleinquants - test_angle) < temp) {
      temp = abs(oldangleinquants - test_angle);
      angleinquants = test_angle;
    }
  }
      
  //Publish to frontwheel Epos:
  Eposmsg.node_id = 1;
  Eposmsg.control_mode = 3;
  Eposmsg.setpoint = angleinquants;
  to_epos_pub.publish(Eposmsg);


  //Assign oldangleinquants:
  oldangleinquants = angleinquants;
}

void IONode::IOBoardCallback(ioboard::IOFromBoard from_escon_message) {
  double v_left, v_right;

  ros::Time current_time = ros::Time::now();
  ros::Duration time_increment;
  if(from_escon_message.status == 2){
    v_left = from_escon_message.velocity;
    v_right = 409;
    time_increment = (current_time - last_time_left);
    last_time_left = current_time;

  }
  else {
    v_right = from_escon_message.velocity;
    v_left  = 409;
    time_increment = (current_time - last_time_right);
    last_time_right = current_time;
  }
  
  double d_time_inc = time_increment.toSec();

  // 4.94V is absolute max (equals 1023 ticks)
  v_left = v_left / 1023 * 4.94;
  v_right = v_right / 1023 * 4.94;

  // 0 == -4000, 3.94 == + 4000
  double rpm_left = -4000 + (v_left / 3.94) * 8000;
  double rpm_right = -4000 + (v_right / 3.94) * 8000;

  double vel_right_wheel_read_by_escon = rpm_right / (60 * 47) * 0.725;
  double vel_left_wheel_read_by_escon =  rpm_left / (60 * 47) * 0.725;

  abs_vel_read_by_escon = (vel_right_wheel_read_by_escon + vel_left_wheel_read_by_escon) / 2;
  ang_vel_read_by_escon = (vel_right_wheel_read_by_escon - vel_left_wheel_read_by_escon) / BB_WIDTH;

  //Integration:
  double delta_pose_theta_read_by_escon = ang_vel_read_by_escon * d_time_inc;
  double delta_pose_x_read_by_escon = abs_vel_read_by_escon * cos(pose_theta_read_by_escon) * d_time_inc;
  double delta_pose_y_read_by_escon = abs_vel_read_by_escon * sin(pose_theta_read_by_escon) * d_time_inc;

  //Update:
  pose_x_read_by_escon_ += delta_pose_x_read_by_escon;
  pose_y_read_by_escon_ += delta_pose_y_read_by_escon;

  pose_theta_read_by_escon += delta_pose_theta_read_by_escon;

  pose_theta_read_by_escon_leapfrog_ =  (old_pose_theta_read_by_escon_ + pose_theta_read_by_escon)/2;

  old_pose_theta_read_by_escon_ = pose_theta_read_by_escon;
  ++odom_count;
  if(this->odom_count >= 12) {
    this->PublishOdometry();
    this->odom_count = 0;
  }
}

void IONode::PublishOdometry() {    
  //if((ros::Time::now() - last_publish_time).nsec > 40000000) {

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_theta_read_by_escon_leapfrog_);

    odom.header.seq = 1;

    last_publish_time = ros::Time::now();
    // last_publish time is current time
    odom.header.stamp = last_publish_time;
    odom.header.frame_id = "robot_frame";

    odom.pose.pose.position.x = pose_x_read_by_escon_;
    odom.pose.pose.position.y = pose_y_read_by_escon_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "robot_frame";
    odom.twist.twist.linear.x = abs_vel_read_by_escon;
    odom.twist.twist.angular.z = ang_vel_read_by_escon;

    odom_pub.publish(odom);
  //}
}

IONode::IONode() {

  from_escon_sub = n.subscribe("io_from_board", 1, &IONode::IOBoardCallback, this);

  to_escon_pub = n.advertise<ioboard::IOToBoard>("to_ioboard", 1);
  to_epos_pub = n.advertise<EposManager::EPOSControl>("/motors/BeachBot/Motor_Control", 1);

  odom_pub = n.advertise<nav_msgs::Odometry>("odometry", 100);
  
  last_time_right = ros::Time::now();
  last_time_left = ros::Time::now();

  old_pose_theta_read_by_escon_ = 0;
  pose_x_read_by_escon_ = 0;
  pose_y_read_by_escon_ = 0;
  pose_theta_read_by_escon_leapfrog_ = 0;
  phi_front_wheel_ = 0;
  abs_vel_read_by_escon = 0;
  ang_vel_read_by_escon = 0;
  pose_theta_read_by_escon = 0;
  oldangleinquants = 0;
  odom_count = 0;



  // Put all pins up and stop robot
  bb_state::TwistWithID stop_twist;
  stop_twist.twist.linear.x = 0;
  stop_twist.twist.angular.z = 0;
  stop_twist.id = 0;
  stop_twist.rake = 0;
  SendTwist(stop_twist);
}

IONode::~IONode() {
  bb_state::TwistWithID stop_twist;
  stop_twist.twist.linear.x = 0;
  stop_twist.twist.angular.z = 0;
  stop_twist.id = 0;
  stop_twist.rake = 0;
  SendTwist(stop_twist);
  
  //Shut down escon motors
  io_to_board_message_.motor_left = 0;
  io_to_board_message_.motor_right = 0;
  io_to_board_message_.status = 0;
  to_escon_pub.publish(io_to_board_message_);
  
}
