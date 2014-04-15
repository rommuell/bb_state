#include "ros/ros.h"
#include <bb_state/TwistWithID.h>
#include "Initialization.cpp"
#include <bb_state/IONode.h>

using bb_state::TwistWithID;

// Yolo

class StateMachine {

private: 
  ros::NodeHandle n;
  ros::Subscriber state_sub;
  ros::Subscriber move_sub;
  ros::AsyncSpinner spinner;
  IONode * io;
  ros::Publisher io_pub;
  int current_state_;
  int joystick;
  void loop() { //main loop
    // ros::Rate loop_rate(25);
    spinner.start();
    ros::waitForShutdown();

    // while (ros::ok() && current_state_ != TwistWithID::SHUT_DOWN) {
    //   ros::spinOnce();
      // if (current_state_ == TwistWithID::INITIALIZE) {  //do stuff for initialization
      //   current_state_ = TwistWithID::PATH_FOLLOWING; //set to joystick if success
      // }
    //   loop_rate.sleep();
    // }
  };

  void SetState(const int &new_state) {
    if(new_state != current_state_) {
      current_state_ = new_state;
      if (new_state == TwistWithID::EMERGENCY_STOP) {
        spinner.stop();
        ros::shutdown();
        ROS_ERROR("EMERGENCY STOP");
      } 
      if (new_state == TwistWithID::INITIALIZE) {
        ROS_INFO("Starting initialization");
        ros::Rate init_rate(1);   //Only try once a second
        Initialization *init = new Initialization();
        while(ros::ok() && !(init->Initialize())) init_rate.sleep();  //loop until initialize is success
        current_state_ = TwistWithID::PATH_FOLLOWING;
        ROS_INFO("Starting joystick operation");
      }
      if (new_state == TwistWithID::JOYSTICK) ROS_INFO("Starting joystick operation");
      if (new_state == TwistWithID::PATH_FOLLOWING) ROS_INFO("Starting path following");
      if (new_state == TwistWithID::PARKING) ROS_INFO("Starting parking");
      if (new_state == TwistWithID::SHUT_DOWN) ROS_WARN("Shutting down");
    }
  }

  void Move(const bb_state::TwistWithID &twist) {    //translates velocity messages to motor control
    io->SendTwist(twist);
    //io_pub.publish(twist);
  }

  void StateCallback(const uint8_t &new_state) {  //callback for state messages
    SetState(new_state);
  }

  void MoveCallback(const bb_state::TwistWithID &data) {  //callback for move messages
    if(data.id == TwistWithID::JOYSTICK &&
       (data.twist.linear.x != 0 || joystick > 0) &&
       data.id != TwistWithID::INITIALIZE && 
       data.id != TwistWithID::EMERGENCY_STOP) {
      if(data.twist.linear.x != 0) {
        joystick = 10;
      } else {
        joystick--;
      }
      Move(data);
    } else {
      joystick--;
    }
    ROS_INFO("Joystick: %d", joystick);
    if (data.id == current_state_ && joystick < 0) {  //check if command is permitted
      Move(data);
    }

    else if (data.id == TwistWithID::EMERGENCY_STOP) ROS_WARN("Received unpermitted move command from Initialization");
    else if (data.id == TwistWithID::INITIALIZE) ROS_WARN("Received unpermitted move command from Initialization");
    else if (data.id == TwistWithID::JOYSTICK) ROS_WARN("Received unpermitted move command from Joystick");
    else if (data.id == TwistWithID::PATH_FOLLOWING) ROS_WARN("Received unpermitted move command from Path Following");
    else if (data.id == TwistWithID::PARKING) ROS_WARN("Received unpermitted move command from Parking");
    else if (data.id == TwistWithID::SHUT_DOWN) ROS_WARN("Received unpermitted move command from Shut Down");
  }
public:
  StateMachine() : spinner(4) {
    current_state_ = -1;
    SetState(TwistWithID::INITIALIZE);
    io = new IONode();
    // state_sub = n.subscribe("robot_state", 10, &StateMachine::StateCallback, this);
    move_sub = n.subscribe("move_io", 1, &StateMachine::MoveCallback, this);
    loop();
  }

  ~StateMachine() {
    // delete io;
    ros::shutdown();
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "state_machine");
  StateMachine *state_machine = new StateMachine();
  delete state_machine;
  return 0;
}
