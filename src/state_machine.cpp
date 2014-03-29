#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8.h"
#include "geometry_msgs/Twist.h"
#include "bbcontrol/TwistWithID.h"
#include "initialize.cpp"

static const int EMERGENCY_STOP = 0;
static const int INITIALIZE = 1;
static const int JOYSTICK = 2;
static const int PATH_FOLLOWING = 3;
static const int PARKING = 4;
static const int SHUT_DOWN = 5;

class StateMachine {
 public:
 	StateMachine() {
    current_state_ = -1;
		SetState(INITIALIZE);
 		state_sub = n.subscribe("robot_state",1000, &StateMachine::StateCallback, this);
 		move_sub = n.subscribe("robot_move", 1000, &StateMachine::MoveCallback, this);
		io_pub = n.advertise<geometry_msgs::Twist>("move_io",1000);
 		loop();
 	}

	~StateMachine() {
		ros::shutdown();
	}

 private:	
	ros::NodeHandle n;
	ros::Subscriber state_sub;
	ros::Subscriber move_sub;
	ros::Publisher io_pub;
	int current_state_;

	void loop() {	//main loop
		ros::Rate loop_rate(25);
		while (ros::ok() && current_state_ != SHUT_DOWN) {
			ros::spinOnce();
			if (current_state_ == INITIALIZE) {	//do stuff for initialization
				ros::Rate init_rate(1);		//Only try once a second
				Initialization *init = new Initialization();
				while(ros::ok() && !(init->Initialize())) init_rate.sleep();	//loop until initialize is success
				current_state_ = JOYSTICK;	//set to joystick if success
				ROS_INFO("Starting joystick operation");
			}
			loop_rate.sleep();
		}
	};

	void SetState(const int &new_state) {
		if(new_state != current_state_) {
			current_state_ = new_state;
			if (new_state == EMERGENCY_STOP) ROS_ERROR("EMERGENCY STOP");
			if (new_state == INITIALIZE) ROS_INFO("Starting initialization");
			if (new_state == JOYSTICK) ROS_INFO("Starting joystick operation");
			if (new_state == PATH_FOLLOWING) ROS_INFO("Starting path following");
			if (new_state == PARKING) ROS_INFO("Starting parking");
			if (new_state == SHUT_DOWN) ROS_WARN("Shutting down");
		}
	}

	void Move(const geometry_msgs::Twist &twist) {		//translates velocity messages to motor control
		io_pub.publish(twist);
	}

	void StateCallback(const std_msgs::UInt8 &new_state) {	//callback for state messages
		SetState(new_state.data);
	}

	void MoveCallback(const bbcontrol::TwistWithID &data) {	//callback for move messages
		if (data.id.data == current_state_) {	//check if command is permitted
			Move(data.twist);
		}
		else if (data.id.data == EMERGENCY_STOP) ROS_WARN("Received unpermitted move command from Initialization");
		else if (data.id.data == INITIALIZE) ROS_WARN("Received unpermitted move command from Initialization");
		else if (data.id.data == JOYSTICK) ROS_WARN("Received unpermitted move command from Joystick");
		else if (data.id.data == PATH_FOLLOWING) ROS_WARN("Received unpermitted move command from Path Following");
		else if (data.id.data == PARKING) ROS_WARN("Received unpermitted move command from Parking");
		else if (data.id.data == SHUT_DOWN) ROS_WARN("Received unpermitted move command from Shut Down");
	}
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "state_machine");
	StateMachine *state_machine = new StateMachine();
	delete state_machine;

	return 0;
}
