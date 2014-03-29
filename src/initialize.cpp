#include "EPOSControl.h"

class Initialization {
 public:
	Initialization() {
		to_epos_pub_ = n_.advertise<EposManager::EPOSControl>("/motors/BeachBot/Motor_Control", 10, true);
		ros::Rate poll_rate(100);
		while(to_epos_pub_.getNumSubscribers() == 0 && ros::ok()) poll_rate.sleep();	//wait until publisher is connected
	}

	~Initialization() {

	}

	bool Initialize() {
		bool initialization_success = false;
		initialization_success = HomeFrontWheel();
		return initialization_success;
	}	

 private:
 	ros::NodeHandle n_;
 	ros::Publisher to_epos_pub_;

 	bool HomeFrontWheel() {
		bool home_success = false;
		ROS_INFO("Homing front wheel...");
		//EposManager Instance:
		EposManager::EPOSControl Eposmsg;
		//Publish to frontwheel Epos:
		Eposmsg.node_id = 1;
		Eposmsg.control_mode = 6;
		Eposmsg.setpoint = 95;
		to_epos_pub_.publish(Eposmsg);	
		ros::Time start = ros::Time::now();
		while ((ros::Time::now()-start).sec < 10 && ros::ok()) {
			//wait for homing for 10 seconds
		}
		ROS_INFO("Homing success");
		home_success = true;
	  if (!home_success) ROS_WARN("Homing failed");
	  return home_success;
	}
};
