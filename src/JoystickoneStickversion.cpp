#include "sensor_msgs/Joy.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "bbcontrol/TwistWithID.h"


class JoystickoneStickversion
{
  public:
    double absolutespeed;
    double angularspeed;
    double maxabsvelocity;
    double maxangvelocity;



    void joystickCallback(const sensor_msgs::Joy &joymsg)
    {
      absolutespeed = joymsg.axes[4];
      angularspeed = joymsg.axes[3];
      //ROS_INFO("angularspeed: %f, absolute: %f", joymsg.axes[3], joymsg.axes[4]);
    }

};



int main(int argc, char **argv){

        JoystickoneStickversion joyinstance;
    joyinstance.maxabsvelocity = 0.4;
    joyinstance.maxangvelocity = 1;
        ros::init(argc, argv, "JoystickoneStickversion");
        ros::NodeHandle n;
        ros::Subscriber joysticksub = n.subscribe("joy", 1000, &JoystickoneStickversion::joystickCallback, &joyinstance);
        ros::Publisher Twistfromonestick_pub = n.advertise<bbcontrol::TwistWithID>("robot_move", 1000);
        ros::Rate loop_rate(25);
	geometry_msgs::Twist velocitymsg;
        while (ros::ok())
        {
	    
	    velocitymsg.linear.x = joyinstance.maxabsvelocity*joyinstance.absolutespeed;
	    velocitymsg.angular.z = joyinstance.maxangvelocity*joyinstance.angularspeed;
	    
            //ROS_INFO("angularspeed: %f, absolute: %f", joyinstance.angularspeed, joyinstance.absolutespeed);
            
			bbcontrol::TwistWithID twist_publish;
			twist_publish.twist = velocitymsg;
			twist_publish.id.data = 2;	//ID for joystick state is 2
	    Twistfromonestick_pub.publish(twist_publish);
	    




	ros::spinOnce();
	loop_rate.sleep();

        }
}
