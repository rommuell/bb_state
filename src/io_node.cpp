#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "EPOSControl.h"
#include "bbcontrol/TwistWithID.h"
#include "bbcontrol/Rake.h"
#include "time.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include "rosserial_avr_tutorial/io_from_board.h"
#include "rosserial_avr_tutorial/io_to_board.h"

// Threading for async IO
// #include <thread>


class io_node
{
 private:
  static constexpr double beachbot_width_ = 0.264;
  static constexpr double lengthcenterofrotation_frontwheel_= 0.31;
  double abs_vel_input_;
  double ang_vel_input_;
  double vel_right_wheel_;
  double vel_left_wheel_;
  double hexvel_left_wheel_;
  double hexvel_right_wheel_;
  double vel_right_wheel_read_by_escon_;
  double vel_left_wheel_read_by_escon_;
  double abs_vel_read_by_escon_;
  double ang_vel_read_by_escon_;
  double delta_pose_x_read_by_escon_;
  double delta_pose_y_read_by_escon_;
  double delta_pose_theta_read_by_escon_;
  double pose_x_read_by_escon_;
  double pose_y_read_by_escon_;
  double pose_theta_read_by_escon_;
  int counter;
  double old_pose_theta_read_by_escon_;
  double pose_theta_read_by_escon_leapfrog_;
  int var_vel_left;
  int var_vel_right;
  double number_of_volts_left;
  double number_of_volts_right;
  double time_increment;
  double phi_front_wheel_;
  int angleinquants;
  int oldangleinquants;
  int sign_of_velocity_for_escon_reader_left_wheel_;
  int sign_of_velocity_for_escon_reader_right_wheel_;
  ros::NodeHandle n;
  ros::Publisher to_epos_pub_;
  ros::Subscriber rake_io_sub;
  ros::Subscriber twist_from_laser_sub;
  ros::Publisher to_laser_pub_;
  ros::Publisher to_escon_pub;
  ros::Subscriber from_escon_sub_;
  FILE * deviceFilePointer;
  FILE * deviceFilePointer2;
  rosserial_avr_tutorial::io_to_board io_to_board_message_;
	
  int roundsturnedfrontwheelmotor;
  std::vector<int> closest_equivalent;
  ros::Time current_time, last_time;


  // Output Variables:
  uint64_t Output_Message;
  uint64_t Output_Status;

  // Input Variables:
  uint64_t Input_Message;

public:
  void twist_io_Callback(const geometry_msgs::Twist velocitymessage) {
		//ROS_INFO("Twist Callback");
			
    abs_vel_input_ = velocitymessage.linear.x;
    ang_vel_input_ = velocitymessage.angular.z;  

	//Determine sign of abs_vel_input for escon reader:

    //from here: stuff that was in int main()
    int roundsturnedfrontwheelmotor;
    std::vector<int> closest_equivalent;
    


    //Calculation of Wheel velocities:
    vel_left_wheel_ = (abs_vel_input_ + ang_vel_input_ * this->beachbot_width_);
    vel_right_wheel_ = (abs_vel_input_ - ang_vel_input_ * this->beachbot_width_);
    //

     if(vel_left_wheel_ < 0)
     {
        sign_of_velocity_for_escon_reader_left_wheel_ = -1;
     }
  	
     else
     {
        sign_of_velocity_for_escon_reader_right_wheel_ = 1;
     }

     if(vel_right_wheel_ < 0)
     {
        sign_of_velocity_for_escon_reader_right_wheel_ = -1;
     }
  	
     else
     {
        sign_of_velocity_for_escon_reader_right_wheel_ = 1;
     }

    //Conversion to numbers that the escon takes:
    hexvel_left_wheel_ = (32512 * vel_left_wheel_ + 32768);
    hexvel_right_wheel_ = (32512 * vel_right_wheel_ + 32768);
    //

    //Calculation of Frontwheel Steeringangle:
    if(abs_vel_input_ < 0.00001 && abs_vel_input_ > -0.00001) {
      if(ang_vel_input_ > 0.00002 || ang_vel_input_ < -0.00002) {
        if(ang_vel_input_ < 0) {
          phi_front_wheel_ = -3.14159/2;
          }
        else {
          phi_front_wheel_ = 3.14159/2;
        }
      }
      else if(angleinquants == 0) {
        phi_front_wheel_ = 0;
      }
    }
    else {
        //ROS_INFO("%d", fread(&Input_Message , 1, 7, deviceFilePointer));
      phi_front_wheel_ = (-1) * atan2( (lengthcenterofrotation_frontwheel_ * ang_vel_input_),abs_vel_input_);
    }
    //
        
        

        
    //Conversion of Frontwheel angle into numbers corresponding to the quants of front wheel motor:
    angleinquants = floor(phi_front_wheel_/3.14159 * 1464);
    //
       
    //Ensuring that the front wheel does sensible movements:
    roundsturnedfrontwheelmotor = floor(oldangleinquants/1464);

    angleinquants = roundsturnedfrontwheelmotor * 1464 + angleinquants;
    //ROS_INFO("anglequants %d", angleinquants);

    closest_equivalent.clear();
    closest_equivalent.push_back(angleinquants);
    closest_equivalent.push_back(angleinquants + 1464);
    closest_equivalent.push_back(angleinquants + 2928);
    closest_equivalent.push_back(angleinquants - 1464);
    closest_equivalent.push_back(angleinquants - 2928);

    int temporary = 300000;

    for(int i = 0; i < 5; i++) {
      if (abs(oldangleinquants - closest_equivalent.at(i)) < temporary){
          temporary = abs(oldangleinquants - closest_equivalent.at(i));
          angleinquants = closest_equivalent.at(i);
      }

    }
    //
    
    //Visualization:
    //ROS_INFO("angleinquants: %d",angleinquants);
    //

    //Declaration of Publisher for Epos Front Wheel:

    //EposManager Instance:
    EposManager::EPOSControl Eposmsg;
        
    //Publish to frontwheel Epos:
    Eposmsg.node_id = 1;
    Eposmsg.control_mode = 2;
    Eposmsg.setpoint = angleinquants;
    to_epos_pub_.publish(Eposmsg);
    //
        
    //Assign oldangleinquants:
    oldangleinquants = angleinquants;
    //
    ////////////////////////////////////////
    this->WriteToFile();
  }

  void WriteToFile() {


    io_to_board_message_.motor_left = hexvel_left_wheel_;
    io_to_board_message_.motor_right = hexvel_right_wheel_;
    io_to_board_message_.status = 1;

//adjust status!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    to_escon_pub.publish(io_to_board_message_);

//  	//ROS_INFO("Writing to file...");
//		//Generate Hexadecimals to write into /dev/beachbot.ioboard (for escons):

//      ROS_INFO("vel: %f",hexvel_left_wheel_);
//        Output_Status = 0x01;
//        Output_Message = 0 | (Output_Status<<32) | (((uint64_t)hexvel_left_wheel_)<<16) | (((uint64_t)hexvel_right_wheel_));

//		//Write into file /dev/beachbot.ioboard (for escons):
//        Output_Message = __builtin_bswap64(Output_Message);
//        fwrite (&Output_Message , 8, 1, deviceFilePointer);
//        fflush(deviceFilePointer);
  }

//  int ReadFromFile() {
//    FILE *fp = fopen("/dev/beachbot.ioboard", "r");
//    ROS_INFO("Opening File\n");
//    if(fp == NULL) {
//      ROS_ERROR("Failed to open USB for reading!");
//      return EXIT_FAILURE;
//    }

//     /*int read = fread(&Input_Message, 1, 7, deviceFilePointer);
//     ROS_INFO("Reading bytes: %d", read);
//     int read2 = fread(&Input_Message, 1, 7, deviceFilePointer);
//     ROS_INFO("Reading bytes: %d", read2);*/
////     while(fread(&Input_Message, 1, 8, deviceFilePointer2)==8  && ros::ok())
////     {
////
////        ROS_INFO("written");
////
////     }

//    ros::Rate read_rate(100);

//    while(ros::ok()) {
//      ROS_INFO("Reading File\n");
//      //ros::spinOnce();
//      while(!feof(fp)) {
//        unsigned char buffer[8] = {0};
//        int i;
//        ROS_INFO("\nMSG: ");
//        for(i = 0; i < 4; i++) {
//          int rc = getc(fp);
//          if(rc == EOF) {
//            ROS_ERROR("Error while reading file: Terminated too early");
//            continue;
//            // return EXIT_FAILURE;
//          }
//          buffer[i] = rc;
//          ROS_INFO("Ausgegeben! %x", buffer[i]);
//        }
//        // char message_malformed = 0;
//        // for(i = 0; i < 2; i++) {
//        //   if(buffer[i] != 0xff) {
//        //     message_malformed = 1;
//        //     // Handle error!
//        //   }
//        // }
//        // if(message_malformed) {
//        //   ROS_ERROR("Message malformed, rewinding head.");
//        //   if(!fseek(fp, -7, SEEK_CUR)){
//        //     ROS_ERROR("Rewinding head raised error.");
//        //     return EXIT_FAILURE;
//        //   }
//        //   continue;
//        // }
//        long int result = -1;
//        // result = buffer[4] * 2 + buffer[5] * 255 + buffer[6];
//        ROS_INFO("RESULT = %ld", result);
//        switch(buffer[2] | 0xf0) {
//          case 0x10:
//            // result = strtol(&buffer[5], NULL, 16);
//            ROS_INFO("ODOM LEFT");
//            // ROS_INFO("ODOM LEFT: %ld\n", result);
//            // send left wheel
//            break;
//          case 0x20:
//            // result = strtol(&buffer[5], NULL, 16);
//            ROS_INFO("ODOM RIGHT");
//            // ROS_INFO("ODOM RIGHT: %ld\n", result  );
//            // send left wheel
//            break;
//          case 0x30:
//            // result = strtol(&buffer[5], NULL, 16);
//            ROS_INFO("NICHT CASE");
//            // ROS_INFO("ODOM RIGHT: %ld\n", result  );
//            // send left wheel
//            break;

//          case 0xa0:
//            ROS_WARN("NOTAUS\n");
//            // send emergency stop
//            break;
//          case 0xf0:
//            // Pointer is wrong for one
//            if(!fseek(fp, -7, SEEK_CUR)){
//              ROS_ERROR("Rewinding head (case mismatch) raised error.");
//              return EXIT_FAILURE;
//            }
//            break;
//          default:
//            ROS_ERROR("Case not handled." );
//            break;
//        }
//      }
//       read_rate.sleep();
//    }


  void from_escon_Callback(const rosserial_avr_tutorial::io_from_board from_escon_message)
  {
    if(from_escon_message.status == 2){
      var_vel_left = from_escon_message.velocity;
      var_vel_right = 0;
    }

    else{
      var_vel_right = from_escon_message.velocity;
      var_vel_left  = 0;
    }

ROS_INFO("numberofvoltsleft: %d, numberofvoltsright: %d", var_vel_left, var_vel_right);

     //Calculation:
    number_of_volts_left = (double)var_vel_left / 1023 * 5;
    number_of_volts_right = (double)var_vel_right / 1023 * 5;

    ROS_INFO("numberofvoltsleft: %f, numberofvoltsright: %f", number_of_volts_left, number_of_volts_right);
    ROS_INFO("numberofvoltsleft: %f, numberofvoltsright: %f", __builtin_bswap32(number_of_volts_left), __builtin_bswap32(number_of_volts_right));

    vel_right_wheel_read_by_escon_ = ((-4000 + (number_of_volts_right/4) * 8000) / (60 * 47) * 0.725);
    vel_left_wheel_read_by_escon_ = ((-4000 + (number_of_volts_left/4) * 8000) / (60 * 47) * 0.725);

    ROS_INFO("int_vel_right: %f, int_vel_left: %f", vel_right_wheel_read_by_escon_, vel_left_wheel_read_by_escon_);

    abs_vel_read_by_escon_ = (vel_right_wheel_read_by_escon_ + vel_left_wheel_read_by_escon_)/2;
    ang_vel_read_by_escon_ = (vel_left_wheel_read_by_escon_ - vel_right_wheel_read_by_escon_)/beachbot_width_;



    current_time = ros::Time::now();

    //Variables do still have to be declared in Class!!!
    time_increment = (current_time-last_time).toSec();

    //Integration:
    delta_pose_theta_read_by_escon_ = ang_vel_read_by_escon_ * time_increment;
    delta_pose_x_read_by_escon_ = abs_vel_read_by_escon_ * cos(pose_theta_read_by_escon_) * time_increment;
    delta_pose_y_read_by_escon_ = abs_vel_read_by_escon_ * sin(pose_theta_read_by_escon_) * time_increment;

    //Update:
    pose_x_read_by_escon_ += delta_pose_x_read_by_escon_;
    pose_y_read_by_escon_ += delta_pose_y_read_by_escon_;

    last_time = current_time;
    pose_theta_read_by_escon_ += delta_pose_theta_read_by_escon_;

    pose_theta_read_by_escon_leapfrog_ =  (old_pose_theta_read_by_escon_ + pose_theta_read_by_escon_)/2;


    old_pose_theta_read_by_escon_ = pose_theta_read_by_escon_;

    //If loop to synchronize with the laser:
    counter += 1;
    if(counter > 10){   // Bedingung noch anzupassen !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
       nav_msgs::Odometry odometry_from_escon_to_laser_;
       //since all odometry is 6DOF we'll need a quaternion created from yaw

       geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_theta_read_by_escon_leapfrog_);

       odometry_from_escon_to_laser_.pose.pose.position.x = pose_x_read_by_escon_;
       odometry_from_escon_to_laser_.pose.pose.position.y = pose_y_read_by_escon_;
       odometry_from_escon_to_laser_.pose.pose.position.z = 0.0;
       odometry_from_escon_to_laser_.pose.pose.orientation = odom_quat;

       //set the velocity
       odometry_from_escon_to_laser_.child_frame_id = "base_link";
       odometry_from_escon_to_laser_.twist.twist.linear.x = abs_vel_read_by_escon_ * cos(pose_theta_read_by_escon_leapfrog_);
       odometry_from_escon_to_laser_.twist.twist.linear.y = abs_vel_read_by_escon_ * sin(pose_theta_read_by_escon_leapfrog_);
       odometry_from_escon_to_laser_.twist.twist.angular.z = ang_vel_read_by_escon_;

       to_laser_pub_.publish(odometry_from_escon_to_laser_);
       pose_x_read_by_escon_ = 0;
       pose_y_read_by_escon_ = 0;
       pose_theta_read_by_escon_ = 0;
       pose_theta_read_by_escon_ = 0;
       old_pose_theta_read_by_escon_ = 0;

       counter = 0;
     }
  }

  void rake_io_Callback(const bbcontrol::Rake rake)
  {
    //$$$ ToDo D: ? = rake.rakepin1;
  }

  
  io_node() {
    rake_io_sub = n.subscribe("rake_io", 1000, &io_node::rake_io_Callback, this);
    twist_from_laser_sub = n.subscribe("move_io", 1000, &io_node::twist_io_Callback, this);
    to_escon_pub = n.advertise<rosserial_avr_tutorial::io_to_board>("to_ioboard", 10);
    from_escon_sub_ = n.subscribe("io_from_board", 1000, &io_node::from_escon_Callback, this);
    to_epos_pub_ = n.advertise<EposManager::EPOSControl>("/motors/BeachBot/Motor_Control", 10);
    to_laser_pub_ = n.advertise<nav_msgs::Odometry>("bot_pose", 10);////////////////////////!!!!!!!!!!!!!!!!!anschauen!!!
    //deviceFilePointer = fopen ("/dev/beachbot.ioboard", "w");
    //deviceFilePointer2 = fopen ("/dev/ttyACM0", "wb");
    //setvbuf(deviceFilePointer2,NULL,_IONBF,8);
    angleinquants = 0;
    oldangleinquants = 0;
    abs_vel_input_ = 0;
    ang_vel_input_ = 0;
    sign_of_velocity_for_escon_reader_right_wheel_ = 0;
    sign_of_velocity_for_escon_reader_right_wheel_ = 0;
    //Timer for Odometry Integration:
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    //
    counter = 0;
    geometry_msgs::Twist stop_twist;
    stop_twist.linear.x = 0;
    stop_twist.angular.z = 0;
    twist_io_Callback(stop_twist);
  }

  ~io_node() {
    geometry_msgs::Twist stop_twist;
    stop_twist.linear.x = 0;
    stop_twist.angular.z = 0;
    twist_io_Callback(stop_twist);
    
    //Shut down escon motors
    io_to_board_message_.motor_left = 0;
    io_to_board_message_.motor_right = 0;
    io_to_board_message_.status = 0;
    to_escon_pub.publish(io_to_board_message_);
    
    //rake_io_Callback(...)
    //  $$$ ToDo D: Motoren abschalten
    //  $$$ ToDo D: Stop_Rake_msg und entsprechenden Callback aufrufen
    //WriteToFile();
    
    //fclose (deviceFilePointer);
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "io_node");
  io_node *io = new io_node();
  ros::Rate loop_rate(25);
  ROS_INFO("11");


//  std::thread read_thread (&io_node::ReadFromFile, io);

  while (ros::ok()) {
    ros::spinOnce();
     // io->WriteToFile();
     ROS_INFO("12");
    loop_rate.sleep();
    // ROS_INFO("13");
  }
	delete io;
  return 0;
}
