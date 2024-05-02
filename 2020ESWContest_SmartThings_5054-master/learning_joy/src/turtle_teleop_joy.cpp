// joy teleop turtlesim example 2015-02-08 LLW
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <unistd.h>

// publisher for a geometry_msgs::Twist topic
ros::Publisher  turtle_vel;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	float vel_stra = 3.0;
	float vel_trun = 2.5;
	
  // declare local variable
  geometry_msgs::Twist command_velocity;

  // X vel driven by left joystick forward and aft
  command_velocity.linear.x  = vel_stra*joy->axes[1];
 
 // heading driven by left joystick left and right
  command_velocity.angular.z  = vel_stra*joy->axes[0];

  // Z vel driven by Y and A buttons
  // button Y
  command_velocity.linear.z = vel_trun*joy->buttons[3];
  // button A
  command_velocity.linear.z = -vel_trun*joy->buttons[0];
  // rotation vel about X driven by X and B buttons
    //button X
  command_velocity.angular.x = vel_trun*joy->buttons[2];
    //button B
  command_velocity.angular.x = -vel_trun*joy->buttons[1];
  // rotation vel about Y driven by right joystick forward and aft
  command_velocity.angular.y = vel_trun*joy->axes[4];
  // rotation vel about Z driven by right joystick left and right
  command_velocity.angular.z = vel_trun*joy->axes[3];
  // publish the cmd vel
  turtle_vel.publish(command_velocity);

}


int main(int argc, char** argv)
{

  // init ros
  ros::init(argc, argv, "turtle_teleop_joy");

  // create node handle
  ros::NodeHandle node;

  // advertise topic that this node will publish
  turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 10);

  // subcscribe to joy topic
  ros::Subscriber sub = node.subscribe("joy", 10, &joyCallback);

  // spin
  ros::spin();

  return 0;

};


