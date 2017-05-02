/*
 * A node that produces creates string messages based on various
 * factors (events, velocity, etc.)
 * 
 * Created by Christian Onuogu
 * Edited by Jordan Taylor
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

ros::Publisher T2S_pub;
std_msgs::String msg;
geometry_msgs::Twist velocity;
bool moving = false;

/*
 * The cb function that turns received velocity into strings
 */
void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& vel)
{
        velocity = *vel;
	if(moving == false && velocity.linear.x > 0) {
		moving = true;
		msg.data = "Started movement";
		T2S_pub.publish(msg);
	}
	else if(moving == true && velocity.linear.x == 0) {
		moving = false;
		msg.data = "Stopped movement";
		T2S_pub.publish(msg);
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	
	T2S_pub = n.advertise<std_msgs::String>("T2S", 1000);
	ros::Subscriber sub = n.subscribe("/cmd_vel", 1, cmd_vel_cb );
	
	while(ros::ok()) {
		ros::spinOnce();
	}
	
	return 0;
}
