/*
 * A node that produces creates string messages based on various
 * factors (events, velocity, etc.)
 * 
 * Created by Christian Onuogu
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include <math.h>

ros::Publisher T2S_pub;
std_msgs::String msg;
geometry_msgs::Twist velocity;
bool moving = false;
bool heardGoal = false;
double xPos;
double yPos;
double goalX;
double goalY;

/*
 * The cb function that turns received velocity into strings
 */
void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& vel)
{
        velocity = *vel;
	if(moving == false && velocity.linear.x > 0) {
		moving = true;
		ROS_INFO("Hello?");
		msg.data = "Started movement";
		T2S_pub.publish(msg);
	}
	else if(moving == true && velocity.linear.x == 0) {
		moving = false;
		ROS_INFO("Bye?");
		msg.data = "Stopped movement";
		T2S_pub.publish(msg);
	}
}

/*
 * The cb function that gets the position of the robot's goal
 */
void goal_cb(const move_base_msgs::MoveBaseActionGoal::ConstPtr& gl)
{
	move_base_msgs::MoveBaseActionGoal goal = *gl;
	goalX = goal.goal.target_pose.pose.position.x;
	goalY = goal.goal.target_pose.pose.position.y;
	heardGoal = true;
}

/*
 * The cb function that gets the position of the robot and
 * checks if it has reached its goal
 */
void pos_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
	nav_msgs::Odometry odometer = *odom;
	xPos = odometer.pose.pose.position.x;
	yPos = odometer.pose.pose.position.y;
	if(heardGoal){
		if(fabs(xPos-goalX) < .5 && fabs(yPos-goalY) < .5) {
			ROS_INFO("Please");
			heardGoal = false;
			msg.data = "Destination reached";
			T2S_pub.publish(msg);
		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	
	T2S_pub = n.advertise<std_msgs::String>("T2S", 1000);
	ros::Subscriber sub = n.subscribe("/cmd_vel", 1, cmd_vel_cb );
	ros::Subscriber sub2 = n.subscribe("/move_base/goal", 1, goal_cb );
	ros::Subscriber sub3 = n.subscribe("/odom", 1, pos_cb );
	
	while(ros::ok()) {
		ros::spinOnce();
	}
	
	return 0;
}
