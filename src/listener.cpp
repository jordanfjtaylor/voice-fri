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
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
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


void say(char* words) {
	msg.data = words;
	T2S_pub.publish(msg);
	ros::Duration(2).sleep();
}
/*
 * The cb function that turns received velocity into strings
 */
void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& vel)
{
        velocity = *vel;
	if(moving == false && velocity.linear.x > 0) {
		moving = true;
		say("Started movement");
	}
	else if(moving == true && velocity.linear.x == 0) {
		moving = false;
		say("Stopped movement");
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
			heardGoal = false;
			say("Destination reached");
		}
	}
}

/*
 *  The cb function that receives the robot's point cloud
 *  and checks for delocalization
 */
void cloud_cb(const geometry_msgs::PoseArray::ConstPtr& pose_object) {
	int size = 2000;
	double threshold = 5.0;
	double x = 0;
	double y = 0;
	double sum_x = 0;
	double sum_x2 = 0;
	
	double sum_y = 0;
	double sum_y2 = 0;
	
	for(int i = 0; i < size; i++) {
		geometry_msgs::Point p = pose_object->poses[i].position;
		x = p.x;
		y = p.y;
		
		sum_x += x;
		sum_x2 += std::pow(x, 2.0);
		
		sum_y += y;
		sum_y2 += std::pow(y, 2.0);
	}
	double mean_x = sum_x/size;
	double mean_y = sum_y/size;
	
	double stddev_x = std::pow((sum_x2 - (2 * mean_x * sum_x) + (size*std::pow(mean_x, 2)))/(size-1), .5);
	double stddev_y = std::pow((sum_y2 - (2 * mean_y * sum_y) + (size*std::pow(mean_y, 2)))/(size-1), .5);

	ROS_INFO("Size: %d", size);
	ROS_INFO("Standard deviation of x: %f", stddev_x);
	ROS_INFO("Standard deviation of y: %f", stddev_y);
	
	if(stddev_x > threshold || stddev_y > threshold) {
		say("I'm delocalized!");
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	
	T2S_pub = n.advertise<std_msgs::String>("T2S", 10);
	ros::Subscriber vel_sub = n.subscribe("/cmd_vel", 1, cmd_vel_cb );
	ros::Subscriber goal_sub = n.subscribe("/move_base/goal", 1, goal_cb );
	ros::Subscriber odom_sub = n.subscribe("/odom", 1, pos_cb );
	ros::Subscriber cloud_sub = n.subscribe("/particlecloud", 1, cloud_cb);
	
	while(ros::ok()) {
		ros::spinOnce();
	}
	
	return 0;
}
