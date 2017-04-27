/*
 * A node that produces voice commands from string messages sent to it
 * Facilitates the communication between sound_play and a normal node
 * 
 * Created by Jordan Taylor
 */

#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <unistd.h>
#include <std_msgs/String.h>

sound_play::SoundClient *sc;

/*
 * The cb function that turns received strings into speech
 */
void voiceCb(const std_msgs::String::ConstPtr& msg) {
	(*sc).say(msg->data);
}

void initVoice() {
	sc = new sound_play::SoundClient();
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "voice");
	ros::NodeHandle nh;
	initVoice();
	ros::Subscriber sub = nh.subscribe("T2S", 5, voiceCb);

	
	ros::Rate r(10);
	while(ros::ok()) {
		ros::spinOnce();
		r.sleep();
	}
	
	return 0;
}
