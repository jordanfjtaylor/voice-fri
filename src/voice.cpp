#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <unistd.h>

void initializeVoice() {
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "voice");
	ros::NodeHandle nh;
	sound_play::SoundClient sc;
	
	while(ros::ok()) {
		sc.say("Voice has been initialized");
		ros::Duration(5).sleep();
	}
	return 0;
}
