#include "ros/ros.h"

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "onset_strength_node");
	
	pid_t pid = fork();
	if (pid == 0)
	{
		// child process
		system("/home/user/catkin_ws/src/dance/third_parties/chuck-1.3.5.2/src/chuck /home/user/catkin_ws/src/dance/src/onsetStrength.ck");
	}
	else if (pid > 0)
	{
		// parent process
		ros::spin();
	}
	return 0;
}
