#include "beat_pub.hpp"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <stdlib.h>

#define MIN_CHANGE_FREQUENCY_PERIOD 5
#define CHANGE_FREQUENCY_PROBABILITY 75
#define MIN_FREQUENCY 0.1
#define MAX_FREQUENCY 0.6

int main(int argc, char **argv)
{
    ros::init(argc, argv, "beat_publisher");
    ros::NodeHandle n;
    
    ros::Publisher freq_pub = n.advertise<std_msgs::Float64>("beats", 1000);
    ros::Rate loop_rate(0.4);
    
    srand(time(NULL));
    while (ros::ok())
    {
        int random = rand()%100;
        ROS_INFO("Random: %f", random);
        if (random < 50) {
            std_msgs::Float64 msg;
            msg.data = random;
            
            ROS_INFO("Sent beat: %f", msg.data);
            freq_pub.publish(msg);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
