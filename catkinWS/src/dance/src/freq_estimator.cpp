#include "freq_estimator.hpp"
#include "ros/ros.h"
#include <stdlib.h>

#define MIN_CHANGE_FREQUENCY_PERIOD 5
#define CHANGE_FREQUENCY_PROBABILITY 75
#define MIN_FREQUENCY 0.1
#define MAX_FREQUENCY 0.6

int main(int argc, char **argv)
{
    ros::init(argc, argv, "freq_estimator");
    ros::NodeHandle n;
    
    ros::Publisher freq_pub = n.advertise<std_msgs::Float64>("musicFrequency", 1000);
    ros::Rate loop_rate(1/MIN_CHANGE_FREQUENCY_PERIOD);
    
    srand(time(NULL));
    while (ros::ok())
    {
        if (rand()%100 < 75) {
            std_msgs::Float64 msg;
            float num = (float) (rand()%6 + 1);
            num = num % 10;
            msg.data = num;
            
            ROS_INFO("Sent frequency: %f", msg.data);
            freq_pub.publish(msg);
            
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    return 0;
}