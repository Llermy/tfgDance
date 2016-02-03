#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <string>

#include "steps_file_reader.h"

#define NUM_BODY_PARTS 4
#define LEFT_ARM 0
#define RIGHT_ARM 1
#define BASE 2
#define HEAD 3


class DanceStepsPublisher
{
public:
    // Moving frequency
    double move_rate;
    
    // Publisher for each body part
    ros::Publisher pubLA;
    ros::Publisher pubRA;
    ros::Publisher pubBase;
    ros::Publisher pubHead;
    
    void command_move(std::string move, int body_part)
    {
        if (move == "")
        {
            return;
        }
        std_msgs::Float64 msg;
        float number = boost::lexical_cast<float>(move);
        switch(body_part) {
            case LEFT_ARM:
                msg.data = number;
                pubLA.publish(msg);
                ROS_INFO("Left arm: %f", msg.data);
                break;
            case RIGHT_ARM:
                msg.data = number;
                pubRA.publish(msg);
                ROS_INFO("Right arm: %f", msg.data);
                break;
            case BASE:
                msg.data = number;
                pubBase.publish(msg);
                ROS_INFO("Base: %f", msg.data);
                break;
            case HEAD:
                msg.data = number;
                pubHead.publish(msg);
                ROS_INFO("Head: %f", msg.data);
                break;
        }
    }
    
    void publish_step(std::string step)
    {
        std::cout << step << "\n";
        int end = -1;
        int start = 0;
        int i;
        for(i = 0; i < NUM_BODY_PARTS; i++)
        {
            start = end + 1;
            end = step.find(";", start);
            std::string command = step.substr(start, end-start);
            command_move(command, i);
        }
    }
    
    DanceStepsPublisher()
    {
        ros::NodeHandle n;
        pubLA = n.advertise<std_msgs::Float64>("/alz/leftArm/command", 1000);
        pubRA = n.advertise<std_msgs::Float64>("/alz/rightArm/command", 1000);
        pubBase = n.advertise<std_msgs::Float64>("/alz/base/command", 1000);
        pubHead = n.advertise<std_msgs::Float64>("/alz/head/command", 1000);
        move_rate = 0.5;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dance_steps_publisher");

    DanceStepsPublisher* steps_pub = new DanceStepsPublisher();
    //std::string test_dance[] = {"0.2;0.2;0.5;0", "1;1;;", "0.8;0.8;;", "1;1;;", "0.2;0.2;-0.5;", "1;1;;", "0.8;0.8;;", "1;1;;", "0.2;0.2;0;", ";1;;", "1;0.2;;", "0.2;;;", ";1;;", "1;0.2;;", ";1;;", "0.2;0.2;;"};
    StepsFileReader* reader = new StepsFileReader();
    reader->set_dance(0);
    ros::Rate loop_rate(steps_pub->move_rate);

    while (ros::ok())
    {
        steps_pub->publish_step(reader->get_next_step());
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
