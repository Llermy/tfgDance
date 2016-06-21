#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "etts_msgs/Utterance.h"

#include <string>
#include <stdlib.h>
#include <time.h>
#include <ctime>

#include "steps_file_reader.h"

#define ETTS_PRIMITIVE etts_msgs::Utterance::PRIM_LOQUENDO
#define ETTS_LANGUAGE "es"
#define NUM_OF_SENTENCES 4

#define NUM_DANCES 3
#define NUM_BODY_PARTS 5
#define LEFT_ARM 0
#define RIGHT_ARM 1
#define BASE 2
#define HEAD 3
#define NECK 4

class DanceStepsPublisher
{
public:
    // Moving frequency
    double move_rate;
    
    int thereIsMusic;
    std::clock_t start;
    int timer;
    
    // Subscriber to read the frequency
    ros::Subscriber freq_sub;
    
    // Publisher for each body part
    ros::Publisher pubLA;
    ros::Publisher pubRA;
    ros::Publisher pubBase;
    ros::Publisher pubHead;
    ros::Publisher pubNeck;
    ros::Publisher etts_say_text;
    
    int shut_down;
    etts_msgs::Utterance etts_msg;
    std::string sentences_to_dance[NUM_OF_SENTENCES];
    
    // Dance steps file reader
    StepsFileReader* reader;
    
    void random_timer()
    {
        timer = rand() % 10 + 15;
    }
    
    void musicCallback(const std_msgs::Float64::ConstPtr& msg)
    {
        float read_value = -1;
        read_value = msg->data;
        if (read_value == 1)
        {
            shut_down = 0;
            thereIsMusic = 1;
            publish_step(reader->get_next_step());
            ROS_INFO("I heard a peak!");
        }
        else if (read_value == 0)
        {
            shut_down = 0;
            change_dance();
            if (thereIsMusic)
            {
                start = std::clock();
                random_timer();
            }
            thereIsMusic = 0;
        }
        else if (read_value == 2)
        {
            shut_down = 1;
        }
    }
    
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
            case NECK:
                msg.data = number;
                pubNeck.publish(msg);
                ROS_INFO("Neck: %f", msg.data);
                break;
        }
    }
    
    void change_dance()
    {
        int new_dance = (rand() % NUM_DANCES) + 1;
        reader->set_dance(new_dance);
        ROS_INFO("Dance changed to %d\n", new_dance);
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
    
    void tell_want_to_dance()
    {
        int sentence_num = rand() % NUM_OF_SENTENCES;
        etts_msg.text = sentences_to_dance[sentence_num];
        etts_say_text.publish(etts_msg);
    }
    
    DanceStepsPublisher()
    {
        ros::NodeHandle n;
        freq_sub = n.subscribe("beats", 1000, &DanceStepsPublisher::musicCallback, this);
        pubLA = n.advertise<std_msgs::Float64>("/alz/leftArm/command", 1000);
        pubRA = n.advertise<std_msgs::Float64>("/alz/rightArm/command", 1000);
        pubBase = n.advertise<std_msgs::Float64>("/alz/base/command", 1000);
        pubHead = n.advertise<std_msgs::Float64>("/alz/head/command", 1000);
        pubNeck = n.advertise<std_msgs::Float64>("/alz/neck/command", 1000);
        etts_say_text = n.advertise<etts_msgs::Utterance>("etts", 100);
        
        //Default params for etts_msg
        etts_msg.primitive = ETTS_PRIMITIVE;
        etts_msg.language = ETTS_LANGUAGE;
        sentences_to_dance[0] = "Necesito música para bailar.";
        sentences_to_dance[1] = "Si no quieres que baile, dime que pare.";
        sentences_to_dance[2] = "Jo, no pones música.";
        sentences_to_dance[3] = "Ponme una canción que te guste.";
        
        shut_down = 1;
        thereIsMusic = 0;
        reader = new StepsFileReader();
        change_dance();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dance_steps_pub");
    
    /* initialize random seed: */
    srand (time(NULL));
    
    //std::string test_dance[] = {"0.2;0.2;0.5;0", "1;1;;", "0.8;0.8;;", "1;1;;", "0.2;0.2;-0.5;", "1;1;;", "0.8;0.8;;", "1;1;;", "0.2;0.2;0;", ";1;;", "1;0.2;;", "0.2;;;", ";1;;", "1;0.2;;", ";1;;", "0.2;0.2;;"};
    DanceStepsPublisher* steps_pub = new DanceStepsPublisher();
    
    while(1)
    {
        if(!steps_pub->shut_down)
        {
            if(!steps_pub->thereIsMusic)
            {
                double duration = ( std::clock() - steps_pub->start ) / (double) CLOCKS_PER_SEC;
                if (duration > steps_pub->timer)
                {
                    steps_pub->random_timer();
                    steps_pub->start = std::clock();
                    steps_pub->tell_want_to_dance();
                }
            }
        }
        ros::spinOnce();
    }
    return 0;
}
