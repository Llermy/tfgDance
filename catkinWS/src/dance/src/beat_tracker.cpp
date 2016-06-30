#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/String.h"

#include <stdio.h>
#include <unistd.h>
#include <string>
#include "osc_float_receiver.hpp"

#include <stdlib.h>

#include <signal.h>
#include <unistd.h>

#define OSC_READ_PORT 7001
#define MIN_PERIOD_uSEC 200000

#define MIN_ONSET_STRENGTH_THRESHOLD 1
#define MAX_ONSET_STRENGTH_THRESHOLD 6
#define ONSET_THRESHOLD_CHANGE_STEP 1
#define FEWER_BEATS_THRESHOLD 300

#define ONSETS_BUFFER_LENGTH 1200
#define MAX_PERIODS_TO_COMPARE 3

#define NO_MUSIC 0
#define SHUT_DOWN 2

ros::Publisher beats_pub;
ros::Subscriber period_sub;
ros::Subscriber status_sub;

int current_period = 107; // The units of current_period is the period in which we receive floats of the discrete onset strength signal
int period_count = 0;

int count = 0;
float current_threshold = MIN_ONSET_STRENGTH_THRESHOLD;
float current_onset_signal[ONSETS_BUFFER_LENGTH];

int fewer_beats_counter = 0;
int running = 0;
std::clock_t start;
int timer;

void send_other_signal(int sig)
{
	std_msgs::Float64 msg;
	msg.data = sig;
    beats_pub.publish(msg);
    ROS_INFO("SHUT DOWN!\n");
}

void statusCallback(const std_msgs::String::ConstPtr& msg)
{
    if(strcmp(msg->data.c_str(),"start") == 0)
    {
        ROS_INFO("%s! Running...\n", msg->data.c_str());
        running = 1;
    }
    else if (strcmp(msg->data.c_str(), "finish") == 0)
    {
        ROS_INFO("%s! Stopped.\n", msg->data.c_str());
        running = 0;
        send_other_signal(SHUT_DOWN);
    }
}

void periodCallback(const std_msgs::Int32::ConstPtr& msg)
{
    int new_period = msg->data;
    ROS_INFO("I heard period %d!", new_period);
	current_period = (int) new_period;
}

void checkIfWeakerBeats(float onset)
{
    if(onset < MIN_ONSET_STRENGTH_THRESHOLD)
    {
        fewer_beats_counter++;
        if(fewer_beats_counter > FEWER_BEATS_THRESHOLD &&
           current_threshold > MIN_ONSET_STRENGTH_THRESHOLD)
        {
            current_threshold -= ONSET_THRESHOLD_CHANGE_STEP;
            printf("Lower threshold: %f", current_threshold);
            fewer_beats_counter = 0;
        }
    } else {
        fewer_beats_counter = 0;
    }
}

bool isBeat(float onset)
{
    float prevPeriodsOnset = 0;
    float dispersedMean;
    int i;
    for(i = 1; i <= MAX_PERIODS_TO_COMPARE; i++)
    {
        dispersedMean = current_onset_signal[i*current_period] +
            current_onset_signal[i*current_period + 1] +
            current_onset_signal[i*current_period - 1];
        
        prevPeriodsOnset += dispersedMean/MAX_PERIODS_TO_COMPARE;
    }
    
    if(onset > current_threshold && prevPeriodsOnset > ((float)current_threshold*0.7))
    {
        printf("WOW!: %f; prev: %f; thres: %f", onset, prevPeriodsOnset, current_threshold);
        if(current_threshold < MAX_ONSET_STRENGTH_THRESHOLD)
        {
            current_threshold += ONSET_THRESHOLD_CHANGE_STEP;
        }
        return true;
    }
    return false;
}

void process_beat()
{
	std_msgs::Float64 msg;
	msg.data = 1;
    beats_pub.publish(msg);
    ROS_INFO("%d PEAK! %f\n", count, current_onset_signal[0]);
    count++;
}

void my_handler(int s)
{
    exit(0);
}

void catchCtrlC()
{
    struct sigaction sigIntHandler;
    
    sigIntHandler.sa_handler = my_handler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    
    sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char *argv[])
{
    running = 0;
    start = std::clock();
    
    ros::init(argc, argv, "beat_tracker");
	ros::NodeHandle n;
    beats_pub = n.advertise<std_msgs::Float64>("beats", 1000);
	period_sub = n.subscribe("music_period", 1000, periodCallback);
	status_sub = n.subscribe("dance_command", 1000, statusCallback);    
    
	catchCtrlC();	

    if(argc == 2)
    {
        //int bpm = std::stoi(argv[1]);
        //printf("Set period for testing: %d BPM", bpm);
        //current_period = 60/(bpm*ONSET_SIGNAL_PERIOD_SEC);
    }
    
    memset(current_onset_signal, 0, sizeof(current_onset_signal));
    OscFloatReceiver *osc_rec = new OscFloatReceiver(OSC_READ_PORT);
    
    while(1)
    {
        if(!running)
        {
            //send_other_signal(SHUT_DOWN);
        }
        else
        {
            memmove(current_onset_signal + 1, current_onset_signal, sizeof(current_onset_signal) - sizeof(float));
            current_onset_signal[0] = osc_rec->read_float();
            //printf("%f\n", current_onset_signal[0]);
            
            if(current_onset_signal[0] == -1000) // That means music is off
            {   
                current_onset_signal[0] = 0;
                period_count = 0;
                current_threshold = MIN_ONSET_STRENGTH_THRESHOLD;
                send_other_signal(NO_MUSIC);
            }
            else
            {
                checkIfWeakerBeats(current_onset_signal[0]);
                
                if (isBeat(current_onset_signal[0]))
                {
                    period_count = 0;
                    process_beat();
                    //usleep(MIN_PERIOD_uSEC);
                }
                else if (period_count > current_period)
                {
                    period_count = 0;
                    if(/*current_onset_signal[0] + 1 > 0.03*/1)
                    {
                        process_beat();
                    }
                }
                
                period_count++;
            }
        }
        ros::spinOnce();
    }
    return 0;
}
