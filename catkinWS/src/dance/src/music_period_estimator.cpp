#include "ros/ros.h"
#include "std_msgs/Int32.h"

#include <string.h>
#include <stdio.h>
#include <cmath>
#include <stdlib.h>
#include "osc_float_receiver.hpp"

#include <signal.h>
#include <unistd.h>

#define PERIOD_OSC_PORT 7000

#define MAX_BPM 240
#define MIN_BPM 40
#define MAX_PERIOD ((int) (60/(MIN_BPM*ONSET_SIGNAL_PERIOD_SEC)) + 1)
#define MIN_PERIOD ((int) (60/(MAX_BPM*ONSET_SIGNAL_PERIOD_SEC)) - 1)
#define MAX_SIGNAL_BUFFER_LENGTH 1200 // corresponds roughly with 7-8 seconds
#define HIST_MAX_NUMBER 500
#define PERIOD_RELATIONS_PRECISION 0.1

#define PREPARE_COUNTER_THRESHOLD (MAX_PERIOD + 40)
#define STABILISATION_COUNTER_THRESHOLD 80

#define AUTOCORR_CYCLES 4
#define HIST_NUMBER_ADDITIONS 3
#define CENTER_BPM 80
#define CENTER_PERIOD ((int) (60/(CENTER_BPM*ONSET_SIGNAL_PERIOD_SEC)))
#define GLOBAL_PERIOD_THRESHOLD 25

// ROS period publisher
ros::Publisher period_pub;

// Autocorrelation-limiting variables
int max_period;
int min_period;

// Autocorrelation buffer
float current_autocorr[MAX_PERIOD - MIN_PERIOD + 1];
float current_signal[MAX_SIGNAL_BUFFER_LENGTH];
int period_hist[MAX_PERIOD - MIN_PERIOD + 1]; // histogram of all previous found periods
int test_hist[MAX_PERIOD - MIN_PERIOD + 1]; // for error rate tests

int current_instant_periods[HIST_NUMBER_ADDITIONS];
int current_global_period = -1;

int prepare_counter = 0;
int stabilisation_counter = 0;

float autocorrAt(int tao)
{
    float sum = 0;
    int i;
    for(i = 0; i < MAX_SIGNAL_BUFFER_LENGTH - tao; i++)
    {
        sum += current_signal[i + tao]*current_signal[i];
    }
    return sum;
}

// Calculate 4-cycled autocorrelation and find maximum values
void autocorr()
{
    float max = -10;
    
    int i;
    for(i = MIN_PERIOD; i <= MAX_PERIOD; i++)
    {
        current_autocorr[i - MIN_PERIOD] = 0;
        
        int j;
        for(j = 1; j <= AUTOCORR_CYCLES; j++)
        {
            current_autocorr[i - MIN_PERIOD] += autocorrAt(i*j);
            if(i*(j+1) > MAX_SIGNAL_BUFFER_LENGTH)
            {
                break;
            }
        }
        current_autocorr[i - MIN_PERIOD] = current_autocorr[i - MIN_PERIOD]/j;
        
        if(current_autocorr[i - MIN_PERIOD] > max) {
            max = current_autocorr[i - MIN_PERIOD];
            memmove(current_instant_periods + 1, current_instant_periods, sizeof(current_instant_periods) - sizeof(int));
            current_instant_periods[0] = i;
        }
        
        /* DEBUG
        if(current_autocorr[i - MIN_PERIOD] != 0)
        {
            printf("Autocorr: %d, %f\n", i, current_autocorr[i - MIN_PERIOD]);
        }*/
    }
}

void updateHist()
{
    bool peakIsSurpassed = false;
    int i;
    for(i = 0; i < HIST_NUMBER_ADDITIONS; i++)
    {
        if(++period_hist[current_instant_periods[i] - MIN_PERIOD] > HIST_MAX_NUMBER)
        {
            peakIsSurpassed = true;
        }
        // DEBUG: printf("Hist update: %d\n", period_hist[current_instant_periods[i] - MIN_PERIOD]);
    }
    
    if(peakIsSurpassed)
    {
        int i;
        for(i = 0; i < MAX_PERIOD - MIN_PERIOD + 1; i++)
        {
            if(period_hist[i] > 0)
            {
                period_hist[i]--;
            }
        }
    }
}

bool areRelatedPeriods(int period1, int period2)
{
    float relation = period1/period2;
    if(fabs(relation - 2) < PERIOD_RELATIONS_PRECISION)
        return true;
    else if (fabs(relation - 3) < PERIOD_RELATIONS_PRECISION)
        return true;
    else if (fabs(relation - 0.5) < PERIOD_RELATIONS_PRECISION)
        return true;
    else if (fabs(relation - 0.33333) < PERIOD_RELATIONS_PRECISION)
        return true;
    return false;
}

// Returns the nearest period from the array to the center period defined above
int getNearestToCenterBPM(int *periods, int length)
{
    int smallestDistance = abs(periods[0] - CENTER_PERIOD);
    int nearestPeriod = periods[0];
    int i;
    for(i = 1; i < length; i++)
    {
        if(abs(periods[i] - CENTER_PERIOD) < smallestDistance)
        {
            smallestDistance = abs(periods[i] - CENTER_PERIOD);
            nearestPeriod = periods[i];
        }
    }
    return nearestPeriod;
}

void updateGlobalPeriod()
{
    int max = GLOBAL_PERIOD_THRESHOLD;
    int best_periods[HIST_NUMBER_ADDITIONS] = {1, 1, 1};
    int numBestPeriods = 0;
    
    // Find best three periods (if they are above 25)
    int i;
    for(i = MIN_PERIOD; i <= MAX_PERIOD; i++)
    {
        if(period_hist[i - MIN_PERIOD] > max) {
            max = period_hist[i - MIN_PERIOD];
            memmove(best_periods + 1, best_periods, sizeof(best_periods) - sizeof(int));
            best_periods[0] = i;
            numBestPeriods++;
            // DEBUG: printf("HEY!!!: %d, %d\n", i, max);
        }
    }
    
    if(numBestPeriods > 3)
    {
        numBestPeriods = 3;
    }
    
    // DEBUG: printf("Best periods: %d: %d, %d: %d, %d: %d\n", best_periods[0], period_hist[best_periods[0] - MIN_PERIOD], best_periods[1], period_hist[best_periods[1] - MIN_PERIOD], best_periods[2], period_hist[best_periods[2] - MIN_PERIOD]);
    
    // Discard non-related periods
    bool isRelatedPeriod[HIST_NUMBER_ADDITIONS] = {false, false, false};
    if(areRelatedPeriods(best_periods[0], best_periods[1]))
    {
        isRelatedPeriod[0] = true;
        isRelatedPeriod[1] = true;
    }
    if(areRelatedPeriods(best_periods[0], best_periods[2]))
    {
        isRelatedPeriod[0] = true;
        isRelatedPeriod[2] = true;
    }
    if(areRelatedPeriods(best_periods[1], best_periods[2]))
    {
        isRelatedPeriod[1] = true;
        isRelatedPeriod[2] = true;
    }
    
    // Count remaining periods
    int numRelatedPeriods = 0;
    int relPeriods[HIST_NUMBER_ADDITIONS] = {1, 1, 1};
    for(i = 0; i < numBestPeriods; i++)
    {
        if(isRelatedPeriod[i])
        {
            relPeriods[numRelatedPeriods] = best_periods[i];
            numRelatedPeriods++;
            
        }
    }
    
    // Get the period nearest to the center period among the related periods only
    int best_period = -1;
    if(numRelatedPeriods == 0)
    {
        int numImportantPeriods = 1;
        if(period_hist[best_periods[1] - MIN_PERIOD] > period_hist[best_periods[0] - MIN_PERIOD]/2)
        {
            numImportantPeriods++;
        }
        if (period_hist[best_periods[2] - MIN_PERIOD] > period_hist[best_periods[0] - MIN_PERIOD]/2)
        {
            numImportantPeriods++;
        }
        if(numImportantPeriods > numBestPeriods)
        {
            numImportantPeriods = numBestPeriods;
        }
        best_period = getNearestToCenterBPM(best_periods, numImportantPeriods);
    }
    else
    {
        best_period = getNearestToCenterBPM(relPeriods, numRelatedPeriods);
    }
    
    current_global_period = best_period;
}

void sendPeriod()
{
    if(current_global_period == -1)
    {
        printf("wtf?¿?\n");
    }
	
    ROS_INFO("Sent period %f bpm\n", 60/(current_global_period*ONSET_SIGNAL_PERIOD_SEC));
    test_hist[current_global_period-MIN_PERIOD]++;
	
	std_msgs::Int32 msg;
	msg.data = current_global_period;
    period_pub.publish(msg);
}

void test_autocorr()
{
    printf("Minimum period: %d\n", MIN_PERIOD);
    printf("Maximum period: %d\n", MAX_PERIOD);
    int min_impulse_period = 40;
    
    int i;
    for (i = 0; i < MAX_SIGNAL_BUFFER_LENGTH; i++)
    {
        current_signal[i] = 0;
        if(i % min_impulse_period == 0)
        {
            current_signal[i] = 10;
        }
    }
    
    autocorr();
}

void printHist()
{
    int i;
    for(i = MIN_PERIOD; i <= MAX_PERIOD; i++)
    {
        printf("%f BPM: %d\n",
               60/(i*ONSET_SIGNAL_PERIOD_SEC),
               test_hist[i-MIN_PERIOD]);
    }
}

void my_handler(int s)
{
    printHist();
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

int main(int argc, char **argv)
{
	ros::init(argc, argv, "music_period_estimator");
	ros::NodeHandle n;
    period_pub = n.advertise<std_msgs::Int32>("/music_period", 1000);
	
    catchCtrlC();
    
    int cnt = 0;
    OscFloatReceiver *osc_rec = new OscFloatReceiver(PERIOD_OSC_PORT);
    memset(current_autocorr, 0, sizeof(current_autocorr));
    memset(current_instant_periods, 0, sizeof(current_instant_periods));
    memset(current_signal, 0, sizeof(current_signal));
    memset(period_hist, 0, sizeof(period_hist));
    
    /* DEBUG PART
    auto prevms = std::chrono::high_resolution_clock::now();
    auto nowms = std::chrono::high_resolution_clock::now();*/
    
    float last_onset_strength;
    while(1)
    {
        last_onset_strength = osc_rec->read_float();
        
         /* DEBUG PART
        prevms = nowms;
        nowms = std::chrono::high_resolution_clock::now();
        auto dur = nowms - prevms;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
        printf("Passed ms: %f\n", ms);*/
        
        current_signal[0] = last_onset_strength;
        
        //printf("%f\n", last_onset_strength);
        
        if(last_onset_strength == -1000) // That means music is off
        {
            printf("MUSIC OFF\n");
            // reset period histogram
            memset(period_hist, 0, sizeof(period_hist));
            
            current_global_period = -1;
            current_signal[0] = 0;
            
            prepare_counter = 0;
            stabilisation_counter = 0;
        }
        else if(prepare_counter < MAX_PERIOD*2)
        {
            prepare_counter++;
        }
        else
        {
            autocorr();
            updateHist();
            updateGlobalPeriod();
            
            if(stabilisation_counter > STABILISATION_COUNTER_THRESHOLD)
            {
                sendPeriod();
                stabilisation_counter = 0;
            }
            else
            {
                stabilisation_counter++;
            }
        }
        
        /* DEBUG
        int i;
        for(i = 0; i < 7; i++)
        {
            printf("%f, ", current_signal[i]);
        }
        printf("\n"); */
        
        // shift current_signal to receive new value in the next iteration
        memmove(current_signal + 1, current_signal, sizeof(current_signal) - sizeof(float));
    }
    return 0;
}
