#include <string.h>
#include <stdio.h>
#include <cmath>
#include "osc_float_receiver.hpp"

#define MAX_BPM 250
#define MIN_BPM 30
#define MAX_PERIOD (int) (60/(MIN_BPM*ONSET_SIGNAL_PERIOD_SEC)) + 1
#define MIN_PERIOD (int) (60/(MAX_BPM*ONSET_SIGNAL_PERIOD_SEC)) - 1
#define MAX_SIGNAL_BUFFER_LENGTH 500 // corresponds roughly with 7-8 seconds
#define HIST_MAX_NUMBER 100
#define PERIOD_RELATIONS_PRECISION 0.05

#define PREPARE_COUNTER_THRESHOLD MAX_PERIOD + 10
#define STABILISATION_COUNTER_THRESHOLD 20

// Autocorrelation-limiting variables
int max_period;
int min_period;

// Autocorrelation buffer
float current_autocorr[MAX_PERIOD - MIN_PERIOD + 1];
float current_signal[MAX_SIGNAL_BUFFER_LENGTH];
int period_hist[MAX_PERIOD - MIN_PERIOD + 1]; // histogram of all previous found periods

int current_instant_period = -1;
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

// Calculate autocorrelation and find maximum value
void autocorr()
{
    float max = 0;
    
    int i;
    for(i = MIN_PERIOD; i <= MAX_PERIOD; i++)
    {
        current_autocorr[i - MIN_PERIOD] = autocorrAt(i);
        if(current_autocorr[i - MIN_PERIOD] > max) {
            max = current_autocorr[i - MIN_PERIOD];
            current_instant_period = i;
        }
        
        if(current_autocorr[i - MIN_PERIOD] != 0)
        {
            printf("Autocorr: %d, %f\n", i, current_autocorr[i - MIN_PERIOD]);
        }
    }
}

void updateHist()
{
    int hist_peak = ++period_hist[current_instant_period - MIN_PERIOD];
    if(hist_peak > HIST_MAX_NUMBER)
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

void updateGlobalPeriod()
{
    int max = 0;
    int best_period = -1;
    int i;
    for(i = MIN_PERIOD; i <= MAX_PERIOD; i++)
    {
        if(period_hist[i - MIN_PERIOD] > max) {
            max = period_hist[i];
            best_period = i;
        }
    }
    
    float relation = current_global_period/best_period;
    if(fabs(relation - 2) < PERIOD_RELATIONS_PRECISION)
        return;
    else if (fabs(relation - 3) < PERIOD_RELATIONS_PRECISION)
        return;
    else if (fabs(relation - 0.5) < PERIOD_RELATIONS_PRECISION)
        return;
    else if (fabs(relation - 0.33333) < PERIOD_RELATIONS_PRECISION)
        return;
    
    current_global_period = best_period;
}

void sendPeriod()
{
    if(current_global_period == -1)
    {
        printf("wtf?¿?\n");
    }
    printf("Sent period %f bpm\n", 60/(current_global_period*ONSET_SIGNAL_PERIOD_SEC));
}

void test_autocorr()
{
    printf("Minimum period: %d\n", MIN_PERIOD);
    printf("Maximum period: %d\n", MAX_PERIOD);
    int min_impulse_period = 40;
    
    int i;
    for (i = 0; i<MAX_SIGNAL_BUFFER_LENGTH; i++)
    {
        current_signal[i] = 0;
        if(i % min_impulse_period == 0)
        {
            current_signal[i] = 10;
        }
    }
    
    autocorr();
}

int main()
{
    int cnt = 0;
    //OscFloatReceiver *osc_rec = new OscFloatReceiver(7000);
    memset(current_autocorr, 0, sizeof(current_autocorr));
    memset(current_signal, 0, sizeof(current_signal));
    memset(period_hist, 0, sizeof(period_hist));
    
    test_autocorr();
    return 0;
    
    float last_onset_strength;
    while(1)
    {
        //last_onset_strength = osc_rec->read_float();
        current_signal[0] = last_onset_strength;
        
        //printf("%f\n", last_onset_strength);
        
        if(last_onset_strength == -1000) // That means music is off
        {
            // reset period histogram
            memset(period_hist, 0, sizeof(period_hist));
            
            current_global_period = -1;
            current_signal[0] = 0;
            
            prepare_counter = 0;
            stabilisation_counter = 0;
        }
        else if(prepare_counter < MAX_PERIOD)
        {
            prepare_counter++;
        }
        else
        {
            autocorr();
            updateGlobalPeriod();
            updateHist();
            
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