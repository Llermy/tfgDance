#include <stdio.h>
#include <unistd.h>
#include <string>
#include "osc_float_receiver.hpp"

#define OSC_READ_PORT 7000
#define MIN_PERIOD_uSEC 200000

#define MIN_ONSET_STRENGTH_THRESHOLD 1
#define MAX_ONSET_STRENGTH_THRESHOLD 6
#define ONSET_THRESHOLD_CHANGE_STEP 1
#define FEWER_BEATS_THRESHOLD 300

#define ONSETS_BUFFER_LENGTH 1200
#define MAX_PERIODS_TO_COMPARE 3

int current_period = 107; // The units of current_period is the period in which we receive floats of the discrete onset strength signal
int period_count = 0;

int count = 0;
float current_threshold = MIN_ONSET_STRENGTH_THRESHOLD;
float current_onset_signal[ONSETS_BUFFER_LENGTH];

int fewer_beats_counter = 0;

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
    printf("%d PEAK! %f\n", count, current_onset_signal[0]);
    count++;
}

int main(int argc, char *argv[])
{
    if(argc == 2)
    {
        int bpm = std::stoi(argv[1]);
        printf("Set period for testing: %d BPM", bpm);
        current_period = 60/(bpm*ONSET_SIGNAL_PERIOD_SEC);
    }
    
    memset(current_onset_signal, 0, sizeof(current_onset_signal));
    OscFloatReceiver *osc_rec = new OscFloatReceiver(OSC_READ_PORT);
    
    while(1)
    {
        memmove(current_onset_signal + 1, current_onset_signal, sizeof(current_onset_signal) - sizeof(float));
        current_onset_signal[0] = osc_rec->read_float();
        //printf("%f\n", current_onset_signal[0]);
        
        if(current_onset_signal[0] == -1000) // That means music is off
        {
            current_onset_signal[0] = 0;
            period_count = 0;
            current_threshold = MIN_ONSET_STRENGTH_THRESHOLD;
            continue;
        }
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
    return 0;
}