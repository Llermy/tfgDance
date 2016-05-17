#include <stdio.h>
#include <unistd.h>
#include "osc_float_receiver.hpp"

#define OSC_READ_PORT 7000
#define MIN_PERIOD_uSEC 200000

#define MIN_ONSET_STRENGTH_THRESHOLD 1
#define MAX_ONSET_STRENGTH_THRESHOLD 5

#define FEWER_BEATS_THRESHOLD 300

int current_period = 80; // The units of current_period is the period in which we receive floats of the discrete onset strength signal
int period_count = 0;

int count = 0;
int current_threshold = MIN_ONSET_STRENGTH_THRESHOLD;

int fewer_beats_counter = 0;

void checkIfWeakerBeats(float onset)
{
    if(onset < MIN_ONSET_STRENGTH_THRESHOLD)
    {
        fewer_beats_counter++;
        if(fewer_beats_counter > FEWER_BEATS_THRESHOLD &&
           current_threshold > MIN_ONSET_STRENGTH_THRESHOLD)
        {
            current_threshold--;
            printf("Lower threshold: %d", current_threshold);
            fewer_beats_counter = 0;
        }
    } else {
        fewer_beats_counter = 0;
    }
}

bool isPeak(float onset)
{
    if(onset > current_threshold)
    {
        printf("WOW!: %f", onset);
        if(current_threshold < MAX_ONSET_STRENGTH_THRESHOLD)
        {
            current_threshold++;
        }
        return true;
    }
    return false;
}

void process_peak()
{
    printf("PEAK!\n %d", count);
    count++;
}

int main()
{
    OscFloatReceiver *osc_rec = new OscFloatReceiver(OSC_READ_PORT);
    
    float last_onset_strength;
    while(1)
    {
        last_onset_strength = osc_rec->read_float();
        //printf("%f\n", last_onset_strength);
        
        if(last_onset_strength == -1000) // That means music is off
        {
            period_count = 0;
            current_threshold = MIN_ONSET_STRENGTH_THRESHOLD;
            continue;
        }
        checkIfWeakerBeats(last_onset_strength);
        
        if (isPeak(last_onset_strength))
        {
            period_count = 0;
            process_peak();
            usleep(MIN_PERIOD_uSEC);
        } else if (period_count > current_period)
        {
            period_count = 0;
            if(last_onset_strength + 1 > 0.03)
            {
                process_peak();
            }
        }
        period_count++;
    }
    return 0;
}