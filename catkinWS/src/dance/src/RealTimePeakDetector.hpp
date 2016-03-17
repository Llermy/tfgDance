/**
 *
 * ______________________________
 * REAL-TIME AUDIO PEAK DETECTION (header file)
 * ------------------------------
 *
 **/

#ifndef RealTimePeakDetector_hpp
#define RealTimePeakDetector_hpp

#define INSTANT_SAMPLES 1024
#define BUFFER_LENGTH 43
#define NUM_FALLS_TO_EXIT_PEAK 4
#define SENSIBILITY 1.3

#include <stdio.h>
#include <iostream>

class RealTimePeakDetector {
    // Variables related to the instants. An instant is the quadratic sum of N audio samples, where N equals INSTANT_SAMPLES
    int formInstant_counter;
    double currentInstant_energy;
    double prevInstant_energy;
    
    // Variables to control the buffer in order to save the energy of the last received instants
    bool buffer_loaded;
    double instants_buffer[BUFFER_LENGTH];
    int last_instant_index;
    
    // Variables to control if we are inside a peak
    int insidePeak_counter;
    bool insidePeak;
    double currentPeakMaximum;
    
public:
    RealTimePeakDetector();
    double mean(double* array, int length);
    void processAudio(double sample);
    bool peakWasExited();
    void checkIfPeak();
    void processPeak();
};

#endif /* RealTimePeakDetector_hpp */