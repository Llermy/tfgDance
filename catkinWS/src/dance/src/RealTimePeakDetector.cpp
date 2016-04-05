
/**
 *
 * ______________________________
 * REAL-TIME AUDIO PEAK DETECTION
 * ------------------------------
 *
 **/

#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "RealTimePeakDetector.hpp"

using namespace std;

RealTimePeakDetector::RealTimePeakDetector() {
    // Peak publisher
    ros::Publisher peak_pub;
    
    buffer_loaded = false;
    last_instant_index = 0;
    
    formInstant_counter = 1;
    currentInstant_energy = 0;
    prevInstant_energy = -1;
    
    insidePeak_counter = 0;
    insidePeak = false;
    currentPeakMaximum = -1;
}

// The MAIN FUNCTION to process the single audio samples to search if there is a peak.
// First of all, we gather "INSTANT_SAMPLES" samples and we sum them quadratically to form an instant (see member variables comments).
// We add the instant to our buffer (and we wait to load completely the buffer if it's not) and we estimate if this instant was in a peak.
// If a peak is found, we wait until we exit the peak to search for other peaks.
void RealTimePeakDetector::processAudio(double sample) {
    // The samples are added to currentInstant_energy until an instant is formed
    currentInstant_energy += sample*sample;
    
    if (formInstant_counter == INSTANT_SAMPLES) {
        // An instant has finally been formed and we save it into our buffer of last instants
        cout << "Instant " << currentInstant_energy << " was formed and added to the buffer.\n";
        formInstant_counter = 0;
        instants_buffer[last_instant_index] = currentInstant_energy;
        
        // First we have to fill the buffer of last instants to have a ground to compare for a peak
        if (!buffer_loaded) {
            cout << "Buffer loading..." << last_instant_index << "\n";
            buffer_loaded = last_instant_index >= BUFFER_LENGTH-1;
        } else {
            // If we found a peak, we wait till we exit the peak and then we restart searching for peaks
            if (insidePeak) {
                cout << "Right now inside a peak.\n";
                insidePeak = !peakWasExited();
            } else {
                checkIfPeak();
            }
        }
        
        // We prepare last_instant_index so that we have the latest instants in instants_buffer by basically cycling the buffer
        last_instant_index++;
        if (last_instant_index >= BUFFER_LENGTH)
            last_instant_index = 0;
        
        // We save this instant and initialise the next one
        prevInstant_energy = currentInstant_energy;
        currentInstant_energy = 0;
        
        cout << "\n";
    }
    
    formInstant_counter++;
}

// Function to calculate the mean of the values of an array
double RealTimePeakDetector::mean(double* array, int length) {
    double sum = 0;
    for (int i = 0; i < length; i++) {
        sum += array[i];
    }
    return sum/length;
}

// Finds if the currentInstant is now outside the last found peak. The peak is decided to be exited if there were "NUM_FALLS_TO_EXIT_PEAK" instants below the peak maximum
bool RealTimePeakDetector::peakWasExited() {
    if (currentInstant_energy > currentPeakMaximum) {
        currentPeakMaximum = currentInstant_energy;
        insidePeak_counter = 0;
    } else {
        insidePeak_counter++;
        cout << insidePeak_counter;
        if (insidePeak_counter == NUM_FALLS_TO_EXIT_PEAK) {
            return true;
        }
    }
    return false;
}

// Checks if the currentInstant has entered a peak and if so, we process the peak. We enter a peak if the currentInstant is a certain amount above the mean of the instants_buffer
void RealTimePeakDetector::checkIfPeak() {
    if (currentInstant_energy > prevInstant_energy) {
        double bufferMean = mean(instants_buffer, BUFFER_LENGTH);
        // DELETED: This restricts very low peaks to be found.
        insidePeak = currentInstant_energy > SENSIBILITY*bufferMean;
        if (insidePeak) {
            cout << "Found peak!\n";
            currentPeakMaximum = currentInstant_energy;
            //if (energyThreshold < 1) {
            //    energyThreshold = 1;
            //}
            processPeak();
        }
    }
}

// Decides the action to carry when a peak is found.
void RealTimePeakDetector::processPeak() {
    std_msgs::Float64 msg;
    msg.data = (float) currentInstant_energy;
    
    ROS_INFO("Sent beat: %f", msg.data);
    peak_pub.publish(msg);
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "RT_peak_detector");
    ros::NodeHandle n;
    peak_pub = n.advertise<std_msgs::Float64>("beats", 1000);
    
    RealTimePeakDetector* rtpd = new RealTimePeakDetector();
    
    for (int i = 0; i < 1024*63; i++) {
        if (i == 1024*60 + 3) {
            rtpd->processAudio(0.2210);
        } else {
            rtpd->processAudio(0);
        }
    }
    
    return 0;
}





