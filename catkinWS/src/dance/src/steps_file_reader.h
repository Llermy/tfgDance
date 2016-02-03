#ifndef STEPS_FILE_READER_H
#define STEPS_FILE_READER_H

#include <fstream>

#define DANCE_FILE_NAME "dance_save.dnc"
#define DANCE_LENGTH 16

class StepsFileReader
{
public:
    int next_step;
    std::string last_dance;
    std::fstream dance_file;
    
    std::string get_whole_dance(int dance_number);
    
    void set_dance(int dance_number);
    
    std::string get_step(int step_number);
    
    std::string get_next_step();
    
    StepsFileReader();
};


#endif
