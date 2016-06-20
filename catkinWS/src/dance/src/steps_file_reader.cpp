#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <fstream>
#include <string>

#include "steps_file_reader.h"


std::string StepsFileReader::get_whole_dance(int dance_number)
{
    set_dance(dance_number);
    std::string dance;
    dance_file >> dance;
    return dance;
}

void StepsFileReader::set_dance(int dance_number)
{
    dance_file.seekg(std::ios::beg);
    for(int i = 0; i < dance_number - 1; i++)
    {
        dance_file.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
    }
    dance_file >> last_dance;
}

std::string StepsFileReader::get_step(int step_number)
{
    std::string dance = last_dance;
    int start = 0;
    int end = last_dance.find("/", start);
    for(int i = 0; i < step_number; i++)
    {
        start = end + 1;
        end = last_dance.find("/", start);
    }
    return last_dance.substr(start, end-start);
}

std::string StepsFileReader::get_next_step()
{
    next_step++;
    if(next_step > DANCE_LENGTH)
    {
        next_step = 1;
    }
    return get_step(next_step-1);
}

StepsFileReader::StepsFileReader()
{
    dance_file.open(DANCE_FILE_NAME, std::ios::in);
    if(!dance_file.is_open())
    {
        ROS_INFO("ERROR: File containing the dances not found.");
    }
    next_step = 0;
}
