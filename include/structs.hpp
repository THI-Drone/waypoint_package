#pragma once

#include <string>
#include <cinttypes>

struct Command
{
    bool values_set = false; /// Flag that is true when the values have been set and false when not initialized with values
    std::string target_coordinate_lat;
    std::string target_coordinate_lon;
    uint32_t pre_wait_time_ms = 0;
    uint32_t post_wait_time_ms = 0;
    float cruise_height_cm;
    float target_height_cm;
    float horizontal_speed_mps;
    float vertical_speed_mps;
};
