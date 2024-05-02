#pragma once

#include <cinttypes>
#include <string>

struct Command
{
    /// Flag that is true when the values have been set and false when not initialized with values
    bool values_set = false;
    double target_coordinate_lat;
    double target_coordinate_lon;
    uint32_t pre_wait_time_ms = 0;
    uint32_t post_wait_time_ms = 0;
    uint32_t cruise_height_cm;
    uint32_t target_height_cm;
    float horizontal_speed_mps;
    float vertical_speed_mps;
};

struct Position
{
    /// Flag that is true when the values have been set and false when not initialized with values
    bool values_set = false;
    double coordinate_lat;
    double coordinate_lon;
    uint32_t height_cm;
};
