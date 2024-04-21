#pragma once

#include <string>
#include <cinttypes>

struct command
{
    std::string target_coordinate;
    uint32_t pre_wait_time_ms = 0;
    uint32_t post_wait_time_ms = 0;
    float travel_height;
    float target_height;
    float horizontal_speed;
    float vertical_speed;
};
