#pragma once

#include <cinttypes>
#include <string>

/**
 * @brief Struct representing a set of values for a waypoint.
 *
 * This struct contains various parameters that define a waypoint, such as
 * target coordinates, wait times, heights, and speeds.
 */
struct Command {
    static constexpr uint32_t min_wait_time_ms =
        100;  //!< Minimum pre- and post-wait-time in ms

    bool values_set =
        false;  //!< Flag that is true when the values have been set and false
                //!< when not initialized with values
    double target_coordinate_lat;
    double target_coordinate_lon;
    uint32_t pre_wait_time_ms = min_wait_time_ms;
    uint32_t post_wait_time_ms = min_wait_time_ms;
    uint32_t cruise_height_cm;
    uint32_t target_height_cm;
    float horizontal_speed_mps;
    float vertical_speed_mps;
};

/**
 * @brief Represents the current position with latitude, longitude, and height.
 */
struct Position {
    bool values_set =
        false;  //!< Flag that is true when the values have been set and false
                //!< when not initialized with values
    double coordinate_lat;
    double coordinate_lon;
    uint32_t height_cm;
};
