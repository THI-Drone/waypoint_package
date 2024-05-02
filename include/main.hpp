#pragma once

#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

#include "common_package/commands.hpp"
#include "common_package/common_node.hpp"
#include "structs.hpp"

// Message includes
#include "interfaces/msg/control.hpp"
#include "interfaces/msg/job_finished.hpp"
#include "interfaces/msg/gps_position.hpp"
#include "interfaces/msg/waypoint.hpp"
#include "interfaces/msg/uav_waypoint_command.hpp"

typedef enum NodeState
{
    init,
    pre_wait_time,
    reach_cruise_height,
    fly_to_waypoint,
    reach_target_height,
    post_wait_time,
} NodeState_t;

/**
 * @brief A class for the waypoint node.
 */
class WaypointNode : public common_lib::CommonNode
{
private:
    /// Current state of the node
    NodeState_t node_state = init;
    /// Command that will be executed when active
    Command cmd;
    /// If set to true, the first event loop after activating the node is happening. Will be set to false when calling get_state_first_loop().
    bool state_first_loop = true;
    /// Current position
    Position pos;
    /// Maximum age of a position message from FCC bridge
    static constexpr uint16_t max_position_msg_time_difference_ms = 500;

    // Thresholds
    /// Height threshold
    static constexpr uint32_t height_treshold_cm = 50; // cm

    // Event Loop
    const uint32_t event_loop_time_delta_ms = 100;
    rclcpp::TimerBase::SharedPtr event_loop_timer;

    /// Publisher for the "uav_waypoint_command" topic
    rclcpp::Publisher<interfaces::msg::UAVWaypointCommand>::SharedPtr uav_waypoint_command_publisher;

    // Wait Timer
    rclcpp::TimerBase::SharedPtr wait_timer;

    // Subscriptions
    rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_subscription;
    rclcpp::Subscription<interfaces::msg::GPSPosition>::SharedPtr gps_position_subscription;

public:
    WaypointNode();

private:
    // Event Loop
    void event_loop();

    // Reset node
    void reset_node();

    // Node State
    void set_node_state(NodeState_t new_state);
    constexpr NodeState_t get_node_state() const { return node_state; }

    bool get_state_first_loop();

    // Callbacks
    void callback_control(const interfaces::msg::Control &msg);
    void callback_position(const interfaces::msg::GPSPosition &msg);
    void callback_wait_time();

    // Modes
    void mode_init();
    void mode_reach_cruise_height();
    void mode_fly_to_waypoint();
    void mode_reach_target_height();

    // Check cmd is valid
    bool check_cmd(const char* function_name);
};
