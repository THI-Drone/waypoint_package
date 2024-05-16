#pragma once

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

#include "common_package/commands.hpp"
#include "common_package/common_node.hpp"
#include "common_package/topic_names.hpp"
#include "rclcpp/rclcpp.hpp"
#include "structs.hpp"

// Message includes
#include "interfaces/msg/control.hpp"
#include "interfaces/msg/gps_position.hpp"
#include "interfaces/msg/job_finished.hpp"
#include "interfaces/msg/mission_progress.hpp"
#include "interfaces/msg/uav_waypoint_command.hpp"
#include "interfaces/msg/waypoint.hpp"

/**
 * @brief A class for the waypoint node.
 */
class WaypointNode : public common_lib::CommonNode {
   private:
    typedef enum NodeState {
        init,
        pre_wait_time,
        reach_cruise_height,
        fly_to_waypoint,
        reach_target_height,
        post_wait_time,
    } NodeState_t;

    NodeState_t node_state = init;  //!< Current state of the node

    Command cmd;  //!< Command that will be executed when active

    bool state_first_loop =
        true;  //!< If set to true, the first event loop after activating the
               //!< node is happening. Will be set to false when calling
               //!< get_state_first_loop().

    Position pos;  //!< Current position

    static constexpr uint32_t max_position_msg_time_difference_ms =
        100;  //!< Maximum age of a position message from FCC bridge

    static constexpr uint32_t max_progress_msg_time_difference_ms =
        500;  //!< Maximum age of a progress message from FCC bridge

    float mission_progress = 0.0;  //!< Current mission progress

    // Event Loop
    const uint32_t event_loop_time_delta_ms = 100;
    rclcpp::TimerBase::SharedPtr event_loop_timer;

    // Publisher for the "uav_waypoint_command" topic
    rclcpp::Publisher<interfaces::msg::UAVWaypointCommand>::SharedPtr
        uav_waypoint_command_publisher;

    // Wait Timer
    rclcpp::TimerBase::SharedPtr wait_timer;

    // Subscriptions
    rclcpp::Subscription<interfaces::msg::Control>::SharedPtr
        control_subscription;
    rclcpp::Subscription<interfaces::msg::GPSPosition>::SharedPtr
        gps_position_subscription;
    rclcpp::Subscription<interfaces::msg::MissionProgress>::SharedPtr
        mission_progress_subscription;

   public:
    WaypointNode();

   private:
    // Event Loop
    void event_loop();

    // Reset node
    void reset_node();

    // Node State
    void set_node_state(const NodeState_t new_state);
    constexpr NodeState_t get_node_state() const { return node_state; }
    const char *get_node_state_str() const;
    const char *get_node_state_str(const NodeState_t node_state) const;

    bool get_state_first_loop();

    // Mission Progress
    bool current_mission_finished();

    // Callbacks
    void callback_control(const interfaces::msg::Control &msg);
    void callback_position(const interfaces::msg::GPSPosition &msg);
    void callback_mission_progress(const interfaces::msg::MissionProgress &msg);
    void callback_wait_time();

    // Modes
    void mode_init();
    void mode_reach_cruise_height();
    void mode_fly_to_waypoint();
    void mode_reach_target_height();

    // Check cmd is valid
    bool check_cmd(const char *function_name);
};
