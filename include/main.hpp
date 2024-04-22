#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <stdexcept>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"

#include "common_package/common_node.hpp"
#include "common_package/commands.hpp"
#include "structs.hpp"

// Message includes
#include "interfaces/msg/control.hpp"
#include "interfaces/msg/job_finished.hpp"

/**
 * @brief A class for the waypoint node.
 */
class WaypointNode : public common_lib::CommonNode
{
private:
    command cmd;                  /// Command that will be executed when active
    bool state_first_loop = true; /// If set to true, the first event loop after activating the node is happening. Will be set to false when calling get_state_first_loop().

    // Event Loop
    const uint32_t event_loop_time_delta_ms = 100;
    rclcpp::TimerBase::SharedPtr event_loop_timer;

    // Subscriptions
    rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_subscription;

public:
    WaypointNode();

private:
    // Event Loop
    void event_loop();

    bool get_state_first_loop();

    void callback_control(const interfaces::msg::Control &msg);
};
