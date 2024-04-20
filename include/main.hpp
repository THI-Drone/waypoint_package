#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include "rclcpp/rclcpp.hpp"

#include "common_package/common_node.hpp"
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
    std::vector<command> commands;  ///< Vector of commands that will be executed in the given order

    // Subscriptions
    rclcpp::Subscription<interfaces::msg::Control>::SharedPtr control_subscription;

    // Publisher
    rclcpp::Publisher<interfaces::msg::JobFinished>::SharedPtr job_finished_publisher;

public:
    WaypointNode();

private:
    void callback_control(const interfaces::msg::Control &msg);
};
