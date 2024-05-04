#include "main.hpp"

using std::placeholders::_1;

WaypointNode::WaypointNode() : CommonNode("waypoint_node") {
    // Create a subscription for the "control" topic
    control_subscription = this->create_subscription<interfaces::msg::Control>(
        "control", 10, std::bind(&WaypointNode::callback_control, this, _1));

    // Create a subscription for the "uav_gps_position" topic
    gps_position_subscription =
        this->create_subscription<interfaces::msg::GPSPosition>(
            "uav_gps_position", 10,
            std::bind(&WaypointNode::callback_position, this, _1));

    // Create a subscription for the "uav_mission_progress" topic
    mission_progress_subscription =
        this->create_subscription<interfaces::msg::MissionProgress>(
            "uav_mission_progress", 10,
            std::bind(&WaypointNode::callback_mission_progress, this, _1));

    // Create a publisher for the "uav_waypoint_command" topic
    uav_waypoint_command_publisher =
        this->create_publisher<interfaces::msg::UAVWaypointCommand>(
            "uav_waypoint_command", 10);

    // Initialize Event Loop
    event_loop_timer = this->create_wall_timer(
        std::chrono::milliseconds(event_loop_time_delta_ms),
        std::bind(&WaypointNode::event_loop, this));
}

/**
 * @brief Checks if the first loop was already triggered and returns the result.
 *
 * This function checks the state of the first loop and returns false if it was
 * already triggered, otherwise it returns true. After returning the state, it
 * sets the first loop state to false.
 *
 * @return False if the first loop was already triggered, true otherwise.
 */
bool WaypointNode::get_state_first_loop() {
    // Check if first loop was already triggered
    if (!state_first_loop) return false;

    // Set first loop to false and return true
    state_first_loop = false;
    return true;
}

/**
 * Checks if the current mission is finished.
 *
 * @return true if the mission is finished, false otherwise.
 */
bool WaypointNode::current_mission_finished() {
    if (mission_progress >= 1.0) {
        mission_progress = 0.0;
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Executes the event loop for the WaypointNode.
 *
 * This function is responsible for executing the event loop of the WaypointNode
 * class. It checks the node state and performs the corresponding actions based
 * on the state. The possible states are:
 * - init: Initializes the mode.
 * - pre_wait_time: Wait a predefined time.
 * - reach_cruise_height: Fly drone to the crusing height.
 * - fly_to_waypoint: Executes the mode to fly to the waypoint.
 * - reach_target_height: Fly drone to the target height.
 * - post_wait_time: Wait a predefined time.
 *
 * If the node state is unknown, an error message is logged and a job_finished
 * error message is sent.
 */
void WaypointNode::event_loop() {
    if (!this->get_active()) return;

    switch (get_node_state()) {
        case init:
            mode_init();
            break;
        case pre_wait_time:
            break;  // Wait for the `wait_timer` to trigger
        case reach_cruise_height:
            mode_reach_cruise_height();
            break;
        case fly_to_waypoint:
            mode_fly_to_waypoint();
            break;
        case reach_target_height:
            mode_reach_target_height();
            break;
        case post_wait_time:
            break;  // Wait for the `wait_timer` to trigger
        default:
            RCLCPP_ERROR(this->get_logger(),
                         "WaypointNode::event_loop: Unknown mission_state: %d",
                         get_node_state());
            this->job_finished(
                "WaypointNode::event_loop: Unknown mission_state");
            reset_node();
    }
}

/**
 * @brief Resets the node for the next command.
 *
 * This function resets the state of the `WaypointNode` object for the next
 * command. It sets the node state to `init`, sets the `state_first_loop` flag
 * to `true`, resets the `mission_progress` to 0, and clears the `cmd` object.
 */
void WaypointNode::reset_node() {
    set_node_state(init);
    state_first_loop = true;
    mission_progress = 0.0;
    cmd = Command();
}

/**
 * @brief Sets the node state to a new value.
 *
 * This function sets the node state to the specified new value. If the new
 * value is different from the current node state, the `state_first_loop` flag
 * is set to true. After updating the node state, a debug message is logged.
 * Additionally, `mission_progress` is reset to 0.
 *
 * @param new_mission_state The new mission state to set.
 */
void WaypointNode::set_node_state(NodeState_t new_state) {
    if (node_state != new_state) {
        state_first_loop = true;
    }

    mission_progress = 0.0;
    node_state = new_state;
    RCLCPP_DEBUG(this->get_logger(),
                 "WaypointNode::set_node_state: Set node state to %d",
                 node_state);
}

/**
 * @brief Checks if the command and position values are set.
 *
 * This function is used to check if the command and position values are set
 * before executing a command. If either the command or position values are not
 * set, the function logs an error message and returns false.
 *
 * @param function_name The name of the calling function.
 * @return True if both the command and position values are set, false
 * otherwise.
 */
bool WaypointNode::check_cmd(const char *function_name) {
    if (!cmd.values_set) {
        // Cancel job and reset node

        RCLCPP_FATAL(this->get_logger(),
                     "WaypointNode::%s::check_cmd: No command specified. "
                     "Aborting job and resetting node.",
                     function_name);

        this->job_finished("WaypointNode::" + (std::string)function_name +
                           "::check_cmd: No command specified. Aborting job "
                           "and resetting node.");

        reset_node();
        return false;
    }

    if (!pos.values_set) {
        // Cancel job and reset node

        RCLCPP_FATAL(this->get_logger(),
                     "WaypointNode::%s::check_cmd: No position received."
                     " Aborting job and resetting node.",
                     function_name);

        this->job_finished("WaypointNode::" + (std::string)function_name +
                           "::check_cmd: No position received. Aborting "
                           "job and resetting node.");

        reset_node();
        return false;
    }

    return true;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNode>());
    rclcpp::shutdown();
    return 0;
}
