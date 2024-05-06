#include "main.hpp"

/**
 * @brief Callback function for the control message.
 *
 * This function is called when a control message is received. It processes the
 * message, activates the node based on the message, parses the payload, and
 * saves the data in a Command struct.
 *
 * @param msg The control message received.
 */
void WaypointNode::callback_control(const interfaces::msg::Control &msg) {
    // Ignore messages that don't target this node
    if (msg.target_id != this->get_name()) {
        RCLCPP_DEBUG(this->get_logger(),
                     "WaypointNode::%s: Ignored control message, "
                     "as it was not targeted for this node",
                     __func__);
        return;
    }

    // Init node state
    reset_node();

    // Activate or deactivate node based on the message
    if (msg.active != this->get_active()) {
        if (msg.active) {
            this->activate();
        } else {
            this->deactivate();
        }
    }

    // If the node should be deactivated, don't parse the payload as it is only
    // relevant in active mode
    if (!this->get_active()) {
        return;
    }

    // Parse payload
    nlohmann::json cmd_json;
    try {
        cmd_json = common_lib::CommandDefinitions::parse_check_json_str(
            msg.payload,
            common_lib::CommandDefinitions::get_waypoint_command_definition());
    } catch (const std::runtime_error &e) {
        RCLCPP_FATAL(this->get_logger(),
                     "WaypointNode::%s: Received invalid json as "
                     "payload: %s",
                     __func__, e.what());
        this->job_finished(
            (std::string) "WaypointNode::" + (std::string) __func__ +
            ": Received "
            "invalid json as payload: " +
            e.what());
        reset_node();
        return;
    }

    // Set flag to true to indicate that the object has values in it
    cmd.values_set = true;

    // Required parameters
    cmd.target_coordinate_lat = cmd_json.at("target_coordinate_lat");
    cmd.target_coordinate_lon = cmd_json.at("target_coordinate_lon");
    cmd.cruise_height_cm = cmd_json.at("cruise_height_cm");
    cmd.target_height_cm = cmd_json.at("target_height_cm");
    cmd.horizontal_speed_mps = cmd_json.at("horizontal_speed_mps");
    cmd.vertical_speed_mps = cmd_json.at("vertical_speed_mps");

    // Optional parameters
    if (cmd_json.contains("pre_wait_time_ms"))
        cmd.pre_wait_time_ms = cmd_json.at("pre_wait_time_ms");

    if (cmd_json.contains("post_wait_time_ms"))
        cmd.post_wait_time_ms = cmd_json.at("post_wait_time_ms");
}

/**
 * @brief Callback function for receiving GPS position messages.
 *
 * This function is called when a GPS position message is received. It extracts
 * the latitude, longitude, and height information from the message and updates
 * the corresponding variables in the `pos` object.
 *
 * @param msg The GPS position message containing the latitude, longitude, and
 * height information.
 */
void WaypointNode::callback_position(const interfaces::msg::GPSPosition &msg) {
    // Store current timestamp for later
    rclcpp::Time timestamp_now = this->now();

    RCLCPP_DEBUG(this->get_logger(),
                 "WaypointNode::%s: Received position from "
                 "'%s': lat: %f, lon: %f, height: %f",
                 __func__, msg.sender_id.c_str(), msg.latitude_deg,
                 msg.longitude_deg, msg.relative_altitude_m);

    // Check timestamp
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
            max_position_msg_time_difference_ms))) {
        // Message is too old, react based on the current 'active' state

        // Reset pos as we can no longer reliabely know where we are
        pos = Position();

        if (this->get_active()) {
            // Abort job if currently active

            RCLCPP_FATAL(get_logger(),
                         "WaypointNode::%s: Received too old "
                         "timestamp in position message: %s. Aborting Job.",
                         __func__, msg.sender_id.c_str());
            this->job_finished("WaypointNode::" + (std::string) __func__ +
                               ": A too old timestamp was received in a "
                               "position message while "
                               "being active");
            reset_node();
        } else {
            // Warn but still store values

            RCLCPP_WARN(get_logger(),
                        "WaypointNode::%s: Received too old "
                        "timestamp in position message: %s",
                        __func__, msg.sender_id.c_str());
        }
    }

    // Store values
    pos.values_set = true;
    pos.coordinate_lat = msg.latitude_deg;
    pos.coordinate_lon = msg.longitude_deg;
    pos.height_cm = msg.relative_altitude_m * 100.0;
}

/**
 * @brief Callback function for receiving mission progress messages.
 *
 * This function is called when a mission progress message is received. It
 * updates the mission progress of the WaypointNode based on the received
 * message.
 *
 * @param msg The mission progress message received.
 */
void WaypointNode::callback_mission_progress(
    const interfaces::msg::MissionProgress &msg) {
    // Store current timestamp for later
    rclcpp::Time timestamp_now = this->now();

    // Ignore message if node is not active
    if (!this->get_active()) return;

    RCLCPP_DEBUG(this->get_logger(),
                 "WaypointNode::%s: Received mission "
                 "progress from '%s': progress: %f / 1.0",
                 __func__, msg.sender_id.c_str(), msg.progress);

    // Check timestamp
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(
            max_progress_msg_time_difference_ms))) {
        // Warn
        RCLCPP_WARN(
            get_logger(),
            "WaypointNode::%s: Received too "
            "old timestamp in progress message: %s. Using value regardless.",
            __func__, msg.sender_id.c_str());
    }

    // Store value
    mission_progress = msg.progress;
}

/**
 * @brief Callback function for handling wait time completion.
 *
 * This function is called when the wait time for a waypoint is completed.
 * It deactivates the timer, checks if the mission was aborted during the wait
 * time, and reacts according to the current state of the node.
 *
 * @note If the node has an incorrect state, a fatal error is logged and a
 * job_finished error message is sent.
 */
void WaypointNode::callback_wait_time() {
    // Deactivate Timer
    wait_timer->cancel();

    // Check if the mission was aborted during the wait time
    if (!this->get_active()) return;

    // React according to state
    switch (get_node_state()) {
        case pre_wait_time:
            set_node_state(reach_cruise_height);
            break;
        case post_wait_time:
            // Job finished successfully: Reset node for next command
            this->job_finished();
            reset_node();
            break;
        default:
            RCLCPP_FATAL(this->get_logger(),
                         "WaypointNode::%s: Node has incorrect "
                         "state: %d",
                         __func__, get_node_state());
            this->job_finished(
                "WaypointNode::" + (std::string) __func__ +
                ": Node has incorrect state: " +
                std::to_string(
                    get_node_state()));  // TODO ouput node state as string

            reset_node();
            return;
    }
}
