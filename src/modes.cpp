#include "main.hpp"

/**
 * @brief Initializes the mode of the WaypointNode.
 *
 * This function is responsible for initializing the mode of the WaypointNode.
 * It checks if a command is specified and performs the necessary actions based
 * on the command. If a pre_wait_time is specified, it sets the node state to
 * pre_wait_time and initializes a wait timer. If no pre_wait_time is specified,
 * it skips the pre_wait_time state and sets the node state to
 * reach_cruise_height.
 */
void WaypointNode::mode_init() {
    if (get_state_first_loop()) {
        // Check if cmd is specified
        if (!cmd.values_set) {
            RCLCPP_FATAL(this->get_logger(),
                         "WaypointNode::mode_init: Node was "
                         "activated without specifying a command");
            this->job_finished(
                "WaypointNode::mode_init: Node was activated without "
                "specifying a command");
            reset_node();
            return;
        }

        // Check if pos is specified
        if (!pos.values_set) {
            RCLCPP_FATAL(this->get_logger(),
                         "WaypointNode::mode_init: Node was "
                         "activated without receiving a position");
            this->job_finished(
                "WaypointNode::mode_init: Node was activated without "
                "receiving a position");
            reset_node();
            return;
        }

        if (cmd.pre_wait_time_ms > 0) {
            // Set state to pre_wait_time
            set_node_state(pre_wait_time);

            // Initialize Wait Timer
            wait_timer = this->create_wall_timer(
                std::chrono::milliseconds(cmd.pre_wait_time_ms),
                std::bind(&WaypointNode::callback_wait_time, this));
        } else {
            // Skipping pre_wait_time state as it wasn't specified
            RCLCPP_DEBUG(
                this->get_logger(),
                "WaypointNode::mode_init: Skipping 'pre_wait_time' state "
                "because no wait time was specified");
            set_node_state(reach_cruise_height);
        }
    }
}

/**
 * @brief Executes the reach cruise height mode of the WaypointNode.
 *
 * This function is responsible for reaching the cruise height specified in the
 * command. It sends a waypoint command with the new height to the UAV and
 * checks if the drone has arrived at the cruising height. If the drone has
 * arrived, it sets the node state to fly_to_waypoint.
 */
void WaypointNode::mode_reach_cruise_height() {
    if (get_state_first_loop()) {
        // Check that command and position are specified
        if (!check_cmd("mode_reach_cruise_height")) {
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "WaypointNode::mode_reach_cruise_height: Started reaching "
                    "cruise height");

        // Send waypoint command with new height and current position
        interfaces::msg::Waypoint waypoint_msg;
        waypoint_msg.latitude_deg = pos.coordinate_lat;
        waypoint_msg.longitude_deg = pos.coordinate_lon;
        waypoint_msg.relative_altitude_m = cmd.cruise_height_cm / 100.0;

        interfaces::msg::UAVWaypointCommand msg;
        msg.sender_id = this->get_name();
        msg.time_stamp = this->now();
        msg.speed_m_s = cmd.vertical_speed_mps;
        msg.waypoint = waypoint_msg;

        uav_waypoint_command_publisher->publish(msg);

        // Make sure that no old progress makes this function think it already
        // finished the mission
        mission_progress = 0.0;
    }

    if (current_mission_finished()) {
        // Drone arrived at crusing height
        RCLCPP_INFO(this->get_logger(),
                    "WaypointNode::mode_reach_cruise_height: Arrived at "
                    "crusing height");
        set_node_state(fly_to_waypoint);
    }
}

void WaypointNode::mode_fly_to_waypoint() {
    if (get_state_first_loop()) {
        // Check that command and position are specified
        if (!check_cmd("mode_fly_to_waypoint")) {
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "WaypointNode::mode_fly_to_waypoint: Started flying to waypoint");

        // Send waypoint command
        interfaces::msg::Waypoint waypoint_msg;
        waypoint_msg.latitude_deg = cmd.target_coordinate_lat;
        waypoint_msg.longitude_deg = cmd.target_coordinate_lon;
        waypoint_msg.relative_altitude_m = cmd.cruise_height_cm / 100.0;

        interfaces::msg::UAVWaypointCommand msg;
        msg.sender_id = this->get_name();
        msg.time_stamp = this->now();
        msg.speed_m_s = cmd.horizontal_speed_mps;
        msg.waypoint = waypoint_msg;

        uav_waypoint_command_publisher->publish(msg);

        // Make sure that no old progress makes this function think it already
        // finished the mission
        mission_progress = 0.0;
    }

    if (current_mission_finished()) {
        // Drone arrived at waypoint
        RCLCPP_INFO(this->get_logger(),
                    "WaypointNode::mode_fly_to_waypoint: Arrived at waypoint");
        set_node_state(reach_target_height);
    }
}

/**
 * @brief Executes the "mode_reach_target_height" mode of the WaypointNode.
 *
 * This function is responsible for reaching the target height specified in the
 * command. It sends a waypoint command with the new height to the UAV and waits
 * until the target height is reached. Once the target height is reached, it
 * either transitions to the "post_wait_time" state or finishes the job,
 * depending on whether a post wait time was specified in the command.
 */
void WaypointNode::mode_reach_target_height() {
    if (get_state_first_loop()) {
        // Check that command and position are specified
        if (!check_cmd("mode_reach_target_height")) {
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "WaypointNode::mode_reach_target_height: Started reaching "
                    "target height");

        // Send waypoint command with new height and current position
        interfaces::msg::Waypoint waypoint_msg;
        waypoint_msg.latitude_deg = pos.coordinate_lat;
        waypoint_msg.longitude_deg = pos.coordinate_lon;
        waypoint_msg.relative_altitude_m = cmd.target_height_cm / 100.0;

        interfaces::msg::UAVWaypointCommand msg;
        msg.sender_id = this->get_name();
        msg.time_stamp = this->now();
        msg.speed_m_s = cmd.vertical_speed_mps;
        msg.waypoint = waypoint_msg;

        uav_waypoint_command_publisher->publish(msg);

        // Make sure that no old progress makes this function think it already
        // finished the mission
        mission_progress = 0.0;
    }

    if (current_mission_finished()) {
        // Reached target height
        RCLCPP_INFO(
            this->get_logger(),
            "WaypointNode::mode_reach_target_height: Arrived at target height");

        if (cmd.post_wait_time_ms > 0) {
            // Set state to post_wait_time
            set_node_state(post_wait_time);

            // Initialize Wait Timer
            wait_timer = this->create_wall_timer(
                std::chrono::milliseconds(cmd.post_wait_time_ms),
                std::bind(&WaypointNode::callback_wait_time, this));
        } else {
            // Skipping post_wait_time state as it wasn't specified
            RCLCPP_DEBUG(this->get_logger(),
                         "WaypointNode::mode_reach_target_height: Skipping "
                         "'post_wait_time' "
                         "state because no wait time was specified");
            this->job_finished();
            reset_node();
        }
    }
}
