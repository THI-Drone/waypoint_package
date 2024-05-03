#include "main.hpp"

using std::placeholders::_1;

WaypointNode::WaypointNode() : CommonNode("waypoint_node")
{
    // Create a subscription for the "control" topic
    control_subscription = this->create_subscription<interfaces::msg::Control>(
        "control", 10, std::bind(&WaypointNode::callback_control, this, _1));

    // Create a subscription for the "uav_gps_position" topic
    gps_position_subscription = this->create_subscription<interfaces::msg::GPSPosition>(
        "uav_gps_position", 10, std::bind(&WaypointNode::callback_position, this, _1));

    // Create a subscription for the "uav_mission_progress" topic
    mission_progress_subscription = this->create_subscription<interfaces::msg::MissionProgress>(
        "uav_mission_progress", 10, std::bind(&WaypointNode::callback_mission_progress, this, _1));

    // Create a publisher for the "uav_waypoint_command" topic
    uav_waypoint_command_publisher = this->create_publisher<interfaces::msg::UAVWaypointCommand>("uav_waypoint_command", 10);

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
bool WaypointNode::get_state_first_loop()
{
    // Check if first loop was already triggered
    if (!state_first_loop)
        return false;

    // Set first loop to false and return true
    state_first_loop = false;
    return true;
}

/**
 * Checks if the current mission is finished.
 *
 * @return true if the mission is finished, false otherwise.
 */
bool WaypointNode::current_mission_finished()
{
    if (mission_progress >= 1.0)
    {
        mission_progress = 0.0;
        return true;
    }
    else
    {
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
void WaypointNode::event_loop()
{
    if (!this->get_active())
        return;

    switch (get_node_state())
    {
    case init:
        mode_init();
        break;
    case pre_wait_time:
        break; // Do nothing
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
        break; // Do nothing
    default:
        RCLCPP_ERROR(this->get_logger(),
                     "WaypointNode::event_loop: Unknown mission_state: %d",
                     get_node_state());
        this->job_finished("WaypointNode::event_loop: Unknown mission_state");
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
void WaypointNode::reset_node()
{
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
void WaypointNode::set_node_state(NodeState_t new_state)
{
    if (node_state != new_state)
    {
        state_first_loop = true;
    }

    mission_progress = 0.0;
    node_state = new_state;
    RCLCPP_DEBUG(this->get_logger(),
                 "WaypointNode::set_node_state: Set node state to %d",
                 node_state);
}

/**
 * @brief Callback function for the control message.
 *
 * This function is called when a control message is received. It processes the
 * message, activates the node based on the message, parses the payload, and
 * saves the data in a Command struct.
 *
 * @param msg The control message received.
 */
void WaypointNode::callback_control(const interfaces::msg::Control &msg)
{
    // Ignore messages that don't target this node
    if (msg.target_id != this->get_name())
    {
        RCLCPP_DEBUG(this->get_logger(),
                     "WaypointNode::callback_control: Ignored control message not "
                     "targeted for this node");
        return;
    }

    // Init node state
    reset_node();

    // Activate or deactivate node based on the message
    if (msg.active != this->get_active())
    {
        if (msg.active)
        {
            this->activate();
        }
        else
        {
            this->deactivate();
        }
    }

    // If the node should be deactivated, don't parse the payload as it is only
    // relevant in active mode
    if (!this->get_active())
    {
        return;
    }

    nlohmann::json cmd_json;
    try
    {
        cmd_json = common_lib::CommandDefinitions::parse_check_json_str(
            msg.payload,
            common_lib::CommandDefinitions::get_waypoint_command_definition());
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_FATAL(
            this->get_logger(),
            "WaypointNode::callback_control: Received invalid json as payload: %s",
            e.what());
        this->job_finished((std::string) "WaypointNode::callback_control: Received "
                                         "invalid json as payload: " +
                           e.what());
        reset_node();
        return;
    }

    // Set flag to true to indicate that the object has real values in it
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
 * This function is called when a GPS position message is received. It extracts the latitude, longitude, and height
 * information from the message and updates the corresponding variables in the `pos` object.
 *
 * @param msg The GPS position message containing the latitude, longitude, and height information.
 */
void WaypointNode::callback_position(const interfaces::msg::GPSPosition &msg)
{
    RCLCPP_DEBUG(
        this->get_logger(),
        "WaypointNode::callback_position: Received position from '%s': lat: %f, lon: %f, height: %f", msg.sender_id.c_str(), msg.latitude_deg, msg.longitude_deg, msg.relative_altitude_m);

    // Check timestamp
    rclcpp::Time timestamp_now = this->now();
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(max_position_msg_time_difference_ms)))
    {
        // Reset pos as we can no longer reliabely know where we are
        pos = Position();

        if (this->get_active())
        {
            RCLCPP_FATAL(
                get_logger(),
                "MissionControl::callback_position: Received too old timestamp in position message: %s. Aborting Job.",
                msg.sender_id.c_str());
            this->job_finished("A too old timestamp was received in a position message while being active");
            reset_node();
        }
        else
        {
            // Warn but still store values
            RCLCPP_WARN(
                get_logger(),
                "MissionControl::callback_position: Received too old timestamp in position message: %s",
                msg.sender_id.c_str());
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
 * This function is called when a mission progress message is received. It updates the mission progress
 * of the WaypointNode based on the received message.
 *
 * @param msg The mission progress message received.
 */
void WaypointNode::callback_mission_progress(const interfaces::msg::MissionProgress &msg)
{
    // Ignore message if node is not active
    if (!this->get_active())
        return;

    RCLCPP_DEBUG(
        this->get_logger(),
        "WaypointNode::callback_mission_progress: Received mission progress from '%s': progress: %f / 1.0", msg.sender_id.c_str(), msg.progress);

    // Check timestamp
    rclcpp::Time timestamp_now = this->now();
    if (timestamp_now - rclcpp::Time(msg.time_stamp) >
        rclcpp::Duration(std::chrono::duration<int64_t, std::milli>(max_progress_msg_time_difference_ms)))
    {
        // Warn and ignore message
        RCLCPP_WARN(
            get_logger(),
            "MissionControl::callback_mission_progress: Received too old timestamp in progress message: %s. Ignoring message.",
            msg.sender_id.c_str());

        return;
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
void WaypointNode::callback_wait_time()
{
    // Deactivate Timer
    wait_timer->cancel();

    // Check if the mission was aborted during the wait time
    if (!this->get_active())
        return;

    // React according to state
    switch (get_node_state())
    {
    case pre_wait_time:
        set_node_state(reach_cruise_height);
        break;
    case post_wait_time:
        // Job finished successfully: Reset node for next command
        this->job_finished();
        reset_node();
        break;
    default:
        RCLCPP_FATAL(
            this->get_logger(),
            "WaypointNode::callback_wait_time: Node has incorrect state: %d",
            get_node_state());
        this->job_finished(
            "WaypointNode::callback_wait_time: Node has incorrect state: " +
            get_node_state());
        reset_node();
        return;
    }
}

/**
 * @brief Initializes the mode of the WaypointNode.
 *
 * This function is responsible for initializing the mode of the WaypointNode.
 * It checks if a command is specified and performs the necessary actions based
 * on the command. If a pre_wait_time is specified, it sets the node state to
 * pre_wait_time and initializes a wait timer. If no pre_wait_time is specified,
 * it skips the pre_wait_time state and sets the node state to reach_cruise_height.
 */
void WaypointNode::mode_init()
{
    if (get_state_first_loop())
    {
        // Check if cmd is specified
        if (!cmd.values_set)
        {
            RCLCPP_FATAL(this->get_logger(), "WaypointNode::mode_init: Node was "
                                             "activated without specifing a command");
            this->job_finished("WaypointNode::mode_init: Node was activated without "
                               "specifing a command");
            reset_node();
            return;
        }

        // Check if pos is specified
        if (!pos.values_set)
        {
            RCLCPP_FATAL(this->get_logger(), "WaypointNode::mode_init: Node was "
                                             "activated without receiving a position before");
            this->job_finished("WaypointNode::mode_init: Node was activated without "
                               "receiving a position before");
            reset_node();
            return;
        }

        if (cmd.pre_wait_time_ms > 0)
        {
            // Set state to pre_wait_time
            set_node_state(pre_wait_time);

            // Initialize Wait Timer
            wait_timer = this->create_wall_timer(
                std::chrono::milliseconds(cmd.pre_wait_time_ms),
                std::bind(&WaypointNode::callback_wait_time, this));
        }
        else
        {
            // Skipping pre_wait_time state as it wasn't specified
            RCLCPP_DEBUG(this->get_logger(),
                         "WaypointNode::mode_init: Skipping 'pre_wait_time' state "
                         "because no wait time was specified");
            set_node_state(reach_cruise_height);
        }
    }
}

/**
 * @brief Executes the reach cruise height mode of the WaypointNode.
 *
 * This function is responsible for reaching the cruise height specified in the command.
 * It sends a waypoint command with the new height to the UAV and checks if the drone has arrived at the cruising height.
 * If the drone has arrived, it sets the node state to fly_to_waypoint.
 */
void WaypointNode::mode_reach_cruise_height()
{
    if (get_state_first_loop())
    {
        // Check that command and position are specified
        if (!check_cmd("mode_reach_cruise_height"))
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "WaypointNode::mode_reach_cruise_height: Started reaching cruise height");

        // Send waypoint command with new height and current position
        interfaces::msg::Waypoint waypoint_msg;
        waypoint_msg.latitude_deg = pos.coordinate_lat;
        waypoint_msg.longitude_deg = pos.coordinate_lon;
        waypoint_msg.relative_altitude_m = cmd.cruise_height_cm;

        interfaces::msg::UAVWaypointCommand msg;
        msg.sender_id = this->get_name();
        msg.time_stamp = this->now();
        msg.speed_m_s = cmd.vertical_speed_mps;
        msg.waypoint = waypoint_msg;

        uav_waypoint_command_publisher->publish(msg);

        // Make sure that no old progress makes this function think it already finished the mission
        mission_progress = 0.0;
    }

    if (current_mission_finished())
    {
        // Drone arrived at crusing height
        RCLCPP_INFO(this->get_logger(), "WaypointNode::mode_reach_cruise_height: Arrived at crusing height");
        set_node_state(fly_to_waypoint);
    }
}

void WaypointNode::mode_fly_to_waypoint()
{
    if (get_state_first_loop())
    {
        // Check that command and position are specified
        if (!check_cmd("mode_fly_to_waypoint"))
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "WaypointNode::mode_fly_to_waypoint: Started flying to waypoint");

        // Send waypoint command
        interfaces::msg::Waypoint waypoint_msg;
        waypoint_msg.latitude_deg = cmd.target_coordinate_lat;
        waypoint_msg.longitude_deg = cmd.target_coordinate_lon;
        waypoint_msg.relative_altitude_m = cmd.cruise_height_cm;

        interfaces::msg::UAVWaypointCommand msg;
        msg.sender_id = this->get_name();
        msg.time_stamp = this->now();
        msg.speed_m_s = cmd.horizontal_speed_mps;
        msg.waypoint = waypoint_msg;

        uav_waypoint_command_publisher->publish(msg);

        // Make sure that no old progress makes this function think it already finished the mission
        mission_progress = 0.0;
    }

    if (current_mission_finished())
    {
        // Drone arrived at waypoint
        RCLCPP_INFO(this->get_logger(), "WaypointNode::mode_fly_to_waypoint: Arrived at waypoint");
        set_node_state(reach_target_height);
    }
}

/**
 * @brief Executes the "mode_reach_target_height" mode of the WaypointNode.
 *
 * This function is responsible for reaching the target height specified in the command.
 * It sends a waypoint command with the new height to the UAV and waits until the target height is reached.
 * Once the target height is reached, it either transitions to the "post_wait_time" state or finishes the job,
 * depending on whether a post wait time was specified in the command.
 */
void WaypointNode::mode_reach_target_height()
{
    if (get_state_first_loop())
    {
        // Check that command and position are specified
        if (!check_cmd("mode_reach_target_height"))
        {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "WaypointNode::mode_reach_target_height: Started reaching target height");

        // Send waypoint command with new height and current position
        interfaces::msg::Waypoint waypoint_msg;
        waypoint_msg.latitude_deg = pos.coordinate_lat;
        waypoint_msg.longitude_deg = pos.coordinate_lon;
        waypoint_msg.relative_altitude_m = cmd.target_height_cm;

        interfaces::msg::UAVWaypointCommand msg;
        msg.sender_id = this->get_name();
        msg.time_stamp = this->now();
        msg.speed_m_s = cmd.vertical_speed_mps;
        msg.waypoint = waypoint_msg;

        uav_waypoint_command_publisher->publish(msg);

        // Make sure that no old progress makes this function think it already finished the mission
        mission_progress = 0.0;
    }

    if (current_mission_finished())
    {
        // Reached target height
        RCLCPP_INFO(this->get_logger(), "WaypointNode::mode_reach_target_height: Arrived at target height");

        if (cmd.post_wait_time_ms > 0)
        {
            // Set state to post_wait_time
            set_node_state(post_wait_time);

            // Initialize Wait Timer
            wait_timer = this->create_wall_timer(
                std::chrono::milliseconds(cmd.post_wait_time_ms),
                std::bind(&WaypointNode::callback_wait_time, this));
        }
        else
        {
            // Skipping post_wait_time state as it wasn't specified
            RCLCPP_DEBUG(this->get_logger(),
                         "WaypointNode::mode_reach_target_height: Skipping 'post_wait_time' "
                         "state because no wait time was specified");
            this->job_finished();
            reset_node();
        }
    }
}

/**
 * @brief Checks if the command and position values are set.
 *
 * This function is used to check if the command and position values are set before executing a command.
 * If either the command or position values are not set, the function logs an error message and returns false.
 *
 * @param function_name The name of the calling function.
 * @return True if both the command and position values are set, false otherwise.
 */
bool WaypointNode::check_cmd(const char *function_name)
{
    if (!cmd.values_set)
    {
        RCLCPP_FATAL(this->get_logger(), "WaypointNode::%s::check_cmd: No command specified", function_name);
        this->job_finished("WaypointNode::" + (std::string)function_name + "::check_cmd: No command specified");
        reset_node();
        return false;
    }

    if (!pos.values_set)
    {
        RCLCPP_FATAL(this->get_logger(), "WaypointNode::%s::check_cmd: No position received before", function_name);
        this->job_finished("WaypointNode::" + (std::string)function_name + "::check_cmd: No position received before");
        reset_node();
        return false;
    }

    return true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNode>());
    rclcpp::shutdown();
    return 0;
}
