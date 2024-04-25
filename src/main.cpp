#include "main.hpp"

using std::placeholders::_1;

WaypointNode::WaypointNode() : CommonNode("waypoint_node")
{
    control_subscription = this->create_subscription<interfaces::msg::Control>(
        "control", 10, std::bind(&WaypointNode::callback_control, this, _1));

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
 * @brief Executes the event loop for the WaypointNode.
 *
 * This function is responsible for executing the event loop of the WaypointNode
 * class. It checks the node state and performs the corresponding actions based
 * on the state. The possible states are:
 * - init: Initializes the mode.
 * - pre_wait_time: Does nothing.
 * - fly_to_waypoint: Executes the mode to fly to the waypoint.
 * - post_wait_time: Does nothing.
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
    case fly_to_waypoint:
        mode_fly_to_waypoint();
        break;
    case post_wait_time:
        break; // Do nothing
    default:
        RCLCPP_ERROR(this->get_logger(),
                     "WaypointNode::event_loop: Unknown mission_state: %d",
                     get_node_state());
        this->job_finished("WaypointNode::event_loop: Unknown mission_state");
    }
}

/**
 * @brief Resets the node for the next command.
 *
 * This function resets the state of the `WaypointNode` object for the next
 * command. It sets the node state to `init`, sets the `state_first_loop` flag
 * to `true`, and clears the `cmd` object.
 */
void WaypointNode::reset_node()
{
    set_node_state(init);
    state_first_loop = true;
    cmd = Command();
}

/**
 * @brief Sets the node state to a new value.
 *
 * This function sets the node state to the specified new value. If the new
 * value is different from the current node state, the `state_first_loop` flag
 * is set to true. After updating the node state, a debug message is logged.
 *
 * @param new_mission_state The new mission state to set.
 */
void WaypointNode::set_node_state(NodeState_t new_state)
{
    if (node_state != new_state)
    {
        state_first_loop = true;
    }

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
    if (msg.target_id != this->get_fully_qualified_name())
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
        set_node_state(fly_to_waypoint);
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
 * it skips the pre_wait_time state and sets the node state to fly_to_waypoint.
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
                         "WaypointNode::mode_init: Skipping pre_wait_time state "
                         "because no wait time was specified");
            set_node_state(fly_to_waypoint);
        }
    }
}

void WaypointNode::mode_fly_to_waypoint()
{
    if (get_state_first_loop())
    {
        // TODO send waypoint command
    }

    // TODO monitor flight path

    // If drone arrived at waypoint
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
                     "WaypointNode::mode_fly_to_waypoint: Skipping post_wait_time "
                     "state because no wait time was specified");
        this->job_finished();
        reset_node();
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNode>());
    rclcpp::shutdown();
    return 0;
}
