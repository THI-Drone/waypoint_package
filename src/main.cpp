#include "main.hpp"

using std::placeholders::_1;

WaypointNode::WaypointNode() : CommonNode("waypoint_node")
{
    control_subscription = this->create_subscription<interfaces::msg::Control>(
        "control", 10, std::bind(&WaypointNode::callback_control, this, _1));

    // Initialize Event Loop
    event_loop_timer = this->create_wall_timer(std::chrono::milliseconds(event_loop_time_delta_ms), std::bind(&WaypointNode::event_loop, this));

    // Initialize Wait Timer
    wait_timer = this->create_wall_timer(std::chrono::milliseconds(0), std::bind(&WaypointNode::callback_wait_time, this));
}

/**
 * @brief Checks if the first loop was already triggered and returns the result.
 *
 * This function checks the state of the first loop and returns false if it was already triggered,
 * otherwise it returns true. After returning the state, it sets the first loop state to false.
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

void WaypointNode::event_loop()
{
    if (!this->get_active())
        return;

    switch(get_node_state()){
        case init:
            mode_init();
            break;
        
        // TODO implement other cases

        default:
            RCLCPP_ERROR(this->get_logger(), "WaypointNode::event_loop: Unknown mission_state: %d", get_node_state());
            this->job_finished("WaypointNode::event_loop: Unknown mission_state");
    }
}

/**
 * @brief Sets the node state to a new value.
 *
 * This function sets the node state to the specified new value. If the new value is different from the current node state,
 * the `state_first_loop` flag is set to true. After updating the node state, a debug message is logged.
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
    RCLCPP_DEBUG(this->get_logger(), "WaypointNode::set_node_state: Set node state to %d", node_state);
}

/**
 * @brief Callback function for the control message.
 *
 * This function is called when a control message is received. It processes the message,
 * activates the node based on the message, parses the payload, and saves
 * the data in the command struct. It also appends the command to the commands vector.
 * Multiple waypoints can be sent at a time when using a JSON array.
 * If you only want to sent one waypoint, an array is not needed.
 *
 * @param msg The control message to be processed.
 */
void WaypointNode::callback_control(const interfaces::msg::Control &msg)
{
    // Ignore messages that don't target this node
    if (msg.target_id != this->get_fully_qualified_name())
    {
        RCLCPP_DEBUG(this->get_logger(), "WaypointNode::callback_control: Ignored control message not targeted for this node");
        return;
    }

    // Init node state
    set_node_state(init);
    state_first_loop = true;

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

    // If the node should be deactivated, don't parse the payload as it is only relevant in active mode
    if (!this->get_active())
    {
        cmd = command(); // delete previous command
        return;
    }

    nlohmann::json cmd_json;
    try
    {
        cmd_json = common_lib::CommandDefinitions::parse_check_json(msg.payload, common_lib::CommandDefinitions::get_waypoint_command_definition());
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_FATAL(this->get_logger(), "WaypointNode::callback_control: Received invalid json as payload: %s", e.what());
        this->job_finished((std::string) "WaypointNode::callback_control: Received invalid json as payload: " + e.what());
        return;
    }

    cmd = command(); // init with default values
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

void WaypointNode::callback_wait_time()
{
    // Check if the mission was aborted during the wait time
    if(!this->get_active()) return;
}

void WaypointNode::mode_init()
{
    if (get_state_first_loop())
    {
        // Check if cmd is specified
        if(!cmd.values_set)
        {
            RCLCPP_FATAL(this->get_logger(), "WaypointNode::event_loop: Node was activated without specifing a command");
            this->job_finished("WaypointNode::event_loop: Node was activated without specifing a command");
            return;
        }

        if(cmd.pre_wait_time_ms > 0)
            // TODO continue here
            wait_timer->reset(std::chrono::milliseconds(cmd.pre_wait_time_ms));
        else
            callback_wait_time();

        // TODO send waypoint command
    }

    // TODO check flight path
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNode>());
    rclcpp::shutdown();
    return 0;
}
