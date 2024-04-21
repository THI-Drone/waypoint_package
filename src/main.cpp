#include "main.hpp"

using std::placeholders::_1;

WaypointNode::WaypointNode() : CommonNode("waypoint_node")
{
    control_subscription = this->create_subscription<interfaces::msg::Control>(
        "control", 10, std::bind(&WaypointNode::callback_control, this, _1));

    job_finished_publisher = this->create_publisher<interfaces::msg::JobFinished>("job_finished", 10);
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

    // Activate or deactivate node based on the message
    if (msg.active != this->get_active())
    {
        if (msg.active)
            this->activate();
        else
            this->deactivate();
    }

    commands.clear(); // delete all previous commands

    // If the node should be deactivated, don't parse the payload as it only relevant in active mode
    if (!this->get_active())
        return;

    // Parse payload, save data in command struct and append to commands vector
    nlohmann::json payload;

    try
    {
        payload = nlohmann::json::parse(msg.payload);
    }
    catch (const nlohmann::json::parse_error &e)
    {
        RCLCPP_FATAL(this->get_logger(), "WaypointNode::callback_control: Payload could not be parsed");
        this->job_finished("WaypointNode::callback_control: Payload could not be parsed");
        return;
    }

    // Separate payload if it is an array
    std::vector<nlohmann::json> payloads;
    if (payload.is_array())
    {
        for (const auto &p : payload)
        {
            payloads.push_back(p);
        }
    }
    else
    {
        payloads.push_back(payload);
    }

    // Create a struct command from every payload and save it in the commands vector
    for (const auto &p : payloads)
    {
        command cmd;

        // 'post_wait_time_ms' is optional
        if (p.contains("post_wait_time_ms"))
            cmd.post_wait_time_ms = p["post_wait_time_ms"];

        // 'pre_wait_time_ms' is optional
        if (p.contains("pre_wait_time_ms"))
            cmd.pre_wait_time_ms = p["pre_wait_time_ms"];

        // 'target_coordinate' is required
        if (!p.contains("target_coordinate"))
        {
            RCLCPP_FATAL(this->get_logger(), "WaypointNode::callback_control: Payload does not include required field 'target_coordinate'");
            this->job_finished("WaypointNode::callback_control: Payload does not include required field 'target_coordinate'");
            return;
        }
        cmd.target_coordinate = p["target_coordinate"];

        // 'travel_height' is required
        if (!p.contains("travel_height"))
        {
            RCLCPP_FATAL(this->get_logger(), "WaypointNode::callback_control: Payload does not include required field 'travel_height'");
            this->job_finished("WaypointNode::callback_control: Payload does not include required field 'travel_height'");
            return;
        }
        cmd.travel_height = p["travel_height"];

        // 'target_height' is required
        if (!p.contains("target_height"))
        {
            RCLCPP_FATAL(this->get_logger(), "WaypointNode::callback_control: Payload does not include required field 'target_height'");
            this->job_finished("WaypointNode::callback_control: Payload does not include required field 'target_height'");
            return;
        }
        cmd.target_height = p["target_height"];

        // 'horizontal_speed' is required
        if (!p.contains("horizontal_speed"))
        {
            RCLCPP_FATAL(this->get_logger(), "WaypointNode::callback_control: Payload does not include required field 'horizontal_speed'");
            this->job_finished("WaypointNode::callback_control: Payload does not include required field 'horizontal_speed'");
            return;
        }
        cmd.horizontal_speed = p["horizontal_speed"];

        // 'vertical_speed' is required
        if (!p.contains("vertical_speed"))
        {
            RCLCPP_FATAL(this->get_logger(), "WaypointNode::callback_control: Payload does not include required field 'vertical_speed'");
            this->job_finished("WaypointNode::callback_control: Payload does not include required field 'vertical_speed'");
            return;
        }
        cmd.vertical_speed = p["vertical_speed"];

        // Add command to vector
        commands.push_back(cmd);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNode>());
    rclcpp::shutdown();
    return 0;
}
