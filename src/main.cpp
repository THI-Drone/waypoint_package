#include "main.hpp"

/**
 * @brief A class that represents a minimal node.
 */
class WaypointNode : public common_lib::CommonNode
{
public:
    /**
     * @brief Constructs a new WaypointNode object.
     *
     * @param id The unique name for the node.
     */
    WaypointNode() : CommonNode("waypoint_node")
    {
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointNode>());
    rclcpp::shutdown();
    return 0;
}
