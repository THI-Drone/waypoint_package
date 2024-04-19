#include "main.hpp"

/**
 * @brief A class that represents a minimal node.
 */
class MinimalPublisher : public common_lib::CommonNode
{
public:
    /**
     * @brief Constructs a new MinimalPublisher object.
     *
     * @param id The unique name for the node.
     */
    MinimalPublisher(char *id) : CommonNode(id)
    {
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>(argv[1]));
    rclcpp::shutdown();
    return 0;
}
