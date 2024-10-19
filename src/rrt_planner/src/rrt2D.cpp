#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"
#include "TreeNode.h"

class RRT2DNode : public rclcpp::Node
{
public:
    RRT2DNode()
        : Node("RRT2DNode"), start_x(0), start_y(0), goal_x(5), goal_y(5)
}