
#include<rclcpp/rclcpp.hpp>

// timer
class AdvanceNode : public rclcpp::Node {
 public:
  AdvanceNode(std::string node_name) : Node(node_name) {
    rclcpp::get_logger("hello %s"%this->get_name());
  }
};