#include <rclcpp/rclcpp.hpp>


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("minimal_node_cpp");

    RCLCPP_INFO(node->get_logger(), "Hello from %s", node->get_name());

    rclcpp::shutdown();
}