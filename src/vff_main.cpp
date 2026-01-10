#include "vff_avoidance/avoidance_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<vff_avoidance_node::VffAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}