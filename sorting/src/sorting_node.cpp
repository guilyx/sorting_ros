#include "sorting.hpp"

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sorting_topic");
    rclcpp::spin(std::make_shared<Sorting>());
    rclcpp::shutdown();
    return 0;
}