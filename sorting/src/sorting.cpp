#include "sorting.hpp"

Sorting::Sorting() : Node("sorting_topic")
{
    hello_sub_ = this->create_subscription<std_msgs::msg::String>("hello", 10, std::bind(&Sorting::hello_callback_, this, std::placeholders::_1));
    sorting_pub_ = this->create_publisher<std_msgs::msg::String>("sorted", 10);
}

void Sorting::hello_callback_(const std_msgs::msg::String::SharedPtr input_str)
{
    RCLCPP_INFO(this->get_logger(), "Receiving: '%s'", input_str->data.c_str());
    auto message = std_msgs::msg::String();
    std::sort(input_str->data.begin(), input_str->data.end());
    message.data = input_str->data;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    sorting_pub_->publish(message);
}