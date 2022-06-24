#include "sorting.hpp"

Sorting::Sorting() : Node("sorting_topic")
{
    hello_sub_ = this->create_subscription<std_msgs::msg::String>("hello", 10, std::bind(&Sorting::hello_callback_, this, std::placeholders::_1));
    sorting_pub_ = this->create_publisher<std_msgs::msg::String>("sorted", 10);
    enable_sorting_srv_ = this->create_service<sorting_msgs::srv::ToggleSorting>("enable_disable", std::bind(&Sorting::toggle_publisher_callback_, this, std::placeholders::_1, std::placeholders::_2));
}

void Sorting::hello_callback_(const std_msgs::msg::String::SharedPtr input_str)
{
    RCLCPP_INFO(this->get_logger(), "Receiving: '%s'", input_str->data.c_str());
    auto message = std_msgs::msg::String();
    std::sort(input_str->data.begin(), input_str->data.end());
    message.data = input_str->data;
    if (sorting_active_)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        sorting_pub_->publish(message);
    }
}

void Sorting::toggle_publisher_callback_(const std::shared_ptr<sorting_msgs::srv::ToggleSorting::Request>, std::shared_ptr<sorting_msgs::srv::ToggleSorting::Response> response)
{
    sorting_active_ = !sorting_active_;
    response->result = sorting_active_;
}