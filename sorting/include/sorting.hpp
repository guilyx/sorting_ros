#pragma once

/**
 * Pragma once replaces the #ifndef XXXX
 *                          #define XXXX
 *                          #endif
 *
 */


/**
 * Some include might be overkill, remove them if so
 */
#include <functional>
#include <memory>
#include <chrono>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sorting_msgs/srv/toggle_sorting.hpp"

class Sorting : public rclcpp::Node
{
public:
  Sorting();

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hello_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr sorting_pub_;
  rclcpp::Service<sorting_msgs::srv::ToggleSorting>::SharedPtr enable_sorting_srv_;

  void hello_callback_(const std_msgs::msg::String::SharedPtr input_str);
  void toggle_publisher_callback_(const std::shared_ptr<sorting_msgs::srv::ToggleSorting::Request>, std::shared_ptr<sorting_msgs::srv::ToggleSorting::Response> response);

  bool sorting_active_ = false;
};

