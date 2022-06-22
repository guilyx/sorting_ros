#include "gtest/gtest.h"
#include "sorting.hpp"
#include "cmr_tests_utils/single_thread_spinner.hpp"
#include "cmr_tests_utils/basic_subscriber_node_test.hpp"
#include "cmr_tests_utils/basic_publisher_node_test.hpp"
#include "cmr_tests_utils/basic_service_client_test.hpp"

TEST(SortingSubscriber, test_service_disable)
{
    rclcpp::init(0, nullptr);
    auto spinner = cmr_tests_utils::SingleThreadSpinner();
    auto sub_node = std::make_shared<cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::String>>("sub_test_node", "sorted");
    auto pub_node = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::String>>("pub_test_node", "hello", false, 10);
    auto node = std::make_shared<Sorting>();

    EXPECT_FALSE(sub_node->has_data_been_received());

    spinner.add_node(sub_node->get_node_base_interface());
    spinner.add_node(pub_node->get_node_base_interface());
    spinner.add_node(node->get_node_base_interface());

    std_msgs::msg::String msg;
    msg.data = "nbvcxwmlkjhgfdsqpoiuytreza";
    pub_node->publish(msg);

    EXPECT_FALSE(sub_node->has_data_been_received());
    spinner.spin_some_all_nodes();

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_FALSE(sub_node->has_data_been_received());

    rclcpp::shutdown();
}

TEST(SortingSubscriber, test_service_enable_nothing_publish)
{
    rclcpp::init(0, nullptr);
    auto spinner = cmr_tests_utils::SingleThreadSpinner();
    auto sub_node = std::make_shared<cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::String>>("sub_test_node", "sorted");
    auto node = std::make_shared<Sorting>();

    auto service_client = std::make_shared<cmr_tests_utils::BasicServiceClientTest<sorting_msgs::srv::ToggleSorting>>("update_service_client", "enable_disable");

    EXPECT_FALSE(sub_node->has_data_been_received());

    spinner.add_node(sub_node->get_node_base_interface());
    spinner.add_node(node->get_node_base_interface());

    EXPECT_FALSE(sub_node->has_data_been_received());
    spinner.spin_some_all_nodes();

    EXPECT_TRUE(service_client->is_server_ready());

    rclcpp::shutdown();
}

TEST(SortingSubscriber, test_service_enable)
{
    rclcpp::init(0, nullptr);
    auto spinner = cmr_tests_utils::SingleThreadSpinner();
    auto sub_node = std::make_shared<cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::String>>("sub_test_node", "sorted");
    auto pub_node = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::String>>("pub_test_node", "hello", false, 10);
    auto node = std::make_shared<Sorting>();

    auto service_client = std::make_shared<cmr_tests_utils::BasicServiceClientTest<sorting_msgs::srv::ToggleSorting>>("update_service_client", "enable_disable");

    EXPECT_FALSE(sub_node->has_data_been_received());

    spinner.add_node(sub_node->get_node_base_interface());
    spinner.add_node(pub_node->get_node_base_interface());
    spinner.add_node(node->get_node_base_interface());

    std_msgs::msg::String msg;
    msg.data = "nbvcxwmlkjhgfdsqpoiuytreza";
    pub_node->publish(msg);

    EXPECT_FALSE(sub_node->has_data_been_received());
    spinner.spin_some_all_nodes();

    EXPECT_TRUE(service_client->is_server_ready());

    auto result_update = service_client->send_request(std::make_shared<sorting_msgs::srv::ToggleSorting::Request>());
    EXPECT_TRUE(result_update->result);

    pub_node->publish(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_EQ(sub_node->get_received_msg().data, "abcdefghijklmnopqrstuvwxyz");

    msg.data = "aeyuio";
    pub_node->publish(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_EQ(sub_node->get_received_msg().data, "aeiouy");

    rclcpp::shutdown();
}

TEST(SortingSubscriber, test_service_become_disable)
{
    rclcpp::init(0, nullptr);
    auto spinner = cmr_tests_utils::SingleThreadSpinner();
    auto sub_node = std::make_shared<cmr_tests_utils::BasicSubscriberNodeTest<std_msgs::msg::String>>("sub_test_node", "sorted");
    auto pub_node = std::make_shared<cmr_tests_utils::BasicPublisherNodeTest<std_msgs::msg::String>>("pub_test_node", "hello", false, 10);
    auto node = std::make_shared<Sorting>();

    auto service_client = std::make_shared<cmr_tests_utils::BasicServiceClientTest<sorting_msgs::srv::ToggleSorting>>("update_service_client", "enable_disable");

    EXPECT_FALSE(sub_node->has_data_been_received());

    spinner.add_node(sub_node->get_node_base_interface());
    spinner.add_node(pub_node->get_node_base_interface());
    spinner.add_node(node->get_node_base_interface());

    std_msgs::msg::String msg;
    msg.data = "nbvcxwmlkjhgfdsqpoiuytreza";
    pub_node->publish(msg);

    EXPECT_FALSE(sub_node->has_data_been_received());
    spinner.spin_some_all_nodes();

    EXPECT_TRUE(service_client->is_server_ready());

    auto result_update = service_client->send_request(std::make_shared<sorting_msgs::srv::ToggleSorting::Request>());
    EXPECT_TRUE(result_update->result);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    result_update = service_client->send_request(std::make_shared<sorting_msgs::srv::ToggleSorting::Request>());
    EXPECT_FALSE(result_update->result);

    pub_node->publish(msg);

    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_FALSE(sub_node->has_data_been_received());

    rclcpp::shutdown();
}