#include "rclcpp/rclcpp.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <iostream>
#include <memory>
#include <vector>


class CancelGoals : public rclcpp::Node
{
public:
  CancelGoals()
  : Node("GoalCancel")
    {

      this->declare_parameter<int>("cancel_button", 1);
      // get data
      this->get_parameter("cancel_button", cancel_button );
      client=Node::create_client<action_msgs::srv::CancelGoal>("navigate_to_pose/_action/cancel_goal");

      subscription_= this->create_subscription<sensor_msgs::msg::Joy>(
      "joy",10,
      [this](const sensor_msgs::msg::Joy::SharedPtr msg){

      std::vector<int> pressed_buttons=msg->buttons;
      auto cancel_all = std::make_shared<action_msgs::srv::CancelGoal::Request>();
      // second element [1] for B
      if(pressed_buttons[cancel_button]){
RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received goal cancel signal. Cancelling all goals.");
        client->async_send_request(cancel_all);
      }
    });
    }

private:
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr client;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
  int cancel_button;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CancelGoals>());
  rclcpp::shutdown();
  return 0;
}
