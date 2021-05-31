#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
#include <iostream>

class CancelGoals : public rclcpp::Node
{
public:
  CancelGoals()
  : Node("GoalCancel")
    {
      client=Node::create_client<action_msgs::srv::CancelGoal>("navigate_to_pose/_action/cancel_goal");
      //an empty cancel goal message would cancel everything
      auto cancel_all = std::make_shared<action_msgs::srv::CancelGoal::Request>();
      //wait for server first
      while (!client->wait_for_service()) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
          break;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
          }
      //send the request to cancel the goal
      client->async_send_request(cancel_all);

    }

private:
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr client;



};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CancelGoals>());
  rclcpp::shutdown();
  return 0;
}
