#pragma once

#include <memory>
#include <string>

#include <smacc2/smacc.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rsn_smacc2/events.hpp"

namespace rsn_smacc2
{

// 订阅 "hand_ready" 话题：当收到 true 时 -> postEvent<EvHandReady>()
class BhWaitHandReady : public smacc2::SmaccClientBehavior
{
public:
  // 可以在构造时指定 topic 名，不指定则用默认
  explicit BhWaitHandReady(const std::string & topic_name = "/rsn_smacc2/hand_ready")
  : topic_name_(topic_name)
  {
  }

  void onEntry() override
  {
    auto node = this->getNode();
    RCLCPP_INFO(
      node->get_logger(), "[BhWaitHandReady] Subscribing to: %s", topic_name_.c_str());

    triggered_ = false;

    subscription_ = node->create_subscription<std_msgs::msg::Bool>(
      topic_name_, rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        // 只在第一次收到 true 时触发一次事件
        if (!triggered_ && msg->data)
        {
          triggered_ = true;
          auto node = this->getNode();
          RCLCPP_INFO(
            node->get_logger(), "[BhWaitHandReady] Hand ready detected on topic %s",
            topic_name_.c_str());
          this->postEvent<EvHandReady>();
        }
      });
  }

  void onExit() override
  {
    auto node = this->getNode();
    RCLCPP_INFO(node->get_logger(), "[BhWaitHandReady] Exit, unsubscribe");
    subscription_.reset();
    triggered_ = false;
  }

private:
  std::string topic_name_;
  bool triggered_{false};

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

}  // namespace rsn_smacc2

