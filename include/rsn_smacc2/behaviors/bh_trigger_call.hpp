#pragma once

#include <memory>
#include <string>

#include <smacc2/smacc.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rsn_smacc2/events.hpp"

namespace rsn_smacc2
{

// 通过 std_srvs/Trigger 服务触发某个动作（例如：开启“等待手 ready”模式）
// 成功 -> postEvent<EvOk>()
// 失败 -> postEvent<EvFail>()
class BhTriggerCall : public smacc2::SmaccClientBehavior
{
public:
  // 可以在构造时指定服务名，不指定则用默认值
  explicit BhTriggerCall(const std::string & service_name = "/rsn_smacc2/trigger")
  : service_name_(service_name)
  {
  }

  // 进入该 Behavior 时立即调用服务
  void onEntry() override
  {
    auto node = this->getNode();
    RCLCPP_INFO(
      node->get_logger(), "[BhTriggerCall] Calling Trigger service: %s", service_name_.c_str());

    client_ = node->create_client<std_srvs::srv::Trigger>(service_name_);

    // 等待服务可用
    if (!client_->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(
        node->get_logger(), "[BhTriggerCall] Service %s not available", service_name_.c_str());
      this->postEvent<EvFail>();
      return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();

    try
    {
      auto future_result = client_->async_send_request(request);

      // 简单阻塞等待结果（在 SMACC2 默认多线程 executor 下正常工作）
      auto ret = rclcpp::spin_until_future_complete(node, future_result);

      if (ret != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(
          node->get_logger(), "[BhTriggerCall] Failed to call service %s", service_name_.c_str());
        this->postEvent<EvFail>();
        return;
      }

      auto response = future_result.get();
      if (response->success)
      {
        RCLCPP_INFO(
          node->get_logger(), "[BhTriggerCall] Service %s returned success", service_name_.c_str());
        this->postEvent<EvOk>();
      }
      else
      {
        RCLCPP_WARN(
          node->get_logger(), "[BhTriggerCall] Service %s returned failure: %s",
          service_name_.c_str(), response->message.c_str());
        this->postEvent<EvFail>();
      }
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(
        node->get_logger(), "[BhTriggerCall] Exception while calling service %s: %s",
        service_name_.c_str(), e.what());
      this->postEvent<EvFail>();
    }
  }

  void onExit() override
  {
    auto node = this->getNode();
    RCLCPP_INFO(node->get_logger(), "[BhTriggerCall] Exit");
    client_.reset();
  }

private:
  std::string service_name_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
};

}  // namespace rsn_smacc2

