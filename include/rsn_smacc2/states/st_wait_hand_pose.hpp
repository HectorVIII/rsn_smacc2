#pragma once
#include <smacc2/smacc.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <boost/mpl/list.hpp>
#include <chrono>

#include "rsn_smacc2/events.hpp"

namespace rsn_smacc2
{
namespace mpl = boost::mpl;

class RsnSmacc2;
class StMoveAndRelease;

class StWaitHandPose : public smacc2::SmaccState<StWaitHandPose, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvHandPoseReady, StMoveAndRelease>,
    smacc2::Transition<EvFail, StIdle>
  >;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_ready_;

  void onEntry()
  {
    auto logger = this->getLogger();
    auto node = this->getNode();

    RCLCPP_INFO(logger, "[StWaitHandPose] Calling /handover/enable_wait_hand ...");

    auto client =
      node->create_client<std_srvs::srv::Trigger>("/handover/enable_wait_hand");

    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(logger,
                   "[StWaitHandPose] Service /handover/enable_wait_hand not available.");
      this->postEvent<EvFail>();
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    auto resp = future.get();

    if (!resp || !resp->success)
    {
      std::string msg = resp ? resp->message : "no response";
      RCLCPP_ERROR(logger,
                   "[StWaitHandPose] enable_wait_hand FAILED: %s", msg.c_str());
      this->postEvent<EvFail>();
      return;
    }

    RCLCPP_INFO(logger,
                "[StWaitHandPose] enable_wait_hand OK, waiting /handover/hand_ready ...");

    sub_ready_ = node->create_subscription<std_msgs::msg::Bool>(
      "/handover/hand_ready", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data)
        {
          RCLCPP_INFO(this->getLogger(), "[StWaitHandPose] hand_ready == true");
          this->postEvent<EvHandPoseReady>();
        }
      });
  }
};

}  // namespace rsn_smacc2

