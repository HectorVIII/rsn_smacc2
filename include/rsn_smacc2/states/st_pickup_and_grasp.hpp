#pragma once
#include <smacc2/smacc.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <boost/mpl/list.hpp>
#include <chrono>

#include "rsn_smacc2/events.hpp"

namespace rsn_smacc2
{
namespace mpl = boost::mpl;

class RsnSmacc2;
class StWaitHandPose;

class StPickupAndGrasp : public smacc2::SmaccState<StPickupAndGrasp, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvInstrumentReady, StWaitHandPose>,
    smacc2::Transition<EvFail, StIdle>
  >;

  void onEntry()
  {
    RCLCPP_INFO(this->getLogger(),
                "[StPickupAndGrasp] Calling /handover/prepare_tool ...");

    auto node = this->getNode();
    auto client =
      node->create_client<std_srvs::srv::Trigger>("/handover/prepare_tool");

    // 等待 service 可用
    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(this->getLogger(),
                   "[StPickupAndGrasp] Service /handover/prepare_tool not available.");
      this->postEvent<EvFail>();
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);

    // 简单阻塞直到完成（实验版这样就够了）
    auto resp = future.get();
    if (resp && resp->success)
    {
      RCLCPP_INFO(this->getLogger(),
                  "[StPickupAndGrasp] prepare_tool OK: %s",
                  resp->message.c_str());
      this->postEvent<EvInstrumentReady>();
    }
    else
    {
      std::string msg = resp ? resp->message : "no response";
      RCLCPP_ERROR(this->getLogger(),
                   "[StPickupAndGrasp] prepare_tool FAILED: %s",
                   msg.c_str());
      this->postEvent<EvFail>();
    }
  }
};

}  // namespace rsn_smacc2

