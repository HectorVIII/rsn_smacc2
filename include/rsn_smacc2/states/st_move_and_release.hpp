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

class StMoveAndRelease : public smacc2::SmaccState<StMoveAndRelease, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvHandoverDone, StIdle>,
    smacc2::Transition<EvFail, StIdle>
  >;

  void onEntry()
  {
    auto logger = this->getLogger();
    auto node = this->getNode();

    RCLCPP_INFO(logger,
                "[StMoveAndRelease] Calling /handover/execute_handover ...");

    auto client =
      node->create_client<std_srvs::srv::Trigger>("/handover/execute_handover");

    if (!client->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_ERROR(logger,
                   "[StMoveAndRelease] Service /handover/execute_handover not available.");
      this->postEvent<EvFail>();
      return;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);
    auto resp = future.get();

    if (resp && resp->success)
    {
      RCLCPP_INFO(logger,
                  "[StMoveAndRelease] execute_handover OK: %s",
                  resp->message.c_str());
      this->postEvent<EvHandoverDone>();
    }
    else
    {
      std::string msg = resp ? resp->message : "no response";
      RCLCPP_ERROR(logger,
                   "[StMoveAndRelease] execute_handover FAILED: %s",
                   msg.c_str());
      this->postEvent<EvFail>();
    }
  }
};

}  // namespace rsn_smacc2

