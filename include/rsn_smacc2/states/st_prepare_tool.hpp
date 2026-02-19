#pragma once
#include <smacc2/smacc.hpp>
#include <boost/mpl/list.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rsn_smacc2/events.hpp"
#include "rsn_smacc2/sm.hpp"

namespace rsn_smacc2
{
namespace mpl = boost::mpl;

class StEnableWaitHand;

class StPrepareTool : public smacc2::SmaccState<StPrepareTool, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvOk, StEnableWaitHand>,
    smacc2::Transition<EvFail, StEnableWaitHand>   // 失败你也可以跳到 StDone，这里先简单处理
  >;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StPrepareTool] calling /handover/prepare_tool");

    auto client = getNode()->create_client<std_srvs::srv::Trigger>("/handover/prepare_tool");
    client->wait_for_service();

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);

    if (future.get()->success)
    {
      RCLCPP_INFO(getLogger(), "prepare_tool ok");
      postEvent<EvOk>();
    }
    else
    {
      RCLCPP_ERROR(getLogger(), "prepare_tool failed");
      postEvent<EvFail>();
    }
  }
};

} // namespace rsn_smacc2

