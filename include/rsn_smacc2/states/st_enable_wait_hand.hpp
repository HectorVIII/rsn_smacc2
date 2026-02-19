#pragma once
#include <smacc2/smacc.hpp>
#include <boost/mpl/list.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "rsn_smacc2/events.hpp"
#include "rsn_smacc2/sm.hpp"

namespace rsn_smacc2
{
namespace mpl = boost::mpl;

class StWaitHandReady;

class StEnableWaitHand : public smacc2::SmaccState<StEnableWaitHand, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvOk, StWaitHandReady>,
    smacc2::Transition<EvFail, StWaitHandReady>
  >;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StEnableWaitHand] calling /handover/enable_wait_hand");

    auto client = getNode()->create_client<std_srvs::srv::Trigger>("/handover/enable_wait_hand");
    client->wait_for_service();

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = client->async_send_request(req);

    if (future.get()->success)
      postEvent<EvOk>();
    else
      postEvent<EvFail>();
  }
};

} // namespace rsn_smacc2

