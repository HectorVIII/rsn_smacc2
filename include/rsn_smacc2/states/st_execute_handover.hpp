#pragma once
#include <smacc2/smacc.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <boost/mpl/list.hpp>

#include "rsn_smacc2/events.hpp"
#include "rsn_smacc2/sm.hpp"

namespace rsn_smacc2
{
namespace mpl = boost::mpl;

class StDone;

class StExecuteHandover : public smacc2::SmaccState<StExecuteHandover, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvOk, StDone>,
    smacc2::Transition<EvFail, StDone>
  >;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StExecuteHandover] calling /handover/execute_handover");

    auto client = getNode()->create_client<std_srvs::srv::Trigger>("/handover/execute_handover");
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

