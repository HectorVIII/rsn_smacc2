#pragma once
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/bool.hpp>
#include <boost/mpl/list.hpp>

#include "rsn_smacc2/events.hpp"
#include "rsn_smacc2/sm.hpp"

namespace rsn_smacc2
{
namespace mpl = boost::mpl;

class StExecuteHandover;

class StWaitHandReady : public smacc2::SmaccState<StWaitHandReady, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvHandReady, StExecuteHandover>
  >;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StWaitHandReady] waiting on /handover/hand_ready");

    auto node = getNode();
    sub_ = node->create_subscription<std_msgs::msg::Bool>(
      "/handover/hand_ready",
      10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data)
        {
          RCLCPP_INFO(getLogger(), "Hand READY!");
          postEvent<EvHandReady>();
        }
      }
    );
  }
};

} // namespace rsn_smacc2

