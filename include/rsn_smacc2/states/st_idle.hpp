#pragma once
#include <smacc2/smacc.hpp>
#include <std_msgs/msg/bool.hpp>
#include <boost/mpl/list.hpp>

#include "rsn_smacc2/events.hpp"

namespace rsn_smacc2
{
namespace mpl = boost::mpl;

class RsnSmacc2;
class StPickupAndGrasp;

class StIdle : public smacc2::SmaccState<StIdle, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  using reactions = mpl::list<
    smacc2::Transition<EvStart, StPickupAndGrasp>
  >;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_start_;

  void onEntry()
  {
    RCLCPP_INFO(this->getLogger(), "[StIdle] Waiting for /handover/start ...");

    auto node = this->getNode();
    sub_start_ = node->create_subscription<std_msgs::msg::Bool>(
      "/handover/start", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg)
      {
        if (msg->data)
        {
          RCLCPP_INFO(this->getLogger(), "[StIdle] Start signal received.");
          this->postEvent<EvStart>();
        }
      });
  }
};

}  // namespace rsn_smacc2

