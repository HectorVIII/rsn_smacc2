#pragma once
#include <smacc2/smacc.hpp>

#include "rsn_smacc2/sm.hpp"

namespace rsn_smacc2
{

class StDone : public smacc2::SmaccState<StDone, RsnSmacc2>
{
public:
  using SmaccState::SmaccState;

  void onEntry()
  {
    RCLCPP_INFO(getLogger(), "[StDone] Handover finished!");
  }
};

} // namespace rsn_smacc2

