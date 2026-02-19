#pragma once
#include <smacc2/smacc.hpp>
#include "rsn_smacc2/events.hpp"

namespace rsn_smacc2
{
// 先前向声明所有 state
class StIdle;
class StPickupAndGrasp;
class StWaitHandPose;
class StMoveAndRelease;

// 顶层状态机，初始状态为 StIdle
class RsnSmacc2
  : public smacc2::SmaccStateMachineBase<RsnSmacc2, StIdle>
{
public:
  using SmaccStateMachineBase::SmaccStateMachineBase;
};

}  // namespace rsn_smacc2

// 再在文件结尾包含各个 state 的定义
#include "rsn_smacc2/states/st_idle.hpp"
#include "rsn_smacc2/states/st_pickup_and_grasp.hpp"
#include "rsn_smacc2/states/st_wait_hand_pose.hpp"
#include "rsn_smacc2/states/st_move_and_release.hpp"

