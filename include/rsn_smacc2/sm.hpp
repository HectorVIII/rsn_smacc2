// include/rsn_smacc2/sm.hpp
#pragma once
#include <smacc2/smacc.hpp>
#include <boost/statechart/event.hpp>

namespace rsn_smacc2
{
namespace sc = boost::statechart;



// -------- forward declare STATES （只声明名字，不写内容）--------
class StPrepareTool;
class StEnableWaitHand;
class StWaitHandReady;
class StExecuteHandover;
class StDone;

// -------- STATE MACHINE --------
class RsnSmacc2
  : public smacc2::SmaccStateMachineBase<RsnSmacc2, StPrepareTool>
{
public:
  using SmaccStateMachineBase::SmaccStateMachineBase;
};

} // namespace rsn_smacc2

// -------- 在这里 include 各个 state 的头文件（真正的定义）--------
#include "rsn_smacc2/states/st_prepare_tool.hpp"
#include "rsn_smacc2/states/st_enable_wait_hand.hpp"
#include "rsn_smacc2/states/st_wait_hand_ready.hpp"
#include "rsn_smacc2/states/st_execute_handover.hpp"
#include "rsn_smacc2/states/st_done.hpp"

