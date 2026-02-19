#pragma once
#include <boost/statechart/event.hpp>

namespace rsn_smacc2
{
namespace sc = boost::statechart;

// GUI / 外部开始一轮 handover
struct EvStart : sc::event<EvStart> {};

// /handover/prepare_tool 成功（器械已抓好，回到 P0）
struct EvInstrumentReady : sc::event<EvInstrumentReady> {};

// /handover/hand_ready == true （ZED 算出手的位置）
struct EvHandPoseReady : sc::event<EvHandPoseReady> {};

// /handover/execute_handover 成功（移动到手、等待>7N、释放并回到 P0）
struct EvHandoverDone : sc::event<EvHandoverDone> {};

// 任意失败
struct EvFail : sc::event<EvFail> {};

}  // namespace rsn_smacc2

