#pragma once
#include <boost/statechart/event.hpp>

namespace rsn_smacc2
{
namespace sc = boost::statechart;

// GUI / start handover
struct EvStart : sc::event<EvStart> {};

// /handover/prepare_tool 
struct EvInstrumentReady : sc::event<EvInstrumentReady> {};

// /handover/hand_ready == true 
struct EvHandPoseReady : sc::event<EvHandPoseReady> {};

// /handover/execute_handover
struct EvHandoverDone : sc::event<EvHandoverDone> {};

// failed
struct EvFail : sc::event<EvFail> {};

}  // namespace rsn_smacc2

