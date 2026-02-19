#pragma once
#include <boost/statechart/event.hpp>

namespace rsn_smacc2
{
namespace sc = boost::statechart;

struct EvOk : sc::event<EvOk> {};
struct EvFail : sc::event<EvFail> {};
struct EvHandReady : sc::event<EvHandReady> {};

} // namespace rsn_smacc2

