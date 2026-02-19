#include "rclcpp/rclcpp.hpp"
#include "smacc2/smacc.hpp"
#include "rsn_smacc2/sm.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  smacc2::run<rsn_smacc2::RsnSmacc2>();
  rclcpp::shutdown();
  return 0;
}

