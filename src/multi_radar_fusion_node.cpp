#include "multi_radar_fusion/multi_radar_fusion.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<multi_radar_fusion::MultiRadarFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}