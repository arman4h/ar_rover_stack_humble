#include "rover_terrain_costmap/terrain_layer.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <algorithm>
#include <functional>
#include <stdexcept>
#include <string>
namespace rover_terrain_costmap {
void TerrainLayer::onInitialize() {
 auto node = node_.lock();
 if (!node) { throw std::runtime_error("TerrainLayer: lifecycle node expired"); }
 declareParameter("topic", rclcpp::ParameterValue(std::string("/terrain/local_costmap")));
 std::string topic; node->get_parameter(name_ + ".topic", topic);
 rclcpp::QoS qos(rclcpp::KeepLast(1)); qos.reliable();
 sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(topic, qos,
   std::bind(&TerrainLayer::mapCallback, this, std::placeholders::_1));
 current_ = false; enabled_ = true;
}
void TerrainLayer::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
 std::lock_guard<std::mutex> lock(mutex_); map_ = msg; current_ = true;
}
void TerrainLayer::updateBounds(double, double, double, double *min_x, double *min_y, double *max_x, double *max_y) {
 if (!enabled_) return;
 std::lock_guard<std::mutex> lock(mutex_);
 if (!map_) { current_ = false; return; }
 const auto &o = map_->info.origin.position;
 *min_x = std::min(*min_x, o.x); *min_y = std::min(*min_y, o.y);
 *max_x = std::max(*max_x, o.x + map_->info.width * map_->info.resolution);
 *max_y = std::max(*max_y, o.y + map_->info.height * map_->info.resolution);
 current_ = true;
}
void TerrainLayer::updateCosts(nav2_costmap_2d::Costmap2D &master, int min_i, int min_j, int max_i, int max_j) {
 if (!enabled_) return;
 std::lock_guard<std::mutex> lock(mutex_); if (!map_) return;
 for (int j=min_j; j<max_j; ++j) for (int i=min_i; i<max_i; ++i) {
  double wx, wy; master.mapToWorld(i,j,wx,wy);
  const auto &info=map_->info; const double mx=info.origin.position.x, my=info.origin.position.y;
  if (wx<mx || wy<my) continue;
  const int gx=static_cast<int>((wx-mx)/info.resolution), gy=static_cast<int>((wy-my)/info.resolution);
  if (gx<0 || gy<0 || gx>=static_cast<int>(info.width) || gy>=static_cast<int>(info.height)) continue;
  const int value=map_->data[gy*info.width+gx]; if (value<0) continue;
  unsigned char cost = value>=100 ? nav2_costmap_2d::LETHAL_OBSTACLE :
    (value==0 ? nav2_costmap_2d::FREE_SPACE : static_cast<unsigned char>(std::min(252, 1 + value*2)));
  master.setCost(i,j,std::max(master.getCost(i,j),cost));
 }
}
void TerrainLayer::reset() { std::lock_guard<std::mutex> lock(mutex_); map_.reset(); current_=false; }
}
PLUGINLIB_EXPORT_CLASS(rover_terrain_costmap::TerrainLayer, nav2_costmap_2d::Layer)
