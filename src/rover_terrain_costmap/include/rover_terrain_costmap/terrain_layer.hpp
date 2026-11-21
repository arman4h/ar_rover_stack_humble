#pragma once
#include <mutex>
#include "nav2_costmap_2d/layer.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
namespace rover_terrain_costmap {
class TerrainLayer : public nav2_costmap_2d::Layer {
public:
 void onInitialize() override;
 void updateBounds(double, double, double, double *, double *, double *, double *) override;
 void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
 void reset() override;
 bool isClearable() override { return true; }
private:
 void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
 rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
 std::mutex mutex_;
 nav_msgs::msg::OccupancyGrid::SharedPtr map_;
 bool current_{false};
};
}
