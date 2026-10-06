#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

using nav_msgs::msg::OccupancyGrid;

class MapProcessor : public rclcpp::Node
{
public:
  MapProcessor() : Node("map_processor")
  {
    robot_radius_ = declare_parameter("robot_radius", 0.25);
    max_cost_distance_ = declare_parameter("max_cost_distance", 1.0);
    occupied_threshold_ = declare_parameter("occupied_threshold", 65);

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    dilated_pub_ = create_publisher<OccupancyGrid>("map_dilated", qos);
    cost_pub_ = create_publisher<OccupancyGrid>("map_cost", qos);
    sub_ = create_subscription<OccupancyGrid>(
      "map", qos, std::bind(&MapProcessor::onMap, this, std::placeholders::_1));
  }

private:
  // Two-pass chamfer (3-4 weights) distance transform, result in cells.
  std::vector<float> distanceToObstacles(const OccupancyGrid & map) const
  {
    const int w = map.info.width, h = map.info.height;
    const float inf = std::numeric_limits<float>::max() / 4;
    const float a = 1.0f, b = std::sqrt(2.0f);
    std::vector<float> d(static_cast<size_t>(w) * h, inf);
    for (size_t i = 0; i < d.size(); ++i) {
      if (map.data[i] >= occupied_threshold_) {d[i] = 0.0f;}
    }
    auto at = [&](int x, int y) -> float & {return d[static_cast<size_t>(y) * w + x];};
    auto relax = [&](int x, int y, int dx, int dy, float c) {
      int nx = x + dx, ny = y + dy;
      if (nx >= 0 && nx < w && ny >= 0 && ny < h) {
        at(x, y) = std::min(at(x, y), at(nx, ny) + c);
      }
    };
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        relax(x, y, -1, 0, a); relax(x, y, 0, -1, a);
        relax(x, y, -1, -1, b); relax(x, y, 1, -1, b);
      }
    }
    for (int y = h - 1; y >= 0; --y) {
      for (int x = w - 1; x >= 0; --x) {
        relax(x, y, 1, 0, a); relax(x, y, 0, 1, a);
        relax(x, y, 1, 1, b); relax(x, y, -1, 1, b);
      }
    }
    return d;
  }

  void onMap(const OccupancyGrid::SharedPtr msg)
  {
    const double res = msg->info.resolution;
    const auto dist = distanceToObstacles(*msg);

    OccupancyGrid dilated = *msg;
    OccupancyGrid cost = *msg;
    for (size_t i = 0; i < dist.size(); ++i) {
      const double d = dist[i] * res;
      if (d <= robot_radius_) {
        dilated.data[i] = 100;
      }
      if (msg->data[i] < 0) {
        cost.data[i] = -1;
      } else {
        const double c = 100.0 * (1.0 - d / max_cost_distance_);
        cost.data[i] = static_cast<int8_t>(std::clamp(std::lround(c), 0L, 100L));
      }
    }
    dilated_pub_->publish(dilated);
    cost_pub_->publish(cost);
    RCLCPP_INFO(get_logger(), "Published map_dilated and map_cost (%ux%u)",
      msg->info.width, msg->info.height);
  }

  double robot_radius_, max_cost_distance_;
  int occupied_threshold_;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<OccupancyGrid>::SharedPtr dilated_pub_, cost_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapProcessor>());
  rclcpp::shutdown();
  return 0;
}
