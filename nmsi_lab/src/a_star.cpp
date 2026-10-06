#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <utility>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::OccupancyGrid;
using nav_msgs::msg::Path;
using std::string;

class AStarCalculator : public rclcpp::Node
{
public:
  AStarCalculator() : Node("a_star")
  {
    auto pose_callback = [this](PoseStamped::UniquePtr pose) -> void {
      TransformStamped trans;
      try
      {
        trans = tf_buffer->lookupTransform(fixed_frame, robot_frame, tf2::TimePointZero);
      }
      catch (const tf2::TransformException& ex)
      {
        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", robot_frame.c_str(), fixed_frame.c_str(),
                    ex.what());
        return;
      }
      PoseStamped start;
      start.header.frame_id = "map";
      start.header.stamp = now();
      start.pose.position.x = trans.transform.translation.x;
      start.pose.position.y = trans.transform.translation.y;
      start.pose.position.z = trans.transform.translation.z;
      start.pose.orientation = trans.transform.rotation;
      a_star(start, *pose);
    };

    subscription = this->create_subscription<PoseStamped>("/goal_pose", 10, pose_callback);
    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    dilated_map_subscription = this->create_subscription<OccupancyGrid>(
      "/map_dilated", map_qos, [this](OccupancyGrid::UniquePtr map) { map_dilated = std::move(map); });
    cost_map_subscription = this->create_subscription<OccupancyGrid>(
      "/map_cost", map_qos, [this](OccupancyGrid::UniquePtr map) { map_cost = std::move(map); });
    publisher = this->create_publisher<Path>("/path", 10);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
  }

private:
  rclcpp::Subscription<PoseStamped>::SharedPtr subscription;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr dilated_map_subscription;
  rclcpp::Subscription<OccupancyGrid>::SharedPtr cost_map_subscription;
  rclcpp::Publisher<Path>::SharedPtr publisher;
  OccupancyGrid::UniquePtr map_dilated;
  OccupancyGrid::UniquePtr map_cost;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  string fixed_frame = "map";
  string robot_frame = "base_link";
  void a_star(PoseStamped start, PoseStamped end);
  void run_follower(Path path);
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStarCalculator>());
  rclcpp::shutdown();
}

void AStarCalculator::a_star(PoseStamped start, PoseStamped end)
{
  Path calculated_path;
  calculated_path.header.frame_id = "map";
  calculated_path.header.stamp = now();
  calculated_path.poses.push_back(start);
  calculated_path.poses.push_back(end);
  publisher->publish(calculated_path);
}