#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Twist;
using nav_msgs::msg::Path;
using std::string;
using namespace std::chrono_literals;

class PathFollower : public rclcpp::Node
{
public:
  PathFollower() : Node("path_follower")
  {
    state = WAITING_FOR_PATH;
    auto path_callback = [this](Path::UniquePtr path) -> void {
      if (path->poses.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Path is empty");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Path registered");
      PoseStamped goal = path->poses.back();
      RCLCPP_INFO(this->get_logger(), "Goal: (%.2f, %.2f, %.2f)", goal.pose.position.x, goal.pose.position.y,
                  goal.pose.position.z);
      state = ALIGNING_HEADING;
      this->path = std::move(path);
    };
    auto timer_callback = [this]() -> void { follow_path(); };
    subscription = this->create_subscription<Path>("/path", 10, path_callback);
    publisher = this->create_publisher<Twist>("/cmd_vel", 10);
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);
    timer = this->create_timer(100ms, timer_callback);
  }

private:
  rclcpp::Subscription<Path>::SharedPtr subscription;
  rclcpp::Publisher<Twist>::SharedPtr publisher;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::TimerBase::SharedPtr timer;
  string fixed_frame = "map";
  string robot_frame = "base_link";
  double max_angle_vel = 0.1;
  double min_angle_vel = 0.01;
  double max_linear_vel = 1.0;
  double min_linear_vel = 0.1;
  double ahead_radius = 1.0;
  double direction_tolerance = 0.1;
  void follow_path();
  void head_to_direction(PoseStamped direction);
  void head_to_point(PoseStamped point);
  void head_to_finish(PoseStamped point);
  PoseStamped global_to_local(PoseStamped point);
  int state;
  Path::UniquePtr path;
  enum State
  {
    WAITING_FOR_PATH,
    ALIGNING_HEADING,
    FOLLOWING_PATH,
    APPROACHING_GOAL
  };
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
}

void PathFollower::follow_path()
{
  if (state == WAITING_FOR_PATH)
  {
    return;
  }
  if (path->poses.size() < 2)
  {
    RCLCPP_ERROR(this->get_logger(), "To little poses in path, changing state to WAITING_FOR_PATH");
    state = WAITING_FOR_PATH;
    return;
  }
  if (state == ALIGNING_HEADING)
  {
    PoseStamped target = global_to_local(path->poses[1]);
    RCLCPP_INFO(this->get_logger(), "Target local position (%.2f, %.2f, %.2f)", target.pose.position.x,
                target.pose.position.y, target.pose.position.z);
    head_to_direction(target);
  }
}

void PathFollower::head_to_direction(PoseStamped direction)
{
  double angle = atan2(direction.pose.position.y, direction.pose.position.x);
  Twist twist;
  twist.angular.z = min_angle_vel + (max_angle_vel - min_angle_vel) * angle;
  publisher->publish(twist);
}

// void PathFollower::head_to_point(PoseStamped point)
// {
// }

PoseStamped PathFollower::global_to_local(PoseStamped point)
{
  try
  {
    return tf_buffer->transform(point, robot_frame);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", robot_frame.c_str(), fixed_frame.c_str(),
                ex.what());
    return PoseStamped();
  }
}