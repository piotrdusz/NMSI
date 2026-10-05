#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <algorithm>

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
      RCLCPP_INFO(this->get_logger(), "Changing state to ALIGNING_HEADING");
      // state = ALIGNING_HEADING;
      state = FOLLOWING_PATH;
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
  double max_angle_vel = 1.0;
  double min_angle_vel = 0.1;
  double max_linear_vel = 1.0;
  double min_linear_vel = 0.1;
  double direction_tolerance = 0.02;
  double xy_tolerance = 0.1;
  void follow_path();
  void head_to_direction(double angle);
  void head_to_point(PoseStamped point);
  void head_to_goal(PoseStamped goal);
  void stop_moving();
  PoseStamped global_to_local(PoseStamped point);
  double distance(PoseStamped point);
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
  static int pose_id = 1;
  if (state == WAITING_FOR_PATH)
  {
    pose_id = 1;
    return;
  }
  if (path->poses.size() < 2)
  {
    RCLCPP_ERROR(this->get_logger(), "To little poses in path, changing state to WAITING_FOR_PATH");
    state = WAITING_FOR_PATH;
    stop_moving();
    return;
  }
  PoseStamped target;
  try
  {
    target = global_to_local(path->poses[pose_id]);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s", fixed_frame.c_str(), robot_frame.c_str(),
                ex.what());
    stop_moving();
  }
  if (state == ALIGNING_HEADING)
  {
    double angle = atan2(target.pose.position.y, target.pose.position.x);
    head_to_direction(angle);
    if (abs(angle) < direction_tolerance)
    {
      RCLCPP_INFO(this->get_logger(), "Direction aligned, changing state to FOLLOWING_PATH");
      state = FOLLOWING_PATH;
      return;
    }
  }
  if (state == FOLLOWING_PATH)
  {
    if (pose_id == path->poses.size() - 1)
    {
      RCLCPP_INFO(this->get_logger(), "Last point set, changing state to APPROACHING_GOAL");
      state = APPROACHING_GOAL;
      return;
    }
    head_to_point(target);
    if (distance(target) < xy_tolerance)
    {
      pose_id += 1;
    }
  }
  if (state == APPROACHING_GOAL)
  {
    head_to_goal(target);
    if (distance(target) < xy_tolerance)
    {
      stop_moving();
      RCLCPP_INFO(this->get_logger(), "Goal achieved, changing state to WAITING_FOR_PATH");
      state = WAITING_FOR_PATH;
    }
  }
}

void PathFollower::head_to_direction(double angle)
{
  Twist twist;
  double sign = angle > 0 ? 1.0 : -1.0;
  twist.angular.z = min_angle_vel * sign + (max_angle_vel - min_angle_vel) * angle / M_PI;
  publisher->publish(twist);
}

void PathFollower::head_to_point(PoseStamped point)
{
  Twist twist;
  double angle = atan2(point.pose.position.y, point.pose.position.x);
  double angular_component = std::clamp(angle / M_PI_2, -1.0, 1.0);
  double linear_component = 1 - std::abs(angular_component);
  twist.angular.z = angular_component * max_angle_vel;
  twist.linear.x = linear_component * max_linear_vel;
  publisher->publish(twist);
}

void PathFollower::head_to_goal(PoseStamped goal)
{
  Twist twist;
  double angle = atan2(goal.pose.position.y, goal.pose.position.x);
  double sign = angle > 0 ? 1.0 : -1.0;
  double angular_component;
  double linear_component = 1 - std::abs(angular_component);
  double linear_scaled = min_linear_vel + (max_linear_vel - min_linear_vel) * distance(goal);
  twist.angular.z = angular_component * max_angle_vel;
  twist.linear.x = std::min(linear_component * max_linear_vel, linear_scaled);
  publisher->publish(twist);
}

PoseStamped PathFollower::global_to_local(PoseStamped point)
{
  point.header.stamp.sec = 0;
  point.header.stamp.nanosec = 0;
  return tf_buffer->transform(point, robot_frame);
}

void PathFollower::stop_moving()
{
  Twist twist;
  publisher->publish(twist);
}

double PathFollower::distance(PoseStamped point)
{
  return hypot(point.pose.position.x, point.pose.position.y);
}