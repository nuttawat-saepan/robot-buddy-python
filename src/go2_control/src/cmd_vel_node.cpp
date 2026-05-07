#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <chrono>
#include <algorithm>

#include "common/ros2_sport_client.h"
#include "unitree_api/msg/request.hpp"

using namespace std::chrono_literals;

class CmdVelBridgeNode : public rclcpp::Node
{
public:
  CmdVelBridgeNode()
  : Node("cmd_vel_bridge_node"),
    sport_client_(this)
  {
    // ===== PARAMETERS =====
    max_vx_   = this->declare_parameter("max_vx", 1.0);    // m/s
    max_vy_   = this->declare_parameter("max_vy", );    // m/s
    max_vyaw_ = this->declare_parameter("max_vyaw", 1.0);  // rad/s

    deadband_ = this->declare_parameter("deadband", 0.05);
    alpha_    = this->declare_parameter("smoothing_alpha", 0.2);

    max_acc_  = this->declare_parameter("max_acc", 0.05); // per cycle

    timeout_  = this->declare_parameter("timeout", 0.3);

    // ===== SUBSCRIBER =====
    sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&CmdVelBridgeNode::callback, this, std::placeholders::_1)
    );

    // ===== TIMER =====
    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&CmdVelBridgeNode::update, this)
    );

    last_cmd_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "CmdVel Bridge Node (Improved) started");
  }

private:

  // ===== CALLBACK =====
  void callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->now();

    // scale to [-1, 1]
    target_vx_   = clamp(msg->linear.x / max_vx_);
    target_vy_   = clamp(msg->linear.y / max_vy_);
    target_vyaw_ = clamp(msg->angular.z / max_vyaw_);

    // deadband
    target_vx_   = apply_deadband(target_vx_);
    target_vy_   = apply_deadband(target_vy_);
    target_vyaw_ = apply_deadband(target_vyaw_);
  }

  // ===== MAIN LOOP =====
  void update()
  {
    auto now = this->now();
    double dt = (now - last_cmd_time_).seconds();

    // watchdog
    if (dt > timeout_)
    {
      vx_ = 0;
      vy_ = 0;
      vyaw_ = 0;
      sport_client_.StopMove(req_);
      return;
    }

    // smoothing (low-pass)
    vx_   = (1 - alpha_) * vx_   + alpha_ * target_vx_;
    vy_   = (1 - alpha_) * vy_   + alpha_ * target_vy_;
    vyaw_ = (1 - alpha_) * vyaw_ + alpha_ * target_vyaw_;

    // acceleration limit
    vx_   = limit_acc(vx_, prev_vx_);
    vy_   = limit_acc(vy_, prev_vy_);
    vyaw_ = limit_acc(vyaw_, prev_vyaw_);

    prev_vx_ = vx_;
    prev_vy_ = vy_;
    prev_vyaw_ = vyaw_;

    // send command
    sport_client_.Move(req_, vx_, vy_, vyaw_);

    // debug log
    RCLCPP_DEBUG(this->get_logger(),
      "vx: %.2f vy: %.2f yaw: %.2f", vx_, vy_, vyaw_);
  }

  // ===== HELPERS =====
  double clamp(double v)
  {
    return std::max(-1.0, std::min(1.0, v));
  }

  double apply_deadband(double v)
  {
    return (std::abs(v) < deadband_) ? 0.0 : v;
  }

  double limit_acc(double current, double prev)
  {
    double delta = current - prev;

    if (delta > max_acc_) return prev + max_acc_;
    if (delta < -max_acc_) return prev - max_acc_;

    return current;
  }

private:
  SportClient sport_client_;
  unitree_api::msg::Request req_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_cmd_time_;

  // parameters
  double max_vx_, max_vy_, max_vyaw_;
  double deadband_;
  double alpha_;
  double max_acc_;
  double timeout_;

  // state
  double vx_{0}, vy_{0}, vyaw_{0};
  double prev_vx_{0}, prev_vy_{0}, prev_vyaw_{0};
  double target_vx_{0}, target_vy_{0}, target_vyaw_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelBridgeNode>());
  rclcpp::shutdown();
  return 0;
}