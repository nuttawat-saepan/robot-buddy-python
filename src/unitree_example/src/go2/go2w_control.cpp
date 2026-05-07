/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unitree_go/msg/detail/sport_mode_state__struct.hpp>

#include "common/ros2_sport_client.h"
#include "unitree_go/msg/sport_mode_state.hpp"

#define TOPIC_HIGHSTATE "lf/sportmodestate"

struct KeyAction {
    std::string name;
    double vx;
    double vy;
    double wz;
    bool oneshot; // true = สั่งครั้งเดียว เช่น Stand / Sit / StopMove
};

class Go2SportClientNode : public rclcpp::Node {
public:
    Go2SportClientNode()
        : Node("go2_sport_client_node"),
          sport_client_(this),
          running_(true)
    {
        suber_ = this->create_subscription<unitree_go::msg::SportModeState>(
            TOPIC_HIGHSTATE, 1,
            [this](const unitree_go::msg::SportModeState::SharedPtr data) {
                HighStateHandler(data);
            });

        double freq_hz = 1.0; // control loop frequency
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds((int)(1000.0 / freq_hz)),
            std::bind(&Go2SportClientNode::RobotControl, this));

        keyboard_thread_ = std::thread([this]() {
            KeyboardLoop();
        });

        // map ปุ่ม → action
        key_map_ = {
            {'1', {"Stand up", 0,0,0, true}},
            {'2', {"Balance stand", 0,0,0, true}},
            {'3', {"Sit down", 0,0,0, true}},
            {'4', {"Stop move", 0,0,0, true}},
            {'w', {"Forward", 0.3,0,0, false}},
            {'s', {"Backward", -0.3,0,0, false}},
            {'a', {"Left", 0,0.3,0, false}},
            {'d', {"Right", 0,-0.3,0, false}},
            {'q', {"Rotate Left",0,0,0.3,false}},
            {'e', {"Rotate Right", 0,0,-0.3,false}}
        };
    }

    ~Go2SportClientNode() {
        running_ = false;
        if (keyboard_thread_.joinable()) {
            keyboard_thread_.join();
        }
    }

private:
    void RobotControl() {
        // ถ้าเป็น one-shot action
        if (current_action_.oneshot && !one_shot_done_) {
            RCLCPP_INFO(this->get_logger(), "Executing: %s", current_action_.name.c_str());
            ExecuteAction(current_action_);
            one_shot_done_ = true;
        }
        // ถ้าเป็น movement velocity
        else if (!current_action_.oneshot) {
            if (current_action_.vx != 0 || current_action_.vy != 0 || current_action_.wz != 0) {
                RCLCPP_INFO(this->get_logger(), "Moving: %s", current_action_.name.c_str());
                sport_client_.Move(req_, current_action_.vx, current_action_.vy, current_action_.wz);
            }
        }
    }

    void ExecuteAction(const KeyAction &action) {
        if (action.name == "Stand up") {
            sport_client_.StandUp(req_);
        } else if (action.name == "Balance stand") {
            sport_client_.BalanceStand(req_);
        } else if (action.name == "Sit down") {
            sport_client_.Sit(req_);
        } else if (action.name == "Stop move") {
            sport_client_.StopMove(req_);
        } 
    }

    void KeyboardLoop() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);
        char c;

        while (running_) {
            if (read(STDIN_FILENO, &c, 1) > 0) {
                if (c == 'x') {  // กด x ออก
                    running_ = false;
                    rclcpp::shutdown();
                    break;
                }

                auto it = key_map_.find(c);
                if (it != key_map_.end()) {
                    current_action_ = it->second;
                    one_shot_done_ = false;
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    void HighStateHandler(const unitree_go::msg::SportModeState::SharedPtr msg) {
        state_ = *msg;
        RCLCPP_INFO(this->get_logger(),
                    "Position: %f, %f, %f | IMU rpy: %f, %f, %f",
                    state_.position[0], state_.position[1], state_.position[2],
                    state_.imu_state.rpy[0], state_.imu_state.rpy[1], state_.imu_state.rpy[2]);
    }

    unitree_go::msg::SportModeState state_;
    SportClient sport_client_;
    rclcpp::Subscription<unitree_go::msg::SportModeState>::SharedPtr suber_;
    rclcpp::TimerBase::SharedPtr timer_;
    unitree_api::msg::Request req_;

    std::atomic<bool> running_;
    std::atomic<bool> one_shot_done_{false};
    KeyAction current_action_{ "",0,0,0,false };

    std::map<char, KeyAction> key_map_;
    std::thread keyboard_thread_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Go2SportClientNode>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}