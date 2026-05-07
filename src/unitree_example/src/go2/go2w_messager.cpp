#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <termios.h>
#include <unistd.h>
#include <map>
#include <string>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

struct KeyAction {
    std::string name;
    bool oneshot;
};

class Go2CommandPublisher : public rclcpp::Node {
public:
    Go2CommandPublisher()
        : Node("go2_command_publisher"), running_(true)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("go2_command", 10);

        key_map_ = {
            {'0', {"damp", true}},
            {'1', {"stand_up", true}},
            {'2', {"balance_stand", true}},
            {'3', {"sit_down", true}},
            {'4', {"stop_move", true}},
            {'w', {"move_forward", false}},
            {'s', {"move_backward", false}},
            {'a', {"move_left", false}},
            {'d', {"move_right", false}},
            {'q', {"rotate_left", false}},
            {'e', {"rotate_right", false}},
            {'c', {"camera_toggle", true}},
            {'v', {"camera_capture", true}}
        };

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&Go2CommandPublisher::TimerPublish, this)
        );

        keyboard_thread_ = std::thread([this]() { KeyboardLoop(); });
    }

    ~Go2CommandPublisher() {
        running_ = false;
        if (keyboard_thread_.joinable())
            keyboard_thread_.join();
    }

private:
    void KeyboardLoop() {
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);

        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);

        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        char c;

        while (running_) {
            if (read(STDIN_FILENO, &c, 1) > 0) {

                if (c == 'x') {
                    running_ = false;
                    rclcpp::shutdown();
                    break;
                }

                if (c == ' ') {
                    std::lock_guard<std::mutex> lock(command_mutex_);
                    current_command_.clear();
                    PublishCommand("stop_move", true);
                    continue;
                }

                auto it = key_map_.find(c);

                if (it != key_map_.end()) {

                    if (it->second.oneshot) {
                        std::lock_guard<std::mutex> lock(command_mutex_);
                        current_command_.clear();
                        PublishCommand(it->second.name, true);
                    }
                    else {
                        std::lock_guard<std::mutex> lock(command_mutex_);

                        if (current_command_ != it->second.name && !current_command_.empty()) {
                            PublishCommand("stop_move", true);
                        }

                        current_command_ = it->second.name;
                    }
                }
            }
        }

        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    }

    void TimerPublish() {
        std::lock_guard<std::mutex> lock(command_mutex_);

        if (!current_command_.empty()) {
            PublishCommand(current_command_, false);
        }
    }

    void PublishCommand(const std::string& name, bool oneshot) {
        nlohmann::json j;
        j["name"] = name;
        j["oneshot"] = oneshot;

        std_msgs::msg::String msg;
        msg.data = j.dump();

        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Sent command: %s", msg.data.c_str());
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<bool> running_;
    std::thread keyboard_thread_;

    std::map<char, KeyAction> key_map_;

    std::string current_command_;
    std::mutex command_mutex_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Go2CommandPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}