#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <hightorque_robot_ros2_example/msg/motor_state.hpp>

#include <fstream>
#include <string>

#include "robot.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("motors_set_zero");
    auto publisher = node->create_publisher<hightorque_robot_ros2_example::msg::MotorState>(
        "motor_states", 10);
    std::string config_path = ament_index_cpp::get_package_share_directory(
        "hightorque_robot_ros2_example") + "/robot_param/robot_config.yaml";
    std::ifstream config_file(config_path.c_str());
    if (!config_file.good())
    {
        config_path = std::string(HIGHTORQUE_ROBOT_SOURCE_DIR) +
                      "/robot_param/robot_config.yaml";
    }
    const std::string parameter_path = node->declare_parameter<std::string>(
        "robot_config", config_path);
    Robot robot(parameter_path);
    robot.motor_zero_pos_reset();
    rclcpp::WallRate rate(100.0);

    while (rclcpp::ok())
    {
        robot.request_motor_state();
        for (size_t i = 0; i < robot.can_ports.size(); ++i)
        {
            for (const auto& item : robot.can_ports[i]->map_motors_state)
            {
                const motor_state_t& state = item.second;
                hightorque_robot_ros2_example::msg::MotorState message;
                message.can_port_id = static_cast<uint8_t>(i + 1);
                message.id = item.first;
                message.mode = state.mode;
                message.fault = state.fault;
                message.position = state.position;
                message.velocity = state.velocity;
                message.torque = state.torque;
                publisher->publish(message);
                std::printf("ID: %2d, mode: %2d, fault: %2d, pos: %2.3f, vel: %2.3f, tor: %2.3f\n",
                            static_cast<int>(item.first), state.mode, state.fault,
                            state.position, state.velocity, state.torque);
            }
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    robot.stop();
    robot.send();
    rclcpp::shutdown();
    return 0;
}
