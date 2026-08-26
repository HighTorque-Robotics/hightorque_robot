#include <rclcpp/rclcpp.hpp>
#include <cstdio>
#include <hightorque_robot_ros2_example/msg/motor_state.hpp>

#include "canport.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("canport_feedback");
    auto publisher = node->create_publisher<hightorque_robot_ros2_example::msg::MotorState>(
        "motor_states", 10);
    CanPort can_port(1, {1});
    rclcpp::WallRate rate(100.0);

    while (rclcpp::ok())
    {
        can_port.request_motor_state();
        for (const auto& item : can_port.map_motors_state)
        {
            const motor_state_t& state = item.second;
            hightorque_robot_ros2_example::msg::MotorState motor;
            motor.can_port_id = 1;
            motor.id = item.first;
            motor.mode = state.mode;
            motor.fault = state.fault;
            motor.position = state.position;
            motor.velocity = state.velocity;
            motor.torque = state.torque;
            publisher->publish(motor);
            std::printf("ID: %2d, mode: %2d, fault: %2d, pos: %2.3f, vel: %2.3f, tor: %2.3f\n",
                        static_cast<int>(item.first), state.mode, state.fault,
                        state.position, state.velocity, state.torque);
        }
        rclcpp::spin_some(node);
        rate.sleep();
    }

    can_port.stop();
    can_port.send();
    rclcpp::shutdown();
    return 0;
}
