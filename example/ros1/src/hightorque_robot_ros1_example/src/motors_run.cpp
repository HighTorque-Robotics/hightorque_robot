#include <ros/ros.h>
#include <ros/package.h>
#include <hightorque_robot_ros1_example/MotorState.h>

#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>

#include "robot.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motors_run");
    ros::NodeHandle node;
    auto publisher = node.advertise<hightorque_robot_ros1_example::MotorState>(
        "motor_states", 10);
    std::string config_path = ros::package::getPath("hightorque_robot_ros1_example") +
                              "/robot_param/robot_config.yaml";
    std::ifstream config_file(config_path.c_str());
    if (!config_file.good())
    {
        config_path = std::string(HIGHTORQUE_ROBOT_SOURCE_DIR) +
                      "/robot_param/robot_config.yaml";
    }
    node.param<std::string>("robot_config", config_path, config_path);
    Robot robot(config_path);
    float position = 0.314f;
    auto tick = std::chrono::steady_clock::now();
    ros::Rate rate(100.0);

    while (ros::ok())
    {
        if (std::chrono::steady_clock::now() - tick >= std::chrono::seconds(1))
        {
            tick = std::chrono::steady_clock::now();
            position *= -1.0f;
        }
        for (Motor& motor : robot.motors)
        {
            motor.pos_vel_acc(position, 0.314f, 3.14f);
        }
        robot.send();
        for (size_t i = 0; i < robot.can_ports.size(); ++i)
        {
            for (const auto& item : robot.can_ports[i]->map_motors_state)
            {
                const motor_state_t& state = item.second;
                hightorque_robot_ros1_example::MotorState motor;
                motor.can_port_id = static_cast<uint8_t>(i + 1);
                motor.id = item.first;
                motor.mode = state.mode;
                motor.fault = state.fault;
                motor.position = state.position;
                motor.velocity = state.velocity;
                motor.torque = state.torque;
                publisher.publish(motor);
                std::printf("ID: %2d, mode: %2d, fault: %2d, pos: %2.3f, vel: %2.3f, tor: %2.3f\n",
                            static_cast<int>(item.first), state.mode, state.fault,
                            state.position, state.velocity, state.torque);
            }
        }
        ros::spinOnce();
        rate.sleep();
    }

    robot.stop();
    robot.send();
    return 0;
}
