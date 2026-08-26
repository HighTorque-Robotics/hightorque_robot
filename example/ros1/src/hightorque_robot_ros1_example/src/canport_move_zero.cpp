#include <ros/ros.h>
#include <hightorque_robot_ros1_example/MotorState.h>

#include <cstdio>

#include "canport.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "canport_set_zero");
    ros::NodeHandle node;
    auto publisher = node.advertise<hightorque_robot_ros1_example::MotorState>(
        "motor_states", 10);
    CanPort can_port(1, {1, 2, 3});
    ros::Rate rate(100.0);

    while (ros::ok())
    {
        for (const auto& item : can_port.map_motors_state)
        {
            can_port.pos_vel_acc(static_cast<uint8_t>(item.first), 0.0f, 0.314f, 3.14f);
        }
        can_port.send();
        for (const auto& item : can_port.map_motors_state)
        {
            const motor_state_t& state = item.second;
            hightorque_robot_ros1_example::MotorState motor;
            motor.can_port_id = 1;
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
        ros::spinOnce();
        rate.sleep();
    }

    can_port.stop();
    can_port.send();
    return 0;
}
