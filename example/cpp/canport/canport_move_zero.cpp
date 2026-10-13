#include "canport.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

static std::atomic<bool> exit_flag(false);


int main()
{
    constexpr auto kControlPeriod = std::chrono::microseconds(1000);
    std::signal(SIGINT, [](int) { exit_flag.store(true); });

    CanPort can_port(1, {1, 2, 3});

    auto next_tick = std::chrono::steady_clock::now();
    while (!exit_flag.load())
    {
        for (const auto& item : can_port.map_motors_state)
        {
            const float id = item.first;
            can_port.pos_vel_acc(id, 0.0f, 0.314f, 3.14f);
        }
        can_port.send();

        for (const auto& item : can_port.map_motors_state)
        {
            const float id = item.first;
            const motor_state_t& motor = item.second;
            std::printf("ID: %2d, mode: %2d, fault: %2d, pos: %2.3f, vel: %2.3f, tor: %2.3f\n",
                        static_cast<int>(id), motor.mode, motor.fault,
                        motor.position, motor.velocity, motor.torque);
        }

        next_tick += kControlPeriod;
        std::this_thread::sleep_until(next_tick);
    }

    can_port.stop();
    can_port.send();
    return 0;
}