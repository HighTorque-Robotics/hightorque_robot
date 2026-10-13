
#ifndef _HIGHTORQUE_CANPORT_H
#define _HIGHTORQUE_CANPORT_H

#include "serial_struct.h"
#include "serial/serial.h"
#include "common_macros.h"
#include "version.h"

#include <vector>
#include <thread>
#include <map>



typedef struct 
{
    uint8_t mode;           // 电机模式
    uint8_t fault;          // 电机错误码，0为正常状态，非零为异常状态

    float position;         // 电机位置，默认弧度
    float velocity;         // 电机速度，默认弧度
    float torque;           // 电机力矩

    uint32_t num;           // 每次更新电机状态时 +1
    std::chrono::steady_clock::time_point time; // 记录上次电机状态更新时的时间

    version_s fw_version;   // 电机软件版本号，初始化时自动获取
    std::string model;      // 电机型号，初始化时自动获取
    std::string name;       // 自定义命名，比如关节名

    uint8_t flag;           // 标志位，目前只在电机重置零位时有用，用户无需关心
} motor_state_t;


class CanPort
{
public:
    fdcan_state_s can_port_state;
    std::map<int, motor_state_t> map_motors_state;

    CanPort(uint8_t _can_port_id, const std::map<uint8_t, std::string>& map_id_name);
    CanPort(uint8_t _can_port_id, std::initializer_list<int> id_list);
    ~CanPort();

    CanPort(const CanPort&) = delete;
    CanPort& operator=(const CanPort&) = delete;
    CanPort(CanPort&&) = delete;
    CanPort& operator=(CanPort&&) = delete;

    void position(uint8_t id, float pos);
    void velocity(uint8_t id, float vel);
    void torque(uint8_t id, float tqe);
    void vel_acc(uint8_t id, float vel, float acc);
    void pos_vel_acc(uint8_t id, float pos, float vel, float acc);
    void pos_vel_MAXtqe(uint8_t id, float pos, float vel, float max_tqe);
    void pos_vel_tqe_kp_kd(uint8_t id, float pos, float vel, float tqe, float kp, float kd);
    
    void stop();
    void stop(uint8_t id);

    void brake();
    void brake(uint8_t id);

    void reset();
    void reset(uint8_t id);

    void request_motor_state(void);
    uint8_t motor_zero_pos_reset(void);

    motor_state_t *get_motor_state(uint8_t id);

    void send(void);
    fdcan_state_s *get_can_port_state(void);
    version_s get_sdk_version(void);
    version_s get_comm_version(void);
private:
    version_s comm_version;  // 通信板版本号
    version_s sdk_version;   // SDK 版本号
    uint8_t can_port_id = 0;

    prot_cdc2comm_s prot_tdata;
    serial::Serial ser_dev;
    std::thread ser_recv_thread;
    int id_max = 0;

    uint8_t comm_init_flag = 0;
    uint8_t set_cache_num_flag = 0;

    void init(uint8_t _can_port_id, const std::map<uint8_t, std::string>& map_id_name);
    
    std::vector<std::string> get_ser_list();
    void ser_init(std::string port_name);
    void port_tdata_clean(const uint8_t mode, const uint16_t len);
    void comm_init(void);
    void update_comm_version(void);
    void set_cache_num(uint8_t num);

    uint16_t get_data_len(uint8_t mode, uint16_t num);
    void motor_tdata_clean(uint8_t cmd);

    void request_motor_version(void);
    void check_motor_version(void);

    void request_motor_model(void);
    void check_motor_model(void);

    void request_zero_pos_reset(void);

    void recv();
};

#endif
