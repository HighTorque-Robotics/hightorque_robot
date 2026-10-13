#include "canport.h"
#include "crc.h"
#include "convert.h"

#include <algorithm>
#include <cctype>
#include <string>


// #define  CANPORT_LOG_PATH "./log"
#ifdef CANPORT_LOG_PATH
#include <mutex>

static std::mutex canport_log_mutex[256];
static FILE *canport_log_file[256] = {};
#endif


static std::vector<std::string> ser_list{};



CanPort::CanPort(uint8_t _can_port_id, const std::map<uint8_t, std::string>& map_id_name)
{
    init(_can_port_id, map_id_name);
}


CanPort::CanPort(uint8_t _can_port_id, std::initializer_list<int> id_list)
{
    std::map<uint8_t, std::string> map_id_name;

    for (int id : id_list)
    {
        map_id_name.insert({static_cast<uint8_t>(id), ""});
    }

    init(_can_port_id, map_id_name);
}


CanPort::~CanPort()
{
    PRINT_INFO_G("[CanPort%d] close", can_port_id);
    ser_dev.close();

    if (ser_recv_thread.joinable())
    {
        ser_recv_thread.join();
    }

#ifdef CANPORT_LOG_PATH
    std::lock_guard<std::mutex> lock(canport_log_mutex[can_port_id]);
    if (canport_log_file[can_port_id] != nullptr)
    {
        std::fclose(canport_log_file[can_port_id]);
        canport_log_file[can_port_id] = nullptr;
    }
#endif
}


void CanPort::init(uint8_t _can_port_id, const std::map<uint8_t, std::string>& map_id_name)
{
    sdk_version.data32 = HIGHTORQUE_ROBOT_VERSION;
    comm_version.data32 = 0;

    PRINT_INFO_G("SDK version: v%d.%d.%d", sdk_version.major, sdk_version.minor, sdk_version.patch);

    if (_can_port_id < 1)
    {
        PRINT_ERROR("can_port_id err: got %d, must be >= 1", _can_port_id);
        exit(1);
    }
    can_port_id = _can_port_id;

#ifdef CANPORT_LOG_PATH
    {
        std::string canport_log_file_path = CANPORT_LOG_PATH;
        if (!canport_log_file_path.empty() && canport_log_file_path.back() != '/')
        {
            canport_log_file_path += '/';
        }
        canport_log_file_path += "canport_" + std::to_string(can_port_id) + ".log";

        std::lock_guard<std::mutex> lock(canport_log_mutex[can_port_id]);
        canport_log_file[can_port_id] = std::fopen(canport_log_file_path.c_str(), "a");
        if (canport_log_file[can_port_id] == nullptr)
        {
            PRINT_ERROR("[CanPort%d] log file open err: %s", can_port_id, canport_log_file_path.c_str());
        }
    }
#endif

    const size_t id_num = map_id_name.size();
    if (id_num < 1 || id_num > 30)
    {
        PRINT_ERROR("map_id_name err: got %zu, valid range [1, 30]", id_num);
        exit(1);
    }

    for (const auto& it : map_id_name)
    {
        const uint8_t id = it.first;
        if (id < 1 || id > 30)
        {
            PRINT_ERROR("map_id_name err: id %d, valid range [1, 30]", id);
            exit(1);
        }
        if (id_max < id)
        {
            id_max = id;
        }
        motor_state_t state = {};
        state.position = 999.0f;
        state.name = it.second;
        map_motors_state.emplace(id, state);
    }

    if (ser_list.size() == 0)
    {
        ser_list = get_ser_list();
        if (ser_list.size() < 1)
        {
            PRINT_ERROR("no comm board serial ports found (VID:PID=CAF1:FFFF)");
            exit(1);
        }
    }

    if (_can_port_id > ser_list.size())
    {
        PRINT_ERROR("[CanPort%d] not enough serial ports: found %zu, need %d", _can_port_id, ser_list.size(), _can_port_id);
        exit(1);
    }
    PRINT_INFO_G("[CanPort%d] init", can_port_id);
    ser_init(ser_list[can_port_id - 1]);
    update_comm_version();
    if (comm_version.data32 < VER_COMBINE(6, 0, 0))
    {
        PRINT_ERROR("[CanPort%d] comm board version too low: v%d.%d.%d, require >= v6.0.0", can_port_id, comm_version.major, comm_version.minor, comm_version.patch);
        exit(1);
    }

    comm_init();
    set_cache_num(map_motors_state.size());

    check_motor_version();
    check_motor_model();

    request_motor_state();
    request_motor_state();

    PRINT_INFO_G("[CanPort%d] init ok", can_port_id);
}


void CanPort::port_tdata_clean(const uint8_t mode, const uint16_t len)
{
    if (prot_tdata.head.s.cmd != mode)
    {
        prot_tdata.head.s.head = PROT_HEAD;
        prot_tdata.head.s.cmd = mode;
        prot_tdata.head.s.len = len;
        memset(&prot_tdata.data, 0, len);
    }
}


std::vector<std::string> CanPort::get_ser_list()
{
    std::vector<serial::PortInfo> all_ports = serial::list_ports();
    std::vector<serial::PortInfo> com_board_ports;

    for (const auto& port_info : all_ports)
    {
        std::string hw = port_info.hardware_id;
        std::transform(hw.begin(), hw.end(), hw.begin(),
            [](unsigned char ch) { return static_cast<char>(std::toupper(ch)); });

        const bool linux_id = hw.find("VID:PID=CAF1:FFFF") != std::string::npos;
        const bool windows_id = hw.find("VID_CAF1") != std::string::npos &&
                                hw.find("PID_FFFF") != std::string::npos;

        if (linux_id || windows_id)
        {
            com_board_ports.push_back(port_info);
        }
    }

#ifdef _WIN32
    std::sort(com_board_ports.begin(), com_board_ports.end(),
        [](const serial::PortInfo& left, const serial::PortInfo& right)
        {
            return left.hardware_id < right.hardware_id;
        });
#endif

    PRINT_INFO("detected %zu serial port(s):", com_board_ports.size());
    for (uint8_t i = 0; i < com_board_ports.size(); i++)
    {
        PRINT_INFO("  [CanPort%d] serial -> %s", i + 1, com_board_ports[i].port.c_str());
    }

    std::vector<std::string> port_names;
    port_names.reserve(com_board_ports.size());
    for (const auto& port_info : com_board_ports)
    {
        port_names.push_back(port_info.port);
    }

    return port_names;
}


void CanPort::ser_init(std::string port_name)
{
    PRINT_INFO_G("[CanPort%d] serial port: %s", can_port_id, port_name.c_str());

    ser_dev.setPort(port_name.c_str());
    ser_dev.setBaudrate(4000000);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    ser_dev.setTimeout(timeout);

    try
    {
        ser_dev.open();
    }
    catch(const std::exception& e)
    {
        PRINT_ERROR("[CanPort%d] serial open err: %s, %s", can_port_id, port_name.c_str(), e.what());
        exit(1);
    }
    
    PRINT_INFO_G("[CanPort%d] serial port %s init ok", can_port_id, port_name.c_str());
    ser_recv_thread = std::thread(&CanPort::recv, this);
}


void CanPort::update_comm_version()
{
    port_tdata_clean(MODE_COMM_VERSION, 0);

    for (int i = 0; i < 5; i++)
    {
        send();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (comm_version.data32 >= VER_COMBINE(3, 0, 0))
        {
            PRINT_INFO_G("[CanPort%d] comm board version: v%d.%d.%d", can_port_id, comm_version.major, comm_version.minor, comm_version.patch);
            return;
        }
    }

    PRINT_ERROR("[CanPort%d] failed to get comm board version", can_port_id);
    exit(1);
}


void CanPort::comm_init()
{
    port_tdata_clean(MODE_COMM_INIT, 0);

    prot_tdata.head.s.len = 0;

    comm_init_flag = 0;
    for (int i = 0; i < 5; i++)
    {
        send();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (comm_init_flag != 0)
        {
            PRINT_INFO_G("[CanPort%d] comm init ok", can_port_id);
            return;
        }
    }

    PRINT_ERROR("[CanPort%d] comm init err", can_port_id);
    exit(1);
}


void CanPort::set_cache_num(uint8_t num)
{
    port_tdata_clean(MODE_CACHE_NUM, 1);

    prot_tdata.data.raw[0] = num;

    set_cache_num_flag = 0;
    for (int i = 0; i < 5; i++)
    {
        send();
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        if (set_cache_num_flag != 0)
        {
            PRINT_INFO_G("[CanPort%d] set cache num ok: %d", can_port_id, num);
            return;
        }
    }

    PRINT_ERROR("[CanPort%d] set cache num err: %d", can_port_id, num);
    exit(1);
}


uint16_t CanPort::get_data_len(uint8_t mode, uint16_t num)
{
    uint8_t motor_one_len = 0;
    switch (mode)
    {
        case MODE_STOP:
        case MODE_BRAKE:
            motor_one_len = 1;
            break;
        case MODE_POSITION:
        case MODE_VELOCITY:
        case MODE_TORQUE:
        case MODE_VOLTAGE:
        case MODE_CURRENT:
            motor_one_len = 2;
            break;
        case MODE_VEL_ACC:
            motor_one_len = 4;
            break;
        case MODE_POS_VEL_TQE:
        case MODE_POS_VEL_ACC:
            motor_one_len = 6;
            break;
        case MODE_POS_VEL_TQE_KP_KD:
            motor_one_len = 10;
            break;
        default:
            motor_one_len = 0;
            PRINT_ERROR("mode %d has been deprecated", mode);
            exit(1);
    }

    uint8_t fdcan_one_len = 60;
    const uint16_t len = motor_one_len * num;
    const uint8_t mul = len / fdcan_one_len;
    const uint8_t rem = len % fdcan_one_len;
    uint8_t rem_len = 0;

    if (rem <= 7)
    {
        rem_len = rem;
    }
    else if (rem <= 11)
    {
        rem_len = 11;
    }
    else if (rem <= 15)
    {
        rem_len = 15;
    }
    else if (rem <= 19)
    {
        rem_len = 19;
    }
    else if (rem <= 23)
    {
        rem_len = 23;
    }
    else if (rem <= 31)
    {
        rem_len = 31;
    }
    else if (rem <= 47)
    {
        rem_len = 47;
    }
    else 
    {
        rem_len = fdcan_one_len;
    }

    return mul * fdcan_one_len + rem_len + 2;
}


void CanPort::motor_tdata_clean(uint8_t cmd)
{
    if (prot_tdata.head.s.cmd != cmd)
    {
        prot_tdata.head.s.head = PROT_HEAD;
        prot_tdata.head.s.cmd = cmd;
        prot_tdata.head.s.len = get_data_len(cmd, id_max);

        if (cmd >= MODE_STOP && cmd <= MODE_BRAKE)
        {
            for (int i = 0; i < id_max; i++)
            {
                prot_tdata.data.contr.raw8[i] = 0;
            }
        }
        else
        {
            for (int i = 0; i < id_max; i++)
            {
                prot_tdata.data.contr.raw16[i] = 0x8000;
            }
        }
        prot_tdata.data.contr.data_type = TINT16_NOHDR;
        prot_tdata.data.contr.query = QUERY_MODE_FAULT_POS_VEL_TQE;
    }
}


void CanPort::position(uint8_t id, float pos)
{
    motor_tdata_clean(MODE_POSITION);
    prot_tdata.data.contr.raw16[id -1] = pos_float2int(pos);
}


void CanPort::velocity(uint8_t id, float vel)
{
    motor_tdata_clean(MODE_VELOCITY);
    prot_tdata.data.contr.raw16[id -1] = vel_float2int(vel);
}


void CanPort::torque(uint8_t id, float tqe)
{
    motor_tdata_clean(MODE_TORQUE);
    prot_tdata.data.contr.raw16[id -1] = tqe_float2int(tqe);
}


void CanPort::vel_acc(uint8_t id, float vel, float acc)
{
    motor_tdata_clean(MODE_VEL_ACC);
    prot_tdata.data.contr.va[id -1].vel = vel_float2int(vel);
    prot_tdata.data.contr.va[id -1].acc = acc_float2int(acc);
}


void CanPort::pos_vel_acc(uint8_t id, float pos, float vel, float acc)
{
    motor_tdata_clean(MODE_POS_VEL_ACC);
    prot_tdata.data.contr.pva[id -1].pos = pos_float2int(pos);
    prot_tdata.data.contr.pva[id -1].vel = vel_float2int(vel);
    prot_tdata.data.contr.pva[id -1].acc = acc_float2int(acc);
}


void CanPort::pos_vel_MAXtqe(uint8_t id, float pos, float vel, float max_tqe)
{   
    motor_tdata_clean(MODE_POS_VEL_TQE);
    prot_tdata.data.contr.pvt[id -1].pos = pos_float2int(pos);
    prot_tdata.data.contr.pvt[id -1].vel = vel_float2int(vel);
    prot_tdata.data.contr.pvt[id -1].tqe = tqe_float2int(max_tqe);
}   


void CanPort::pos_vel_tqe_kp_kd(uint8_t id, float pos, float vel, float tqe, float kp, float kd)
{
    motor_tdata_clean(MODE_POS_VEL_TQE_KP_KD);
    prot_tdata.data.contr.pvtpd[id -1].pos = pos_float2int(pos);
    prot_tdata.data.contr.pvtpd[id -1].vel = vel_float2int(vel);
    prot_tdata.data.contr.pvtpd[id -1].tqe = tqe_float2int(tqe);
    prot_tdata.data.contr.pvtpd[id -1].kp = kp_float2int(kp);
    prot_tdata.data.contr.pvtpd[id -1].kd = kd_float2int(kd);
}


void CanPort::stop(uint8_t id)
{
    motor_tdata_clean(MODE_STOP);
    prot_tdata.data.contr.raw8[id - 1] = 1;
}


void CanPort::stop()
{
    motor_tdata_clean(MODE_STOP);
    for (auto it : map_motors_state)
    {
        prot_tdata.data.contr.raw8[it.first - 1] = 1;
    }
}


void CanPort::reset(uint8_t id)
{
    motor_tdata_clean(MODE_RESET);
    prot_tdata.data.contr.raw8[id - 1] = 1;
}


void CanPort::reset()
{
    motor_tdata_clean(MODE_RESET);
    for (auto it : map_motors_state)
    {
        prot_tdata.data.contr.raw8[it.first - 1] = 1;
    }
}


void CanPort::brake(uint8_t id)
{
    motor_tdata_clean(MODE_BRAKE);
    prot_tdata.data.contr.raw8[id - 1] = 1;
}


void CanPort::brake()
{
    motor_tdata_clean(MODE_BRAKE);
    for (auto it : map_motors_state)
    {
        prot_tdata.data.contr.raw8[it.first - 1] = 1;
    }
}


void CanPort::request_motor_version()
{
    port_tdata_clean(MODE_MOTOR_VERSION, 0);
    send();
}


void CanPort::request_motor_state()
{
    port_tdata_clean(MODE_MOTOR_STATE, 2);
    prot_tdata.data.contr.data_type = TINT16_NOHDR;
    prot_tdata.data.contr.query = QUERY_MODE_FAULT_POS_VEL_TQE;
    send();
}


void CanPort::request_zero_pos_reset()
{
    port_tdata_clean(MODE_MOTOR_POS_RESET, id_max);

    for (auto it : map_motors_state)
    {
        prot_tdata.data.raw[it.first - 1] = 1;
    }

    send();
}

uint8_t CanPort::motor_zero_pos_reset()
{
    stop();
    send();
    send();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::vector<uint8_t> failed_id_list;
    for (auto it : map_motors_state)
    {
        it.second.flag = 0;
    }

    for (int i = 0; i < 10; i++)
    {
        request_zero_pos_reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        failed_id_list.clear();
        for (auto it : map_motors_state)
        {
            if (it.second.flag == 0)
            {
                failed_id_list.push_back(it.first);
            }
        }

        if (failed_id_list.size() == 0)
        {
            PRINT_INFO_G("[CanPort%d] motor zero pos reset ok, %zu motor(s)", can_port_id, map_motors_state.size());
            return 0;
        }
    }

    PRINT_ERROR("[CanPort%d] motor zero pos reset err, %zu motor(s) failed:", can_port_id, failed_id_list.size());
    for (auto it : failed_id_list)
    {
        PRINT_ERROR("  motor[%d]", it);
    }

    return 1;
}


void CanPort::check_motor_version()
{
    std::vector<uint8_t> failed_id_list;

    for (int i = 0; i < 100; i++)
    {
        request_motor_version();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        failed_id_list.clear();
        for (auto it : map_motors_state)
        {
            if (it.second.fw_version.data32 == 0)
            {
                failed_id_list.push_back(it.first);
            }
        }

        if (failed_id_list.size() == 0)
        {
            return;
        }
    }

}


void CanPort::request_motor_model()
{
    port_tdata_clean(MODE_MOTOR_MODEL, 0);
    send();
}


void CanPort::check_motor_model()
{
    std::vector<uint8_t> failed_id_list;

    for (int i = 0; i < 100; i++)
    {
        request_motor_model();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));

        failed_id_list.clear();
        for (auto it : map_motors_state)
        {
            if (it.second.fw_version.data32 == 0 || it.second.model.size() == 0)
            {
                failed_id_list.push_back(it.first);
            }
        }

        if (failed_id_list.size() == 0)
        {
            for (auto it : map_motors_state)
            {
                PRINT_INFO("[CanPort%d] motor[%d] fw version: v%d.%d.%d, model: %s", can_port_id, it.first,
                    it.second.fw_version.major, it.second.fw_version.minor, it.second.fw_version.patch,
                    it.second.model.c_str());
            }
            return;
        }
    }

    PRINT_ERROR("[CanPort%d] motor fw version/model check err, %zu motor(s) failed:", can_port_id, failed_id_list.size());
    for (auto id : failed_id_list)
    {
        const auto& motor = map_motors_state.at(id);
        PRINT_ERROR("[CanPort%d] motor[%d] fw version: v%d.%d.%d, model: %s", can_port_id, id,
            motor.fw_version.major, motor.fw_version.minor, motor.fw_version.patch,
            motor.model.c_str());
    }
}


motor_state_t *CanPort::get_motor_state(uint8_t id)
{
    auto it = map_motors_state.find(id);
    if (it == map_motors_state.end())
    {
        PRINT_ERROR("[CanPort%d] motor[%d] not found", can_port_id, id);
        return nullptr;
    }

    return &(it->second);
}


fdcan_state_s *CanPort::get_can_port_state()
{
    return &can_port_state;
}


version_s CanPort::get_sdk_version(void)
{
    return sdk_version;
}


version_s CanPort::get_comm_version(void)
{
    return comm_version;
}


void CanPort::send()
{
    prot_tdata.head.s.crc_head = crc8_ccitt(&(prot_tdata.head.raw[1]), 3);
    prot_tdata.head.s.crc_data = crc8_ccitt(&(prot_tdata.data.raw[0]), prot_tdata.head.s.len);

    #if 0
    uint8_t *byte_ptr = (uint8_t *)&prot_tdata.head.s.head;
    printf("send:\n");
    for (size_t i = 0; i < prot_tdata.head.s.len + 6; i++)
    {
        printf("0x%.2X ", byte_ptr[i]);
    }
    printf("\n\n");
    #endif

#ifdef CANPORT_LOG_PATH
    {
        std::lock_guard<std::mutex> lock(canport_log_mutex[can_port_id]);
        if (canport_log_file[can_port_id] != nullptr)
        {
            uint8_t *byte_ptr = (uint8_t *)&prot_tdata.head.s.head;
            std::fprintf(canport_log_file[can_port_id], "[CanPort%d] tx:", can_port_id);
            for (size_t i = 0; i < prot_tdata.head.s.len + sizeof(prot_head_s); i++)
            {
                std::fprintf(canport_log_file[can_port_id], " 0x%.2X", byte_ptr[i]);
            }
            std::fprintf(canport_log_file[can_port_id], "\n");
            std::fflush(canport_log_file[can_port_id]);
        }
    }
#endif

    try
    {
        if(ser_dev.isOpen())
        {
            ser_dev.write((const uint8_t *)&prot_tdata.head.s.head, prot_tdata.head.s.len + sizeof(prot_head_s));
        }
    }
    catch(const std::exception& e)
    {
        PRINT_ERROR("[CanPort%d] serial send err: %s", can_port_id, e.what());
        ser_dev.close();
    }
}


void CanPort::recv()
{
    prot_comm2cdc_s prot_rdata = {0};

    while(ser_dev.isOpen())
    {
        prot_rdata.head.s.head = 0;
        prot_rdata.head.s.cmd = 0;
        prot_rdata.head.s.len = 0;
        try
        {  
            ser_dev.read(&prot_rdata.head.raw[0], 1);
            const std::chrono::steady_clock::time_point time = std::chrono::steady_clock::now();
            for (auto &it : map_motors_state)
            {
                if (time - it.second.time >= std::chrono::milliseconds(200))
                {
                    it.second.position = 999.0f;
                    it.second.time = time;
                }
            }

            if (prot_rdata.head.s.head != PROT_HEAD)
            {
                continue;
            }

            ser_dev.read(&prot_rdata.head.raw[1], 4);
            if (prot_rdata.head.s.crc_head != crc8_ccitt(&prot_rdata.head.raw[1], 3))
            {
                continue;
            }

            ser_dev.read(&prot_rdata.head.raw[5], prot_rdata.head.s.len + 1);
            if (prot_rdata.head.s.crc_data != crc8_ccitt(&prot_rdata.data.raw[0], prot_rdata.head.s.len))
            {
                continue;
            }

            #if 0
            printf("cmd %02X, len = %d   ", prot_rdata.head.s.cmd, prot_rdata.head.s.len);
            for (int i = 0; i < prot_rdata.head.s.len; i++)
            {
                printf("0x%02X ", prot_rdata.data.raw[i]);
            }
            printf("\n");
            #endif

            #ifdef CANPORT_LOG_PATH
            {
                std::lock_guard<std::mutex> lock(canport_log_mutex[can_port_id]);
                if (canport_log_file[can_port_id] != nullptr)
                {
                    uint8_t *byte_ptr = (uint8_t *)&prot_rdata.head.s.head;
                    std::fprintf(canport_log_file[can_port_id], "[CanPort%d] rx:", can_port_id);
                    for (size_t i = 0; i < prot_rdata.head.s.len + sizeof(prot_head_s); i++)
                    {
                        std::fprintf(canport_log_file[can_port_id], " 0x%.2X", byte_ptr[i]);
                    }
                    std::fprintf(canport_log_file[can_port_id], "\n");
                    std::fflush(canport_log_file[can_port_id]);
                }
            }
            #endif

            can_port_state = prot_rdata.data.s.fdcan_state;
            if (can_port_state.fault > FDCAN_STATUS_ERROR_WARNING || can_port_state.fault == FDCAN_STATUS_UNKNOWN)
            {
                PRINT_ERROR("[CanPort%d] FDCAN fault: %d, rx_err: %d, tx_err: %d", can_port_id, can_port_state.fault, can_port_state.rx_err_num, can_port_state.tx_err_num);
            }
            else if (can_port_state.fault == FDCAN_STATUS_ERROR_WARNING)
            {
                PRINT_INFO("[CanPort%d] FDCAN warning, fault: %d, rx_err: %d, tx_err: %d", can_port_id, can_port_state.fault, can_port_state.rx_err_num, can_port_state.tx_err_num);
            }

            switch (prot_rdata.head.s.cmd)
            {
            case MODE_COMM_VERSION:
                comm_version.major = prot_rdata.data.s.version.major;
                comm_version.minor = prot_rdata.data.s.version.minor;
                comm_version.patch = prot_rdata.data.s.version.patch;
                break;
            case MODE_COMM_INIT:
                comm_init_flag = prot_rdata.data.s.raw[0];
                break;
            case MODE_CACHE_NUM:
                set_cache_num_flag = prot_rdata.data.s.raw[0];
                break;
            case MODE_MOTOR_STATE:
                switch (prot_rdata.data.s.motor_state.query)
                {
                case QUERY_MODE_FAULT_POS_VEL_TQE:
                    for (int i = 0; i < (prot_rdata.head.s.len - 1 - 3) / sizeof(query_mode_fault_pos_vel_tqe_t); i++)
                    {
                        auto it = map_motors_state.find(prot_rdata.data.s.motor_state.mfpvt[i].id);
                        if (it != map_motors_state.end())
                        {
                            it->second.mode = prot_rdata.data.s.motor_state.mfpvt[i].mode;
                            it->second.fault = prot_rdata.data.s.motor_state.mfpvt[i].fault;

                            it->second.position = pos_int2float(prot_rdata.data.s.motor_state.mfpvt[i].pos);
                            it->second.velocity = vel_int2float(prot_rdata.data.s.motor_state.mfpvt[i].vel);
                            it->second.torque = tqe_int2float(prot_rdata.data.s.motor_state.mfpvt[i].tqe);
                            it->second.num++;
                            it->second.time = std::chrono::steady_clock::now();
                        }
                    }
                    break;
                }
                break;
            case MODE_MOTOR_VERSION:
                for (int i = 0; i < (prot_rdata.head.s.len - 3) / sizeof(prot_motor_version_t); i++)
                {
                    auto it = map_motors_state.find(prot_rdata.data.s.motor_version[i].id);
                    if (it != map_motors_state.end())
                    {
                        it->second.fw_version.major = prot_rdata.data.s.motor_version->major;
                        it->second.fw_version.minor = prot_rdata.data.s.motor_version->minor;
                        it->second.fw_version.patch = prot_rdata.data.s.motor_version->patch;
                    }
                }
                break;
            case MODE_MOTOR_MODEL:
                for (int i = 0; i < (prot_rdata.head.s.len - 3) / sizeof(prot_motor_model_t); i++)
                {
                    auto it = map_motors_state.find(prot_rdata.data.s.motor_model[i].id);
                    if (it != map_motors_state.end())
                    {
                        it->second.model = std::string(prot_rdata.data.s.motor_model[i].data, prot_rdata.data.s.motor_model[i].len);
                    }
                }
                break;
            case MODE_MOTOR_POS_RESET:
                for (int i = 0; i < (prot_rdata.head.s.len - 3) / sizeof(prot_motor_flag_t); i++)
                {
                    auto it = map_motors_state.find(prot_rdata.data.s.motor_flag[i].id);
                    if (it != map_motors_state.end())
                    {
                        it->second.flag = !prot_rdata.data.s.motor_flag[i].flag;
                    }
                }
                break;
            default:
                break;
            }
        }
        catch(const std::exception& e)
        {
            ser_dev.close();
        }
    }
}
