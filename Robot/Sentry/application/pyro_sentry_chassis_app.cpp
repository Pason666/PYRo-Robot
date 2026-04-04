#include "pyro_sentry_func_config.h"
#if BOARD_ID == CHASSIS_ID

#include "pyro_rud_chassis.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_yaw.h"
#include "pyro_uart_drv.h"
#include "pyro_referee.h"
#include "pyro_com_cantx.h"
#include "pyro_com_canrx.h"
#include "pyro_crc.h"
#include "pyro_uart_comm.h"
#include "pyro_powermeter.h"
#include "protocol.h"
#include "pyro_sentry_message_frame.h"

using namespace pyro;

// ------------------- 函数声明 -------------------
void chassis_config(rud_cfg_t &rud_cfg);
void yaw_config(yaw_cfg_t &yaw_cfg);
void gimbal2chassis();
void imu2chassis();
void referee_process(const referee_drv_t *referee_drv);
void chassis2gimbal();
void mcu2nav_process();
void sentry_chassis_thread(void *argument);
status_t sentry_chassis_init(void *argument);

// ------------------- 全局对象定义 -------------------
rud_chassis_t *rud_chassis_ptr             = nullptr;
rud_cmd_t *rud_cmd_ptr                     = nullptr;
rud_cfg_t *rud_cfg_ptr                     = nullptr;

yaw_t *yaw_ptr                             = nullptr;
yaw_cmd_t *yaw_cmd_ptr                     = nullptr;
yaw_cfg_t *yaw_cfg_ptr                     = nullptr;

uart_comm_t *comm                          = nullptr;
referee_data_t referee_data{};

// ------------------- 功率计相关 -------------------
/* powermeter_drv_t *power_meter;
 powermeter_data power_data; */

__attribute__((section(".dma_heap"))) sentry_cmd_t sentry_cmd;
__attribute__((section(".dma_heap"))) nav2mcu_msg_t nav2mcu_msg;
__attribute__((section(".dma_heap"))) mcu2nav_msg_t mcu2nav_msg;

void chassis_config(rud_cfg_t &rud_cfg)
{
    rud_cfg.motor.rudder[0] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                    can_hub_t::can2); // FL Rudder
    rud_cfg.motor.rudder[1] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                    can_hub_t::can2); // BL Rudder
    rud_cfg.motor.rudder[2] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                    can_hub_t::can1); // BR Rudder
    rud_cfg.motor.rudder[3] =
        new dji_gm_6020_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                    can_hub_t::can1); // FR Rudder

    rud_cfg.motor.wheel[0] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_1,
                                  can_hub_t::can2); // FL Wheel
    rud_cfg.motor.wheel[1] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                  can_hub_t::can2); // BL Wheel
    rud_cfg.motor.wheel[2] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                  can_hub_t::can1); // BR Wheel
    rud_cfg.motor.wheel[3] =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_4,
                                  can_hub_t::can1); // FR Wheel

    rud_cfg.pid.wheel_pid[0]   = new pyro::pid_t(15.0f, 0.0f, 0.00f, 0.00f, 20.0f);
    rud_cfg.pid.wheel_pid[1]   = new pyro::pid_t(15.0f, 0.0f, 0.00f, 0.00f, 20.0f);
    rud_cfg.pid.wheel_pid[2]   = new pyro::pid_t(15.0f, 0.0f, 0.00f, 0.00f, 20.0f);
    rud_cfg.pid.wheel_pid[3]   = new pyro::pid_t(15.0f, 0.0f, 0.00f, 0.00f, 20.0f);

    rud_cfg.pid.rud_pos_pid[0] = new pyro::pid_t(20.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    rud_cfg.pid.rud_pos_pid[1] = new pyro::pid_t(20.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    rud_cfg.pid.rud_pos_pid[2] = new pyro::pid_t(20.0f, 0.0f, 0.00f, 0.0f, 10.0f);
    rud_cfg.pid.rud_pos_pid[3] = new pyro::pid_t(20.0f, 0.0f, 0.00f, 0.0f, 10.0f);

    rud_cfg.pid.rud_spd_pid[0] = new pyro::pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    rud_cfg.pid.rud_spd_pid[1] = new pyro::pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    rud_cfg.pid.rud_spd_pid[2] = new pyro::pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);
    rud_cfg.pid.rud_spd_pid[3] = new pyro::pid_t(0.3f, 0.0f, 0.00f, 0.0f, 3.0f);

    rud_cfg.pid.follow_yaw_pid = new pyro::pid_t(14.0f, 0.0f, 1.8, 0, 30.0f);

    rud_cfg.rud_pos_moving_offset[0] = 1.01472831f;
    rud_cfg.rud_pos_moving_offset[1] = -0.29145637f;
    rud_cfg.rud_pos_moving_offset[2] = -1.87299052f;
    rud_cfg.rud_pos_moving_offset[3] = -1.04003897f;

    power_control_drv_t &power_controller =
        power_control_drv_t::get_instance(4);
    power_control_drv_t::motor_coefficient_t coef1{};
    coef1.k1 = 2.8755f;
    coef1.k2 = 8.3800f;
    coef1.k3 = 0.1760f;
    coef1.k4 = 0.5043f;
    power_controller.set_motor_coefficient(1, coef1);

    power_control_drv_t::motor_coefficient_t coef2{};
    coef2.k1 = 2.8755f;
    coef2.k2 = 8.3800f;
    coef2.k3 = 0.1760f;
    coef2.k4 = 0.5043f;
    power_controller.set_motor_coefficient(2, coef2);

    power_control_drv_t::motor_coefficient_t coef3{};
    coef3.k1 = 2.8755f;
    coef3.k2 = 8.3800f;
    coef3.k3 = 0.1760f;
    coef3.k4 = 0.5043f;
    power_controller.set_motor_coefficient(3, coef3);

    power_control_drv_t::motor_coefficient_t coef4{};
    coef4.k1 = 2.8755f;
    coef4.k2 = 8.3800f;
    coef4.k3 = 0.1760f;
    coef4.k4 = 0.5043f;
    power_controller.set_motor_coefficient(4, coef4);
}

void yaw_config(yaw_cfg_t &yaw_cfg)
{
    yaw_cfg.motor.yaw = new dm_motor_drv_t(0x11, 0x12, can_hub_t::can2);
    yaw_cfg.motor.yaw->set_position_range(-PI, PI);
    yaw_cfg.motor.yaw->set_rotate_range(-20, 20);
    yaw_cfg.motor.yaw->set_torque_range(-10, 10);

    yaw_cfg.pid.yaw_pos_pid = new pyro::pid_t(28, 0, 2, 1, 40);
    yaw_cfg.pid.yaw_spd_pid = new pyro::pid_t(1.3, 0, 0, 2, 7);

    yaw_cfg.yaw_offset      = 0.509001732f;
}

extern "C"
{
    void gimbal2chassis()
    {
        std::array<uint8_t, 8> raw_data{};
        if (can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x123, raw_data))
        {
            if (raw_data[4] >> 1 & 0x01)
            {
                rud_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;
                yaw_cmd_ptr->mode = cmd_base_t::mode_t::ACTIVE;
            }
            else
            {
                rud_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
                yaw_cmd_ptr->mode = cmd_base_t::mode_t::PASSIVE;
            }
            yaw_cmd_ptr->nav_enable = static_cast<bool>(raw_data[5]);
            rud_cmd_ptr->follow_yaw = static_cast<bool>(raw_data[4] & 0x01);
            if (cmd_base_t::mode_t::PASSIVE == yaw_cmd_ptr->mode)
            {
                yaw_cmd_ptr->target_yaw_imu_angle =
                    yaw_cmd_ptr->current_yaw_imu_rad;
            }
            if (yaw_cmd_ptr->nav_enable == false)
            {
                rud_cmd_ptr->vx =
                    2 * static_cast<float>(static_cast<int8_t>(raw_data[0])) /
                    127.0f;
                rud_cmd_ptr->vy =
                    2 * static_cast<float>(static_cast<int8_t>(raw_data[1])) /
                    127.0f;
                rud_cmd_ptr->wz =
                    static_cast<float>(static_cast<int8_t>(raw_data[2]));

                yaw_cmd_ptr->target_yaw_imu_angle -=
                    static_cast<float>(static_cast<int8_t>(raw_data[3])) /
                    127.0f * 0.005f;
                memset(&nav2mcu_msg, 0, sizeof(nav2mcu_msg));
            }
            else
            {
                rud_cmd_ptr->vx                   = nav2mcu_msg.data.vx;
                rud_cmd_ptr->vy                   = nav2mcu_msg.data.vy;
                rud_cmd_ptr->wz                   = 10;
                yaw_cmd_ptr->target_yaw_imu_angle = nav2mcu_msg.data.wz;
            }

            rud_cmd_ptr->yaw_error = yaw_ptr->get_yaw_error();
        }
    }

    void imu2chassis()
    {
        std::array<uint8_t, 8> raw_data{};
        can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x103, raw_data);
        float imu_angle;
        memcpy(&imu_angle, raw_data.data(), 4);
        if (imu_angle == 0)
            return;
        yaw_cmd_ptr->current_yaw_imu_rad = imu_angle / 180 * PI;
    }

    void referee_process(const referee_drv_t *referee_drv)
    {
        referee_data = referee_drv->get_data();
    }

    void chassis2gimbal()
    {
        const uint8_t bullet_speed_int =
            floor(referee_data.shoot.initial_speed);
        uint8_t bullet_speed_dec =
            static_cast<uint8_t>(referee_data.shoot.initial_speed -
                                 static_cast<float>(bullet_speed_int)) *
            100;
        uint8_t enemy_color;
        if (referee_data.robot_status.robot_id > 100)
            enemy_color = 1;
        else
            enemy_color = 0;
        bool game_started   = referee_data.game_status.game_progress == 4;
        uint8_t power_heat =
            referee_data.power_heat.shooter_17mm_barrel_heat / 10;

        can_tx_drv_t::clear(0x102);
        can_tx_drv_t::add_data(0x102, 8, bullet_speed_int);
        can_tx_drv_t::add_data(0x102, 8, bullet_speed_dec);
        can_tx_drv_t::add_data(0x102, 8, enemy_color);
        can_tx_drv_t::add_data(0x102, 8, game_started);
        can_tx_drv_t::add_data(0x102, 8, power_heat);
        can_tx_drv_t::send(0x102, can_hub_t::get_instance()->hub_get_can_obj(
                                      can_hub_t::which_can::can3));
    }

    void mcu2nav_process()
    {
        mcu2nav_msg.data.self_hp = referee_data.robot_status.current_hp;
        mcu2nav_msg.data.self_ammo =
            referee_data.allowance.projectile_allowance_17mm;
        mcu2nav_msg.data.game_state   = referee_data.game_status.game_progress;
        mcu2nav_msg.data.self_base_hp = referee_data.game_robot_hp.base_hp;
        mcu2nav_msg.data.self_outpost_hp =
            referee_data.game_robot_hp.outpost_hp;
        mcu2nav_msg.data.game_time =
            referee_data.game_status.stage_remain_time; // 当前阶段剩余时间

        append_crc16_check_sum(reinterpret_cast<uint8_t *>(&mcu2nav_msg),
                               sizeof(mcu2nav_msg)); // 添加CRC校验
        comm->write(mcu2nav_msg);
    }

    void sentry_chassis_thread(void *argument)
    {
        while (true)
        {
            comm->read(nav2mcu_msg);
            verify_crc16_check_sum(reinterpret_cast<uint8_t *>(&nav2mcu_msg),
                                   sizeof(nav2mcu_msg));
            imu2chassis();
            sentry_cmd.sentry_posture = nav2mcu_msg.data.mode;
            referee_process(referee_drv_t::get_instance());
            referee_drv_t::get_instance()->send_robot_interaction(
                0x8080, 0x0120, &sentry_cmd, sizeof(sentry_cmd));
            mcu2nav_process();

            gimbal2chassis();
            chassis2gimbal();

            rud_chassis_ptr->set_command(*rud_cmd_ptr);
            yaw_ptr->set_command(*yaw_cmd_ptr);

            // ------------------- 功率计相关 -------------------
            // power_meter->get_data(power_data);

            vTaskDelay(1);
        }
    }
    uint8_t nav2mcu_msg_header = 0xA5;
    status_t sentry_chassis_init(void *argument)
    {
        // 初始化区域
        can_rx_drv_t::subscribe(can_hub_t::which_can::can3, 0x123);
        can_rx_drv_t::subscribe(can_hub_t::which_can::can3, 0x103);
        rud_cmd_ptr = new rud_cmd_t();
        rud_cfg_ptr = new rud_cfg_t();
        yaw_cmd_ptr = new yaw_cmd_t();
        yaw_cfg_ptr = new yaw_cfg_t();
        comm        = new uart_comm_t(uart_comm_t::, 0x01);

        // 注册区域
        mcu2nav_msg.header.sof = 0xA5;

        comm->register_msg_type(sizeof(nav2mcu_msg), &nav2mcu_msg_header,
                                sizeof(nav2mcu_msg.header));

        // 发送命令区域
        rud_chassis_ptr = rud_chassis_t::instance();
        chassis_config(*rud_cfg_ptr);
        rud_chassis_ptr->configure(*rud_cfg_ptr);
        yaw_ptr = yaw_t::instance();
        yaw_config(*yaw_cfg_ptr);
        yaw_ptr->configure(*yaw_cfg_ptr);
        rud_chassis_ptr->start();
        yaw_ptr->start();

        // power_meter = new powermeter_drv_t(0x212, can_hub_t::can2);
        // power_meter->init();

        xTaskCreate(sentry_chassis_thread, "sentry_chassis_thread", 512,
                    nullptr, configMAX_PRIORITIES - 1, nullptr);
        vTaskDelete(nullptr);
        return PYRO_OK;
    }
}

#endif