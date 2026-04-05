#include "pyro_core_config.h"
#if BOARD_ID == GIMBAL_ID

#include "pyro_bsp_uart.h"
#include "pyro_module_base.h"
#include "pyro_sentry_gimbal.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_com_cantx.h"
#include "pyro_com_canrx.h"
#include "pyro_uart_comm.h"
#include "pyro_crc.h"
#include "pyro_sentry_message_frame.h"

using namespace pyro;

bool game_started{};
bool autoaim = false;
bool enemy_color{};
uint8_t in_aim{};
bool scan{};

gimbal_t *gimbal_ptr                       = nullptr;
gimbal_cmd_t *gimbal_cmd_ptr               = nullptr;
gimbal_cfg_t *gimbal_cfg_ptr               = nullptr;
uart_comm_t *comm                          = nullptr;

__attribute__((section(".dma_heap"))) mcu2aim_msg_t mcu2aim_msg;
__attribute__((section(".dma_heap"))) aim2mcu_msg_t aim2mcu_msg;

void gimbal_config(gimbal_cfg_t &gimbal_cfg)
{
    gimbal_cfg.motor.yaw = new dji_gm_6020_motor_drv_t(
        dji_motor_tx_frame_t::id_1, can_hub_t::can1);
    gimbal_cfg.motor.pitch = new dm_motor_drv_t(0x01, 0x00, can_hub_t::can2);
    gimbal_cfg.motor.pitch->set_position_range(-PI, PI);
    gimbal_cfg.motor.pitch->set_rotate_range(-20, 20);
    gimbal_cfg.motor.pitch->set_torque_range(-10, 10);

    gimbal_cfg.pitch_max_rad     = 0.43f; // 最高的时候
    gimbal_cfg.pitch_min_rad     = 0.18f; // 最低的时候
    gimbal_cfg.yaw_max_rad       = 0.70f;
    gimbal_cfg.yaw_min_rad       = -0.70f;

    gimbal_cfg.pid.pitch_pos_pid = new pyro::pid_t(90.0f, 0.001f, 1.8f, 0.5f, 20);
    gimbal_cfg.pid.pitch_spd_pid = new pyro::pid_t(1.1f, 0.0f, 0.0f, 0.5f, 6.0f);

    gimbal_cfg.pid.yaw_pos_pid =
        new pyro::pid_t(50.0f, 0.0f, 0.8f, 0, 50.0f, 0, 90, 2);
    gimbal_cfg.pid.yaw_spd_pid = new pyro::pid_t(0.85f, 0.0f, 0.0f, 0.2f, 6);

    gimbal_cfg.yaw_offset      = -1.10293245f;
}

extern "C"
{
    void aim2mcu_process()
    {
        gimbal_cmd_ptr->is_aiming         = aim2mcu_msg.data.fire;
        auto_fire                         = aim2mcu_msg.data.fire;
        gimbal_cmd_ptr->aim_imu_yaw_rad   = aim2mcu_msg.data.shoot_yaw;
        gimbal_cmd_ptr->aim_imu_pitch_rad = aim2mcu_msg.data.shoot_pitch;
    }

    void gimbal_rc2cmd()
    {
        pyro::read_scope_lock lock(pyro::dr16_drv_t::instance().get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        if (sw_pos_t::UP == vrc.switches.right.current_pos)
        {
            gimbal_cmd_ptr->mode = gimbal_cmd_t::mode_t::PASSIVE;
            gimbal_cmd_ptr->target_delta_pitch_rad = 0.0f;
            gimbal_cmd_ptr->target_delta_yaw_rad   = 0.0f;
            autoaim                                = false;
        }
        else if (sw_pos_t::MID== vrc.switches.right.current_pos)
        {
            gimbal_cmd_ptr->mode        = gimbal_cmd_t::mode_t::ACTIVE;
            gimbal_cmd_ptr->gimbal_mode = gimbal_cmd_t::gimbal_mode_t::MANUAL;
            gimbal_cmd_ptr->target_delta_pitch_rad = vrc.axes.ry * 0.005f;
            gimbal_cmd_ptr->target_delta_yaw_rad   = vrc.axes.rx * 0.02f;
            autoaim                                = false;
        }
        else if (sw_pos_t::DOWN == vrc.switches.right.current_pos)
        {
            gimbal_cmd_ptr->mode        = gimbal_cmd_t::mode_t::ACTIVE;
            gimbal_cmd_ptr->gimbal_mode = gimbal_cmd_t::gimbal_mode_t::SCANNING;
            autoaim                     = true;
            aim2mcu_process();
        }
    }

    void chassis_rc2cmd()
    {
        pyro::read_scope_lock lock(pyro::dr16_drv_t::instance().get_lock());
        auto &vrc = pyro::rc_drv_t::read();
        static int8_t vx        = 0;
        static int8_t vy        = 0;
        static int8_t wz        = 0;
        static int8_t delta_yaw = 0;
        static bool active      = false;
        static bool follow_yaw  = false;
        static bool nav_enable  = false;

        can_tx_drv_t::clear(0x123);

        if (sw_pos_t::UP == vrc.switches.right.current_pos)
        {
            vx         = 0;
            vy         = 0;
            wz         = 0;
            delta_yaw  = 0;
            follow_yaw = false;
            active     = false;
            nav_enable = false;
        }
        else if (sw_pos_t::MID== vrc.switches.right.current_pos)
        {
            if (abs(vrc.axes.lx) < 0.1f)
                vx = 0;
            else
                vx = static_cast<int8_t>(vrc.axes.lx * 127);
            if (abs(vrc.axes.ly) < 0.1f)
                vy = 0;
            else
                vy = static_cast<int8_t>(vrc.axes.ly * 127);
            wz         = 0;
            delta_yaw  = static_cast<int8_t>(vrc.axes.rx * 127);
            follow_yaw = true;
            active     = true;
            nav_enable = false;
        }
        else if (sw_pos_t::DOWN == vrc.switches.right.current_pos)
        {
            follow_yaw = false;
            active     = true;
            nav_enable = true;
        }

        can_tx_drv_t::add_data(0x123, 8, vx);
        can_tx_drv_t::add_data(0x123, 8, vy);
        can_tx_drv_t::add_data(0x123, 8, wz);
        can_tx_drv_t::add_data(0x123, 8, delta_yaw);
        can_tx_drv_t::add_data(0x123, 1, static_cast<uint8_t>(follow_yaw));
        can_tx_drv_t::add_data(0x123, 1, static_cast<uint8_t>(active));
        can_tx_drv_t::add_data(0x123, 1, static_cast<uint8_t>(nav_enable));

        can_tx_drv_t::send(0x123, can_hub_t::get_instance()->hub_get_can_obj(
                                      can_hub_t::which_can::can3));
    }

    void chassis2gimbal()
    {
        std::array<uint8_t, 8> raw_data{};
        can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x102, raw_data);
        uint8_t bullet_speed_int = raw_data[0];
        uint8_t bullet_speed_dec = raw_data[1];
        bullet_speed             = bullet_speed_int + bullet_speed_dec / 100.0f;
        power_heat               = raw_data[2];
        in_aim                   = raw_data[3];
        game_started             = raw_data[4] & 0x01;
        enemy_color              = raw_data[4] >> 1 & 0x01;
        scan                     = raw_data[4] >> 2 & 0x01;
    }

    void mcu2aim_process()
    {
        float yaw, pitch, roll;
        ins_drv_t *ins = ins_drv_t::get_instance();
        ins->get_angles_b(&yaw, &pitch, &roll);
        yaw                               = yaw / 180 * PI;
        pitch                             = pitch / 180 * PI;

        mcu2aim_msg.data.curr_yaw         = yaw;
        mcu2aim_msg.data.curr_pitch       = pitch;
        mcu2aim_msg.data.self_v_magnitude = 0;
        mcu2aim_msg.data.self_v_angle     = 0;
        mcu2aim_msg.data.curr_speed       = bullet_speed;
        mcu2aim_msg.data.shoot_delay      = 0;
        mcu2aim_msg.data.state            = in_aim;
        mcu2aim_msg.data.stop_record      = 0;
        mcu2aim_msg.data.autoaim          = autoaim;
        mcu2aim_msg.data.enemy_color      = enemy_color;
        append_crc16_check_sum(reinterpret_cast<uint8_t *>(&mcu2aim_msg),
                               sizeof(mcu2aim_msg) - 1);
        comm->write(mcu2aim_msg);
    }

    void sentry_gimbal_thread(void *argument)
    {
        while (true)
        {
            comm->read(aim2mcu_msg);
            chassis_rc2cmd();
            gimbal_rc2cmd();
            chassis2gimbal();
            mcu2aim_process();
            gimbal_ptr->set_command(*gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }
    uint8_t header = 0xA5;
    status_t sentry_gimbal_init()
    {
        gimbal_cmd_ptr = new gimbal_cmd_t();
        gimbal_cfg_ptr = new gimbal_cfg_t();
        comm           = new uart_comm_t(PYRO_UART10, 0x01);

        mcu2aim_msg.header.sof  = 0xA5;
        mcu2aim_msg.enter.enter = '\n';

        comm->register_msg_type(sizeof(aim2mcu_msg), &header,
                                sizeof(aim2mcu_msg.header));

        gimbal_ptr = gimbal_t::instance();
        gimbal_config(*gimbal_cfg_ptr);
        gimbal_ptr->configure(*gimbal_cfg_ptr);
        gimbal_ptr->start();
        
        xTaskCreate(sentry_gimbal_thread, "sentry_gimbal_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, nullptr);
        return PYRO_OK;
    }
}

#endif