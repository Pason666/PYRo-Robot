#if BOARD_ID == GIMBAL_ID

#include "pyro_17mm_booster.h"
#include "pyro_com_canrx.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_uart_comm.h"
#include "pyro_crc.h"
#include "pyro_sentry_gimbal.h"
#include "pyro_sentry_message_frame.h"
#include <algorithm>

using namespace pyro;

// -----------------------------------------------

constexpr uint32_t EVENT_BIT_FIRE = (1 << 0);

shoot_17mm_control_t *booster_ptr     = nullptr;
booster_cmd_t *booster_cmd_ptr        = nullptr;
booster_cfg_t *booster_cfg_ptr        = nullptr;

static TaskHandle_t booster_thread_handle = nullptr;

void booster_config(booster_cfg_t &cfg)
{
    cfg.motor.fric[0] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_3,
                                                  can_hub_t::can1); // 右摩擦轮
    cfg.motor.fric[1] = new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_2,
                                                  can_hub_t::can1); // 左摩擦轮
    cfg.motor.trigger =
        new dji_m2006_motor_drv_t(dji_motor_tx_frame_t::id_1, can_hub_t::can1);

    cfg.pid.fric_pid[0]      = new pyro::pid_t(0.9f, 0.0f, 0.0f, 0.8f, 20.0f);
    cfg.pid.fric_pid[1]      = new pyro::pid_t(0.9f, 0.0f, 0.0f, 0.8f, 20.0f);
    cfg.pid.trig_pos_pid     = new pyro::pid_t(1000.0f, 0.0f, 0.0f, 100.0f, 1000.0f);
    cfg.pid.trig_spd_pid     = new pyro::pid_t(5.0f, 1.5f, 0.0f, 5.0f, 10.0f);
    cfg.pid.bullet_speed_pid = new pyro::pid_t(0.01f, 0.0f, 0.00f, 5.00f, 10.0f);

    cfg.target_fric_speed    = SHOOT_FIRE_RADPS;
}

extern "C"
{
    void booster_rc2cmd(uint32_t notify_val)
    {
        static uint8_t down_time;
        read_scope_lock lock(pyro::dr16_drv_t::instance().get_lock());
        auto &vrc = pyro::rc_drv_t::read();

        if (sw_pos_t::MID  == vrc.switches.left.current_pos ||
            sw_pos_t::DOWN == vrc.switches.left.current_pos ||
            auto_fire)
        {
            booster_cmd_ptr->is_fric_on = true;

            // 情况 A：拨杆从中拨到下 -> 单发模式
            // 只有在中档时才响应拨杆的单发开火事件
            if(sw_pos_t::MID  == vrc.switches.left.current_pos)
            {
                if (notify_val & EVENT_BIT_FIRE)
                {
                    booster_cmd_ptr->single_shoot = true;
                    booster_cmd_ptr->continue_shoot = false;
                }
            }

            // 情况 A：拨杆保持在下方 (SW_DOWN) -> 连发模式
            if (sw_pos_t::DOWN == vrc.switches.left.current_pos ||
                auto_fire)
            {
                down_time++;
                if (down_time > 200)
                {
                    booster_cmd_ptr->continue_shoot = true;
                    booster_cmd_ptr->single_shoot   = false;
                    // 注意：连发模式下，不要触发单发，防止逻辑冲突
                }
            }
            else
            {
                // 拨杆不在下方，关闭连发
                booster_cmd_ptr->continue_shoot = false;
                down_time                       = 0;
            }
        }
        else
        {
            booster_cmd_ptr->is_fric_on     = false;
            booster_cmd_ptr->continue_shoot = false;
            booster_cmd_ptr->single_shoot   = false;
            down_time                       = 0;
        }
    }

    void chassis2booster()
    {
        booster_cmd_ptr->current_bullet_mps = bullet_speed;
        booster_cmd_ptr->power_heat = power_heat;
    }

    // void speed_control(void)
    // {
    //     static float filtered_speed_error          = 0.0f;
    //     static bool first_ball_received            = false;
    //     std::array<uint8_t, 8> raw_data{};
    //     if (can_rx_drv_t::get_data(can_hub_t::which_can::can3, 0x102,
    //     raw_data))
    //     {
    //         float current_speed = raw_data[0] + raw_data[1] / 100.0f;
    //         if (current_speed < 1.0f)
    //         {
    //             current_speed = booster_cfg_ptr->target_fric_speed;
    //         }
    //         if (booster_cfg_ptr->target_fric_speed > 7.5f)
    //         {
    //             // --- A. 计算当前瞬时误差 ---
    //             float error_now =
    //                 current_speed - booster_cfg_ptr->target_fric_speed;
    //
    //             // --- B. 一阶低通滤波 (核心逻辑) ---
    //             // 公式: Output = Alpha * Input + (1 - Alpha) * Output_Last
    //             if (!first_ball_received)
    //             {
    //                 // 第一发弹：直接初始化滤波器
    //                 filtered_speed_error = error_now;
    //                 first_ball_received  = true;
    //             }
    //             else
    //             {
    //                 // 后续发弹：平滑累积误差
    //                 filtered_speed_error =
    //                     FILTER_ALPHA * error_now +
    //                     (1 - FILTER_ALPHA) * filtered_speed_error;
    //             }
    //
    //             // --- C. 构造带符号的“类平方误差” ---
    //             // 作用：让大误差被更大权重地修正，小误差被抑制
    //             float pid_input =
    //                 filtered_speed_error * std::abs(filtered_speed_error);
    //
    //             // --- D. PID 计算速度增量 ---
    //             // 逻辑保持不变：将 pid_input 视为误差，期望将其控制到 0
    //             float speed_increment =
    //                 booster_cfg_ptr->pid.bullet_speed_pid->calculate(0.0f,
    //                                                                  pid_input);
    //
    //             // --- E. 执行与限幅 ---
    //             booster_cfg_ptr->target_fric_speed += speed_increment;
    //
    //             constexpr float MAX_FRIC1_MPS = 730.0f;
    //             constexpr float MIN_FRIC1_MPS = 710.0f;
    //
    //             // 使用 std::clamp (C++17) 更简洁，若不支持则换回 if-else
    //             booster_cfg_ptr->target_fric_speed =
    //                 std::clamp(booster_cfg_ptr->target_fric_speed,
    //                            MIN_FRIC1_MPS, MAX_FRIC1_MPS);
    //         }
    //     }
    // }

    void booster_thread(void *argument)
    {
        while (true)
        {
            chassis2booster();

            uint32_t notify_val = 0;
            // 非阻塞提取任务通知（超时为0），捕获按键与拨杆脉冲
            xTaskNotifyWait(0x00, 0xFFFFFFFF, &notify_val, 0);
            // 【架构精髓】：在此处默认重置所有脉冲触发变量，形成严格的 1 帧电平脉冲
            booster_cmd_ptr->single_shoot = false;
            if (dr16_drv_t::instance().check_online())
            {
                booster_rc2cmd(notify_val);
            }
            
            booster_ptr->set_command(*booster_cmd_ptr);
            vTaskDelay(1);
        }
    }

    status_t sentry_booster_init(void *argument)
    {
        can_rx_drv_t::subscribe(can_hub_t::which_can::can3, 0x102);

        booster_cmd_ptr = new booster_cmd_t();
        booster_cfg_ptr = new booster_cfg_t();
        booster_ptr     = shoot_17mm_control_t::instance();

        booster_config(*booster_cfg_ptr);
        booster_ptr->configure(*booster_cfg_ptr);
        booster_ptr->start();
        
        // 1. 获取任务句柄
        xTaskCreate(booster_thread, "booster_thread", 512, nullptr,
                    configMAX_PRIORITIES - 1, &booster_thread_handle);
        
        // 2. 利用泛型 Broker 登记所有会触发动作的事件
        auto &vrc = pyro::rc_drv_t::read();

        // --- DR16 拨杆绑定 ---
        pyro::sw_broker::subscribe(&vrc.switches.left,
            pyro::sw_event_t::MID_TO_DOWN, booster_thread_handle, EVENT_BIT_FIRE);
        
        vTaskDelete(nullptr);
        return PYRO_OK;
    }
}

#endif
