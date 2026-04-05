#include "pyro_module_base.h"
#include "pyro_mutex.h"
#include "pyro_dr16_rc_drv.h"
#include "pyro_vt03_rc_drv.h"
#include "pyro_rc_base_drv.h"
#include "pyro_screw_gimbal.h"
#include "pyro_com_cantx.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"

using namespace pyro;

// 定义任务通知的位掩码 (Event Bits)
constexpr uint32_t EVENT_BIT_TRACK_TOGGLE = (1 << 0);
constexpr uint32_t EVENT_BIT_LEG_TOGGLE   = (1 << 1);

static TaskHandle_t gimbal_task_handle                = nullptr;
static pyro::screw_gimbal_t *screw_gimbal_ptr         = nullptr;
static pyro::screw_gimbal_cmd_t *screw_gimbal_cmd_ptr = nullptr;
static pyro::screw_gimbal_deps_t *screw_gimbal_deps   = nullptr;

static void gimbal_dr162cmd();
static void chassis_dr162cmd();
static void gimbal_vt032cmd();
static void chassis_vt032cmd(uint32_t notify_val);
static void deps_init();

extern "C"
{
    void hero_gimbal_thread(void *argument)
    {
        while (true)
        {
            uint32_t notify_val = 0;
            xTaskNotifyWait(0x00, UINT32_MAX, &notify_val, 0);

            if (vt03_drv_t::instance().check_online())
            {
                chassis_vt032cmd(notify_val);
                gimbal_vt032cmd();
            }
            else if (dr16_drv_t::instance().check_online())
            {
                chassis_dr162cmd();
                gimbal_dr162cmd();
            }
            else
            {
                screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::PASSIVE;
            }
            screw_gimbal_ptr->set_command(*screw_gimbal_cmd_ptr);
            vTaskDelay(1);
        }
    }

    void hero_gimbal_init(void *argument)
    {
        screw_gimbal_cmd_ptr = new pyro::screw_gimbal_cmd_t();
        screw_gimbal_ptr     = pyro::screw_gimbal_t::instance();

        deps_init();
        screw_gimbal_ptr->configure(*screw_gimbal_deps);
        screw_gimbal_ptr->start();

        xTaskCreate(hero_gimbal_thread, "start_app_thread", 128, nullptr,
                    configMAX_PRIORITIES - 1, &gimbal_task_handle);

        auto &vrc = pyro::rc_drv_t::read();
        pyro::btn_broker::subscribe(&vrc.buttons.pause, pyro::btn_event_t::PRESS_DOWN, gimbal_task_handle, EVENT_BIT_TRACK_TOGGLE);
        pyro::btn_broker::subscribe(&vrc.buttons.fn_r, pyro::btn_event_t::PRESS_DOWN, gimbal_task_handle, EVENT_BIT_LEG_TOGGLE);

        vTaskDelete(nullptr);
    }
}

void gimbal_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::MID != vrc.switches.right.current_pos)
    {
        screw_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        screw_gimbal_cmd_ptr->pitch_delta_angle = 0;
        screw_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        return;
    }
    screw_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::ACTIVE;
    screw_gimbal_cmd_ptr->pitch_delta_angle = -vrc.axes.ry * 0.0025f;
    screw_gimbal_cmd_ptr->yaw_delta_angle   = -vrc.axes.rx * 0.0035f;
}

void chassis_dr162cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    static int8_t vx        = 0;
    static int8_t vy        = 0;
    static int8_t wz        = 0;
    static bool active      = false;
    static bool track_en    = false;
    static bool leg_retract = false;

    pyro::can_tx_drv_t::clear(0x101);

    if (pyro::sw_pos_t::DOWN == vrc.switches.right.current_pos)
    {
        vx          = 0;
        vy          = 0;
        wz          = 0;
        active      = false;
        track_en    = false;
        leg_retract = false;
        pyro::can_tx_drv_t::add_data(0x101, 8, vx);
        pyro::can_tx_drv_t::add_data(0x101, 8, vy);
        pyro::can_tx_drv_t::add_data(0x101, 8, wz);
        pyro::can_tx_drv_t::add_data(0x101, 1, active);
        pyro::can_tx_drv_t::add_data(0x101, 1, track_en);
        pyro::can_tx_drv_t::add_data(0x101, 1, leg_retract);
        pyro::can_tx_drv_t::send(
            0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                       pyro::can_hub_t::which_can::can1));
        return;
    }

    vx     = static_cast<int8_t>(vrc.axes.ly * 127);
    vy     = static_cast<int8_t>(-vrc.axes.lx * 127);
    wz     = 0;
    active = true;

    // if (pyro::sw_pos_t::DOWN != vrc.switches.left.current_pos)
    // {
    //     track_en = true;
    //     if (pyro::sw_pos_t::MID == vrc.switches.left.current_pos)
    //     {
    //         leg_retract = true;
    //     }
    //     else
    //     {
    //         leg_retract = false;
    //     }
    // }
    // else
    // {
    //     track_en = true;
    // }

    track_en = false;

    pyro::can_tx_drv_t::add_data(0x101, 8, vx);
    pyro::can_tx_drv_t::add_data(0x101, 8, vy);
    pyro::can_tx_drv_t::add_data(0x101, 8, wz);
    pyro::can_tx_drv_t::add_data(0x101, 1, active);
    pyro::can_tx_drv_t::add_data(0x101, 1, track_en);
    pyro::can_tx_drv_t::add_data(0x101, 1, leg_retract);
    pyro::can_tx_drv_t::send(0x101,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can1));
}

void gimbal_vt032cmd()
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos)
    {
        screw_gimbal_cmd_ptr->mode              = pyro::cmd_base_t::mode_t::PASSIVE;
        screw_gimbal_cmd_ptr->pitch_delta_angle = 0;
        screw_gimbal_cmd_ptr->yaw_delta_angle   = 0;
        return;
    }

    screw_gimbal_cmd_ptr->mode = pyro::cmd_base_t::mode_t::ACTIVE;
    screw_gimbal_cmd_ptr->auto_aim = false; // 清除外部视觉依赖，仅走手控

    // 补偿归一化乘积
    screw_gimbal_cmd_ptr->pitch_delta_angle =
        -vrc.axes.ry * 0.0025f - vrc.mouse_axes.y * 0.25f;
    screw_gimbal_cmd_ptr->yaw_delta_angle =
        -vrc.axes.rx * 0.0025f - vrc.mouse_axes.x * 0.6f;
}

void chassis_vt032cmd(uint32_t notify_val)
{
    pyro::read_scope_lock lock(pyro::rc_drv_t::get_lock());
    auto &vrc = pyro::rc_drv_t::read();

    static int8_t vx        = 0;
    static int8_t vy        = 0;
    static int8_t wz        = 0;
    static bool active      = false;
    static bool track_en    = false;
    static bool leg_retract = false;

    pyro::can_tx_drv_t::clear(0x101);

    if (pyro::sw_pos_t::UP == vrc.switches.gear.current_pos)
    {
        vx          = 0;
        vy          = 0;
        wz          = 0;
        active      = false;
        track_en    = false;
        leg_retract = false;
        pyro::can_tx_drv_t::add_data(0x101, 8, vx);
        pyro::can_tx_drv_t::add_data(0x101, 8, vy);
        pyro::can_tx_drv_t::add_data(0x101, 8, wz);
        pyro::can_tx_drv_t::add_data(0x101, 1, active);
        pyro::can_tx_drv_t::add_data(0x101, 1, track_en);
        pyro::can_tx_drv_t::add_data(0x101, 1, leg_retract);
        pyro::can_tx_drv_t::send(
            0x101, pyro::can_hub_t::get_instance()->hub_get_can_obj(
                       pyro::can_hub_t::which_can::can1));
        return;
    }

    vx     = static_cast<int8_t>(vrc.keys.w.current_level   ? 127
                                 : vrc.keys.s.current_level ? -127
                                                            : vrc.axes.ly * 127);
    vy     = static_cast<int8_t>(vrc.keys.a.current_level   ? 127
                                 : vrc.keys.d.current_level ? -127
                                                            : -vrc.axes.lx * 127);
    wz     = 0;
    active = true;

    // 根据任务通知位判断边沿事件
    if (notify_val & EVENT_BIT_TRACK_TOGGLE)
    {
        track_en = !track_en;
        if (!track_en)
        {
            leg_retract = false;
        }
    }

    if (notify_val & EVENT_BIT_LEG_TOGGLE)
    {
        if (track_en)
        {
            leg_retract = !leg_retract;
        }
    }

    pyro::can_tx_drv_t::add_data(0x101, 8, vx);
    pyro::can_tx_drv_t::add_data(0x101, 8, vy);
    pyro::can_tx_drv_t::add_data(0x101, 8, wz);
    pyro::can_tx_drv_t::add_data(0x101, 1, active);
    pyro::can_tx_drv_t::add_data(0x101, 1, track_en);
    pyro::can_tx_drv_t::add_data(0x101, 1, leg_retract);
    pyro::can_tx_drv_t::send(0x101,
                             pyro::can_hub_t::get_instance()->hub_get_can_obj(
                                 pyro::can_hub_t::which_can::can1));
}

void deps_init()
{
    screw_gimbal_deps = new pyro::screw_gimbal_deps_t();
    // 1. 初始化电机

    // Pitch: 使用 DM 电机 (示例 ID: Master 0x11, Slave 0x21, CAN1)
    // 根据 hybrid 中的用法进行配置
    screw_gimbal_deps->motor_deps.pitch =
        new dji_m3508_motor_drv_t(dji_motor_tx_frame_t::id_5, can_hub_t::can3);

    // Yaw: 使用 DJI GM6020 (ID 2, CAN1)

    screw_gimbal_deps->motor_deps.yaw = new dji_gm_6020_motor_drv_t(
        dji_motor_tx_frame_t::id_3, can_hub_t::can1);


    // 3. 初始化串级 PID
    screw_gimbal_deps->pid_deps.pitch_pos =
        new pid_t(11.5f, 0.108f, 0.01f, 0.5f, 10.0f, 40, 10,
                  4); // 位置环输出为 rad/s，限制在电机可接受范围内
    screw_gimbal_deps->pid_deps.pitch_spd =
        new pid_t(22.0f, 0.102f, 0.014f, 1.0f, 20.0f, 20, 10,
                  4); // 输出限制匹配电机 Nm 级

    // Yaw 轴 (DJI GM6020，输出为电流值/电压值，通常量级较大，如 +/- 30000)
    screw_gimbal_deps->pid_deps.yaw_pos =
        new pid_t(6.2f, 0.01f, 0.22f, 0.8f, 10.0f);
    screw_gimbal_deps->pid_deps.yaw_spd =
        new pid_t(4.0f, 0.0003f, 0.0001f, 0.2f, 3.0f);
}