#ifndef __PYRO_17MM_BOOSTER_H__
#define __PYRO_17MM_BOOSTER_H__

#include "pyro_core_fsm.h"
#include "pyro_algo_pid.h"
#include "pyro_algo_common.h"
#include "pyro_module_base.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_17mm_config.h"
#include "pyro_func_control.h"

namespace pyro
{
struct booster_cmd_t : cmd_base_t
{
    bool is_fric_on;            // 摩擦轮是否开启
    bool single_shoot;          // 触发单发
    bool continue_shoot;        // 触发连发
    bool fire_licence{}; // 发射许可，为false时拨弹盘绝对不允许转动

    uint8_t power_heat{};       // 当前热量（除以10 0~26）
    float current_bullet_mps{}; // 当前弹速（裁判系统反馈）

    booster_cmd_t()
        : is_fric_on(false), single_shoot(false), continue_shoot(false)
    {
    }
};

struct booster_cfg_t
{
    struct motor_cfg_t
    {
        motor_base_t *fric[2]{nullptr};
        motor_base_t *trigger{nullptr};
    };
    struct pid_cfg_t
    {
        pid_t *trig_pos_pid{nullptr};
        pid_t *trig_spd_pid{nullptr};
        pid_t *fric_pid[2]{nullptr};
        pid_t *bullet_speed_pid{nullptr};
    };

    motor_cfg_t motor;
    pid_cfg_t pid;
    float target_fric_speed;
};

class shoot_17mm_control_t final
    : public module_base_t<shoot_17mm_control_t, booster_cmd_t, booster_cfg_t>
{
    friend class module_base_t;
    friend class vofa_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct booster_ctx_t;

  public:
    shoot_17mm_control_t(const shoot_17mm_control_t &)            = delete;
    shoot_17mm_control_t &operator=(const shoot_17mm_control_t &) = delete;

  private:
    shoot_17mm_control_t();
    ~shoot_17mm_control_t() override    = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    static void _fric_control(shoot_17mm_control_t *ctx);
    static void _trig_control(shoot_17mm_control_t *ctx);
    static void _fire_check(booster_ctx_t *ctx);
    static void _send_motor_command(booster_ctx_t *ctx);

    struct data_ctx_t
    {
        // 供状态机内部读取的状态变量
        uint16_t block_time     = 0;
        bool fric_pid_active    = true;
        bool trig_pid_active    = true;
        enum class trig_mode_e
        {
            SPEED,
            POSITION
        } trig_mode          = trig_mode_e::SPEED;
        bool is_first_update = true; // 计圈辅助变量
        float last_rotor_rad = 0.0f; // 计圈辅助变量
        float current_fric_radps[2]{};
        float current_trig_rad{};
        float current_trig_radps{};
        float target_fric_radps[2]{};
        float target_trig_radps{};
        float target_trig_rad{};
        float out_trig_radps{};
        float out_trig_torque{};
        float out_fric_torque[2]{};
    };

    struct booster_ctx_t
    {
        booster_cfg_t booster_cfg;
        data_ctx_t data;
        booster_cmd_t *cmd{};
    };

    struct debug_ctx_t
    {
        float debug_rud_torque[4]{};
    };

    booster_ctx_t _ctx;
    debug_ctx_t debug_data;

    // ================== FSM 状态机定义 ==================
    using owner = shoot_17mm_control_t;

    struct state_stop_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_ready_fric_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_ready_shoot_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_single_bullet_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_continue_bullet_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };
    struct state_done_t : public state_t<owner>
    {
        void enter(owner *ctx) override;
        void execute(owner *ctx) override;
        void exit(owner *ctx) override;
    };

    fsm_t<owner> _main_fsm;
    state_stop_t _state_stop;
    state_ready_fric_t _state_ready_fric;
    state_ready_shoot_t _state_ready_shoot;
    state_single_bullet_t _state_single_bullet;
    state_continue_bullet_t _state_continue_bullet;
    state_done_t _state_done;
};

} // namespace pyro

#endif // PYRO_17MM_BOOSTER_H
