#ifndef __PYRO_RUD_CHASSIS_H__
#define __PYRO_RUD_CHASSIS_H__

#define POWER_CONTROL_USE 1

#include "pyro_algo_pid.h"
#include "pyro_module_base.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_kin_rudder.h"
#include "pyro_powermeter.h"
#include "pyro_power_control_drv.h"
#include "pyro_referee.h"
#include "pyro_supercap_drv.h"

namespace pyro
{
// 定义舵轮特有的命令结构（如果有额外参数）
struct rud_cmd_t : cmd_base_t
{
    float vx, vy, wz, yaw_error;
    bool follow_yaw;
    rud_cmd_t() : vx(0), vy(0), wz(0), yaw_error(0), follow_yaw(false)
    {
    }
};

struct rud_cfg_t
{
    // 电机句柄
    struct motor_cfg_t
    {
        motor_base_t *rudder[4]{nullptr};
        motor_base_t *wheel[4]{nullptr};
    };

    struct pid_cfg_t
    {
        pid_t *rud_pos_pid[4]{nullptr};
        pid_t *rud_spd_pid[4]{nullptr};
        pid_t *wheel_pid[4]{nullptr};
        pid_t *follow_yaw_pid{nullptr};
    };

    motor_cfg_t motor;
    pid_cfg_t pid;
    float rud_pos_moving_offset[4]{};
};

// 继承模板基类，传入具体的命令类型
class rud_chassis_t final
    : public module_base_t<rud_chassis_t, rud_cmd_t, rud_cfg_t>
{
    friend class module_base_t;
    friend class vofa_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct rud_ctx_t;

  public:
    rud_chassis_t(const rud_chassis_t &)            = delete;
    rud_chassis_t &operator=(const rud_chassis_t &) = delete;

  private:
    rud_chassis_t();
    ~rud_chassis_t() override = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    void _kinematics_solve();
    static void _chassis_control(rud_ctx_t *ctx);
    static void _send_motor_command(rud_ctx_t *ctx);
    void _send_supercap_command() const;
    void _decide_cap();

    rudder_kin_t *_kinematics{nullptr};

    struct data_ctx_t
    {
        rudder_kin_t::rudder_states_t current_states{};
        rudder_kin_t::rudder_states_t target_states{};

        float current_rud_radps[4]{};
        float out_rud_torque[4]{};
        float out_wheel_torque[4]{};
    };

    struct hardware_ctx_t
    {
        powermeter_drv_t *power_meter{nullptr};
    };

    struct power_ctx_t
    {
        powermeter_data *data{nullptr};
    };

    enum class drive_mode_t
    {
        MOVING,  // Normal driving mode
        BRAKING, // Braking mode (Stopping)
        TURNING,
    };

    struct rud_ctx_t
    {
        rud_cfg_t rud_config;
        hardware_ctx_t hardware;
        power_ctx_t power;
        data_ctx_t data;
        rud_cmd_t *cmd;
        drive_mode_t drive_mode;
        supercap_drv_t::chassis_cmd_t supercap_cmd;
        supercap_drv_t::cap_feedback_t cap_feedback;
    };

    struct debug_ctx_t
    {
        float debug_rud_torque[4]{};
    };

    rud_ctx_t _ctx;
    debug_ctx_t debug_data;

    // --- FSM 状态定义 ---
    using owner = rud_chassis_t;

    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    // struct fsm_active_t : public fsm_t<owner>
    // {
    //     // 子状态
    //     struct state_moving_t : public state_t<owner>
    //     {
    //         void enter(owner *owner) override;
    //         void execute(owner *owner) override;
    //         void exit(owner *owner) override;
    //     };
    //
    //     struct state_braking_t : public state_t<owner>
    //     {
    //         void enter(owner *owner) override;
    //         void execute(owner *owner) override;
    //         void exit(owner *owner) override;
    //     };
    //
    //     struct state_turning_t : public state_t<owner>
    //     {
    //         void enter(owner *owner) override;
    //         void execute(owner *owner) override;
    //         void exit(owner *owner) override;
    //     };
    //
    //     void on_enter(owner *owner) override;
    //     void on_execute(owner *owner) override;
    //     void on_exit(owner *owner) override;
    //
    //   private:
    //     state_moving_t _moving_state;
    //     state_braking_t _braking_state;
    //     state_turning_t _turning_state;
    // };

    struct state_active_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    state_passive_t _state_passive;
    state_active_t _state_active;
    // fsm_active_t _state_active;

    fsm_t<owner> _main_fsm;

    static constexpr float RUD_RADIUS         = 0.060f;
    static constexpr uint8_t POWERCONTROL_NUM = 4;
};


} // namespace pyro
#endif