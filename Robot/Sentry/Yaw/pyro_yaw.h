//
// Created by pason on 2026/2/2.
//

#ifndef PYRO_YAW_H
#define PYRO_YAW_H

#include "pyro_module_base.h"
#include "pyro_algo_pid.h"
#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"
#include "pyro_ins.h"

namespace pyro
{
struct yaw_cmd_t : cmd_base_t
{
    float target_yaw_imu_angle;
    float current_yaw_imu_rad;
    float radar_imu{};
    bool nav_enable{};

    yaw_cmd_t() : target_yaw_imu_angle(0)
    {
    }
};
struct yaw_cfg_t
{
    struct motor_cfg_t
    {
        dm_motor_drv_t *yaw{nullptr};
    };

    struct pid_cfg_t
    {
        pid_t *yaw_pos_pid{nullptr};
        pid_t *yaw_spd_pid{nullptr};
    };

    motor_cfg_t motor;
    pid_cfg_t pid;
    float yaw_offset;
};

class yaw_t final : public module_base_t<yaw_t, yaw_cmd_t, yaw_cfg_t>
{
    friend class module_base_t;
    friend class vofa_drv_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct yaw_ctx_t;

  public:
    yaw_t(const yaw_t &)            = delete;
    yaw_t &operator=(const yaw_t &) = delete;

    [[nodiscard]] float get_yaw_error() const;

  private:
    yaw_t();
    ~yaw_t() override = default;

    // --- 基类接口 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 派生方法 ---
    static void _yaw_control(yaw_ctx_t *ctx);
    static void _send_motor_command(yaw_ctx_t *ctx);

    struct data_ctx_t
    {
        float chassis_world_yaw;
        float target_yaw_imu_angle;
        float current_yaw_imu_angle;
        float current_yaw_angle;
        float current_yaw_radps;
        float out_yaw_radps;
        float out_yaw_torque;
        float world_yaw_error;
        float chassis_wz;
    };

    struct yaw_ctx_t
    {
        yaw_cfg_t yaw_config;
        data_ctx_t data{};
        yaw_cmd_t *cmd{};
    };

    struct debug_ctx_t
    {
        float debug_yaw_torque;
    };

    yaw_ctx_t _ctx;
    debug_ctx_t debug_data{};

    using owner = yaw_t;

    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t : public fsm_t<owner>
    {
        struct state_scanning_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct state_manual_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        void on_enter(owner *owner) override;
        void on_execute(yaw_t *ctx) override;
        void on_exit(owner *owner) override;

      private:
        state_scanning_t _scanning_state;
        state_manual_t _manual_state;
    };

    state_passive_t _passive_state;
    fsm_active_t _active_state;
    fsm_t<owner> _main_fsm;
};

} // namespace pyro

#endif // PYRO_YAW_H
