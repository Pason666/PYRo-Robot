#ifndef __PYRO_SENTRY_GIMBAL_H__
#define __PYRO_SENTRY_GIMBAL_H__

#include "pyro_module_base.h"
#include "pyro_algo_pid.h"
#include "pyro_algo_common.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h"
#include "pyro_motor_base.h"
#include "pyro_ins.h"

inline uint8_t auto_fire;
inline float bullet_speed;
inline uint8_t power_heat;

namespace pyro
{

struct gimbal_cmd_t final : cmd_base_t
{
    enum class gimbal_mode_t
    {
        MANUAL,
        SCANNING,
        TRACKING
    };

    float target_delta_yaw_rad;
    float target_delta_pitch_rad;
    float aim_imu_pitch_rad{};
    float aim_imu_yaw_rad{};
    bool is_aiming = false;

    gimbal_mode_t gimbal_mode;

    gimbal_cmd_t()
        : target_delta_yaw_rad(0), target_delta_pitch_rad(0),
          gimbal_mode(gimbal_mode_t::MANUAL)
    {
    }
};

struct gimbal_cfg_t
{
    struct motor_cfg_t
    {
        motor_base_t *yaw{nullptr};
        dm_motor_drv_t *pitch{nullptr};
    };

    struct pid_cfg_t
    {
        pid_t *yaw_pos_pid{};
        pid_t *yaw_spd_pid{};
        pid_t *pitch_pos_pid{};
        pid_t *pitch_spd_pid{};
    };

    motor_cfg_t motor;
    pid_cfg_t pid;
    float yaw_offset{};
    float yaw_max_rad{};
    float yaw_min_rad{};
    float pitch_max_rad{};
    float pitch_min_rad{};
};

class gimbal_t final
    : public module_base_t<gimbal_t, gimbal_cmd_t, gimbal_cfg_t>
{
    friend class module_base_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct gimbal_context_t;

  public:
    gimbal_t(const gimbal_t &)            = delete;
    gimbal_t &operator=(const gimbal_t &) = delete;

  private:
    gimbal_t();
    ~gimbal_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    static void _gimbal_mec_control(gimbal_context_t *ctx);
    static void _gimbal_imu_control(gimbal_context_t *ctx);
    static void _send_motor_command(gimbal_context_t *ctx);

    // --- 成员变量 ---

    // 电机句柄
    struct motor_ctx_t
    {
        motor_base_t *pitch{nullptr};
        motor_base_t *yaw{nullptr};
    };

    // 算法对象 (串级 PID)
    struct pid_ctx_t
    {
        pid_t *pitch_pos_pid{nullptr};
        pid_t *pitch_spd_pid{nullptr};
        pid_t *yaw_pos_pid{nullptr};
        pid_t *yaw_spd_pid{nullptr};
    };

    // 运行时数据
    struct data_ctx_t
    {
        float current_pitch_rad{0.0f};
        float current_pitch_radps{0.0f};
        float current_yaw_rad{0.0f};
        float current_yaw_radps{0.0f};

        float target_pitch_rad{0.0f};
        float target_pitch_radps{0.0f};
        float target_yaw_rad{0.0f};
        float target_yaw_radps{0.0f};

        float out_pitch_torque{0.0f};
        float out_yaw_torque{0.0f};

        float aim_imu_max_yaw{};
        float aim_imu_min_yaw{};
    };



    struct gimbal_context_t
    {
        gimbal_cfg_t gimbal_config;
        motor_ctx_t motor;
        pid_ctx_t pid;
        data_ctx_t data;
        gimbal_cmd_t *cmd{};
    };

    struct debug_ctx_t
    {
        float debug_rud_torque[4]{};
    };

    gimbal_context_t _ctx;
    debug_ctx_t debug_data;

    // --- FSM 状态定义 ---
    using owner = gimbal_t;

    struct state_passive_t : public state_t<owner>
    {
        void enter(owner *owner) override;
        void execute(owner *owner) override;
        void exit(owner *owner) override;
    };

    struct fsm_active_t : public fsm_t<owner>
    {
        // 子状态
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

        struct state_tracking_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

      private:
        state_scanning_t _scanning_state;
        state_manual_t _manual_state;
        state_tracking_t _tracking_state;
    };

    state_passive_t _passive_state;
    fsm_active_t _active_state;
    fsm_t<owner> _main_fsm;
};


} // namespace pyro

#endif
