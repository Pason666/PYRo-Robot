#ifndef __PYRO_SCREW_GIMBAL_H__
#define __PYRO_SCREW_GIMBAL_H__

#include "pyro_algo_pid.h"
#include "pyro_dji_motor_drv.h"
#include "pyro_dm_motor_drv.h" // 新增 DM 电机驱动
#include "pyro_module_base.h"
#include "pyro_motor_base.h"

namespace pyro
{

// =========================================================
// 1. 命令定义
// =========================================================
struct screw_gimbal_cmd_t final : public cmd_base_t
{
    float pitch_delta_angle; // 目标 Pitch 角度 (rad)
    float yaw_delta_angle;   // 目标 Yaw 角度 (rad)

    // 校准触发标志位 (外部传入，0->1 触发校准)
    bool trigger_calibration;

    // 自瞄数据
    bool auto_aim;
    float target_pitch;
    float target_yaw;

    screw_gimbal_cmd_t()
        : pitch_delta_angle(0.0f), yaw_delta_angle(0.0f), trigger_calibration(false), auto_aim(false),
          target_pitch(0.0f), target_yaw(0.0f)
    {
    }
};

struct screw_gimbal_deps_t
{
    // 电机句柄
    struct motor_deps_t
    {
        motor_base_t *pitch{nullptr};
        motor_base_t *yaw{nullptr};
    };

    // 算法对象 (串级 PID)
    struct pid_deps_t
    {
        pid_t *pitch_pos{nullptr};
        pid_t *pitch_spd{nullptr};
        pid_t *yaw_pos{nullptr};
        pid_t *yaw_spd{nullptr};
    };

    motor_deps_t motor_deps{};
    pid_deps_t pid_deps{};
};

// =========================================================
// 2. 云台类
// =========================================================
class screw_gimbal_t final
    : public module_base_t<screw_gimbal_t, screw_gimbal_cmd_t,
                           screw_gimbal_deps_t>
{
    friend class module_base_t;

    struct motor_ctx_t;
    struct pid_ctx_t;
    struct data_ctx_t;
    struct gimbal_context_t;

  public:
    [[nodiscard]] gimbal_context_t get_ctx() const;

  private:
    screw_gimbal_t();
    ~screw_gimbal_t() override = default;

    // --- 基类接口实现 ---
    status_t _init() override;
    void _update_feedback() override;
    void _fsm_execute() override;

    // --- 私有辅助方法 ---
    void _gimbal_control();
    static void _send_motor_command(gimbal_context_t *ctx);
    void _communicate_chassis();
    void _calculate_relative_angles();

    // --- 新增：核心运动学与校准方法 ---
    bool _calibrate_pitch_offset();
    float _pitch_rad_to_motor_rad(float pitch_rad) const;
    float _motor_rad_to_pitch_rad(float motor_rad) const;
    float _pitch_radps_to_motor_radps(float pitch_radps,
                                      float current_pitch_rad) const;
    float _motor_radps_to_pitch_radps(float motor_radps,
                                      float current_motor_rad) const;

    // --- 成员变量 ---

    // 上电校准逻辑暂存
    uint32_t _calib_tick{0};
    float _calib_pitch_sum{0.0f};

    // 运行时数据
    struct data_ctx_t
    {
        bool is_calibrating{false};
        bool has_initial_calibrated{false}; // 标记上电初次校准是否完成

        float pitch_motor_upper_limit{0}; // 电机多圈角度软限位(上限)
        float pitch_motor_lower_limit{0}; // 电机多圈角度软限位(下限)

        // --- Pitch 电机增量套圈处理变量 ---
        float last_pitch_rotor_rad{0};  // 上一次的电机单圈转子角度 (-PI ~ PI)
        float total_pitch_motor_rad{0}; // 累计的电机多圈总角度 (连续无跳变)

        // --- 反馈 (含 Offset) ---
        float current_pitch_motor_rad{0};   // 通过电机逆解算的 pitch
        float current_pitch_motor_radps{0}; // 通过电机逆解算的 pitch 角速度

        // 姿态反馈 (基于 IMU)
        float pitch_imu_rad{0};
        float pitch_imu_radps{0};
        float yaw_imu_rad{0};
        float yaw_imu_radps{0};
        float roll_imu_rad{0};
        float roll_imu_radps{0};

        // 加速度反馈 (基于 IMU)
        float z_accel_imu{0};
        float x_accel_imu{0};
        float y_accel_imu{0};

        // 四元数与相对角反馈
        float current_chassis_pitch_rad{0};
        float chassis_q[4]{};
        float gimbal_q[4]{};
        float relative_pitch_rad{0};       // 四元数计算得到
        float relative_roll_rad{0};        // 四元数计算得到
        float relative_yaw_motor_rad{0};   // 6020反馈计算得到

        // 目标
        float target_pitch_rad{0};
        float target_pitch_radps{0};
        float target_yaw_rad{0};
        float target_yaw_radps{0};

        // 输出
        float out_pitch_torque{0};
        float out_yaw_torque{0};
    };

    // 总 Context
    struct gimbal_context_t
    {
        screw_gimbal_deps_t::motor_deps_t motor;
        screw_gimbal_deps_t::pid_deps_t pid;
        data_ctx_t data;
        screw_gimbal_cmd_t *cmd{};
    };

    gimbal_context_t _ctx;

    // =====================================================
    // 状态定义 (HFSM)
    // =====================================================
    using owner = screw_gimbal_t;

    struct fsm_passive_t : public fsm_t<owner>
    {
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;

        struct calibration_state_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        struct idle_state_t : public state_t<owner>
        {
            void enter(owner *owner) override;
            void execute(owner *owner) override;
            void exit(owner *owner) override;
        };

        calibration_state_t calibration_state;
        idle_state_t idle_state;
        bool _last_calib_flag{false}; // 用于边缘检测
    };

    struct fsm_active_t : public fsm_t<owner>
    {
        void on_enter(owner *owner) override;
        void on_execute(owner *owner) override;
        void on_exit(owner *owner) override;
    };

    fsm_passive_t _fsm_passive;
    fsm_active_t _fsm_active;
    fsm_t<owner> _main_fsm;

    // =====================================================
    // 静态配置 (编译器常量)
    // =====================================================
    static constexpr float PITCH_OFFSET_RAD = 0.0f;
};

} // namespace pyro

#endif