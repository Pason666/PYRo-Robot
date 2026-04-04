#include "pyro_screw_gimbal.h"
#include "pyro_ins.h"
#include "pyro_com_canrx.h"
#include "pyro_algo_common.h"
#include "pyro_dwt_drv.h"
#include "screw_config.h"

#include <algorithm>
namespace pyro
{

// =========================================================
// 构造与初始化
// =========================================================

screw_gimbal_t::screw_gimbal_t() : module_base_t("screw_gimbal")
{
    _ctx = {};
}

status_t screw_gimbal_t::_init()
{
    _ctx.motor = _module_deps.motor_deps;
    _ctx.pid   = _module_deps.pid_deps;

    // 只保留解算需要的底盘四元数订阅
    pyro::can_rx_drv_t::subscribe(can_hub_t::which_can::can1, 0x103);
    return PYRO_OK;
}

// =========================================================
// 核心循环回调
// =========================================================

void screw_gimbal_t::_update_feedback()
{
    _ctx.motor.pitch->update_feedback();
    _ctx.motor.yaw->update_feedback();

    // =========================================================
    // 处理 Pitch 增量式电机过零点套圈逻辑
    // =========================================================
    float now_pitch_rotor_rad = _ctx.motor.pitch->get_current_position();
    float delta_rotor = now_pitch_rotor_rad - _ctx.data.last_pitch_rotor_rad;

    if (delta_rotor > PI)
    {
        delta_rotor -= 2.0f * PI;
    }
    else if (delta_rotor < -PI)
    {
        delta_rotor += 2.0f * PI;
    }

    // 数学模型已适配物理硬件：直接 += 累加物理增量
    _ctx.data.total_pitch_motor_rad += delta_rotor;
    _ctx.data.last_pitch_rotor_rad = now_pitch_rotor_rad;

    // Yaw相对角解算
    float current_angle = _ctx.motor.yaw->get_current_position() - YAW_OFFSET_RAD;
    _ctx.data.relative_yaw_motor_rad = loop_fp32_constrain(current_angle, -PI, PI);

    // 读取 IMU 数据作为云台姿态反馈
    ins_drv_t::get_instance()->get_rads_n(&_ctx.data.yaw_imu_rad,
                                          &_ctx.data.pitch_imu_rad,
                                          &_ctx.data.roll_imu_rad);

    ins_drv_t::get_instance()->get_gyro_b(&_ctx.data.yaw_imu_radps,
                                          &_ctx.data.pitch_imu_radps,
                                          &_ctx.data.roll_imu_radps);

    ins_drv_t::get_instance()->get_accel_b(&_ctx.data.x_accel_imu,
                                           &_ctx.data.y_accel_imu,
                                           &_ctx.data.z_accel_imu);

    ins_drv_t::get_instance()->get_quaternion(
        &_ctx.data.gimbal_q[0], &_ctx.data.gimbal_q[1],
        &_ctx.data.gimbal_q[2], &_ctx.data.gimbal_q[3]);

    _communicate_chassis();
    _calculate_relative_angles();

    // =========================================================
    // 基于电机多圈角度解算 Pitch 实际姿态，存入专属变量供闭环使用
    // =========================================================
    _ctx.data.current_pitch_motor_rad = _motor_rad_to_pitch_rad(_ctx.data.total_pitch_motor_rad);

    _ctx.data.current_pitch_motor_radps = _motor_radps_to_pitch_radps(
        _ctx.motor.pitch->get_current_rotate(),
        _ctx.data.total_pitch_motor_rad
    );
}

void screw_gimbal_t::_gimbal_control()
{
    // --- Pitch 串级控制 ---
    // 反馈量改用通过电机逆解算出的绝对顺滑数据
    _ctx.data.target_pitch_radps = _ctx.pid.pitch_pos->calculate(
        _ctx.data.target_pitch_rad, _ctx.data.current_pitch_motor_rad);

    _ctx.data.out_pitch_torque = _ctx.pid.pitch_spd->calculate(
        _ctx.data.target_pitch_radps, _ctx.data.current_pitch_motor_radps);

    // --- Yaw 串级控制 ---
    _ctx.data.target_yaw_radps = _ctx.pid.yaw_pos->calculate(
        _ctx.data.target_yaw_rad, _ctx.data.yaw_imu_rad);

    _ctx.data.out_yaw_torque = _ctx.pid.yaw_spd->calculate(
        _ctx.data.target_yaw_radps, _ctx.data.yaw_imu_radps);
}

screw_gimbal_t::gimbal_context_t screw_gimbal_t::get_ctx() const
{
    return _ctx;
}

void screw_gimbal_t::_send_motor_command(gimbal_context_t *ctx)
{
    ctx->motor.pitch->send_torque(ctx->data.out_pitch_torque);
    ctx->motor.yaw->send_torque(ctx->data.out_yaw_torque);
}

void screw_gimbal_t::_communicate_chassis()
{
    std::array<uint8_t, 8> raw_data{};
    if (pyro::can_rx_drv_t::get_data(pyro::can_hub_t::which_can::can1, 0x103, raw_data))
    {
        auto *src = reinterpret_cast<int16_t *>(raw_data.data());
        _ctx.data.chassis_q[0] = static_cast<float>(src[0]) / 32767.0f; // q0
        _ctx.data.chassis_q[1] = static_cast<float>(src[1]) / 32767.0f; // q1
        _ctx.data.chassis_q[2] = static_cast<float>(src[2]) / 32767.0f; // q2
        _ctx.data.chassis_q[3] = static_cast<float>(src[3]) / 32767.0f; // q3
    }
}

void screw_gimbal_t::_calculate_relative_angles()
{
    const float cw = _ctx.data.chassis_q[0], cx = _ctx.data.chassis_q[1],
                cy = _ctx.data.chassis_q[2], cz = _ctx.data.chassis_q[3];

    const float gw = _ctx.data.gimbal_q[0], gx = _ctx.data.gimbal_q[1],
                gy = _ctx.data.gimbal_q[2], gz = _ctx.data.gimbal_q[3];

    // 从四元数中反解出两个 IMU 认为的世界 Yaw 角
    const float chassis_yaw_imu = std::atan2(2.0f * (cw * cz + cx * cy), 1.0f - 2.0f * (cy * cy + cz * cz));
    const float gimbal_yaw_imu  = std::atan2(2.0f * (gw * gz + gx * gy), 1.0f - 2.0f * (gy * gy + gz * gz));

    // 计算当前的坐标系漂移误差 (去除了低通滤波，直接应用)
    float raw_yaw_error = gimbal_yaw_imu - chassis_yaw_imu - _ctx.data.relative_yaw_motor_rad;
    raw_yaw_error = pyro::loop_fp32_constrain(raw_yaw_error, -PI, PI);

    // 构造补偿四元数
    const float half_err = raw_yaw_error * 0.5f;
    const float comp_w = std::cos(half_err);
    const float comp_z = std::sin(half_err);

    // 四元数相乘：Q_aligned = Q_comp * Q_chassis
    const float aw = comp_w * cw - comp_z * cz;
    const float ax = comp_w * cx - comp_z * cy;
    const float ay = comp_w * cy + comp_z * cx;
    const float az = comp_w * cz + comp_z * cw;

    // 矩阵相乘提取相对 Pitch
    const float RcT_row2_0 = 2.0f * (ax * az + aw * ay);
    const float RcT_row2_1 = 2.0f * (ay * az - aw * ax);
    const float RcT_row2_2 = 1.0f - 2.0f * (ax * ax + ay * ay);

    const float Rg_col0_0 = 1.0f - 2.0f * (gy * gy + gz * gz);
    const float Rg_col0_1 = 2.0f * (gx * gy + gw * gz);
    const float Rg_col0_2 = 2.0f * (gx * gz - gw * gy);

    const float Rg_col2_0 = 2.0f * (gx * gz + gw * gy);
    const float Rg_col2_1 = 2.0f * (gy * gz - gw * gx);
    const float Rg_col2_2 = 1.0f - 2.0f * (gx * gx + gy * gy);

    const float r31 = RcT_row2_0 * Rg_col0_0 + RcT_row2_1 * Rg_col0_1 + RcT_row2_2 * Rg_col0_2;
    const float r33 = RcT_row2_0 * Rg_col2_0 + RcT_row2_1 * Rg_col2_1 + RcT_row2_2 * Rg_col2_2;

    _ctx.data.relative_pitch_rad = std::atan2(-r31, r33);
}

bool screw_gimbal_t::_calibrate_pitch_offset()
{
    _calib_tick++;
    if (_calib_tick < 1000)
    {
        return false;
    }
    _calib_pitch_sum += _ctx.data.relative_pitch_rad;

    if (_calib_tick >= PITCH_CALIB_MAX_TICKS + 1000)
    {
        const float avg_relative_pitch = _calib_pitch_sum / static_cast<float>(PITCH_CALIB_MAX_TICKS);

        const float theoretical_motor_rad = _pitch_rad_to_motor_rad(avg_relative_pitch);
        _ctx.data.total_pitch_motor_rad = theoretical_motor_rad;

        const float limit1 = _pitch_rad_to_motor_rad(PITCH_MAX_RAD);
        const float limit2 = _pitch_rad_to_motor_rad(PITCH_MIN_RAD);

        _ctx.data.pitch_motor_upper_limit = std::max(limit1, limit2);
        _ctx.data.pitch_motor_lower_limit = std::min(limit1, limit2);
        return true;
    }
    return false;
}

float screw_gimbal_t::_pitch_rad_to_motor_rad(float pitch_rad) const
{
    const float theta_rad = SCREW_THETA_ZERO_RAD + pitch_rad;
    const float target_S = std::sqrt(SCREW_L1_SQ_PLUS_L2_SQ - SCREW_TWO_L1_L2 * std::cos(theta_rad));

    // 【修改点】：使模型适应物理方向。电机正转 -> 丝杠变长 -> pitch低头(+)
    const float delta_S = target_S - SCREW_S_LIMIT_BOTTOM;

    return delta_S * 2.0f * PI;
}

float screw_gimbal_t::_motor_rad_to_pitch_rad(float motor_rad) const
{
    const float delta_S = motor_rad / (2.0f * PI);

    // 【修改点】：当前长度 = 极限长度 + 电机带来的伸长量
    const float current_S = SCREW_S_LIMIT_BOTTOM + delta_S;

    float cos_theta = (SCREW_L1_SQ_PLUS_L2_SQ - current_S * current_S) / SCREW_TWO_L1_L2;
    cos_theta = std::clamp(cos_theta, -1.0f, 1.0f);

    const float theta_rad = std::acos(cos_theta);
    return theta_rad - SCREW_THETA_ZERO_RAD;
}

float screw_gimbal_t::_pitch_radps_to_motor_radps(float pitch_radps, float current_pitch_rad) const
{
    const float theta_rad = SCREW_THETA_ZERO_RAD + current_pitch_rad;
    float current_S = std::sqrt(SCREW_L1_SQ_PLUS_L2_SQ - SCREW_TWO_L1_L2 * std::cos(theta_rad));

    if (current_S < 1.0f) current_S = 1.0f;

    const float dS_dpitch = (SCREW_TWO_L1_L2 * std::sin(theta_rad)) / (2.0f * current_S);

    // 【修改点】：丝杠变长对应电机正转，去掉负号
    const float dMotor_dpitch = dS_dpitch * 2.0f * PI;

    return pitch_radps * dMotor_dpitch;
}

float screw_gimbal_t::_motor_radps_to_pitch_radps(float motor_radps, float current_motor_rad) const
{
    const float current_pitch_rad = _motor_rad_to_pitch_rad(current_motor_rad);
    const float theta_rad = SCREW_THETA_ZERO_RAD + current_pitch_rad;
    float current_S = std::sqrt(SCREW_L1_SQ_PLUS_L2_SQ - SCREW_TWO_L1_L2 * std::cos(theta_rad));

    if (current_S < 1.0f) current_S = 1.0f;

    const float dS_dpitch = (SCREW_TWO_L1_L2 * std::sin(theta_rad)) / (2.0f * current_S);

    // 【修改点】：去掉负号
    const float dMotor_dpitch = dS_dpitch * 2.0f * PI;

    if (std::abs(dMotor_dpitch) < 0.001f) return 0.0f;

    return motor_radps / dMotor_dpitch;
}

void screw_gimbal_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    bool allow_active = (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode);

    if (_ctx.data.is_calibrating || !_ctx.data.has_initial_calibrated)
    {
        allow_active = false;
    }

    if (allow_active)
    {
        _main_fsm.change_state(&_fsm_active);
    }
    else
    {
        _main_fsm.change_state(&_fsm_passive);
    }

    _main_fsm.execute(this);
}

} // namespace pyro