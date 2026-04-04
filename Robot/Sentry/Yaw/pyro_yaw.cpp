//
// Created by pason on 2026/2/2.
//
#include "pyro_yaw.h"

namespace pyro
{
float yaw{}, pitch{}, roll{};
float chassis_yaw_radps{}, chassis_pitch_radps{}, chassis_roll_radps{};

yaw_t::yaw_t() : module_base_t("yaw", 512, 512, task_base_t::priority_t::HIGH)
{
    _ctx.data  = {};
    debug_data = {};
}

float yaw_t::get_yaw_error() const
{
    return -_ctx.data.current_yaw_angle;
}

status_t yaw_t::_init()
{
    if (_module_deps.motor.yaw == nullptr)
    {
        // 电机指针未初始化，返回错误码
        return PYRO_ERROR;
    }
    if (_module_deps.pid.yaw_pos_pid == nullptr ||
        _module_deps.pid.yaw_spd_pid == nullptr)
    {
        // PID 指针未初始化，返回错误
        return PYRO_ERROR;
    }
    _ctx.yaw_config = _module_deps;
    return PYRO_OK;
}

void yaw_t::_update_feedback()
{
    ins_drv_t *ins = ins_drv_t::get_instance();
    _ctx.yaw_config.motor.yaw->update_feedback();

    // yaw轴当前角度（电机角度， -PI ~ PI）
    _ctx.data.current_yaw_angle =
        wrap_pi(_ctx.yaw_config.motor.yaw->get_current_position() -
                _ctx.yaw_config.yaw_offset);

    // 这里需要获取底盘imu数据减去大yaw的机械角度得到yaw轴的imu角度
    ins->get_angles_n(&yaw, &pitch, &roll);
    ins->get_gyro_n(&chassis_yaw_radps, &chassis_pitch_radps,
                    &chassis_roll_radps);
    _ctx.data.chassis_world_yaw = yaw / 180 * PI;

    _ctx.data.chassis_wz = chassis_yaw_radps;

    _ctx.data.current_yaw_imu_angle =
        wrap_pi(yaw - _ctx.data.current_yaw_angle);

    // yaw电机当前角速度
    _ctx.data.current_yaw_radps =
        _ctx.yaw_config.motor.yaw->get_current_rotate();
}

void yaw_t::_yaw_control(yaw_ctx_t *ctx)
{


    ctx->data.out_yaw_torque = ctx->yaw_config.pid.yaw_spd_pid->calculate(
        ctx->data.out_yaw_radps, ctx->data.current_yaw_radps);
}

void yaw_t::_send_motor_command(yaw_ctx_t *ctx)
{
    ctx->yaw_config.motor.yaw->send_torque(ctx->data.out_yaw_torque);
}

void yaw_t::_fsm_execute()
{
    _ctx.cmd = &_current_cmd;

    if (cmd_base_t::mode_t::PASSIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_passive_state);
    else if (cmd_base_t::mode_t::ACTIVE == _ctx.cmd->mode)
        _main_fsm.change_state(&_active_state);

    _main_fsm.execute(this);
}

} // namespace pyro
