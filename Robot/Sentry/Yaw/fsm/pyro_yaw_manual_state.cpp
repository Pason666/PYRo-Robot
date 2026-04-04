#include "pyro_yaw.h"

float test_target;
float test_current;
namespace pyro
{
void yaw_t::fsm_active_t::state_manual_t::enter(owner *owner)
{
}

void yaw_t::fsm_active_t::state_manual_t::execute(owner *owner)
{
    float target_yaw = owner->_ctx.cmd->target_yaw_imu_angle;
    const float current = owner->_ctx.cmd->current_yaw_imu_rad;

    while (target_yaw > PI)  target_yaw -= 2 * PI;
    while (target_yaw < -PI) target_yaw += 2 * PI;

    float error = target_yaw - current;
    while (error > PI)  error -= 2 * PI;
    while (error < -PI) error += 2 * PI;

    const float target = current + error;

    test_current = owner->_ctx.cmd->current_yaw_imu_rad;
    test_target = owner->_ctx.cmd->target_yaw_imu_angle;

    owner->_ctx.data.out_yaw_radps =
        -owner->_ctx.yaw_config.pid.yaw_pos_pid->calculate(
            target, current);

    owner->_ctx.data.out_yaw_torque =
        owner->_ctx.yaw_config.pid.yaw_spd_pid->calculate(
            owner->_ctx.data.out_yaw_radps, owner->_ctx.data.current_yaw_radps);

    _send_motor_command(&owner->_ctx);
}

void yaw_t::fsm_active_t::state_manual_t::exit(owner *owner)
{
}

} // namespace pyro