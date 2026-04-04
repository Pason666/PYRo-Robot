#include "pyro_screw_gimbal.h"
#include "screw_config.h"

namespace pyro
{
void screw_gimbal_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.motor.pitch->enable();
    owner->_ctx.motor.yaw->enable();
}

void screw_gimbal_t::fsm_active_t::on_execute(owner *owner)
{
    owner->_ctx.data.target_pitch_rad += owner->_ctx.cmd->pitch_delta_angle;
    owner->_ctx.data.target_yaw_rad += owner->_ctx.cmd->yaw_delta_angle;

    const float dynamic_pitch_max = PITCH_MAX_RAD;
    const float dynamic_pitch_min = PITCH_MIN_RAD;

    if (owner->_ctx.data.target_pitch_rad > dynamic_pitch_max)
    {
        owner->_ctx.data.target_pitch_rad = dynamic_pitch_max;
    }
    else if (owner->_ctx.data.target_pitch_rad < dynamic_pitch_min)
    {
        owner->_ctx.data.target_pitch_rad = dynamic_pitch_min;
    }

    const float yaw_error =
        owner->_ctx.data.target_yaw_rad - owner->_ctx.data.yaw_imu_rad;
    if (yaw_error > PI)
    {
        owner->_ctx.data.target_yaw_rad -= 2.0f * PI;
    }
    else if (yaw_error < -PI)
    {
        owner->_ctx.data.target_yaw_rad += 2.0f * PI;
    }

    owner->_gimbal_control();
    _send_motor_command(&owner->_ctx);
}

void screw_gimbal_t::fsm_active_t::on_exit(owner *owner) {}
} // namespace pyro