#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::state_manual_t::enter(owner *owner)
{
    float yaw, pitch, roll;
    ins_drv_t *ins = ins_drv_t::get_instance();
    ins->get_rads_b(&yaw, &pitch, &roll);
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
    owner->_ctx.gimbal_config.motor.yaw->enable();
    owner->_ctx.gimbal_config.motor.pitch->enable();
    owner->_ctx.data.target_pitch_rad = owner->_ctx.data.current_pitch_rad;
}

void gimbal_t::fsm_active_t::state_manual_t::execute(owner *owner)
{
    if (owner->_ctx.data.target_pitch_rad >
        owner->_ctx.gimbal_config.pitch_max_rad)
        owner->_ctx.data.target_pitch_rad =
            owner->_ctx.gimbal_config.pitch_max_rad;
    if (owner->_ctx.data.target_pitch_rad <
        owner->_ctx.gimbal_config.pitch_min_rad)
        owner->_ctx.data.target_pitch_rad =
            owner->_ctx.gimbal_config.pitch_min_rad;
    if (owner->_ctx.data.target_yaw_rad > owner->_ctx.gimbal_config.yaw_max_rad)
        owner->_ctx.data.target_yaw_rad = owner->_ctx.gimbal_config.yaw_max_rad;
    if (owner->_ctx.data.target_yaw_rad < owner->_ctx.gimbal_config.yaw_min_rad)
        owner->_ctx.data.target_yaw_rad = owner->_ctx.gimbal_config.yaw_min_rad;

    owner->_ctx.data.target_yaw_rad = 0;
    owner->_ctx.data.target_pitch_rad -=
        owner->_ctx.cmd->target_delta_pitch_rad;
    _gimbal_mec_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void gimbal_t::fsm_active_t::state_manual_t::exit(owner *owner)
{
}
} // namespace pyro
