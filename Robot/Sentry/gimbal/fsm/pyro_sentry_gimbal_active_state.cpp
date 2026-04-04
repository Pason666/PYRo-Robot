#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::fsm_active_t::on_enter(owner *owner)
{
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
    owner->_ctx.gimbal_config.motor.yaw->enable();
    owner->_ctx.gimbal_config.motor.pitch->enable();
}

void gimbal_t::fsm_active_t::on_execute(owner *owner)
{
    if (gimbal_cmd_t::gimbal_mode_t::SCANNING == owner->_ctx.cmd->gimbal_mode)
    {
        this->change_state(&_scanning_state);
    }
    else if (gimbal_cmd_t::gimbal_mode_t::MANUAL ==
             owner->_ctx.cmd->gimbal_mode)
    {
        this->change_state(&_manual_state);
    }
}

void gimbal_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro