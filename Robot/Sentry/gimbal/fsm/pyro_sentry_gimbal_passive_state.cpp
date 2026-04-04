#include "pyro_sentry_gimbal.h"

namespace pyro
{
void gimbal_t::state_passive_t::enter(owner *owner)
{
    if (dm_motor_drv_t::ok !=
            owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
    owner->_ctx.gimbal_config.motor.pitch->disable();
    owner->_ctx.gimbal_config.motor.yaw->disable();
}

void gimbal_t::state_passive_t::execute(owner *owner)
{
    if (dm_motor_drv_t::ok !=
        owner->_ctx.gimbal_config.motor.pitch->get_error_code())
    {
        owner->_ctx.gimbal_config.motor.pitch->clear_error();
    }
    owner->_ctx.gimbal_config.motor.pitch->send_torque(0);
    owner->_ctx.gimbal_config.motor.yaw->send_torque(0);
}

void gimbal_t::state_passive_t::exit(owner *owner)
{
}

}