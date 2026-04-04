//
// Created by pason on 2026/2/2.
//
#include "pyro_yaw.h"

namespace pyro
{
void yaw_t::fsm_active_t::on_enter(owner *owner)
{
    if (dm_motor_drv_t::ok !=
        owner->_ctx.yaw_config.motor.yaw->get_error_code())
    {
        owner->_ctx.yaw_config.motor.yaw->clear_error();
    }
    owner->_ctx.yaw_config.motor.yaw->enable();
}

void yaw_t::fsm_active_t::on_execute(owner *owner)
{
    this->change_state(&_manual_state);
}

void yaw_t::fsm_active_t::on_exit(owner *owner)
{
}


} // namespace pyro