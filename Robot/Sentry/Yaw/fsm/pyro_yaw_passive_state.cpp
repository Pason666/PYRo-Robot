//
// Created by pason on 2026/2/2.
//
#include "pyro_yaw.h"

namespace pyro
{
void yaw_t::state_passive_t::enter(owner *owner)
{
    owner->_ctx.yaw_config.motor.yaw->disable();
}

void yaw_t::state_passive_t::execute(owner *owner)
{
    owner->_ctx.data.out_yaw_radps = 0.0f;
    owner->_ctx.yaw_config.motor.yaw->send_torque(0);
}

void yaw_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro