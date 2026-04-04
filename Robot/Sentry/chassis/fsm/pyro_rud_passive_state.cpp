#include "pyro_rud_chassis.h"

namespace pyro
{
void rud_chassis_t::state_passive_t::enter(owner *owner)
{
    owner->_ctx.rud_config.motor.rudder[0]->disable();
    owner->_ctx.rud_config.motor.rudder[1]->disable();
    owner->_ctx.rud_config.motor.rudder[2]->disable();
    owner->_ctx.rud_config.motor.rudder[3]->disable();
    owner->_ctx.rud_config.motor.wheel[0]->disable();
    owner->_ctx.rud_config.motor.wheel[1]->disable();
    owner->_ctx.rud_config.motor.wheel[2]->disable();
    owner->_ctx.rud_config.motor.wheel[3]->disable();
}

void rud_chassis_t::state_passive_t::execute(owner *owner)
{
    owner->_ctx.rud_config.motor.rudder[0]->send_torque(0);
    owner->_ctx.rud_config.motor.rudder[1]->send_torque(0);
    owner->_ctx.rud_config.motor.rudder[2]->send_torque(0);
    owner->_ctx.rud_config.motor.rudder[3]->send_torque(0);
    owner->_ctx.rud_config.motor.wheel[0]->send_torque(0);
    owner->_ctx.rud_config.motor.wheel[1]->send_torque(0);
    owner->_ctx.rud_config.motor.wheel[2]->send_torque(0);
    owner->_ctx.rud_config.motor.wheel[3]->send_torque(0);
}

void rud_chassis_t::state_passive_t::exit(owner *owner)
{
}

} // namespace pyro