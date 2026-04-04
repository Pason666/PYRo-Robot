#include "pyro_rud_chassis.h"

namespace pyro
{
void rud_chassis_t::state_active_t::enter(owner *owner)
{
    // 使能所有电机（有力状态），保留原有使能逻辑
    owner->_ctx.rud_config.motor.rudder[0]->enable();
    owner->_ctx.rud_config.motor.rudder[1]->enable();
    owner->_ctx.rud_config.motor.rudder[2]->enable();
    owner->_ctx.rud_config.motor.rudder[3]->enable();
    owner->_ctx.rud_config.motor.wheel[0]->enable();
    owner->_ctx.rud_config.motor.wheel[1]->enable();
    owner->_ctx.rud_config.motor.wheel[2]->enable();
    owner->_ctx.rud_config.motor.wheel[3]->enable();
}

void rud_chassis_t::state_active_t::execute(owner *owner)
{
    owner->_kinematics_solve();
    _chassis_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void rud_chassis_t::state_active_t::exit(owner *owner)
{
}

} // namespace pyro