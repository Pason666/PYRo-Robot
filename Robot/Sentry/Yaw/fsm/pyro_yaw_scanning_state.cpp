#include "pyro_yaw.h"

namespace pyro
{
void yaw_t::fsm_active_t::state_scanning_t::enter(owner *owner)
{
}

void yaw_t::fsm_active_t::state_scanning_t::execute(owner *owner)
{
    owner->_ctx.data.out_yaw_radps = -5.0f;

    _yaw_control(&owner->_ctx);
    _send_motor_command(&owner->_ctx);
}

void yaw_t::fsm_active_t::state_scanning_t::exit(owner *owner)
{
}

}