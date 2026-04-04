#include "pyro_hybrid_chassis.h"

namespace pyro
{
void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::track_climbing_state_t::
    enter(owner *owner)
{
}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::track_climbing_state_t::
    execute(owner *owner)
{
    // 1. VMC 控制逻辑
    owner->_leg_vmc();
    // 2. 统一发送所有电机指令
    owner->_send_motor_command();

}

void hybrid_chassis_t::fsm_active_t::climbing_fsm_t::track_climbing_state_t::
    exit(owner *owner)
{
}


} // namespace pyro