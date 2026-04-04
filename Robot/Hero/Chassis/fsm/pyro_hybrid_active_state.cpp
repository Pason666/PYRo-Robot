#include "pyro_hybrid_chassis.h"


namespace pyro
{

void hybrid_chassis_t::fsm_active_t::on_enter(owner *owner)
{
    owner->_ctx.motor.mecanum[0]->enable();
    owner->_ctx.motor.mecanum[1]->enable();
    owner->_ctx.motor.mecanum[2]->enable();
    owner->_ctx.motor.mecanum[3]->enable();
    owner->_ctx.motor.track[0]  ->enable();
    owner->_ctx.motor.track[1]  ->enable();
    owner->_ctx.motor.leg[0]    ->enable();
    owner->_ctx.motor.leg[1]    ->enable();
}

void hybrid_chassis_t::fsm_active_t::on_execute(owner *owner)
{
    if (owner->_ctx.cmd->track_en)
    {
        change_state(&climbing_fsm);
    }
    else
    {
        change_state(&cruising_state);
    }

    owner->_communicate_gimbal();
    owner->_kinematics_solve();
}

void hybrid_chassis_t::fsm_active_t::on_exit(owner *owner)
{
}

} // namespace pyro