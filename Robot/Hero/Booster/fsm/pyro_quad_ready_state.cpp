#include "pyro_dwt_drv.h"
#include "pyro_quad_booster.h"

namespace pyro
{
void quad_booster_t::fsm_active_t::state_ready_t::enter(owner *owner)
{

}

void quad_booster_t::fsm_active_t::state_ready_t::execute(owner *owner)
{
    if (owner->_ctx.cmd->fire_enable)
    {
        owner->_ctx.cmd->fire_enable = false;
        owner->_ctx.data.signal_timer = dwt_drv_t::get_timeline_ms();
        owner->_ctx.data.target_trig_rad -= PI / 3.0f; // 每次拨弹60度
        request_switch(&owner->_state_active._busy_state);
    }

    // @TODO: 循环判断摩擦轮转速，不符合要求则退回interim状态
    for (int i = 0; i < 4; i++)
    {
        if (abs(owner->_ctx.data.current_fric_mps[i] - owner->_ctx.data.target_fric_mps[i]) > 0.5f)
        {
            request_switch(&owner->_state_active._interim_state);
            break;
        }
    }

    owner->_trigger_position_control();
    owner->_send_trigger_command();
}

void quad_booster_t::fsm_active_t::state_ready_t::exit(owner *owner)
{

}
}