#include "pyro_17mm_booster.h"
// 3. 发射就绪状态 (SHOOT_READY_SHOOT)
namespace pyro
{

void shoot_17mm_control_t::state_ready_shoot_t::enter(owner *ctx)
{
    ctx->_ctx.data.fric_pid_active    = true;
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.target_trig_rad    = ctx->_ctx.data.current_trig_rad;
    ctx->_ctx.data.target_trig_radps  = 0;
    ctx->_ctx.data.trig_pid_active    = false;
}

void shoot_17mm_control_t::state_ready_shoot_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }
    if (ctx->_ctx.cmd->single_shoot)
    {
        this->request_switch(&ctx->_state_single_bullet);
    }
    else if (ctx->_ctx.cmd->continue_shoot)
    {
        this->request_switch(&ctx->_state_continue_bullet);
    }
}

void shoot_17mm_control_t::state_ready_shoot_t::exit(owner *ctx)
{
}

}