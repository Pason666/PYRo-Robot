#include "pyro_17mm_booster.h"
// 反转状态
namespace pyro
{

void shoot_17mm_control_t::state_reverse_t::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::SPEED; // 切速度模式
    ctx->_ctx.data.target_trig_radps  = -TRIGGER_UNJAM_RADPS;
    ctx->_ctx.booster_cfg.motor.trigger->enable();
}

void shoot_17mm_control_t::state_reverse_t::execute(owner *ctx)
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

void shoot_17mm_control_t::state_reverse_t::exit(owner *ctx)
{
}

}