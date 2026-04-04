#include "pyro_17mm_booster.h"
// 2. 摩擦轮启动状态 (SHOOT_READY_FRIC)
namespace pyro
{

void shoot_17mm_control_t::state_ready_fric_t::enter(owner *ctx)
{
    ctx->_ctx.data.target_fric_radps[0] = -ctx->_ctx.booster_cfg.target_fric_speed;
    ctx->_ctx.data.target_fric_radps[1] = ctx->_ctx.booster_cfg.target_fric_speed;
    ctx->_ctx.data.fric_pid_active      = true;
}
void shoot_17mm_control_t::state_ready_fric_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }

    // 检查摩擦轮速度是否达标
    if (std::abs(ctx->_ctx.data.current_fric_radps[0] -
                 -ctx->_ctx.booster_cfg.target_fric_speed) < 150 &&
        std::abs(ctx->_ctx.data.current_fric_radps[1] -
                 ctx->_ctx.booster_cfg.target_fric_speed) < 150)
    {
        this->request_switch(&ctx->_state_ready_shoot);
    }
}
void shoot_17mm_control_t::state_ready_fric_t::exit(owner *ctx)
{
}

}