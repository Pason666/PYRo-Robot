#include "pyro_17mm_booster.h"
// 7. 连发状态 (SHOOT_CONTINUE_BULLET)

extern pyro::booster_cmd_t *booster_cmd_ptr;

namespace pyro
{

void shoot_17mm_control_t::state_continue_bullet_t::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::SPEED; // 切速度模式
    ctx->_ctx.data.target_trig_radps  = TRIGGER_CONTINUOUS_RADPS;
    ctx->_ctx.data.trig_pid_active    = true;
}

void shoot_17mm_control_t::state_continue_bullet_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->continue_shoot)
    {
        this->request_switch(&ctx->_state_done);
    }
    // 移除堵弹检测逻辑
}

void shoot_17mm_control_t::state_continue_bullet_t::exit(owner *ctx)
{
}


}