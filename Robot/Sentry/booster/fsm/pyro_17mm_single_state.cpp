#include "pyro_17mm_booster.h"
// 6. 单发状态 (SHOOT_SINGLE_BULLET)

extern pyro::booster_cmd_t *booster_cmd_ptr;

namespace pyro
{

void shoot_17mm_control_t::state_single_bullet_t::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode = data_ctx_t::trig_mode_e::POSITION; // 切位置模式
    ctx->_ctx.data.trig_pid_active = true;
    ctx->_ctx.data.target_trig_rad = ctx->_ctx.data.current_trig_rad + PI / 4;
}

void shoot_17mm_control_t::state_single_bullet_t::execute(owner *ctx)
{
    if (!ctx->_ctx.cmd->is_fric_on)
    {
        this->request_switch(&ctx->_state_stop);
        return;
    }
    if (abs(ctx->_ctx.data.current_trig_rad -
                        ctx->_ctx.data.target_trig_rad) < 0.05f)
    {
        this->request_switch(&ctx->_state_done);
    }
}

void shoot_17mm_control_t::state_single_bullet_t::exit(owner *ctx)
{
    booster_cmd_ptr->single_shoot = false;
}

}