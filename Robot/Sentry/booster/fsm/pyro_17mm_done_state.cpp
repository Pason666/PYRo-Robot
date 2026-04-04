#include "pyro_17mm_booster.h"
// 8. 射击完成状态 (SHOOT_DONE)

namespace pyro
{

void shoot_17mm_control_t::state_done_t::enter(owner *ctx)
{
    ctx->_ctx.data.trig_mode          = data_ctx_t::trig_mode_e::SPEED;
    ctx->_ctx.data.trig_pid_active    = false;
    ctx->_ctx.data.target_trig_radps  = 0;
}
void shoot_17mm_control_t::state_done_t::execute(owner *ctx)
{
    this->request_switch(&ctx->_state_ready_shoot);
}
void shoot_17mm_control_t::state_done_t::exit(owner *ctx)
{
}

}