#include "ExplorerStateNode.h"
#include "VehicleContext.h"

void ExplorerStateNode::enter(VehicleContext& ctx) {
  #ifdef PRINT_DEBUG
  printf("Entering Explorer state\r\n");
  #endif
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

void ExplorerStateNode::execute(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Executing Explorer state\r\n");
#endif
  LightLevels lvls = ctx.get_curr_light_lvls();
  float pwm_l = 1.0 - (lvls.lvl_left * m_max_speed);
  float pwm_r = 1.0 - (lvls.lvl_right * m_max_speed);
  ctx.set_motor_speeds(FORWARD, FORWARD, pwm_l, pwm_r);

  if (ctx.get_elapsed_time_in_state() >= ctx.get_min_duration(get_enum())) {
    StateEnum next_state = ctx.sample_next_state();

    if (next_state != get_enum()) {
      ctx.transition_to(next_state);
      return;
    }
  }
}

void ExplorerStateNode::exit(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Exiting Explorer state\r\n");
#endif
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

StateEnum ExplorerStateNode::get_enum() const { return EXPLORER; }
