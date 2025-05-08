#include "AggressiveStateNode.h"
#include "VehicleContext.h"

void AggressiveStateNode::enter(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Entering Aggressive state\r\n");
#endif
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

void AggressiveStateNode::execute(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Executing Aggressive state\r\n");
#endif
  LightLevels lvls = ctx.get_curr_light_lvls();
  float pwm_l = lvls.lvl_right * m_max_speed;
  float pwm_r = lvls.lvl_left * m_max_speed;
  ctx.set_motor_speeds(FORWARD, FORWARD, pwm_l, pwm_r);

  if (ctx.get_elapsed_time_in_state() >= ctx.get_min_duration(get_enum())) {
    StateEnum next_state = ctx.sample_next_state();

    if (next_state != get_enum()) {
      ctx.transition_to(next_state);
      return;
    }
  }
}

void AggressiveStateNode::exit(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Exiting Aggressive state\r\n");
#endif
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

StateEnum AggressiveStateNode::get_enum() const { return AGGRESSIVE; }
