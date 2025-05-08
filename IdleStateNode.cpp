#include "IdleStateNode.h"
#include "VehicleContext.h"

void IdleStateNode::enter(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Entering Idle state\r\n");
#endif
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

void IdleStateNode::execute(VehicleContext& ctx) {
  // todo: implement communication as required
#ifdef PRINT_DEBUG
  printf("Executing Idle state\r\n");
#endif

  if (ctx.get_elapsed_time_in_state() >= ctx.get_min_duration(get_enum())) {
    StateEnum next_state = ctx.sample_next_state();

    if (next_state != get_enum()) {
      ctx.transition_to(next_state);
      return;
    }
  }
}

void IdleStateNode::exit(VehicleContext& ctx) {
#ifdef PRINT_DEBUG
  printf("Exiting Idle state\r\n");
#endif
  ctx.set_motor_speeds(STOP, STOP, 0.0, 0.0);
}

StateEnum IdleStateNode::get_enum() const { return IDLE; }
