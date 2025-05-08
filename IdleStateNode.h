#pragma once

#include "StateNode.h"

class IdleStateNode : public StateNode {
 public:
  void enter(VehicleContext& ctx) override;
  void execute(VehicleContext& ctx) override;
  void exit(VehicleContext& ctx) override;
  StateEnum get_enum(void) const override;
};
