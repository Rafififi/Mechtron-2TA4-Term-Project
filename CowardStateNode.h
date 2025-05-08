#pragma once

#include "StateNode.h"

class CowardStateNode : public StateNode {
 public:
  void enter(VehicleContext& ctx) override;
  void execute(VehicleContext& ctx) override;
  void exit(VehicleContext& ctx) override;
  StateEnum get_enum(void) const override;

 public:
  const float m_max_speed = 1.0f;
};
