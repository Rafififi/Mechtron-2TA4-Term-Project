#pragma once

#include "AggressiveStateNode.h"
#include "CowardStateNode.h"
#include "ExplorerStateNode.h"
#include "IdleStateNode.h"
#include "LoveStateNode.h"
#include "StateNode.h"
#include "Globals.h"
#include "mbed.h"

// todo: add support for comms after

#ifndef NUM_STATES
#define NUM_STATES 5
#endif

class VehicleContext {
 public:
  VehicleContext(PinName ldr_l, PinName ldr_r, PinName ldr_l_gnd,
                 PinName ldr_r_gnd, PinName mtr_l_in1, PinName mtr_l_in2,
                 PinName mtr_r_in3, PinName mtr_r_in4, PinName mtr_l_pwm,
                 PinName mtr_r_pwm, PinName led_g, PinName led_r,
                 float learning_rate = 0.1f, float ci_change_rate = 0.05f);

  /**
   * @brief Is called every FSM "tick". Calls StateNode::execute.
   */
  void run_fsm_cycle(void);

  /**
   * @brief Called by StateNode::execute to indicate what state to
   * transition into next. Handles learning updates, general cleanup, etc.
   * Calls StateNode::exit before performing transition.
   */
  void transition_to(StateEnum next_state);

  /**
   * @returns LightLevels struct containing left and right values of
   * photoresistors. Noramlized between 0.0f - 1.0f.
   */
  LightLevels get_curr_light_lvls(void) const;

  /**
   * @returns Elapsed time in current state as `Kernel::Clock::duration`.
   */
  Kernel::Clock::duration get_elapsed_time_in_state(void) const;

  /**
   * @returns The minimum duration for a given state as
   * `Kernel::Clock::duration`.
   */
  Kernel::Clock::duration get_min_duration(StateEnum state) const;

  /**
   * @brief Sets the direction and "speed" (PWM duty cycle) of the left and
   * right wheels. Direction parameters (`dir_x`) use the following characters:
   * `F` (forwards), `R` (reverse), `S` (stop).
   * @param dir_l Direction of the left wheel.
   * @param dir_r Direction of the right wheel.
   * @param pwm_l "Speed" of the left wheel as a PWM duty cycle (0.0f - 1.0f).
   * @param pwm_r "Speed" of the right wheel as a PWM duty cycle (0.0f - 1.0f).
   */
  void set_motor_speeds(Direction dir_l, Direction dir_r, float pwm_l, float pwm_r);

  /**
   * @brief Updates the probability table using built-in reward mechanisms and
   * internal states. Reward mechanism based on minimizing light levels.
   */
  void update_probability_table(float reward);

  // /**
  //  * @brief Updates the influence of communication from other vehicles on the
  //  * state probabilities. Values updated using internal reward mechanisms.
  //  */
  // void update_comms_influence(void);

  /**
   * @returns The next state based on current probabilities, optionally
   * influenced by communication.
   */
  StateEnum sample_next_state(void);

 private:
  // for photoresistors
  AnalogIn m_ldr_l;
  AnalogIn m_ldr_r;
  DigitalOut m_ldr_l_gnd;
  DigitalOut m_ldr_r_gnd;

  // for motor control
  DigitalOut m_mtr_l_in1;
  DigitalOut m_mtr_l_in2;
  PwmOut m_mtr_l_pwm;
  DigitalOut m_mtr_r_in3;
  DigitalOut m_mtr_r_in4;
  PwmOut m_mtr_r_pwm;

  // state node instances
  IdleStateNode m_state_idle;
  AggressiveStateNode m_state_aggressive;
  CowardStateNode m_state_coward;
  ExplorerStateNode m_state_explorer;
  LoveStateNode m_state_love;
  StateNode* m_state_node_instances[NUM_STATES];

  // for internal FSM state and other values
  StateNode* m_curr_state_ptr;
  StateEnum m_curr_state;
  StateEnum m_prev_state;
  Kernel::Clock::time_point m_time_state_entry;
  LightLevels m_light_lvl_entry;
  LightLevels m_light_lvl_curr;
  LightLevels m_light_lvl_min;
  LightLevels m_light_lvl_max;

  // leds
  DigitalOut m_led_g;
  DigitalOut m_led_r;

  // for learning and other things
  float m_probability_table[NUM_STATES][NUM_STATES];
  float m_comms_influence;
  const float m_learning_rate;
  const float m_ci_change_rate;

  // for miscellaneous configuration
  const Kernel::Clock::duration m_min_state_duration[NUM_STATES] = {
      1000ms,  // IDLE
      2000ms,  // TOWARDS_LIGHT
      2000ms,  // AWAY_LIGHT
      2000ms,  // TOWARDS_DARK
      2000ms,  // AWAY_DARK
  };

  void initialize_fsm(void);
  StateNode* get_state_node(StateEnum state) const;
  void read_sensors(void);
  float calculate_reward(LightLevels before, LightLevels after);
  void normalize_probabilities(StateEnum state);
};
