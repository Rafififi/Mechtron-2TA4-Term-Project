#include "VehicleContext.h"

VehicleContext::VehicleContext(PinName ldr_l, PinName ldr_r, PinName ldr_l_gnd,
                               PinName ldr_r_gnd, PinName mtr_l_in1,
                               PinName mtr_l_in2, PinName mtr_r_in3,
                               PinName mtr_r_in4, PinName mtr_l_pwm,
                               PinName mtr_r_pwm, PinName led_g, PinName led_r,
                               float learning_rate, float ci_change_rate)
    : m_ldr_l(ldr_l),
      m_ldr_r(ldr_r),
      m_ldr_l_gnd(ldr_l_gnd, 0),
      m_ldr_r_gnd(ldr_r_gnd, 0),
      m_mtr_l_in1(mtr_l_in1, 0),
      m_mtr_l_in2(mtr_l_in2, 0),
      m_mtr_l_pwm(mtr_l_pwm),
      m_mtr_r_in3(mtr_r_in3, 0),
      m_mtr_r_in4(mtr_r_in4, 0),
      m_mtr_r_pwm(mtr_r_pwm),
      m_curr_state_ptr(nullptr),
      m_curr_state(IDLE),
      m_prev_state(IDLE),
      m_light_lvl_min({1.0, 1.0}),
      m_light_lvl_max({0.0, 0.0}),
      m_led_g(led_g),
      m_led_r(led_r),
      m_comms_influence(0.0f),
      m_learning_rate(learning_rate),
      m_ci_change_rate(ci_change_rate) {
  // set up photoresistors
  m_ldr_l.set_reference_voltage(3.0f);
  m_ldr_r.set_reference_voltage(3.0f);

  // initialize PWM for drive
  m_mtr_l_pwm.period(0.00005f);  // 1 kHz
  m_mtr_r_pwm.period(0.00005f);
  m_mtr_l_pwm.write(0.0f);
  m_mtr_r_pwm.write(0.0f);

  // set up state tables
  m_state_node_instances[IDLE] = &m_state_idle;
  m_state_node_instances[COWARD] = &m_state_coward;
  m_state_node_instances[AGGRESSIVE] = &m_state_aggressive;
  m_state_node_instances[LOVE] = &m_state_love;
  m_state_node_instances[EXPLORER] = &m_state_explorer;

  // and initialize the fsm
  initialize_fsm();
}

void VehicleContext::initialize_fsm(void) {
  for (int i = 0; i < NUM_STATES; ++i) {
    for (int j = 0; j < NUM_STATES; ++j) {
      m_probability_table[i][j] = 1.0f / static_cast<float>(NUM_STATES);
    }
  }

  m_curr_state_ptr = get_state_node(m_curr_state);
  m_time_state_entry = Kernel::Clock::now();
  read_sensors();
  m_light_lvl_entry = m_light_lvl_curr;

  if (m_curr_state_ptr) {
    m_curr_state_ptr->enter(*this);
  }
}

StateNode* VehicleContext::get_state_node(StateEnum state) const {
  return m_state_node_instances[state];
}

void VehicleContext::read_sensors(void) {
  float raw_ldr_l = m_ldr_l.read();
  float raw_ldr_r = m_ldr_r.read();

  float min_ldr_l = min(raw_ldr_l, m_light_lvl_min.lvl_left);
  float max_ldr_l = max(raw_ldr_l, m_light_lvl_max.lvl_left);
  float min_ldr_r = min(raw_ldr_r, m_light_lvl_min.lvl_right);
  float max_ldr_r = max(raw_ldr_r, m_light_lvl_max.lvl_right);

  m_light_lvl_min = {
      .lvl_left = min_ldr_l,
      .lvl_right = min_ldr_r,
  };
  m_light_lvl_max = {
      .lvl_left = max_ldr_l,
      .lvl_right = max_ldr_r,
  };

  float norm_ldr_l = (raw_ldr_l - m_light_lvl_min.lvl_left) /
                     (m_light_lvl_max.lvl_left - m_light_lvl_min.lvl_left);
  float norm_ldr_r = (raw_ldr_r - m_light_lvl_min.lvl_right) /
                     (m_light_lvl_max.lvl_right - m_light_lvl_min.lvl_right);

  m_light_lvl_curr = {
      .lvl_left = norm_ldr_l,
      .lvl_right = norm_ldr_r,
  };
}

void VehicleContext::run_fsm_cycle(void) {
  read_sensors();

  if (m_curr_state_ptr) {
    m_curr_state_ptr->execute(*this);
  } else {
    // oh shit, something went wrong we should try to recover to IDLE
    transition_to(IDLE);
  }
}

void VehicleContext::transition_to(StateEnum next_state) {
  if (next_state >= NUM_STATES || next_state < 0) {
    // invalid state to be in, so ignore
    return;
  }

  float reward = calculate_reward(m_light_lvl_entry, m_light_lvl_curr);
  update_probability_table(reward);

  if (m_curr_state_ptr) {
    m_curr_state_ptr->exit(*this);
  }

  m_prev_state = m_curr_state;
  m_curr_state = next_state;
  m_curr_state_ptr = get_state_node(m_curr_state);

  if (m_curr_state_ptr) {
    m_time_state_entry = Kernel::Clock::now();
    m_light_lvl_entry = m_light_lvl_curr;
    m_curr_state_ptr->enter(*this);
  } else {
    // shit, try defaulting to idle
    m_curr_state = IDLE;
    m_curr_state_ptr = get_state_node(IDLE);
    if (m_curr_state_ptr) {
      m_curr_state_ptr->enter(*this);
    }
  }
}

float VehicleContext::calculate_reward(LightLevels before, LightLevels after) {
  float avg_before = (before.lvl_left + before.lvl_right) / 2.0f;
  float avg_after = (after.lvl_left + after.lvl_right) / 2.0f;
  return avg_before - avg_after;
}

void VehicleContext::update_probability_table(float reward) {
  float& prob_to_update = m_probability_table[m_prev_state][m_curr_state];
  float delta = m_learning_rate * reward;
  prob_to_update += delta;

  if (NUM_STATES > 1) {
    float reverse_delta = -delta / static_cast<float>(NUM_STATES - 1);
    for (int i = 0; i < NUM_STATES; ++i) {
      if (i != m_curr_state) {
        m_probability_table[m_prev_state][i] += reverse_delta;
      }
    }
  }

  normalize_probabilities(m_prev_state);

#ifdef PRINT_DEBUG
  float* foo = m_probability_table[m_prev_state];
  for (int i = 0; i < NUM_STATES; ++i) {
    printf("State %d | Probability: %f | ", i, foo[i]);
  }
  printf("\r\n");
#endif
}

void VehicleContext::normalize_probabilities(StateEnum state) {
  const float min_prob = 0.01f;
  float sum = 0.0f;

  for (int i = 0; i < NUM_STATES; ++i) {
    if (m_probability_table[state][i] < min_prob) {
      m_probability_table[state][i] = min_prob;
    }
    sum += m_probability_table[state][i];
  }

  if (sum > 0.0f) {
    for (int i = 0; i < NUM_STATES; ++i) {
      m_probability_table[state][i] /= sum;
    }
  } else {
    // something went wrong updating weights, so reset to uniform
    for (int i = 0; i < NUM_STATES; ++i) {
      m_probability_table[state][i] = 1.0f / static_cast<float>(NUM_STATES);
    }
  }
}

StateEnum VehicleContext::sample_next_state(void) {
  float* probabilities = m_probability_table[m_curr_state];

  float sample = ((float)rand() / (float)(RAND_MAX));

  float cum_sum = 0.0f;
  for (int i = 0; i < NUM_STATES; ++i) {
    cum_sum += probabilities[i];
    if (sample <= cum_sum) {
      return static_cast<StateEnum>(i);
    }
  }

  // otherwise we fallback to idle
  return IDLE;
}

LightLevels VehicleContext::get_curr_light_lvls(void) const {
  return m_light_lvl_curr;
}

Kernel::Clock::duration VehicleContext::get_elapsed_time_in_state(void) const {
  return Kernel::Clock::now() - m_time_state_entry;
}

Kernel::Clock::duration VehicleContext::get_min_duration(
    StateEnum state) const {
  return m_min_state_duration[state];
}

void VehicleContext::set_motor_speeds(Direction dir_l, Direction dir_r, float pwm_l, float pwm_r) {
  switch(dir_l) {
    case FORWARD:
      m_mtr_l_in1.write(1);
      m_mtr_l_in2.write(0);
      break;
    case REVERSE:
      m_mtr_l_in1.write(0);
      m_mtr_l_in2.write(1);
      break;
    case STOP:
    default:
      m_mtr_l_in1.write(0);
      m_mtr_l_in2.write(0);
      break;
  }
  m_mtr_l_pwm.write(pwm_l);

  switch (dir_r) {
    case FORWARD:
      m_mtr_r_in3.write(0);
      m_mtr_r_in4.write(1);
      break;
    case REVERSE:
      m_mtr_r_in3.write(1);
      m_mtr_r_in4.write(0);
      break;
    case STOP:
    default:
      m_mtr_r_in3.write(0);
      m_mtr_r_in4.write(0);
      break;
  }
  m_mtr_r_pwm.write(pwm_r);
}
