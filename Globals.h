#pragma once
#include "mbed.h"

#ifndef NUM_STATES
#define NUM_STATES 5
#endif

#ifndef MAIL_SIZE
#define MAIL_SIZE 16
#endif

#ifndef MSG_SIZE
#define MSG_SIZE 32
#endif


enum StateEnum : uint8_t {
  IDLE = 0,
  COWARD,
  AGGRESSIVE,
  LOVE,
  EXPLORER,
};

typedef enum {
  FORWARD,
  REVERSE,
  STOP,
} Direction;

typedef struct {
  float lvl_left;
  float lvl_right;
} LightLevels;

#pragma pack (push, 1)
typedef struct {
  LightLevels prev_lvls;
  LightLevels curr_lvls;
  StateEnum prev_state;
  uint8_t padding[15] = {0};
} CommsMsg;
#pragma pack (pop)
