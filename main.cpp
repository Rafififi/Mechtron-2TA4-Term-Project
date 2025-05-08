#include "mbed.h"
#include "VehicleContext.h"
#include "CommsContext.h"

#define PIN_ENTROPY PF_4

const auto FSM_TICK_RATE = 250ms;
const auto COMMS_TICK_RATE = 100ms;

CommsContext comms_ctx(PE_14, PE_13, PE_12, PE_11, PE_9, 0xE7E7E7E7E7E7, 0XD8D8D8D8D8);
VehicleContext vehicle_ctx(
  PC_1, PF_10, PC_0, PF_9, PF_5, PF_3, PF_1, PC_15, PF_6, PA_3, PG_13, PG_14
);
AnalogIn entropy(PIN_ENTROPY);
Thread thread_fsm;
Thread thread_comms;

void fsm_proc() {
  while (true) {
    auto cycle_start = Kernel::Clock::now();

#ifdef PRINT_DEBUG
    printf("Running FSM tick\r\n");
#endif
    vehicle_ctx.run_fsm_cycle();

    auto cycle_end = Kernel::Clock::now();
    auto cycle_delta = cycle_end - cycle_start;

    if (cycle_delta < FSM_TICK_RATE) {
      ThisThread::sleep_for(FSM_TICK_RATE - cycle_delta);
    }
  }
}

void comms_proc(){
  while(true){
    auto cycle_start = Kernel::Clock::now();

#ifdef PRINT_DEBUG
    printf("Running COMMS tick\r\n");
#endif
    comms_ctx.run_comms_cycle();

    auto cycle_end = Kernel::Clock::now();
    auto cycle_delta = cycle_end - cycle_start;

    if (cycle_delta < COMMS_TICK_RATE) {
      ThisThread::sleep_for(COMMS_TICK_RATE - cycle_delta);
    }
  }
}

int main() {
  srand(entropy.read_u16());
  auto fsm_thread_start_status = thread_fsm.start(fsm_proc);
  if (fsm_thread_start_status != osOK) {
    error("Failed to start FSM thread\r\n");
  }
#ifdef PRINT_DEBUG
  printf("Initialized FSM thread\r\n");
#endif

  auto comms_thread_start_status = thread_comms.start(comms_proc);
  if (comms_thread_start_status != osOK) {
    error("Failed to start COMMS thread\r\n");
  }
#ifdef PRINT_DEBUG
  printf("Initialized COMMS thread\r\n");
#endif

  while (true) {
    ThisThread::sleep_for(5s);
  }

  return 0;
}
