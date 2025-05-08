#pragma once

#include "mbed.h"
#include "Globals.h"
#include "nRF24L01P.h"

using nrf_address = unsigned long long;

class CommsContext {
 public:
  CommsContext(PinName nrf_mosi, PinName nrf_miso, PinName nrf_sck,
               PinName nrf_ncs, PinName nrf_ce, nrf_address addr_tx,
               nrf_address addr_rx);
  
  void run_comms_cycle(void);
  bool try_queue_send(const CommsMsg msg);
  bool try_read(CommsMsg *out);

 private:
    Mail<CommsMsg, MAIL_SIZE> mail_incoming;
    Mail<CommsMsg, MAIL_SIZE> mail_outgoing;
    nRF24L01P nrf;
};
