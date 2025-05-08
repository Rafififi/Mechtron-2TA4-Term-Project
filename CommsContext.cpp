#include "CommsContext.h"

CommsContext::CommsContext(PinName nrf_mosi, PinName nrf_miso, PinName nrf_sck,
                           PinName nrf_ncs, PinName nrf_ce, nrf_address addr_tx,
                           nrf_address addr_rx)
    : nrf(nrf_mosi, nrf_miso, nrf_sck, nrf_ncs, nrf_ce) {
  nrf.powerUp();
  nrf.setTxAddress(addr_tx);
  nrf.setRxAddress(addr_rx);
  nrf.setTransferSize(MSG_SIZE);
  nrf.setReceiveMode();
  nrf.enable();
}

void CommsContext::run_comms_cycle(void) {
  if (nrf.readable()) {
    char buffer[MSG_SIZE];
    nrf.read(NRF24L01P_PIPE_P0, buffer, MSG_SIZE);
    CommsMsg *msg = mail_incoming.try_alloc();

    if (msg == nullptr) {
      return;
    }

    memcpy(msg, buffer, MSG_SIZE);
    mail_incoming.put(msg);
  } else {
    CommsMsg *msg = mail_outgoing.try_get();
    if (msg == nullptr) {
      return;
    }

    int bytes_written = nrf.write(NRF24L01P_PIPE_P0, (char*)msg, MSG_SIZE);
    if (bytes_written < MSG_SIZE) {
      mail_outgoing.put(msg);
    } else {
      mail_outgoing.free(msg);
    }
  }
}

bool CommsContext::try_queue_send(const CommsMsg msg_vals) {
  CommsMsg *msg = mail_outgoing.try_alloc();
  if (msg == nullptr) {
    return false;
  }

  memcpy(msg, &msg_vals, MSG_SIZE);
  mail_outgoing.put(msg);

  return true;
}

bool CommsContext::try_read(CommsMsg *out) {
  CommsMsg *read_msg = mail_incoming.try_get();
  if (read_msg == nullptr) {
    out = nullptr;
    return false;
  }

  memcpy(out, read_msg, MSG_SIZE);
  mail_incoming.free(read_msg);

  return true;
}
