#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace uart_hw_flow {

class UARTHwFlowComponent : public Component {
 public:
  void set_uart(esphome::uart::UARTComponent *uart) { this->uart_ = uart; }
  void set_uart_num(int uart_num) { this->uart_num_ = uart_num; }
  void set_pins(int tx_pin, int rx_pin, int rts_pin, int cts_pin) {
    this->tx_pin_ = tx_pin;
    this->rx_pin_ = rx_pin;
    this->rts_pin_ = rts_pin;
    this->cts_pin_ = cts_pin;
  }
  void set_rx_flow_ctrl_thresh(int threshold) { this->rx_flow_ctrl_thresh_ = threshold; }
  void set_early_rts_assert(bool early_rts_assert) { this->early_rts_assert_ = early_rts_assert; }

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE - 1.0f; }

  void apply();

 protected:
  esphome::uart::UARTComponent *uart_{nullptr};
  int uart_num_{-1};
  int tx_pin_{-1};
  int rx_pin_{-1};
  int rts_pin_{-1};
  int cts_pin_{-1};
  int rx_flow_ctrl_thresh_{122};
  bool early_rts_assert_{true};
};

}  // namespace uart_hw_flow
}  // namespace esphome
