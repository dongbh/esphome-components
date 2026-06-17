#include "uart_hw_flow.h"

#include "esphome/core/log.h"

#ifdef USE_ESP_IDF
#include "driver/gpio.h"
#include "driver/uart.h"
#endif

namespace esphome {
namespace uart_hw_flow {

static const char *const TAG = "uart_hw_flow";

void UARTHwFlowComponent::setup() {
#ifdef USE_ESP_IDF
  if (this->early_rts_assert_ && this->rts_pin_ >= 0) {
    gpio_reset_pin(static_cast<gpio_num_t>(this->rts_pin_));
    gpio_set_direction(static_cast<gpio_num_t>(this->rts_pin_), GPIO_MODE_OUTPUT);
    gpio_set_level(static_cast<gpio_num_t>(this->rts_pin_), 0);
  }
  this->apply();
#else
  ESP_LOGE(TAG, "uart_hw_flow requires ESP-IDF");
  this->mark_failed();
#endif
}

void UARTHwFlowComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "UART HW Flow:");
  ESP_LOGCONFIG(TAG, "  UART num: %d", this->uart_num_);
  ESP_LOGCONFIG(TAG, "  TX/RX/RTS/CTS pins: %d/%d/%d/%d", this->tx_pin_, this->rx_pin_, this->rts_pin_,
                this->cts_pin_);
  ESP_LOGCONFIG(TAG, "  RX flow threshold: %d", this->rx_flow_ctrl_thresh_);
  ESP_LOGCONFIG(TAG, "  Early RTS assert: %s", YESNO(this->early_rts_assert_));
}

void UARTHwFlowComponent::apply() {
#ifdef USE_ESP_IDF
  if (this->uart_num_ < 0 || this->tx_pin_ < 0 || this->rx_pin_ < 0 || this->rts_pin_ < 0 ||
      this->cts_pin_ < 0) {
    ESP_LOGE(TAG, "UART HW flow control is not fully configured");
    this->mark_failed();
    return;
  }

  auto uart_num = static_cast<uart_port_t>(this->uart_num_);
  esp_err_t err = uart_set_pin(uart_num, this->tx_pin_, this->rx_pin_, this->rts_pin_, this->cts_pin_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  err = uart_set_hw_flow_ctrl(uart_num, UART_HW_FLOWCTRL_CTS_RTS, this->rx_flow_ctrl_thresh_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "uart_set_hw_flow_ctrl failed: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }

  ESP_LOGD(TAG, "Applied CTS/RTS hardware flow control on UART%d", this->uart_num_);
#endif
}

}  // namespace uart_hw_flow
}  // namespace esphome
