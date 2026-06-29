#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/socket/socket.h"
#include "esphome/components/uart/uart.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#include <memory>
#include <string>
#include <vector>

namespace esphome {
namespace stream_server {

class StreamServerComponent : public esphome::Component {
public:
    StreamServerComponent() = default;
    explicit StreamServerComponent(esphome::uart::UARTComponent* stream) : stream_{ stream } {}
    void set_uart_parent(esphome::uart::UARTComponent* parent) { this->stream_ = parent; }
    void set_buffer_size(size_t size) { this->buf_size_ = size; }

#ifdef USE_BINARY_SENSOR
    void set_connected_sensor(esphome::binary_sensor::BinarySensor *connected) { this->connected_sensor_ = connected; }
#endif

    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;

    void pause();
    void resume();


    float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }

    void set_port(uint16_t port) { this->port_ = port; }

protected:
    void publish_sensor();

    void accept();
    void cleanup();
    void read();
//    void flush();
    void write();

    std::unique_ptr<esphome::socket::Socket> client_socket{nullptr};
    std::string identifier{};
    
    esphome::uart::UARTComponent *stream_{nullptr};
    uint16_t port_{6636};
    size_t buf_size_{1024};

    bool paused_{false};

#ifdef USE_BINARY_SENSOR
    esphome::binary_sensor::BinarySensor *connected_sensor_;
#endif

    std::unique_ptr<uint8_t[]> buf_{};

    std::unique_ptr<esphome::socket::Socket> socket_{};
};

template <typename... Ts>
class PauseAction : public esphome::Action<Ts...> {
    public:
        explicit PauseAction(StreamServerComponent *ss) : ss_(ss) {}
        void play(const Ts &...x) override { this->ss_->pause(); }

    protected:
        StreamServerComponent *ss_;
};

template <typename... Ts>
class ResumeAction : public esphome::Action<Ts...> {
    public:
        explicit ResumeAction(StreamServerComponent *ss) : ss_(ss) {}
        void play(const Ts &...x) override { this->ss_->resume(); }

    protected:
        StreamServerComponent *ss_;
};

} // namespace stream_server
}  // namespace esphome
