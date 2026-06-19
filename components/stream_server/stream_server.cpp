#include "stream_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/version.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

#include <algorithm>
#include <cerrno>
#include <span>

namespace esphome {
namespace stream_server {

static const char *TAG = "stream_server";

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    // The make_unique() wrapper doesn't like arrays, so initialize the unique_ptr directly.
    this->buf_ = std::unique_ptr<uint8_t[]>{new uint8_t[this->buf_size_]};

    struct sockaddr_storage bind_addr;
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2023, 4, 0)
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), this->port_);
#else
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), htons(this->port_));
#endif

    this->socket_ = socket::socket_ip(SOCK_STREAM, PF_INET);
    this->socket_->setblocking(false);
    this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), bind_addrlen);
    this->socket_->listen(8);

    this->publish_sensor();
}

void StreamServerComponent::loop() {
    if (this->paused_) {
        return;
    }
    this->accept();
    this->read();
    this->write();
}

void StreamServerComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
#if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address(), this->port_);
#else
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address().c_str(), this->port_);
#endif
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Connected:", this->connected_sensor_);
#endif
}

void StreamServerComponent::on_shutdown() {
    if (this->client_socket_)
        this->client_socket_->shutdown(SHUT_RDWR);
}

void StreamServerComponent::publish_sensor() {
#ifdef USE_BINARY_SENSOR
    if (this->connected_sensor_)
        this->connected_sensor_->publish_state(this->client_socket_ != nullptr);
#endif
}

void StreamServerComponent::accept() {
    struct sockaddr_storage client_addr;
    socklen_t client_addrlen = sizeof(client_addr);
    std::unique_ptr<socket::Socket> socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;

    this->cleanup();
    this->client_socket_ = std::move(socket);
    this->client_socket_->setblocking(false);

#if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 1, 0)
    this->identifier_ = std::string{esphome::socket::SOCKADDR_STR_LEN, 0};
    auto identifier_span = std::span<char, esphome::socket::SOCKADDR_STR_LEN>(this->identifier_.data(), this->identifier_.size());
    this->identifier_.resize(this->client_socket_->getpeername_to(identifier_span));
#else
    this->identifier_ = this->client_socket_->getpeername();
#endif

    ESP_LOGD(TAG, "New client connected from %s", this->identifier_.c_str());
    this->publish_sensor();
}

void StreamServerComponent::cleanup() {
    if (this->client_socket_) {
        this->client_socket_->shutdown(SHUT_RDWR);
        this->client_socket_.reset();
        this->identifier_.clear();
    }
    this->publish_sensor();
}

void StreamServerComponent::read() {
    size_t len = 0;
    ssize_t written;

    while ((len = this->stream_->available()) > 0) {
        len = std::min<size_t>(len, this->buf_size_);
        this->stream_->read_array(this->buf_.get(), len);
        if (this->client_socket_) {
            if ((written = this->client_socket_->write(this->buf_.get(), len)) > 0) {
                // Successfully written.
            } else if (written == 0 || errno == ECONNRESET) {
                ESP_LOGD(TAG, "Client %s disconnected", this->identifier_.c_str());
                this->cleanup();
            } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
                // Expected if the TCP transmit buffer is full.
            } else {
                ESP_LOGE(TAG, "Failed to write to client %s with error %d!", this->identifier_.c_str(), errno);
            }
        }
    }
}

void StreamServerComponent::write() {
    ssize_t nread;
    if (this->client_socket_) {
        while ((nread = this->client_socket_->read(this->buf_.get(), this->buf_size_)) > 0) {
            this->stream_->write_array(this->buf_.get(), nread);
        }

        if (nread == 0 || errno == ECONNRESET) {
            ESP_LOGD(TAG, "Client %s disconnected", this->identifier_.c_str());
            this->cleanup();
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Expected if the TCP receive buffer is empty.
        } else {
            ESP_LOGW(TAG, "Failed to read from client %s with error %d!", this->identifier_.c_str(), errno);
        }
    }
}

void StreamServerComponent::pause() {
    if (this->paused_)
        return;
    this->paused_ = true;
    ESP_LOGI(TAG, "Pausing stream server and disconnecting clients");
    this->cleanup();
}

void StreamServerComponent::resume() {
    if (!this->paused_)
        return;
    ESP_LOGI(TAG, "Resuming stream server");
    this->paused_ = false;
}

}  // namespace stream_server
}  // namespace esphome
