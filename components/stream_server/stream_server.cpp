#include "stream_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/version.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"

#include <algorithm>

namespace esphome {
namespace stream_server {

static const char *TAG = "stream_server";

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    // The make_unique() wrapper doesn't like arrays, so initialize the unique_ptr directly.
    this->buf_ = std::unique_ptr<uint8_t[]>{new uint8_t[this->buf_size_]};

    struct sockaddr_in bind_addr = {
        .sin_len = sizeof(struct sockaddr_in),
        .sin_family = AF_INET,
        .sin_port = htons(this->port_),
        .sin_addr = {
            .s_addr = ESPHOME_INADDR_ANY,
        }
    };

    this->socket_ = socket::socket(AF_INET, SOCK_STREAM, PF_INET);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 20000;

    this->socket_->setsockopt(SOL_SOCKET, SO_RCVTIMEO, (char *) &timeout, sizeof(timeout));
    this->socket_->setsockopt(SOL_SOCKET, SO_RCVTIMEO, (char *) &timeout, sizeof(timeout));

    this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(struct sockaddr_in));
    this->socket_->listen(8);

    this->publish_sensor();
}

void StreamServerComponent::loop() {
    if (paused_)
        return;
    this->accept();
    this->read();
    this->write();
    this->cleanup();
}

void StreamServerComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address(), this->port_);
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Connected:", this->connected_sensor_);
#endif
}

void StreamServerComponent::on_shutdown() {
    for (const Client &client : this->clients_)
        client.socket->shutdown(SHUT_RDWR);
}

void StreamServerComponent::publish_sensor() {
#ifdef USE_BINARY_SENSOR
    if (this->connected_sensor_)
        this->connected_sensor_->publish_state(!this->clients_.empty());
#endif
}

void StreamServerComponent::accept() {
    struct sockaddr_in client_addr;
    socklen_t client_addrlen = sizeof(struct sockaddr_in);
    std::unique_ptr<esphome::socket::Socket> socket =
        this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;

    for (Client &client : this->clients_)
        client.disconnected = true;

    uint8_t discard[128];
    size_t len;
    while ((len = this->stream_->available()) > 0) {
        len = std::min<size_t>(len, sizeof(discard));
        this->stream_->read_array(discard, len);
    }

    socket->setblocking(false);

    uint32_t peer = ntohl(client_addr.sin_addr.s_addr);
    char identifier[16];
    std::snprintf(identifier, sizeof(identifier), "%u.%u.%u.%u",
                  static_cast<unsigned>((peer >> 24) & 0xFF),
                  static_cast<unsigned>((peer >> 16) & 0xFF),
                  static_cast<unsigned>((peer >> 8) & 0xFF),
                  static_cast<unsigned>(peer & 0xFF));

    this->clients_.emplace_back(std::move(socket), identifier);
    ESP_LOGD(TAG, "New client connected from %s", identifier);
    this->publish_sensor();
}

void StreamServerComponent::cleanup() {
    auto discriminator = [](const Client &client) { return !client.disconnected; };
    auto last_client = std::partition(this->clients_.begin(), this->clients_.end(), discriminator);
    this->clients_.erase(last_client, this->clients_.end());
    this->publish_sensor();
}

void StreamServerComponent::read() {
    int len;
    while ((len = this->stream_->available()) > 0) {
        char buf[128];
        len = std::min(len, 128);
        this->stream_->read_array(reinterpret_cast<uint8_t *>(buf), len);
        for (Client &client : this->clients_) {
            if (client.disconnected)
                continue;

            ssize_t written = client.socket->write(buf, len);
            if (written > 0) {
                // Successfully written.
            } else if (written == 0 || errno == ECONNRESET) {
                ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
                client.disconnected = true;
            } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
                // Expected if the TCP transmit buffer is full.
            } else {
                ESP_LOGW(TAG, "Failed to write to client %s with error %d", client.identifier.c_str(), errno);
            }
        }
    }
}

void StreamServerComponent::write() {
    uint8_t buf[128];
    ssize_t len;

    for (Client &client : this->clients_) {
        if (client.disconnected)
            continue;

        while ((len = client.socket->read(&buf, sizeof(buf))) > 0) {
            this->stream_->write_array(buf, len);
        }

        if (len == 0 || errno == ECONNRESET) {
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Expected if the TCP receive buffer is empty.
        } else {
            ESP_LOGW(TAG, "Failed to read from client %s with error %d", client.identifier.c_str(), errno);
        }
    }
}

StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier)
    : socket(std::move(socket)), identifier{identifier} {}

void StreamServerComponent::pause() {
    if (paused_)
        return;
    paused_ = true;
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
