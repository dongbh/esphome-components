#include "stream_server.h"

#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/core/version.h"

#include "esphome/components/network/util.h"
#include "esphome/components/socket/socket.h"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <netinet/tcp.h>

namespace esphome {
namespace stream_server {

static const char *TAG = "stream_server";

static void configure_client_socket(int fd) {
    int val = 1;
    if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &val, sizeof(val)) != 0)
        ESP_LOGW(TAG, "setsockopt TCP_NODELAY failed: %s", strerror(errno));

    val = 1;
    if (setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &val, sizeof(val)) != 0) {
        ESP_LOGW(TAG, "setsockopt SO_KEEPALIVE failed: %s", strerror(errno));
        return;
    }

#ifdef TCP_KEEPIDLE
    val = 30;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &val, sizeof(val)) != 0)
        ESP_LOGW(TAG, "setsockopt TCP_KEEPIDLE failed: %s", strerror(errno));
#endif
#ifdef TCP_KEEPINTVL
    val = 10;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &val, sizeof(val)) != 0)
        ESP_LOGW(TAG, "setsockopt TCP_KEEPINTVL failed: %s", strerror(errno));
#endif
#ifdef TCP_KEEPCNT
    val = 3;
    if (setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &val, sizeof(val)) != 0)
        ESP_LOGW(TAG, "setsockopt TCP_KEEPCNT failed: %s", strerror(errno));
#endif
}

void StreamServerComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up stream server...");

    // The make_unique() wrapper doesn't like arrays, so initialize the unique_ptr directly.
    this->buf_ = std::unique_ptr<uint8_t[]>{new uint8_t[this->buf_size_]};

    struct sockaddr_storage bind_addr;
    socklen_t bind_addrlen = socket::set_sockaddr_any(reinterpret_cast<struct sockaddr *>(&bind_addr), sizeof(bind_addr), this->port_);
    this->socket_ = socket::socket_ip(SOCK_STREAM, PF_INET);
    this->socket_->setblocking(false);
    this->socket_->bind(reinterpret_cast<struct sockaddr *>(&bind_addr), bind_addrlen);
    this->socket_->listen(8);

    this->publish_sensor();
}

void StreamServerComponent::loop() {
    if (paused_)
        return;
    this->accept();
    this->read();
//    this->flush();
    this->write();
//    this->cleanup();

}

void StreamServerComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Stream Server:");
    ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_use_address(), this->port_);
#ifdef USE_BINARY_SENSOR
    LOG_BINARY_SENSOR("  ", "Connected:", this->connected_sensor_);
#endif
}

void StreamServerComponent::on_shutdown() {
    if (client_socket)
        client_socket->shutdown(SHUT_RDWR);
}

void StreamServerComponent::publish_sensor() {
#ifdef USE_BINARY_SENSOR
    if (this->connected_sensor_)
        this->connected_sensor_->publish_state(this->client_socket != nullptr);
#endif
}

void StreamServerComponent::accept() {
    std::unique_ptr<esphome::socket::Socket> socket;
    struct sockaddr_storage client_addr;
    socklen_t client_addrlen = sizeof(client_addr);
    socket = this->socket_->accept(reinterpret_cast<struct sockaddr *>(&client_addr), &client_addrlen);
    if (!socket)
        return;
    cleanup();  // Disconnect any existing client

    this->client_socket = std::move(socket);
    this->client_socket->setblocking(false);
    configure_client_socket(this->client_socket->get_fd());
    //identifier = this->client_socket->getpeername();
    char peername[esphome::socket::SOCKADDR_STR_LEN];
    this->client_socket->getpeername_to(peername);
    identifier = peername;
    ESP_LOGD(TAG, "New client connected from %s", identifier.c_str());
    this->publish_sensor();
}

void StreamServerComponent::cleanup() {
    if (client_socket) {
        client_socket->shutdown(SHUT_RDWR);
        client_socket = nullptr;
        identifier = "";
    }
    this->publish_sensor();
}

void StreamServerComponent::read() {
    size_t len = 0;
    ssize_t written;

    while ((len = this->stream_->available()) > 0) {
        len = std::min(len, buf_size_);
        this->stream_->read_array(buf_.get(), len);
        if (client_socket) {
            if ((written = client_socket->write(buf_.get(), len)) > 0) {
                // Successfully written
            } else if (written == 0 || errno == ECONNRESET) {
                ESP_LOGD(TAG, "Client %s disconnected", identifier.c_str());
                cleanup();
            } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
                // Expected if the (TCP) transmit buffer is full, nothing to do.
            } else {
                ESP_LOGE(TAG, "Failed to write to client %s with error %d!", identifier.c_str(), errno);
            }
        }
    }
}

/*
void StreamServerComponent::flush() {
    ssize_t written;
    this->buf_tail_ = this->buf_head_;
    for (Client &client : this->clients_) {
        if (client.disconnected || client.position == this->buf_head_)
            continue;

        // Split the write into two parts: from the current position to the end of the ring buffer, and from the start
        // of the ring buffer until the head. The second part might be zero if no wraparound is necessary.
        struct iovec iov[2];
        iov[0].iov_base = &this->buf_[this->buf_index(client.position)];
        iov[0].iov_len = std::min(this->buf_head_ - client.position, this->buf_ahead(client.position));
        iov[1].iov_base = &this->buf_[0];
        iov[1].iov_len = this->buf_head_ - (client.position + iov[0].iov_len);
        if ((written = client.socket->writev(iov, 2)) > 0) {
            client.position += written;
        } else if (written == 0 || errno == ECONNRESET) {
            ESP_LOGD(TAG, "Client %s disconnected", client.identifier.c_str());
            client.disconnected = true;
            continue;  // don't consider this client when calculating the tail position
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Expected if the (TCP) transmit buffer is full, nothing to do.
        } else {
            ESP_LOGE(TAG, "Failed to write to client %s with error %d!", client.identifier.c_str(), errno);
        }

        this->buf_tail_ = std::min(this->buf_tail_, client.position);
    }
}
*/
void StreamServerComponent::write() {
    ssize_t read;
    if (client_socket) {
        while ((read = client_socket->read(buf_.get(), buf_size_)) > 0) {
            this->stream_->write_array(buf_.get(), read);
        }

        if (read == 0 || errno == ECONNRESET) {
            ESP_LOGD(TAG, "Client %s disconnected", identifier.c_str());
            cleanup();
        } else if (errno == EWOULDBLOCK || errno == EAGAIN) {
            // Expected if the (TCP) receive buffer is empty, nothing to do.
        } else {
            ESP_LOGW(TAG, "Failed to read from client %s with error %d!", identifier.c_str(), errno);
        }

    }
}

//StreamServerComponent::Client::Client(std::unique_ptr<esphome::socket::Socket> socket, std::string identifier, size_t position)
//    : socket(std::move(socket)), identifier{identifier}, position{position} {}

void StreamServerComponent::pause() {
    if (paused_)
        return;
    paused_ = true;
    ESP_LOGI(TAG, "Pausing stream server and disconnecting clients");
    cleanup();
}

void StreamServerComponent::resume() {
    if (!this->paused_)
        return;
    ESP_LOGI(TAG, "Resuming stream server");
    this->paused_ = false;
}


}  // namespace stream_server
}  // namespace esphome
