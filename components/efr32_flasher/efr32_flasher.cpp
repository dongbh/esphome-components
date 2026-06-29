// ====== EFR32 Flasher (external component) ======
#include "esphome/components/efr32_flasher/efr32_flasher.h"
#include "esphome/components/efr32_info/ash_util.h"
#include <vector>
#include <string>
#include <cstring>
#include <algorithm>
#include <strings.h>
#include <cctype>

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/md5/md5.h"

#include <ArduinoJson.h>

#include "esp_http_client.h"
#include "esp_partition.h"
#if __has_include("esp_crt_bundle.h")
#include "esp_crt_bundle.h"
#define EFR32_FLASHER_HAS_CRT_BUNDLE 1
#else
#define EFR32_FLASHER_HAS_CRT_BUNDLE 0
#endif

#ifndef USE_ESP_IDF
#error "EFR32 flasher requires ESP-IDF framework."
#endif

namespace esphome {
namespace efr32_flasher {

static const char* const TAG = "efr32_flasher";

static bool ends_with_ignore_query_(const std::string& s, const char* suffix) {
    size_t end = s.find_first_of("?#");
    if (end == std::string::npos)
        end = s.size();
    size_t n = std::strlen(suffix);
    if (end < n)
        return false;
    return strncasecmp(s.c_str() + end - n, suffix, n) == 0;
}

static bool is_http_redirect_(int status) {
    return status == 301 || status == 302 || status == 303 || status == 307 || status == 308;
}

static std::string resolve_redirect_url_(const std::string& base, const char* location) {
    if (location == nullptr || *location == '\0')
        return "";

    std::string loc(location);
    if (loc.rfind("http://", 0) == 0 || loc.rfind("https://", 0) == 0)
        return loc;

    size_t scheme_end = base.find("://");
    if (scheme_end == std::string::npos)
        return loc;

    std::string scheme = base.substr(0, scheme_end);
    size_t host_start = scheme_end + 3;
    size_t path_start = base.find('/', host_start);
    std::string origin = path_start == std::string::npos ? base : base.substr(0, path_start);

    if (loc.rfind("//", 0) == 0)
        return scheme + ":" + loc;
    if (loc[0] == '/')
        return origin + loc;

    std::string dir = path_start == std::string::npos ? origin + "/" : base.substr(0, base.rfind('/') + 1);
    return dir + loc;
}

static std::string get_redirect_location_(esp_http_client_handle_t client, const std::string& base) {
    const char* names[] = { "Location", "location", "LOCATION" };
    for (const char* name : names) {
        char* loc = nullptr;
        if (esp_http_client_get_header(client, name, &loc) == ESP_OK && loc != nullptr && *loc != '\0')
            return resolve_redirect_url_(base, loc);
    }
    return "";
}

static void replace_all_(std::string& text, const char* from, const char* to) {
    size_t pos = 0;
    size_t from_len = strlen(from);
    while ((pos = text.find(from, pos)) != std::string::npos) {
        text.replace(pos, from_len, to);
        pos += strlen(to);
    }
}

static std::string extract_redirect_href_(const std::string& base, const std::string& body) {
    size_t href = body.find("href=");
    if (href == std::string::npos)
        return "";

    size_t value = href + 5;
    while (value < body.size() && std::isspace(static_cast<unsigned char>(body[value])))
        value++;
    if (value >= body.size())
        return "";

    char quote = body[value];
    size_t start = value;
    size_t end = std::string::npos;
    if (quote == '"' || quote == '\'') {
        start++;
        end = body.find(quote, start);
    } else {
        while (start < body.size() && std::isspace(static_cast<unsigned char>(body[start])))
            start++;
        end = body.find_first_of(" \t\r\n>", start);
    }
    if (end == std::string::npos || end <= start)
        return "";

    std::string loc = body.substr(start, end - start);
    replace_all_(loc, "&amp;", "&");
    return resolve_redirect_url_(base, loc.c_str());
}

static const char* header_or_none_(esp_http_client_handle_t client, const char* name) {
    char* value = nullptr;
    if (esp_http_client_get_header(client, name, &value) == ESP_OK && value != nullptr && *value != '\0')
        return value;
    return "<none>";
}

// Forward declarations for ASH helpers
void write_escaped_(esphome::uart::UARTComponent* uart, const uint8_t* data, size_t len);
bool recv_ash_frame_(esphome::uart::UARTComponent* uart, std::vector<uint8_t>& out, uint32_t timeout_ms);
// Minimal data/ack send helpers (file-static implementations below)
static void ash_send_data_(esphome::uart::UARTComponent* uart, const std::vector<uint8_t>& ezsp_payload, uint8_t frm_num, uint8_t ack_num, bool re_tx);
static void ash_send_ack_(esphome::uart::UARTComponent* uart, uint8_t ack_num, bool ncp_ready);
static void ash_randomize_(uint8_t* data, size_t len);
std::vector<uint8_t> build_ezsp_get_version_();

// XMODEM constants
static constexpr uint8_t SOH = 0x01; // 128B
static constexpr uint8_t STX = 0x02; // 1024B (not used initially)
static constexpr uint8_t EOT = 0x04;
static constexpr uint8_t ACK = 0x06;
static constexpr uint8_t NAK = 0x15;
static constexpr uint8_t CAN = 0x18;
static constexpr uint8_t CCHR = 'C';
static constexpr const char* EFR32_FW_PARTITION_LABEL = "efr32_fw";
static constexpr const char* TEMP_DEFAULT_VARIANT_KEY = "zs3lnone";

EFR32Flasher::~EFR32Flasher() {}

void EFR32Flasher::setup() {
    // Avoid any UART activity at boot to prevent consuming ASH startup frames.
    // If needed, a manual probe action can be invoked later.
    this->apply_runtime_baud_();
}

void EFR32Flasher::set_uart_baud_(uint32_t baud) {
    if (this->uart_ == nullptr)
        return;
    if (baud == 0)
        return;
    if (this->uart_->get_baud_rate() == baud) {
        ESP_LOGV(TAG, "UART baud rate already %u", static_cast<unsigned>(baud));
        return;
    }

    // Ensure any outstanding TX completes before retuning the peripheral.
    this->uart_->flush();

    this->uart_->set_baud_rate(baud);
#if defined(USE_ESP8266) || defined(USE_ESP32)
    this->uart_->load_settings(false);
#endif
    if (this->uart_hw_flow_ != nullptr)
        this->uart_hw_flow_->apply();
    // Give the hardware a moment to settle at the new rate.
    delay_(100);
    uint32_t reported = this->uart_->get_baud_rate();
    ESP_LOGD(TAG, "UART baud rate set to %u (reported=%u)", static_cast<unsigned>(baud),
        static_cast<unsigned>(reported));
}

void EFR32Flasher::apply_runtime_baud_() {
    uint32_t baud = runtime_baud_rate_;
    if (baud == 0 && this->uart_ != nullptr) {
        baud = this->uart_->get_baud_rate();
        runtime_baud_rate_ = baud;
    }
    set_uart_baud_(baud);
}

void EFR32Flasher::apply_bootloader_baud_() {
    set_uart_baud_(bootloader_baud_rate_);
}

bool EFR32Flasher::disable_flow_for_bootloader_() {
    if (this->uart_hw_flow_ == nullptr)
        return false;

    bool restore_flow = this->uart_hw_flow_->is_rtscts_enabled();
    if (restore_flow) {
        ESP_LOGD(TAG, "Disabling UART RTS/CTS for Gecko bootloader");
        if (this->uart_ != nullptr)
            this->uart_->flush();
        this->uart_hw_flow_->set_rtscts_enabled_runtime(false);
        delay_(50);
    }
    return restore_flow;
}

void EFR32Flasher::restore_flow_after_bootloader_(bool restore_flow) {
    if (!restore_flow || this->uart_hw_flow_ == nullptr)
        return;

    ESP_LOGD(TAG, "Restoring UART RTS/CTS for EZSP runtime");
    this->uart_hw_flow_->set_rtscts_enabled_runtime(true);
    delay_(50);
}

bool EFR32Flasher::pause_stream_for_bootloader_() {
    if (this->stream_server_ == nullptr)
        return false;

    ESP_LOGD(TAG, "Pausing stream server for Gecko bootloader");
    this->stream_server_->pause();
    delay_(50);
    return true;
}

void EFR32Flasher::resume_stream_after_bootloader_(bool resume_stream) {
    if (!resume_stream || this->stream_server_ == nullptr)
        return;

    ESP_LOGD(TAG, "Resuming stream server after Gecko bootloader");
    this->stream_server_->resume();
}

void EFR32Flasher::delay_(uint32_t ms) {
    uint32_t start = millis();
    while (millis() - start < ms) {
        App.feed_wdt();
        esphome::delay(1);
    }
}

bool EFR32Flasher::read_byte_(uint8_t& b, uint32_t timeout_ms) {
    uint32_t start = millis();
    while (millis() - start < timeout_ms) {
        if (uart_ && uart_->available()) {
            if (uart_->read_byte(&b))
                return true;
        }
        delay_(1);
    }
    return false;
}

bool EFR32Flasher::await_char_(uint8_t expect, uint32_t timeout_ms) {
    uint8_t b = 0;
    if (!read_byte_(b, timeout_ms))
        return false;
    if (b == expect)
        return true;
    uint32_t end = millis() + timeout_ms;
    while (millis() < end) {
        if (uart_ && uart_->available() && uart_->read_byte(&b)) {
            if (b == expect)
                return true;
        } delay_(1);
    }
    return false;
}

void EFR32Flasher::flush_uart_() {
    uint8_t b;
    for (int i = 0;i < 64;i++) {
        bool any = false;
        while (uart_ && uart_->available()) {
            uart_->read_byte(&b);
            any = true;
            App.feed_wdt();
        }
        if (!any)
            break;
        delay_(2);
    }
}

bool EFR32Flasher::http_open_(const std::string& url, esp_http_client_handle_t& client, int timeout_ms) {
    std::string cur = url;
    for (int redirects = 0; redirects < 5; redirects++) {
        ESP_LOGD(TAG, "HTTP open attempt %d: %s", redirects + 1, cur.c_str());
        esp_http_client_config_t cfg = {};
        cfg.url = cur.c_str();
        cfg.timeout_ms = timeout_ms;
#if EFR32_FLASHER_HAS_CRT_BUNDLE
        cfg.crt_bundle_attach = esp_crt_bundle_attach;
#endif
        cfg.disable_auto_redirect = true; // handle redirects manually for reliability
        cfg.buffer_size = 4096;
        cfg.buffer_size_tx = 1024;
        client = esp_http_client_init(&cfg);
        if (!client) {
            ESP_LOGE(TAG, "HTTP client init failed: %s", cur.c_str());
            return false;
        }
        esp_err_t open_err = esp_http_client_open(client, 0);
        if (open_err != ESP_OK) {
            ESP_LOGE(TAG, "HTTP open failed: %s url=%s", esp_err_to_name(open_err), cur.c_str());
            esp_http_client_cleanup(client);
            return false;
        }

        int64_t header_len = esp_http_client_fetch_headers(client);
        if (header_len < 0)
            ESP_LOGW(TAG, "HTTP fetch headers failed: %lld url=%s", (long long)header_len, cur.c_str());
        int status = esp_http_client_get_status_code(client);
        int64_t content_len = esp_http_client_get_content_length(client);
        ESP_LOGD(TAG, "HTTP response status=%d header_len=%lld content_len=%lld type=%s",
            status, (long long)header_len, (long long)content_len, header_or_none_(client, "Content-Type"));
        if (status == 200) {
            return true;
        }
        if (is_http_redirect_(status)) {
            std::string next = get_redirect_location_(client, cur);
            ESP_LOGW(TAG, "HTTP redirect %d to: %s", status, !next.empty() ? next.c_str() : "<none>");
            if (next.empty()) {
                char first[512];
                int n = esp_http_client_read(client, first, sizeof(first));
                ESP_LOGW(TAG, "HTTP redirect body read returned %d", n);
                if (n > 0) {
                    ESP_LOGW(TAG, "HTTP redirect body first bytes: %.*s", n, first);
                    std::string redirect_body(first, n);
                    next = extract_redirect_href_(cur, redirect_body);
                    ESP_LOGW(TAG, "HTTP redirect href fallback to: %s", !next.empty() ? next.c_str() : "<none>");
                }
            }
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            if (!next.empty()) {
                cur = next;
                continue;
            }
            return false;
        }
        char first[256];
        int n = esp_http_client_read(client, first, sizeof(first));
        ESP_LOGE(TAG, "HTTP status %d, first bytes: %.*s", status, n > 0 ? n : 0, first);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    ESP_LOGE(TAG, "Too many HTTP redirects while fetching: %s", cur.c_str());
    return false;
}

bool EFR32Flasher::fetch_manifest_(const std::string& url, std::string& fw_url_out) {
    if (ends_with_ignore_query_(url, ".gbl")) {
        fw_url_out = url;
        ESP_LOGI(TAG, "Direct GBL URL configured; skipping manifest fetch");
        return true;
    }
    if (ends_with_ignore_query_(url, ".hex") || ends_with_ignore_query_(url, ".bin")) {
        ESP_LOGE(TAG, "HEX/BIN firmware is for CC2652, not EFR32. Use a .gbl file.");
        return false;
    }
    ESP_LOGI(TAG, "fetch_manifest: url=%s", url.c_str());
    std::string body; body.reserve(2048);

    esp_http_client_handle_t client = nullptr;
    if (!http_open_(url, client, 15000)) {
        ESP_LOGE(TAG, "Manifest HTTP open failed");
        return false;
    }

    char buf[512];
    while (true) {
        int n = esp_http_client_read(client, buf, sizeof(buf));
        if (n > 0) {
            body.append(buf, n);
            App.feed_wdt();
            continue;
        }
        if (n == 0)
            break;
        ESP_LOGE(TAG, "Manifest HTTP read failed: %d", n);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    JsonDocument doc;
    auto err = deserializeJson(doc, body);
    ESP_LOGI(TAG, "Manifest fetched:  size=%d, \nContent: %s", body.length(), body.c_str());
    if (err) {
        ESP_LOGE(TAG, "JSON parse error: %s", err.c_str());
        return false;
    }

    fw_url_out.clear();
    expected_md5_.clear();
    expected_size_ = 0;
    manifest_version_.clear();

    // Try variants object: choose based on forced variant or autodetected key
    if (doc["variants"].is<JsonObjectConst>()) {
        auto obj = doc["variants"].as<JsonObjectConst>();
        std::string key = variant_key_override_;
//        if (!variant_key_override_.empty())
//            key = variant_key_override_;
//        else if (variant_force_ == 1)
//            key = "MGM24";
//        else if (variant_force_ == 2)
//            key = "BM24";

        if (!key.empty()) {
            auto v = obj[key.c_str()];
            if (!v.isNull()) {
                if (v["fw_url"].is<const char*>())
                    fw_url_out = v["fw_url"].as<const char*>();
                if (v["md5"].is<const char*>())
                    expected_md5_ = v["md5"].as<const char*>();
                if (v["size"].is<uint32_t>())
                    expected_size_ = v["size"].as<uint32_t>();
                if (v["version"].is<const char*>())
                    manifest_version_ = v["version"].as<const char*>();
                ESP_LOGI(TAG, "Manifest variant '%s' selected", key.c_str());
            } else {
                ESP_LOGW(TAG, "Manifest does not provide variant '%s'", key.c_str());
            }
        }
/*
        if (fw_url_out.empty() && !obj.isNull() && obj.size() > 0) {
            ESP_LOGW(TAG, "Manifest has %u variants but none matched autodetect/override; defaulting to first entry",
                (unsigned)obj.size());
            for (auto kv : obj) {
                auto v = kv.value();
                if (v["fw_url"].is<const char*>())
                    fw_url_out = v["fw_url"].as<const char*>();
                if (v["md5"].is<const char*>())
                    expected_md5_ = v["md5"].as<const char*>();
                if (v["size"].is<uint32_t>())
                    expected_size_ = v["size"].as<uint32_t>();
                ESP_LOGI(TAG, "Using manifest variant '%s' as fallback", kv.key().c_str());
                break;
            }
        }
*/
    }
/*    if (fw_url_out.empty()) {
        if (doc["fw_url"].is<const char*>())
            fw_url_out = doc["fw_url"].as<const char*>();
        else if (doc["firmware_url"].is<const char*>())
            fw_url_out = doc["firmware_url"].as<const char*>();
        else if (doc["url"].is<const char*>())
            fw_url_out = doc["url"].as<const char*>();
    }
    // Optional metadata
    if (expected_md5_.empty() && doc["md5"].is<const char*>())
        expected_md5_ = doc["md5"].as<const char*>();
    if (!expected_size_ && doc["size"].is<uint32_t>())
        expected_size_ = doc["size"].as<uint32_t>();
*/
    if (manifest_version_.empty() && doc["version"].is<const char*>())
        manifest_version_ = doc["version"].as<const char*>();

    if (fw_url_out.empty()) {
        ESP_LOGE(TAG, "Manifest missing firmware URL.");
        return false;
    } else {
        ESP_LOGI(TAG, "Manifest firmware URL=%s", fw_url_out.c_str());
    }
    if (!expected_md5_.empty())
        ESP_LOGI(TAG, "Manifest MD5=%s", expected_md5_.c_str());
    if (expected_size_)
        ESP_LOGI(TAG, "Manifest size=%u", (unsigned)expected_size_);
    if (!manifest_version_.empty())
        ESP_LOGI(TAG, "Manifest version=%s", manifest_version_.c_str());
    return true;
}

void EFR32Flasher::enter_bootloader_() {
    if (!bl_sw_ || !rst_sw_)
        return;
    ESP_LOGI(TAG, "Entering Gecko bootloader (BL+RST)...");
    // Mirror the proven sequence used in your YAML script:
    // 1) Assert boot pin
    // 2) Hold ~1s, then assert reset for ~1s
    // 3) Release reset, keep boot asserted for ~5s
    // 4) Release boot
    bl_sw_->turn_on();           // inverted: drives boot pin active
    delay_(1000);
    rst_sw_->turn_on();          // inverted: drives reset active (low)
    delay_(1000);
    rst_sw_->turn_off();         // release reset
    delay_(5000);
    bl_sw_->turn_off();          // release boot pin
    delay_(200);
    // Clear any spuriously buffered bytes before menu selection
    flush_uart_();
}

void EFR32Flasher::leave_bootloader_() {
    if (!rst_sw_)
        return;
    ESP_LOGD(TAG, "Exiting bootloader via reset...");
    rst_sw_->turn_on();
    delay_(15);
    rst_sw_->turn_off();
}

bool EFR32Flasher::download_firmware_(const std::string& url, uint32_t& content_len_out) {
    content_len_out = 0;
    if (expected_md5_.size() != 32) {
        ESP_LOGE(TAG, "Manifest MD5 is required before flashing EFR32");
        return false;
    }

    const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY, EFR32_FW_PARTITION_LABEL);
    if (partition == nullptr) {
        ESP_LOGE(TAG, "EFR32 firmware cache partition '%s' not found", EFR32_FW_PARTITION_LABEL);
        return false;
    }
    ESP_LOGI(TAG, "EFR32 firmware cache partition '%s': offset=0x%06X size=%u", EFR32_FW_PARTITION_LABEL,
        (unsigned)partition->address, (unsigned)partition->size);

    esp_http_client_handle_t client = nullptr;
    if (!http_open_(url, client)) {
        ESP_LOGE(TAG, "Firmware HTTP open failed");
        return false;
    }

    int64_t len64 = esp_http_client_get_content_length(client);
    uint32_t content_len = 0;
    if (len64 > 0) {
        content_len = (uint32_t)len64;
        if (expected_size_ > 0 && expected_size_ != content_len) {
            ESP_LOGE(TAG, "Firmware size mismatch: manifest=%u http=%u", (unsigned)expected_size_,
                (unsigned)content_len);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return false;
        }
    } else if (expected_size_ > 0) {
        content_len = expected_size_;
        ESP_LOGW(TAG, "Firmware HTTP content length unavailable; using manifest size=%u", (unsigned)content_len);
    } else {
        ESP_LOGE(TAG, "Firmware size is required before flashing EFR32");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    if (content_len == 0 || content_len > partition->size) {
        ESP_LOGE(TAG, "Firmware size %u does not fit cache partition size %u", (unsigned)content_len,
            (unsigned)partition->size);
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    esp_err_t err = esp_partition_erase_range(partition, 0, partition->size);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to erase EFR32 firmware cache: %s", esp_err_to_name(err));
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return false;
    }

    std::vector<uint8_t> buf(4096);
    uint32_t received = 0;
    uint32_t last_pc = 0;
    esphome::md5::MD5Digest md5;
    md5.init();
    while (received < content_len) {
        int r = esp_http_client_read(client, (char*)buf.data(),
            std::min((size_t)buf.size(), (size_t)(content_len - received)));
        if (r < 0) {
            ESP_LOGE(TAG, "Firmware HTTP read failed at %u/%u: %d", (unsigned)received, (unsigned)content_len, r);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return false;
        }
        if (r == 0) {
            ESP_LOGE(TAG, "Firmware HTTP ended early at %u/%u bytes", (unsigned)received, (unsigned)content_len);
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return false;
        }

        err = esp_partition_write(partition, received, buf.data(), r);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to write EFR32 firmware cache at %u: %s", (unsigned)received,
                esp_err_to_name(err));
            esp_http_client_close(client);
            esp_http_client_cleanup(client);
            return false;
        }
        md5.add(buf.data(), r);
        received += r;
        update_progress_(received, content_len, last_pc, 0, 50);
        App.feed_wdt();
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    md5.calculate();
    char md5hex[33];
    md5.get_hex(md5hex);
    md5hex[32] = 0;
    if (strcasecmp(expected_md5_.c_str(), md5hex) != 0) {
        ESP_LOGE(TAG, "Downloaded firmware MD5 mismatch: expected %s, got %s", expected_md5_.c_str(), md5hex);
        return false;
    }

    ESP_LOGI(TAG, "Firmware downloaded and verified. bytes=%u md5=%s", (unsigned)received, md5hex);
    content_len_out = content_len;
    return true;
}

bool EFR32Flasher::xmodem_send_(const esp_partition_t* partition, uint32_t content_len) {
    // XMODEM-CRC 128-byte blocks
    std::vector<uint8_t> block(128, 0x1A);
    uint32_t sent = 0;
    uint8_t seq = 1;
    uint32_t last_pc = 0;
    esphome::md5::MD5Digest md5;
    md5.init();

    ESP_LOGD(TAG, "Waiting for receiver 'C'...");
    uint32_t tstart = millis();
    bool got_c = false;
    uint32_t last_cr = 0;
    bool resent_menu_6s = false;
    bool resent_menu_12s = false;
    uint8_t sample_buf[32];
    size_t sample_len = 0;
    uint32_t last_report = 0;
    while (millis() - tstart < 15000) {
        uint8_t b;
        if (read_byte_(b, 250)) {
            if (b == CCHR) {
                got_c = true;
                break;
            }
            if (sample_len < sizeof(sample_buf)) {
                sample_buf[sample_len++] = b;
            }
            uint32_t now = millis();
            if (now - last_report > 1000) {
                ESP_LOGD(TAG, "Bootloader RX byte 0x%02X while waiting for 'C'", static_cast<unsigned>(b));
                last_report = now;
            }
        }
        uint32_t elapsed = millis() - tstart;
        if (elapsed >= 3000 && (elapsed - last_cr) >= 2000) {
            ESP_LOGD(TAG, "No 'C' yet (~%us), sending CR to prompt bootloader", (unsigned)(elapsed / 1000));
            uart_->write_byte('\r');
            last_cr = elapsed;
        }
        if (!resent_menu_6s && elapsed >= 6000) {
            ESP_LOGD(TAG, "No 'C' yet (~6s), re-sending '1'+CR to select upload");
            uart_->write_byte('1');
            uart_->write_byte('\r');
            resent_menu_6s = true;
        }
        if (!resent_menu_12s && elapsed >= 12000) {
            ESP_LOGD(TAG, "No 'C' yet (~12s), re-sending '1'+CR to select upload");
            uart_->write_byte('1');
            uart_->write_byte('\r');
            resent_menu_12s = true;
        }
    }
    if (!got_c) {
        if (sample_len > 0) {
            char buf[3 * sizeof(sample_buf) + 1];
            size_t pos = 0;
            for (size_t i = 0; i < sample_len && pos + 3 < sizeof(buf); i++) {
                pos += std::snprintf(buf + pos, sizeof(buf) - pos, "%02X", static_cast<unsigned>(sample_buf[i]));
            }
            buf[pos] = '\0';
            ESP_LOGW(TAG, "Bootloader received unexpected bytes while waiting for 'C': %s", buf);
        }
        ESP_LOGE(TAG, "Receiver did not send 'C' to start XMODEM");
        return false;
    }

    while (sent < content_len) {
        const size_t payload_len = std::min<size_t>(128, content_len - sent);
        std::fill(block.begin(), block.end(), 0x1A);

        esp_err_t err = esp_partition_read(partition, sent, block.data(), payload_len);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read cached firmware at %u: %s", (unsigned)sent, esp_err_to_name(err));
            uart_->write_byte(CAN);
            return false;
        }
        App.feed_wdt();

        uint8_t hdr[3] = { SOH, seq, (uint8_t)(0xFF - seq) };
        uint16_t crc = crc16_ccitt_(block.data(), 128);
        uint8_t crc_be[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };

        bool ok = false;
        static constexpr uint8_t MAX_RETRIES = 10;
        for (uint8_t attempt = 1; attempt <= MAX_RETRIES; attempt++) {
            uart_->write_array(hdr, 3);
            uart_->write_array(block.data(), 128);
            uart_->write_array(crc_be, 2);

            uint8_t resp = 0;
            uint32_t start = millis();
            while (millis() - start < 3000) {
                if (read_byte_(resp, 250)) {
                    if (resp == ACK) {
                        ok = true;
                        break;
                    }
                    if (resp == NAK) {
                        ESP_LOGW(TAG, "NAK on block %u attempt %u/%u", (unsigned)seq, (unsigned)attempt,
                            (unsigned)MAX_RETRIES);
                        break;
                    }
                    if (resp == CAN) {
                        ESP_LOGE(TAG, "Receiver cancelled (CAN)");
                        return false;
                    }
                }
            }
            if (ok)
                break;
            if (resp != NAK) {
                ESP_LOGW(TAG, "Timeout on block %u attempt %u/%u", (unsigned)seq, (unsigned)attempt,
                    (unsigned)MAX_RETRIES);
            }
            App.feed_wdt();
        }
        if (!ok) {
            ESP_LOGE(TAG, "Block %u failed after retries; cancelling XMODEM", (unsigned)seq);
            uart_->write_byte(CAN);
            return false;
        }

        md5.add(block.data(), payload_len);
        sent += payload_len;
        seq++;
        update_progress_(sent, content_len, last_pc, 50, 50);
    }

    // Send EOT
    uart_->write_byte(EOT);
    uint8_t resp = 0;
    if (!read_byte_(resp, 2000) || resp != ACK) {
        ESP_LOGE(TAG, "No ACK after EOT");
        return false;
    }
    md5.calculate();
    char md5hex[33];
    md5.get_hex(md5hex);
    md5hex[32] = 0;
    if (!expected_md5_.empty() && expected_md5_.size() == 32 && strcasecmp(expected_md5_.c_str(), md5hex) != 0) {
        ESP_LOGE(TAG, "MD5 mismatch: expected %s, got %s", expected_md5_.c_str(), md5hex);
        return false;
    }
    ESP_LOGI(TAG, "XMODEM transfer complete. bytes=%u md5=%s", (unsigned)sent, md5hex);
    return true;
}

void EFR32Flasher::update_progress_(uint32_t total, uint32_t expected, uint32_t& last_pc, uint8_t base_pc,
    uint8_t range_pc) {
    if (expected > 0) {
        uint32_t pc = base_pc + (uint64_t)total * range_pc / expected;
        if (pc >= last_pc + progress_step_) {
            last_pc = pc - (pc % progress_step_);
            if (show_progress_)
                ESP_LOGI(TAG, "Progress: %u%% (%u/%u)", (unsigned)pc, (unsigned)total, (unsigned)expected);
            if (progress_sensor_ != nullptr)
                progress_sensor_->publish_state(pc);
        }
    } else {
        if (total / 65536 > last_pc) {
            last_pc = total / 65536;
            if (show_progress_)
                ESP_LOGI(TAG, "Progress: %u bytes", (unsigned)total);
        }
    }
}

void EFR32Flasher::run_update_() {
    apply_runtime_baud_();
    ESP_LOGI(TAG, "run_update start variant_force=%u override='%s'", static_cast<unsigned>(variant_force_),
        variant_key_override_.c_str());
    if (!uart_ || !bl_sw_ || !rst_sw_) {
        ESP_LOGE(TAG, "Not configured (uart/switches)");
        set_busy_(false);
        if (progress_sensor_ != nullptr)
            progress_sensor_->publish_state(0);
        return;
    }
    variant_key_override_.clear();
    if (variant_force_ == 0) {
        variant_key_override_ = detect_variant_key_();
        if (variant_key_override_.empty()) {
            variant_key_override_ = TEMP_DEFAULT_VARIANT_KEY;
            ESP_LOGW(TAG, "Auto variant detection did not yield a match; using temporary default '%s'",
                TEMP_DEFAULT_VARIANT_KEY);
        }
    }
    ESP_LOGD(TAG, "Detected override='%s'", variant_key_override_.c_str());
    if (progress_sensor_ != nullptr)
        progress_sensor_->publish_state(0);
    std::string fw_url;
    if (ends_with_ignore_query_(manifest_url_, ".gbl")) {
        expected_md5_.clear();
        expected_size_ = 0;
        fw_url = manifest_url_;
        ESP_LOGI(TAG, "Using direct firmware URL (no manifest): %s", fw_url.c_str());
    } else {
        if (!fetch_manifest_(manifest_url_, fw_url)) {
            ESP_LOGE(TAG, "Manifest fetch/parse failed");
            if (progress_sensor_ != nullptr)
                progress_sensor_->publish_state(0);
            set_busy_(false); return;
        }
        ESP_LOGI(TAG, "Firmware: %s", fw_url.c_str());
    }

    uint32_t content_len = 0;
    if (!download_firmware_(fw_url, content_len)) {
        ESP_LOGE(TAG, "Firmware download/verification failed; not entering bootloader");
        if (progress_sensor_ != nullptr)
            progress_sensor_->publish_state(0);
        set_busy_(false);
        return;
    }

    const esp_partition_t* fw_partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA,
        ESP_PARTITION_SUBTYPE_ANY, EFR32_FW_PARTITION_LABEL);
    if (fw_partition == nullptr) {
        ESP_LOGE(TAG, "EFR32 firmware cache partition '%s' disappeared", EFR32_FW_PARTITION_LABEL);
        if (progress_sensor_ != nullptr)
            progress_sensor_->publish_state(0);
        set_busy_(false);
        return;
    }

    set_busy_(true);
    bool resume_stream = pause_stream_for_bootloader_();
    bool restore_flow = disable_flow_for_bootloader_();
    apply_bootloader_baud_();
    enter_bootloader_();
    ESP_LOGD(TAG, "Bootloader entered, starting upload");
    // Allow BL banner/prompt to appear, then select '1' (upload gbl)
    delay_(1000);
    // Nudge BL prompt
    uart_->write_byte('\r');
    delay_(200);
    flush_uart_();
    ESP_LOGD(TAG, "Selecting 'upload gbl' (sending '1' + CR)");
    uart_->write_byte('1');
    uart_->write_byte('\r');
    delay_(200);

    bool ok = xmodem_send_(fw_partition, content_len);

    leave_bootloader_();
    apply_runtime_baud_();
    restore_flow_after_bootloader_(restore_flow);
    resume_stream_after_bootloader_(resume_stream);
    ESP_LOGI(TAG, "run_update finished ok=%d", ok ? 1 : 0);
    if (ok) {
        if (progress_sensor_ != nullptr)
            progress_sensor_->publish_state(100);
        ESP_LOGI(TAG, "EFR32 update finished OK. Waiting for NCP start marker...");
        if (wait_for_ncp_start_(1500)) {
            ESP_LOGI(TAG, "NCP start marker {~ detected.");
        } else {
            ESP_LOGW(TAG, "No NCP start marker seen in 1.5s (firmware may still boot later).");
        }
        if (!manifest_version_.empty()) {
            if (fw_text_ != nullptr) {
                fw_text_->publish_state(manifest_version_.c_str());
                ESP_LOGI(TAG, "Published installed EFR32 firmware version: %s", manifest_version_.c_str());
            }
            if (latest_text_ != nullptr)
                latest_text_->publish_state(manifest_version_.c_str());
        }
    } else {
        if (progress_sensor_ != nullptr)
            progress_sensor_->publish_state(0);
        ESP_LOGE(TAG, "EFR32 update failed");
    }
    set_busy_(false);
}

void EFR32Flasher::run_check_update_() {
    apply_runtime_baud_();
    ESP_LOGI(TAG, "Checking EFR32 firmware manifest...");
    if (manifest_url_.empty()) {
        ESP_LOGW(TAG, "No manifest URL configured; skipping update check.");
        return;
    }
    if (ends_with_ignore_query_(manifest_url_, ".gbl")) {
        ESP_LOGW(TAG, "Custom firmware URL configured; skipping manifest update check.");
        return;
    }
    if (variant_force_ == 0 && variant_key_override_.empty()) {
        variant_key_override_ = detect_variant_key_();
        if (variant_key_override_.empty()) {
            variant_key_override_ = TEMP_DEFAULT_VARIANT_KEY;
            ESP_LOGW(TAG, "Auto variant detection did not yield a match; using temporary default '%s'",
                TEMP_DEFAULT_VARIANT_KEY);
        }
    }
    ESP_LOGD(TAG, "run_check_update variant override='%s' force=%u", variant_key_override_.c_str(),
        static_cast<unsigned>(variant_force_));
    std::string fw_url;
    if (!fetch_manifest_(manifest_url_, fw_url)) {
        ESP_LOGE(TAG, "Manifest fetch/parse failed"); return;
    }
    if (!manifest_version_.empty() && latest_text_)
        latest_text_->publish_state(manifest_version_.c_str());
}

void EFR32Flasher::start_firmware_update() {
    if (running_ || want_update_ || want_check_) {
        ESP_LOGW(TAG, "EFR32 flasher is busy; ignoring firmware update request");
        return;
    }
    want_update_ = true;
}

void EFR32Flasher::start_check_update() {
    if (running_ || want_update_ || want_check_) {
        ESP_LOGW(TAG, "EFR32 flasher is busy; ignoring update check request");
        return;
    }
    want_check_ = true;
}

void EFR32Flasher::loop() {
    if (want_check_) {
        want_check_ = false;
        running_ = true;
        run_check_update_();
        running_ = false;
        return;
    }
    if (!want_update_)
        return;
    want_update_ = false;
    running_ = true;
    run_update_();
    running_ = false;
}

std::string EFR32Flasher::detect_variant_key_() {
    auto normalize = [](const std::string& input) {
        std::string norm;
        norm.reserve(input.size());
        for (char ch : input) {
            if (ch == ' ' || ch == '-' || ch == '_' || ch == '/' || ch == '\\' || ch == '.')
                continue;
            norm.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(ch))));
        }
        return norm;
    };

    auto match_from_source = [&](const char* label, esphome::text_sensor::TextSensor* sensor) -> std::string {
        if (sensor == nullptr)
            return {};
        std::string value = sensor->get_state();
        if (value.empty())
            return {};
        return value;
/*    
        std::string norm = normalize(value);
        if (norm.empty())
            return {};

        ESP_LOGD(TAG, "Variant probe %s='%s' (norm='%s')", label, value.c_str(), norm.c_str());
        return norm;
*/
        /*
        if (norm.find("mg21none") != std::string::npos || norm.find("MG21NONE") != std::string::npos) {
            ESP_LOGI(TAG, "Variant detected as BM24 via %s='%s'", label, value.c_str());
            return "mg21none";
        }
        if (norm.find("mg21hw") != std::string::npos || norm.find("MG21HW") != std::string::npos) {
            ESP_LOGI(TAG, "Variant detected as MGM24 via %s='%s'", label, value.c_str());
            return "mg21hw";
        }
        return {};
*/
        };

    return match_from_source("board_name", board_name_text_);
/*   

    if (auto key = match_from_source("board_name", board_name_text_); !key.empty())
        return key;
    if (auto key = match_from_source("mfg_string", mfg_string_text_); !key.empty())
        return key;
    if (auto key = match_from_source("manufacturer", manuf_id_text_); !key.empty())
        return key;
    if (auto key = match_from_source("chip", chip_text_); !key.empty())
        return key;

    ESP_LOGW(TAG, "Unable to determine EFR32 variant from probed text sensors (board/mfg/manufacturer)");
    return {};
*/
}

bool EFR32Flasher::wait_for_ncp_start_(uint32_t ms) {
    uint32_t t0 = esphome::millis();
    uint8_t prev = 0;
    while (millis() - t0 < ms) {
        uint8_t b;
        if (uart_ && uart_->available() && uart_->read_byte(&b)) {
            if (prev == '{' && b == '~')
                return true;
            prev = b;
        } else {
            delay_(2);
        }
    }
    return false;
}


// PPP FCS-16 (used by ASH). Polynomial 0x8408, init 0xFFFF, output one's complement, LSB first.
// CRC-HQX (poly 0x1021, init 0xFFFF), big-endian append, as used by bellows' ASH
static uint16_t ash_crc_hqx_be_(const uint8_t* data, size_t len) {
    return esphome::efr32::ash::crc_hqx_be(data, len);
}

void write_escaped_(esphome::uart::UARTComponent* uart, const uint8_t* data, size_t len) {
    esphome::efr32::ash::write_escaped(uart, data, len);
}

// Build and send a generic ASH frame (control+data already prepared)
static void ash_send_frame_(esphome::uart::UARTComponent* uart, const uint8_t* frame, size_t len) {
    if (uart == nullptr)
        return;
    std::vector<uint8_t> tmp(frame, frame + len);
    uint16_t crc = ash_crc_hqx_be_(frame, len);
    tmp.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    tmp.push_back(static_cast<uint8_t>(crc & 0xFF));
    uart->write_byte(esphome::efr32::ash::FLAG);
    write_escaped_(uart, tmp.data(), tmp.size());
    uart->write_byte(esphome::efr32::ash::FLAG);
}

// Pseudo-random data sequence generator (same algorithm as bellows)
static void ash_randomize_(uint8_t* data, size_t len) { esphome::efr32::ash::randomize(data, len); }
// Send a DATA frame with minimal sequencing (frm_num=0, re_tx=0, ack_num=0 by default)
static void ash_send_data_(esphome::uart::UARTComponent* uart, const std::vector<uint8_t>& ezsp_payload, uint8_t frm_num, uint8_t ack_num, bool re_tx) {
    if (uart == nullptr)
        return;
    std::vector<uint8_t> buf;
    uint8_t ctrl = static_cast<uint8_t>(((frm_num & 0x07) << 4) | (re_tx ? 0x08 : 0x00) | (ack_num & 0x07));
    buf.push_back(ctrl);
    std::vector<uint8_t> rnd(ezsp_payload);
    if (!rnd.empty())
        ash_randomize_(rnd.data(), rnd.size());
    buf.insert(buf.end(), rnd.begin(), rnd.end());
    ash_send_frame_(uart, buf.data(), buf.size());
}
// Send an ACK frame (supervisory, not-extended)
static void ash_send_ack_(esphome::uart::UARTComponent* uart, uint8_t ack_num, bool ncp_ready) {
    uint8_t ctrl = static_cast<uint8_t>(0x80 | (ncp_ready ? 0x08 : 0x00) | (ack_num & 0x07));
    ash_send_frame_(uart, &ctrl, 1);
}

} // namespace efr32_flasher
} // namespace esphome
