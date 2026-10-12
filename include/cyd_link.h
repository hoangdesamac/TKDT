#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <atomic>
#include <cstring>

#include "cyd_protocol.h"

class CydLink {
 public:
  using ReceiveHandler = void (*)(const cyd::SystemPacket &packet,
                                  const uint8_t sourceMac[6]);

  bool begin(const uint8_t peerMac[6], uint8_t channel,
             ReceiveHandler receiveHandler);
  bool send(const cyd::SystemPacket &packet);
  // Call from loop(): the user handler never runs in the Wi-Fi task.
  void poll();

  bool ready() const { return ready_; }
  uint32_t sentOk() const { return sentOk_.load(); }
  uint32_t sentFailed() const { return sentFailed_.load(); }
  uint32_t invalidReceived() const { return invalidReceived_.load(); }
  uint32_t droppedReceived() const { return droppedReceived_.load(); }
  int8_t lastRssi() const { return lastRssi_.load(); }

 private:
  static CydLink *instance_;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
  static void onSent(const wifi_tx_info_t *info, esp_now_send_status_t status);
#else
  static void onSent(const uint8_t *mac, esp_now_send_status_t status);
#endif
#if ESP_IDF_VERSION_MAJOR >= 5
  static void onReceived(const esp_now_recv_info_t *info, const uint8_t *data,
                         int length);
#else
  static void onReceived(const uint8_t *mac, const uint8_t *data, int length);
#endif
  void handleReceived(const uint8_t sourceMac[6], const uint8_t *data,
                      int length, int8_t rssi);

  struct RxEvent {
    cyd::SystemPacket packet;
    uint8_t mac[6];
    int8_t rssi;
  };
  QueueHandle_t rxQueue_ = nullptr;

  uint8_t peerMac_[6]{};
  std::atomic<uint32_t> sentOk_{0};
  std::atomic<uint32_t> sentFailed_{0};
  std::atomic<uint32_t> invalidReceived_{0};
  std::atomic<uint32_t> droppedReceived_{0};
  std::atomic<int8_t> lastRssi_{0};
  std::atomic<bool> txPending_{false};
  bool ready_ = false;
  ReceiveHandler receiveHandler_ = nullptr;
};
