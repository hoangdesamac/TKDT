#include "cyd_link.h"

CydLink *CydLink::instance_ = nullptr;

bool CydLink::begin(const uint8_t peerMac[6], uint8_t channel,
                    ReceiveHandler receiveHandler) {
  // One link per device; a failed begin can be retried.
  if (instance_ || !peerMac || !receiveHandler || channel < 1 || channel > 14)
    return false;
  memcpy(peerMac_, peerMac, sizeof(peerMac_));
  if (!WiFi.mode(WIFI_STA)) return false;
  WiFi.disconnect();
  if (esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE) != ESP_OK)
    return false;

  rxQueue_ = xQueueCreate(8, sizeof(RxEvent));
  if (!rxQueue_) return false;
  if (esp_now_init() != ESP_OK) {
    vQueueDelete(rxQueue_);
    rxQueue_ = nullptr;
    return false;
  }

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, peerMac_, sizeof(peerMac_));
  peer.channel = channel;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  receiveHandler_ = receiveHandler;
  instance_ = this;
  if (esp_now_add_peer(&peer) != ESP_OK ||
      esp_now_register_send_cb(onSent) != ESP_OK ||
      esp_now_register_recv_cb(onReceived) != ESP_OK) {
    esp_now_deinit();
    instance_ = nullptr;
    receiveHandler_ = nullptr;
    vQueueDelete(rxQueue_);
    rxQueue_ = nullptr;
    return false;
  }
  ready_ = true;
  return true;
}

bool CydLink::send(const cyd::SystemPacket &packet) {
  if (!ready_) return false;
  bool expected = false;
  if (!txPending_.compare_exchange_strong(expected, true)) return false;

  const esp_err_t result = esp_now_send(
      peerMac_, reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
  if (result != ESP_OK) {
    txPending_.store(false);
    sentFailed_.fetch_add(1);
    return false;
  }
  return true;  // Queued only: delivery is counted by onSent().
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void CydLink::onSent(const wifi_tx_info_t *, esp_now_send_status_t status) {
#else
void CydLink::onSent(const uint8_t *, esp_now_send_status_t status) {
#endif
  if (!instance_) return;
  if (status == ESP_NOW_SEND_SUCCESS)
    instance_->sentOk_.fetch_add(1);
  else
    instance_->sentFailed_.fetch_add(1);
  instance_->txPending_.store(false);
}

#if ESP_IDF_VERSION_MAJOR >= 5
void CydLink::onReceived(const esp_now_recv_info_t *info, const uint8_t *data,
                         int length) {
  if (instance_ && info)
    instance_->handleReceived(info->src_addr, data, length,
                              info->rx_ctrl ? info->rx_ctrl->rssi : 0);
}
#else
void CydLink::onReceived(const uint8_t *mac, const uint8_t *data, int length) {
  if (instance_) instance_->handleReceived(mac, data, length, 0);
}
#endif

void CydLink::handleReceived(const uint8_t sourceMac[6], const uint8_t *data,
                             int length, int8_t rssi) {
  // Check bytes before copying a potentially invalid bool into the struct.
  if (!sourceMac || !data || length != sizeof(cyd::SystemPacket) ||
      memcmp(sourceMac, peerMac_, sizeof(peerMac_)) != 0 ||
      data[0] != cyd::PKT_CMD_TOGGLE || data[1] > 1) {
    invalidReceived_.fetch_add(1);
    return;
  }

  RxEvent event{};
  memcpy(&event.packet, data, sizeof(event.packet));
  memcpy(event.mac, sourceMac, sizeof(event.mac));
  event.rssi = rssi;
  if (xQueueSend(rxQueue_, &event, 0) != pdTRUE)
    droppedReceived_.fetch_add(1);
}

void CydLink::poll() {
  if (!ready_) return;
  RxEvent event{};
  // Bound work per iteration so serial and status updates are not starved.
  for (uint8_t i = 0; i < 8 &&
       xQueueReceive(rxQueue_, &event, 0) == pdTRUE; ++i) {
    lastRssi_.store(event.rssi);
    receiveHandler_(event.packet, event.mac);
  }
}
