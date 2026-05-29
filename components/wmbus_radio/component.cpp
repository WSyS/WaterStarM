#include "component.h"

#include "freertos/queue.h"
#include "freertos/task.h"

#define ASSERT(expr, expected, before_exit)                                    \
  {                                                                            \
    auto result = (expr);                                                      \
    if (!!result != expected) {                                                \
      ESP_LOGE(TAG, "Assertion failed: %s -> %d", #expr, result);              \
      before_exit;                                                             \
      return;                                                                  \
    }                                                                          \
  }

#define ASSERT_SETUP(expr) ASSERT(expr, 1, this->mark_failed())

namespace esphome {
namespace wmbus_radio {
static const char *TAG = "wmbus";

void Radio::setup() {
  ASSERT_SETUP(this->packet_queue_ = xQueueCreate(3, sizeof(Packet *)));
  ASSERT_SETUP(this->decode_queue_ = xQueueCreate(3, sizeof(Packet *)));

  // High priority receiver to avoid FIFO overflow.
#if portNUM_PROCESSORS > 1
  ASSERT_SETUP(xTaskCreatePinnedToCore((TaskFunction_t)this->receiver_task, "radio_recv",
                           128 * 1024, this, 24, &(this->receiver_task_handle_), 1));
#else
  ASSERT_SETUP(xTaskCreate((TaskFunction_t)this->receiver_task, "radio_recv",
                           96 * 1024, this, 24, &(this->receiver_task_handle_)));
#endif

  ESP_LOGI(TAG, "Receiver task created [%p]", this->receiver_task_handle_);
  if (this->receiver_task_handle_ != nullptr) {
    auto hw = uxTaskGetStackHighWaterMark(this->receiver_task_handle_);
    ESP_LOGI(TAG, "Receiver task initial stack high-water mark: %lu words",
             (unsigned long)hw);
  }

  // Decoder task (heavy frame conversion + handlers)
#if portNUM_PROCESSORS > 1
  ASSERT_SETUP(xTaskCreatePinnedToCore((TaskFunction_t)this->decode_task, "wmbus_decode",
                           128 * 1024, this, 18, &(this->decode_task_handle_), 1));
#else
  ASSERT_SETUP(xTaskCreate((TaskFunction_t)this->decode_task, "wmbus_decode",
                           64 * 1024, this, 18, &(this->decode_task_handle_)));
#endif

  ESP_LOGI(TAG, "Decoder task created [%p]", this->decode_task_handle_);
  if (this->decode_task_handle_ != nullptr) {
    auto hw = uxTaskGetStackHighWaterMark(this->decode_task_handle_);
    ESP_LOGI(TAG, "Decoder task initial stack high-water mark: %lu words",
             (unsigned long)hw);
  }

  // Only attach IRQ after tasks exist.
  if (this->receiver_task_handle_ == nullptr) {
    ESP_LOGE(TAG, "receiver_task_handle_ is null, not attaching IRQ");
    return;
  }

  ESP_LOGI(TAG, "Attaching CC1101 IRQ to wake receiver task");
  this->radio->attach_data_interrupt(Radio::wakeup_receiver_task_from_isr,
                                     &(this->receiver_task_handle_));
}

void Radio::wakeup_receiver_task_from_isr(TaskHandle_t *arg) {
  if (arg == nullptr || *arg == nullptr)
    return;

  BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(*arg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



void Radio::loop() {
  // Decoding/handler execution is handled exclusively in decode_task().
  // Keep this loop empty to avoid any potential double-ownership between
  // loopTask and the dedicated FreeRTOS tasks.
}



void Radio::wakeup_receiver_task_from_isr(TaskHandle_t *arg) {
  BaseType_t xHigherPriorityTaskWoken;
  vTaskNotifyGiveFromISR(*arg, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void Radio::decode_task(Radio *arg) {
  // Periodic stack watermark logging to identify which task overflows.
  uint32_t last_log_ms = 0;

  while (true) {
    Packet *p = nullptr;
    if (xQueueReceive(arg->decode_queue_, &p, portMAX_DELAY) != pdPASS)
      continue;

    if (p == nullptr)
      continue;

    auto frame = p->convert_to_frame();
    delete p;

    if (!frame)
      continue;

    for (auto &handler : arg->handlers_)
      handler(&frame.value());

    const uint32_t now_ms = millis();
    if (now_ms - last_log_ms > 5000) {
      last_log_ms = now_ms;
      if (arg->decode_task_handle_ != nullptr) {
        auto hw = uxTaskGetStackHighWaterMark(arg->decode_task_handle_);
        ESP_LOGW(TAG,
                 "Decoder stack high-water mark: %lu words; handlers=%zu",
                 (unsigned long)hw, arg->handlers_.size());
      }
    }
  }
}





void Radio::receive_frame() {
  if (!ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000))) {
    this->radio->restart_rx();
    return;
  }

  auto packet = std::make_unique<Packet>();

  if (!this->radio->read_in_task(packet->rx_data_ptr(), packet->rx_capacity(), 0)) {
    this->radio->restart_rx();
    return;
  }

  if (!packet->calculate_payload_size()) {
    this->radio->restart_rx();
    return;
  }

  if (!this->radio->read_in_task(packet->rx_data_ptr(), packet->rx_capacity(), 3)) {
    this->radio->restart_rx();
    return;
  }

  packet->set_rssi(this->radio->get_rssi());

  // Re-arm sync word detector for next packet
  this->radio->restart_rx();

  auto packet_ptr = packet.get();

  if (xQueueSend(this->packet_queue_, &packet_ptr, 0) == pdTRUE) {
    ESP_LOGV(TAG, "Queue items: %zu",
             uxQueueMessagesWaiting(this->packet_queue_));
    ESP_LOGV(TAG, "Queue send success");
    packet.release();
  } else
    ESP_LOGW(TAG, "Queue send failed");
}

void Radio::receiver_task(Radio *arg) {
  // Temporary isolation: disable radio reception to determine whether
  // this component is responsible for the previous-boot stack overflow.
  while (true)
    vTaskDelay(pdMS_TO_TICKS(1000));
}



void Radio::add_frame_handler(std::function<void(Frame *)> &&callback) {
  this->handlers_.push_back(std::move(callback));
}

} // namespace wmbus_radio
} // namespace esphome
