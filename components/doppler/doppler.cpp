#include "doppler.h"

#ifdef USE_ESP_IDF

#include "esphome/core/log.h"

#include <esp_heap_caps.h>
#include <esp_timer.h>

#include <cinttypes>
#include <cstdio>

namespace esphome {
namespace doppler {

static const char *const TAG = "doppler";

// Ridge storage and fit buffers: PSRAM, where there is plenty.
static void *big_alloc(size_t n) {
  void *p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return p != nullptr ? p : heap_caps_malloc(n, MALLOC_CAP_8BIT);
}

// The decimator and the detector object (about 1.5 kB) are touched every
// 85 ms: internal RAM if that still leaves plenty for the network stack and
// DMA, PSRAM otherwise. Everything else, the 156 kB spectrum included, is
// PSRAM: at boot internal RAM looks free, but the network and stream clients
// claim it later.
static void *fast_alloc(size_t n) {
  if (heap_caps_get_free_size(MALLOC_CAP_INTERNAL) > n + 128 * 1024) {
    void *p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (p != nullptr)
      return p;
  }
  return big_alloc(n);
}

static void task_trampoline(void *arg) { static_cast<Doppler *>(arg)->task_loop(); }

void Doppler::setup() {
  if (this->source_->sample_rate() != dsp::kFsIn) {
    ESP_LOGE(TAG, "source runs at %" PRIu32 " Hz; the detector needs %" PRIu32, this->source_->sample_rate(),
             dsp::kFsIn);
    this->mark_failed();
    return;
  }
  this->det_ = static_cast<dsp::Detector *>(fast_alloc(sizeof(dsp::Detector)));
  this->ring_ = static_cast<int32_t *>(big_alloc(sizeof(int32_t) * kRing));
  if (this->det_ == nullptr || this->ring_ == nullptr) {
    ESP_LOGE(TAG, "out of memory");
    this->mark_failed();
    return;
  }
  new (this->det_) dsp::Detector();
  if (!this->det_->init(this->params_, big_alloc, fast_alloc)) {
    ESP_LOGE(TAG, "out of memory for ridge storage");
    this->mark_failed();
    return;
  }
  this->passes_ = xQueueCreate(8, sizeof(Item));
  // Core 1 with the capture task, below its priority 5: a long fit delays
  // this task, never the capture. Core 0 has the network and the main loop.
  if (this->passes_ == nullptr ||
      xTaskCreatePinnedToCore(task_trampoline, "doppler", 12288, this, 3, &this->task_, 1) != pdPASS) {
    ESP_LOGE(TAG, "could not start the detector task");
    this->mark_failed();
    return;
  }
  this->source_->add_consumer([this](const int32_t *s, uint32_t n) { this->push_(s, n); });
  this->load_since_us_ = esp_timer_get_time();
  ESP_LOGI(TAG, "internal heap free %u after setup", (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

// Capture task. Copies and wakes the detector; nothing else.
void Doppler::push_(const int32_t *s, uint32_t n) {
  const uint32_t w = this->write_pos_.load(std::memory_order_relaxed);
  const uint32_t at = w & (kRing - 1);
  const uint32_t first = std::min(n, kRing - at);
  memcpy(this->ring_ + at, s, sizeof(int32_t) * first);
  if (first < n)
    memcpy(this->ring_, s + first, sizeof(int32_t) * (n - first));
  this->write_pos_.store(w + n, std::memory_order_release);
  if (this->task_ != nullptr)
    xTaskNotifyGive(this->task_);
}

void Doppler::task_loop() {
  static constexpr uint32_t kChunk = 1024;
  float buf[kChunk];
  for (;;) {
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));
    const uint32_t w = this->write_pos_.load(std::memory_order_acquire);
    if (w - this->read_pos_ > kRing - kChunk) {
      // Fell more than the ring behind: skip to half a ring back.
      const uint32_t to = w - kRing / 2;
      this->skipped_.fetch_add(to - this->read_pos_, std::memory_order_relaxed);
      this->read_pos_ = to;
    }
    this->det_->set_sound_speed(this->sound_speed_.load(std::memory_order_relaxed));
    const int64_t t0 = esp_timer_get_time();
    while (this->read_pos_ != w) {
      const uint32_t at = this->read_pos_ & (kRing - 1);
      const uint32_t n = std::min({w - this->read_pos_, kChunk, kRing - at});
      for (uint32_t i = 0; i < n; i++)
        buf[i] = (float) this->ring_[at + i] * (1.0f / 8388608.0f);
      this->det_->feed(buf, n);
      this->read_pos_ += n;
    }
    this->busy_us_.fetch_add((uint32_t) (esp_timer_get_time() - t0), std::memory_order_relaxed);
    Item it;
    while (this->det_->pop(it.pass)) {
      it.ago_s = (float) (this->det_->now() - it.pass.best.t_heard);
      if (xQueueSend(this->passes_, &it, 0) != pdTRUE)
        ESP_LOGW(TAG, "pass queue full; a pass was dropped");
    }
  }
}

void Doppler::loop() {
  if (this->temp_ != nullptr && this->temp_->has_state() && !std::isnan(this->temp_->state)) {
    const float c = this->temp_f_ ? (this->temp_->state - 32.0f) * 5.0f / 9.0f : this->temp_->state;
    if (c > -50.0f && c < 60.0f)
      this->sound_speed_.store(dsp::sound_speed(c), std::memory_order_relaxed);
  }
  Item it;
  while (this->passes_ != nullptr && xQueueReceive(this->passes_, &it, 0) == pdTRUE)
    this->publish_(it.pass, it.ago_s);
}

void Doppler::publish_(const dsp::Pass &p, float ago_s) {
  const dsp::Fit &b = p.best;
  this->count_total_++;
  ESP_LOGI(TAG, "pass %.1f s ago: %.1f m/s, closest %.0f m, f0 %.1f Hz, swing %.1f Hz, res %.2f Hz, %u line%s",
           ago_s, b.speed, b.dist, b.f0, b.swing, b.res, (unsigned) p.lines, p.lines == 1 ? "" : "s");
  if (this->speed_ != nullptr)
    this->speed_->publish_state(b.speed);
  if (this->distance_ != nullptr)
    this->distance_->publish_state(b.dist);
  if (this->frequency_ != nullptr)
    this->frequency_->publish_state(b.f0);
  if (this->swing_ != nullptr)
    this->swing_->publish_state(b.swing);
  if (this->level_ != nullptr)
    this->level_->publish_state(b.level);
  if (this->count_ != nullptr)
    this->count_->publish_state((float) this->count_total_);
  if (this->last_ != nullptr) {
    // Everything needed to redraw the fitted curve: f0, beta, T and when
    // closest approach was heard. Kept under the 255-character state limit.
    char js[256];
    int len = snprintf(js, sizeof(js),
                       "{\"ago\":%.1f,\"v\":%.1f,\"d\":%.0f,\"f0\":%.2f,\"beta\":%.5f,\"T\":%.2f,\"swing\":%.1f,"
                       "\"res\":%.2f,\"level\":%.1f,\"rise\":%.1f,\"lines\":%u,\"f0s\":[",
                       ago_s, b.speed, b.dist, b.f0, b.beta, b.T, b.swing, b.res, b.level, b.rise, (unsigned) p.lines);
    for (uint32_t i = 0; i < std::min<uint32_t>(p.lines, 4) && len > 0 && len < 240; i++)
      len += snprintf(js + len, sizeof(js) - len, "%s%.1f", i ? "," : "", p.f0s[i]);
    if (len > 0 && len < 250)
      snprintf(js + len, sizeof(js) - len, "]}");
    this->last_->publish_state(js);
  }
}

void Doppler::update() {
  const int64_t now = esp_timer_get_time();
  const uint32_t busy = this->busy_us_.exchange(0, std::memory_order_relaxed);
  const int64_t wall = now - this->load_since_us_;
  this->load_since_us_ = now;
  const uint32_t skipped = this->skipped_.exchange(0, std::memory_order_relaxed);
  if (skipped > 0)
    ESP_LOGW(TAG, "detector fell behind and skipped %.1f s of audio", skipped / (float) dsp::kFsIn);
  if (this->load_ != nullptr && wall > 0)
    this->load_->publish_state(100.0f * (float) busy / (float) wall);
  const uint32_t min_free = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  if (min_free < 32 * 1024 && !this->low_heap_warned_) {
    ESP_LOGW(TAG, "internal heap has been down to %u bytes", (unsigned) min_free);
    this->low_heap_warned_ = true;
  }
  if (this->det_ != nullptr && this->updates_++ % 10 == 0) {
    ESP_LOGI(TAG, "%u live ridges, %u fits, %u ridges dropped, %u truncated, %u passes dropped; "
             "internal heap %u free, %u lowest, largest block %u",
             (unsigned) this->det_->live_ridges(), (unsigned) this->det_->fits(),
             (unsigned) this->det_->dropped_ridges(), (unsigned) this->det_->truncated_ridges(),
             (unsigned) this->det_->dropped_passes(), (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned) min_free, (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL));
  }
}

void Doppler::dump_config() {
  ESP_LOGCONFIG(TAG, "Doppler pass detector:");
  ESP_LOGCONFIG(TAG, "  %.0f-%.0f Hz, %u-point FFT at %u Hz (%.2f Hz bins, %.1f ms hops)", this->params_.f_lo,
                this->params_.f_hi, (unsigned) dsp::kNfft, (unsigned) dsp::kFs, dsp::kBinHz, dsp::kDt * 1000.0);
  ESP_LOGCONFIG(TAG, "  peaks >= %.1f dB over the local median; ridges >= %.1f s; fit residual <= %.2f Hz",
                this->params_.peak_db, this->params_.min_track_s, this->params_.max_res_hz);
  ESP_LOGCONFIG(TAG, "  speed of sound %s", this->temp_ != nullptr ? "from the temperature sensor" : "340.3 m/s (15 C)");
}

}  // namespace doppler
}  // namespace esphome

#endif  // USE_ESP_IDF
