#pragma once

#include "esphome/core/component.h"

#ifdef USE_ESP_IDF

#include <driver/i2s_std.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <functional>
#include <vector>

namespace esphome {
namespace audio_source {

// Owns the I2S peripheral and hands captured audio to everything that wants
// it: a spectrum analyser, a streaming server, anything added later.
//
// Why this is not built on ESPHome's `microphone` abstraction, which would
// otherwise be the natural choice: i2s_audio hardcodes dma_desc_num = 4 and
// dma_frame_num = 256, which is 21 ms of DMA buffering at 48 kHz. A consumer
// that blocks - an FFT takes ~114 ms at 32768 points - overruns that buffer
// and silently loses audio. That cost 14% of real time before it was found.
// Here the DMA depth is configurable and defaults to 341 ms.
//
// Samples are delivered as 24-bit values sign-extended into int32, which is
// what the common I2S MEMS microphones (ICS-43434, INMP441, SPH0645) produce.
class AudioSource : public Component {
 public:
  // Called from the capture task with a block of samples. Consumers must not
  // block for longer than the DMA buffer holds - see dma_duration_ms().
  using Consumer = std::function<void(const int32_t *samples, uint32_t count)>;

  void setup() override;
  void dump_config() override;
  // Ahead of DATA so consumers can register in their own setup().
  float get_setup_priority() const override { return setup_priority::IO; }

  void set_pins(int bclk, int ws, int din) {
    bclk_pin_ = bclk;
    ws_pin_ = ws;
    din_pin_ = din;
  }
  void set_sample_rate(uint32_t sr) { sample_rate_ = sr; }
  void set_use_right_slot(bool r) { use_right_slot_ = r; }
  void set_use_apll(bool a) { use_apll_ = a; }
  void set_dma_buffers(uint32_t n) { dma_buffers_ = n; }
  void set_dma_frame_size(uint32_t n) { dma_frame_size_ = n; }
  void set_block_size(uint32_t n) { block_size_ = n; }

  void add_consumer(Consumer c) { consumers_.push_back(std::move(c)); }

  uint32_t sample_rate() const { return sample_rate_; }
  uint32_t block_size() const { return block_size_; }
  // How long the DMA ring can cover while a consumer is busy. A consumer that
  // takes longer than this loses audio, and the loss is silent.
  uint32_t dma_duration_ms() const {
    return (uint32_t) ((uint64_t) dma_buffers_ * dma_frame_size_ * 1000 / sample_rate_);
  }

  void capture_loop();  // public so the task trampoline can reach it

 protected:
  bool start_i2s_();

  int bclk_pin_{0}, ws_pin_{0}, din_pin_{0};
  uint32_t sample_rate_{48000};
  bool use_right_slot_{false};
  bool use_apll_{true};
  uint32_t dma_buffers_{16};
  uint32_t dma_frame_size_{1023};
  uint32_t block_size_{1024};

  i2s_chan_handle_t rx_chan_{nullptr};
  int32_t *block_{nullptr};
  std::vector<Consumer> consumers_;
};

}  // namespace audio_source
}  // namespace esphome

#endif  // USE_ESP_IDF
