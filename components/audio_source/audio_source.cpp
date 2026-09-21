#include "audio_source.h"

#ifdef USE_ESP_IDF

#include "esphome/core/log.h"

#include <esp_heap_caps.h>

namespace esphome {
namespace audio_source {

static const char *const TAG = "audio_source";

static void capture_trampoline(void *arg) {
  static_cast<AudioSource *>(arg)->capture_loop();
  vTaskDelete(nullptr);
}

void AudioSource::setup() {
  block_ = static_cast<int32_t *>(
      heap_caps_malloc(sizeof(int32_t) * block_size_, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (block_ == nullptr)
    block_ = static_cast<int32_t *>(heap_caps_malloc(sizeof(int32_t) * block_size_, MALLOC_CAP_8BIT));
  if (block_ == nullptr || !this->start_i2s_()) {
    ESP_LOGE(TAG, "setup failed");
    this->mark_failed();
    return;
  }
  // Core 1: the network stack and ESPHome's main loop live on core 0, and
  // capture must not be preempted by them.
  xTaskCreatePinnedToCore(capture_trampoline, "audio_source", 4096, this, 5, nullptr, 1);
}

bool AudioSource::start_i2s_() {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  // The DMA ring is what covers a consumer that blocks. See dma_duration_ms().
  chan_cfg.dma_desc_num = dma_buffers_;
  chan_cfg.dma_frame_num = dma_frame_size_;
  chan_cfg.auto_clear = false;

  esp_err_t err = i2s_new_channel(&chan_cfg, nullptr, &rx_chan_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2s_new_channel failed: %s", esp_err_to_name(err));
    return false;
  }

  // These microphones emit 24 bits MSB-first, left-justified in a 32-bit slot,
  // so clock 32-bit slots and shift the payload down in software.
  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg =
          {
              .mclk = I2S_GPIO_UNUSED,
              .bclk = (gpio_num_t) bclk_pin_,
              .ws = (gpio_num_t) ws_pin_,
              .dout = I2S_GPIO_UNUSED,
              .din = (gpio_num_t) din_pin_,
              .invert_flags = {false, false, false},
          },
  };
  // Must match the mic's L/R strap: L/R to GND -> left slot.
  std_cfg.slot_cfg.slot_mask = use_right_slot_ ? I2S_STD_SLOT_RIGHT : I2S_STD_SLOT_LEFT;

  // Clock from the audio PLL rather than the default.
  //
  // A delta-sigma microphone puts bit-clock jitter straight into its noise
  // floor. The default source is a general-purpose PLL shared with everything
  // else the chip does, so heavy CPU or memory activity modulates it. On one
  // node this was plainly audible: a hiss pulsing at exactly the FFT frame
  // rate, 41 dB above the envelope floor, which followed the frame rate when
  // the FFT size was halved. APLL took it to 15 dB and lowered the noise floor
  // by 2-3 dB. ESPHome's own i2s_audio offers this and defaults it to false;
  // here it defaults to true, because for measurement it matters.
#if SOC_I2S_SUPPORTS_APLL
  if (use_apll_)
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_APLL;
#endif

  err = i2s_channel_init_std_mode(rx_chan_, &std_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2s_channel_init_std_mode failed: %s", esp_err_to_name(err));
    return false;
  }
  err = i2s_channel_enable(rx_chan_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "i2s_channel_enable failed: %s", esp_err_to_name(err));
    return false;
  }
  return true;
}

void AudioSource::capture_loop() {
  const size_t want = sizeof(int32_t) * block_size_;
  while (true) {
    size_t got = 0;
    esp_err_t err = i2s_channel_read(rx_chan_, block_, want, &got, pdMS_TO_TICKS(5000));
    if (err != ESP_OK || got < want)
      continue;
    for (uint32_t i = 0; i < block_size_; i++)
      block_[i] >>= 8;  // 24-bit payload sits in the top bits
    for (auto &c : consumers_)
      c(block_, block_size_);
  }
}

void AudioSource::dump_config() {
  ESP_LOGCONFIG(TAG, "Audio source:");
  ESP_LOGCONFIG(TAG, "  Pins: BCLK %d, WS %d, DIN %d", bclk_pin_, ws_pin_, din_pin_);
  ESP_LOGCONFIG(TAG, "  Sample rate: %u Hz, %s slot", (unsigned) sample_rate_,
                use_right_slot_ ? "right" : "left");
  ESP_LOGCONFIG(TAG, "  Clock source: %s", use_apll_ ? "APLL" : "default PLL");
  ESP_LOGCONFIG(TAG, "  DMA: %u x %u samples = %u ms", (unsigned) dma_buffers_,
                (unsigned) dma_frame_size_, (unsigned) dma_duration_ms());
  ESP_LOGCONFIG(TAG, "  Block size: %u samples", (unsigned) block_size_);
  ESP_LOGCONFIG(TAG, "  Consumers: %u", (unsigned) consumers_.size());
}

}  // namespace audio_source
}  // namespace esphome

#endif  // USE_ESP_IDF
