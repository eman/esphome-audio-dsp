#pragma once

#include "esphome/core/component.h"
#include "esphome/components/audio_source/audio_source.h"

#ifdef USE_ESP_IDF

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstdint>

namespace esphome {
namespace audio_stream {

// An endless WAV stream of the microphone, over plain HTTP.
//
// The point is that everything already speaks it: ffmpeg, sox, VLC, Audacity
// and go2rtc can all open the URL directly, so the node stays a dumb source and
// anything downstream does the fan-out, transcoding or recording.
//
// The capture task pushes samples into a ring buffer and never waits on a
// socket. If a client cannot keep up it loses audio, which is the right trade:
// the FFT analysis must not stutter because someone opened a stream on a slow
// link.
class AudioStream : public Component {
 public:
  void setup() override;
  void dump_config() override;
  // After the audio source, and after the network stack is up.
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  void set_source(audio_source::AudioSource *src) { source_ = src; }

  void set_port(uint16_t port) { port_ = port; }
  void set_bits(uint8_t bits) { bits_ = bits; }
  void set_max_clients(uint8_t n) { max_clients_ = n; }
  void set_buffer_ms(uint32_t ms) { buffer_ms_ = ms; }
  void set_sample_rate(uint32_t sr) { sample_rate_ = sr; }

  // Allocates the ring (PSRAM when available) and starts the accept task.
  bool start();
  // Called from the capture task with 24-bit samples sign-extended into int32.
  void push(const int32_t *samples, uint32_t n);

  bool active() const { return ring_ != nullptr; }
  uint16_t port() const { return port_; }
  uint8_t bits() const { return bits_; }
  uint32_t clients() const { return clients_; }
  uint8_t max_clients() const { return max_clients_; }
  uint32_t buffer_samples() const { return capacity_; }

  // Task entry points; public so the trampolines can reach them.
  void accept_loop();
  void client_loop(int fd);

 protected:
  audio_source::AudioSource *source_{nullptr};

  uint16_t port_{8080};
  uint8_t bits_{24};
  uint8_t max_clients_{2};
  uint32_t buffer_ms_{1000};
  uint32_t sample_rate_{48000};

  int32_t *ring_{nullptr};
  uint32_t capacity_{0};  // samples
  // Monotonic count of samples ever written. 64-bit so it never wraps: at
  // 48 kHz a 32-bit counter would fold after about a day and glitch every
  // client at the same moment.
  volatile uint64_t write_pos_{0};
  volatile uint32_t clients_{0};
};

}  // namespace audio_stream
}  // namespace esphome

#endif  // USE_ESP_IDF
