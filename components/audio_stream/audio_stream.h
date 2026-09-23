#pragma once

#include "esphome/core/component.h"
#include "esphome/components/audio_source/audio_source.h"

#ifdef USE_ESP_IDF

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <atomic>
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
//
// The ring holds the samples already packed as 24-bit little-endian bytes,
// which is the WAV payload. Packing happens once, in the capture task, and a
// client that wants the stream as-is (no gain, 24-bit) sends straight out of
// the ring with no conversion and no buffer of its own. Only a client that
// asks for gain, or a 16-bit stream, walks the samples.
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
  // Decimation factor applied before the ring, so every client gets the same
  // rate and the ring holds proportionally more time.
  void set_decimation(uint8_t d) { decim_ = d < 1 ? 1 : d; }
  // Requested output rate; the factor is worked out in setup(), once the
  // source's rate is known, and refused if it does not divide exactly.
  void set_stream_rate(uint32_t r) { want_rate_ = r; }
  uint32_t out_rate() const { return sample_rate_ / decim_; }

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
  // Clients sleep on a task notification that push() gives after every
  // block, rather than polling. Slots are claimed and released by the client
  // tasks under wake_mux_; push() copies the handles out under it and
  // notifies outside it.
  static constexpr uint8_t kMaxClientSlots = 8;
  int claim_wake_slot_();
  void release_wake_slot_(int slot);
  void wake_clients_();
  TaskHandle_t wake_[kMaxClientSlots] = {};
  portMUX_TYPE wake_mux_ = portMUX_INITIALIZER_UNLOCKED;
  // Rate limit for the fell-behind warning; per client, in client_loop().
  static constexpr uint32_t kBehindLogMs = 10000;

  audio_source::AudioSource *source_{nullptr};

  uint16_t port_{8080};
  uint8_t bits_{24};
  uint8_t max_clients_{2};
  uint32_t buffer_ms_{1000};
  uint32_t sample_rate_{48000};
  uint8_t decim_{1};
  uint32_t want_rate_{0};
  // Box-filter accumulator for decimation: sum decim_ samples, emit the mean.
  int64_t decim_acc_{0};
  uint8_t decim_n_{0};

  uint8_t *ring_{nullptr};  // capacity_ samples x 3 bytes, packed 24-bit LE
  uint32_t capacity_{0};    // samples
  // Where push() packs a block before it goes into the ring: internal RAM,
  // sized for one source block, so the capture task does one or two memcpys
  // into PSRAM instead of three byte stores per sample.
  uint8_t *stage_{nullptr};
  uint32_t stage_samples_{0};
  // Count of samples ever written, modulo 2^32. It wraps after about a day
  // at 48 kHz and that is fine: readers only ever compute (write - read) in
  // unsigned arithmetic, which is exact across the wrap. A 64-bit counter was
  // tried and is worse on a 32-bit core: the writer is on one core and the
  // readers on the other, and a reader could see one half updated, which
  // happened once per wrap of the low word, and skipped the client forward.
  //
  // Release/acquire so a reader that sees the new count also sees the samples
  // behind it; the ring is in PSRAM on a dual-core part.
  std::atomic<uint32_t> write_pos_{0};
  std::atomic<uint32_t> clients_{0};
};

}  // namespace audio_stream
}  // namespace esphome

#endif  // USE_ESP_IDF
