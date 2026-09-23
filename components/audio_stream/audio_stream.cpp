#include "audio_stream.h"

#ifdef USE_ESP_IDF

#include "esphome/core/log.h"
#include "esphome/components/network/util.h"

#include <esp_heap_caps.h>
#include <lwip/sockets.h>
#include <algorithm>
#include <cstring>
#include <cmath>

namespace esphome {
namespace audio_stream {

static const char *const TAG = "audio_stream";

// Samples handed to a client per write. Large enough that the syscall overhead
// disappears, small enough that a 24-bit conversion buffer stays modest.
static constexpr uint32_t kChunkSamples = 2048;

// How long a client may stall before it is given up on. Long enough to ride
// out a WiFi hiccup, short enough that a laptop that has gone to sleep does
// not hold a slot indefinitely.
static constexpr uint32_t kStallWaitMs = 20;
static constexpr uint8_t kMaxStalls = 250;  // 5 s

struct ClientArg {
  AudioStream *stream;
  int fd;
};

static void accept_trampoline(void *arg) {
  static_cast<AudioStream *>(arg)->accept_loop();
  vTaskDelete(nullptr);
}

static void client_trampoline(void *arg) {
  auto *ca = static_cast<ClientArg *>(arg);
  ca->stream->client_loop(ca->fd);
  delete ca;
  vTaskDelete(nullptr);
}

void AudioStream::setup() {
  if (source_ == nullptr) {
    ESP_LOGE(TAG, "no audio source");
    this->mark_failed();
    return;
  }
  sample_rate_ = source_->sample_rate();
  if (want_rate_ > 0 && want_rate_ != sample_rate_) {
    if (want_rate_ == 0 || sample_rate_ % want_rate_ != 0) {
      ESP_LOGW(TAG, "%u Hz does not divide the source's %u Hz; streaming at full rate",
               (unsigned) want_rate_, (unsigned) sample_rate_);
    } else {
      decim_ = (uint8_t) (sample_rate_ / want_rate_);
      ESP_LOGI(TAG, "streaming at %u Hz (1/%u of the source)", (unsigned) out_rate(),
               (unsigned) decim_);
    }
  }
  if (!this->start()) {
    this->mark_failed();
    return;
  }
  source_->add_consumer([this](const int32_t *s, uint32_t n) { this->push(s, n); });
}

void AudioStream::dump_config() {
  ESP_LOGCONFIG(TAG, "Audio stream:");
  ESP_LOGCONFIG(TAG, "  http://<node>:%u/audio.wav", (unsigned) port_);
  ESP_LOGCONFIG(TAG, "  %u-bit, %u Hz, up to %u clients, %.1f s buffer", (unsigned) bits_,
                (unsigned) out_rate(), (unsigned) max_clients_,
                (float) capacity_ / (float) out_rate());
  ESP_LOGCONFIG(TAG, "  Add ?gain=<dB> to listen; leave it off to measure.");
}

bool AudioStream::start() {
  capacity_ = (uint32_t) ((uint64_t) out_rate() * buffer_ms_ / 1000ULL);
  if (capacity_ < kChunkSamples * 2)
    capacity_ = kChunkSamples * 2;

  const size_t bytes = sizeof(int32_t) * capacity_;
  ring_ = static_cast<int32_t *>(heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (ring_ == nullptr)
    ring_ = static_cast<int32_t *>(heap_caps_malloc(bytes, MALLOC_CAP_8BIT));
  if (ring_ == nullptr) {
    ESP_LOGE(TAG, "ring buffer allocation failed (%u bytes)", (unsigned) bytes);
    return false;
  }
  memset(ring_, 0, bytes);

  // Core 0: the I2S capture and FFT task owns core 1.
  if (xTaskCreatePinnedToCore(accept_trampoline, "audio_accept", 6144, this, 3, nullptr, 0) !=
      pdPASS) {
    ESP_LOGE(TAG, "accept task creation failed");
    return false;
  }
  return true;
}

void AudioStream::push(const int32_t *samples, uint32_t n) {
  if (ring_ == nullptr)
    return;

  // Decimating here rather than per client keeps one copy of the audio and
  // means the ring covers decim_ times as much time for the same memory.
  //
  // The filter is a box average over decim_ samples, which is a crude
  // anti-alias - it has nulls at multiples of the output rate but only about
  // 13 dB of first-sidelobe rejection. That is fine for listening, which is
  // what decimation is for, and wrong for measurement. Analysis should read
  // the undecimated path.
  if (decim_ > 1) {
    for (uint32_t i = 0; i < n; i++) {
      decim_acc_ += samples[i];
      if (++decim_n_ < decim_)
        continue;
      const int32_t v = (int32_t) (decim_acc_ / decim_);
      decim_acc_ = 0;
      decim_n_ = 0;
      const uint32_t pos = write_pos_.load(std::memory_order_relaxed);
      ring_[pos % capacity_] = v;
      write_pos_.store(pos + 1, std::memory_order_release);
    }
    return;
  }
  const uint32_t start = write_pos_.load(std::memory_order_relaxed);
  uint32_t offset = start % capacity_;
  uint32_t remaining = n;
  const int32_t *src = samples;
  while (remaining > 0) {
    const uint32_t run = std::min(remaining, capacity_ - offset);
    memcpy(ring_ + offset, src, sizeof(int32_t) * run);
    src += run;
    remaining -= run;
    offset = (offset + run) % capacity_;
  }
  // Published last, with release ordering: a reader that sees the new count
  // is guaranteed the samples behind it are already in the ring.
  write_pos_.store(start + n, std::memory_order_release);
}

void AudioStream::accept_loop() {
  // The component sets up at DATA priority, ahead of the ethernet component,
  // so the TCP/IP stack is not ready yet: calling socket() here at boot trips
  // an lwIP assertion and aborts, which the OTA rollback then blames on the
  // whole firmware. Wait for a link before touching the socket API.
  while (!network::is_connected())
    vTaskDelay(pdMS_TO_TICKS(250));

  const int listen_fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (listen_fd < 0) {
    ESP_LOGE(TAG, "socket() failed: errno %d", errno);
    return;
  }
  int yes = 1;
  ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(port_);
  if (::bind(listen_fd, (struct sockaddr *) &addr, sizeof(addr)) != 0 ||
      ::listen(listen_fd, 2) != 0) {
    ESP_LOGE(TAG, "bind/listen on port %u failed: errno %d", (unsigned) port_, errno);
    ::close(listen_fd);
    return;
  }
  ESP_LOGI(TAG, "WAV stream on http://<node>:%u/audio.wav", (unsigned) port_);

  while (true) {
    struct sockaddr_in peer = {};
    socklen_t peer_len = sizeof(peer);
    const int fd = ::accept(listen_fd, (struct sockaddr *) &peer, &peer_len);
    if (fd < 0)
      continue;

    // Counted here, by the one task that admits clients, so two arrivals
    // cannot both pass the check; the client task counts itself out.
    if (clients_.load() >= max_clients_) {
      static const char kBusy[] = "HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n";
      ::send(fd, kBusy, sizeof(kBusy) - 1, 0);
      ::close(fd);
      ESP_LOGW(TAG, "refused a client: %u already streaming", (unsigned) clients_.load());
      continue;
    }
    clients_++;

    auto *ca = new ClientArg{this, fd};
    if (xTaskCreatePinnedToCore(client_trampoline, "audio_client", 6144, ca, 3, nullptr, 0) !=
        pdPASS) {
      ESP_LOGE(TAG, "client task creation failed");
      delete ca;
      ::close(fd);
      clients_--;
    }
  }
}

void AudioStream::client_loop(int fd) {
  // Don't let a stalled client pin a task forever, and send promptly: this is
  // a live stream, so latency matters more than packing full segments.
  struct timeval tv = {.tv_sec = 5, .tv_usec = 0};
  ::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  int yes = 1;
  ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

  // Consume the request line. The path itself is ignored - any GET gets the
  // stream - but ?gain=<dB> is honoured.
  //
  // Why this exists: the mic is a measurement device, not a listening one. An
  // ICS-43434 puts 94 dB SPL at -26 dBFS, so ordinary outdoor ambient lands
  // near -73 dBFS and occupies about 14 of the 24 bits. That is correct, and
  // it is also inaudible in every normal player - ffplay, go2rtc, a browser,
  // Frigate's audio classifier all see samples near zero. Analysis wants the
  // untouched samples; listening wants them lifted. A query parameter serves
  // both from one stream rather than forcing a choice in the config.
  char req[256] = {};
  const int req_len = ::recv(fd, req, sizeof(req) - 1, 0);
  if (req_len > 0)
    req[req_len] = '\0';

  float gain = 1.0f;
  if (const char *q = strstr(req, "gain=")) {
    // Clamped at 60 dB: beyond that the quantization of a quiet signal is
    // louder than anything worth hearing.
    float db = strtof(q + 5, nullptr);
    if (db < 0.0f)
      db = 0.0f;
    if (db > 60.0f)
      db = 60.0f;
    gain = powf(10.0f, db / 20.0f);
    ESP_LOGI(TAG, "client wants %.1f dB of gain", db);
  }

  const uint8_t bytes_per_sample = bits_ / 8;
  const uint32_t byte_rate = out_rate() * bytes_per_sample;

  // Streaming WAV: the RIFF and data sizes are unknowable in advance, so use
  // the usual 0xFFFFFFFF sentinel. ffmpeg, sox, VLC and Audacity all accept it.
  uint8_t hdr[44];
  memcpy(hdr, "RIFF", 4);
  const uint32_t kUnknown = 0xFFFFFFFFu;
  memcpy(hdr + 4, &kUnknown, 4);
  memcpy(hdr + 8, "WAVEfmt ", 8);
  const uint32_t fmt_len = 16;
  memcpy(hdr + 16, &fmt_len, 4);
  const uint16_t pcm = 1, channels = 1;
  memcpy(hdr + 20, &pcm, 2);
  memcpy(hdr + 22, &channels, 2);
  const uint32_t hdr_rate = out_rate();
  memcpy(hdr + 24, &hdr_rate, 4);
  memcpy(hdr + 28, &byte_rate, 4);
  const uint16_t block_align = bytes_per_sample;
  const uint16_t bits16 = bits_;
  memcpy(hdr + 32, &block_align, 2);
  memcpy(hdr + 34, &bits16, 2);
  memcpy(hdr + 36, "data", 4);
  memcpy(hdr + 40, &kUnknown, 4);

  char head[256];
  const int head_len = snprintf(head, sizeof(head),
                                "HTTP/1.1 200 OK\r\n"
                                "Content-Type: audio/wav\r\n"
                                "Cache-Control: no-cache, no-store\r\n"
                                "Connection: close\r\n"
                                "\r\n");
  bool ok = ::send(fd, head, head_len, 0) == head_len &&
            ::send(fd, hdr, sizeof(hdr), 0) == (int) sizeof(hdr);

  auto *out = static_cast<uint8_t *>(
      heap_caps_malloc(kChunkSamples * 3, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (out == nullptr)
    out = static_cast<uint8_t *>(heap_caps_malloc(kChunkSamples * 3, MALLOC_CAP_8BIT));
  if (out == nullptr)
    ok = false;

  // Start at the live edge rather than replaying the buffer.
  uint32_t read_pos = write_pos_.load(std::memory_order_acquire);
  uint32_t dropped = 0;
  // The writer keeps going while a chunk is being copied out, so a reader
  // within one chunk of being lapped would copy samples as they are being
  // overwritten. Treat that as already behind.
  const uint32_t safe_depth = capacity_ - kChunkSamples;

  while (ok) {
    const uint32_t head_pos = write_pos_.load(std::memory_order_acquire);
    uint32_t available = head_pos - read_pos;  // exact across the counter's wrap

    if (available > safe_depth) {
      // The client fell behind the writer. Skip to half a buffer back: keep
      // some history so the stream resumes smoothly rather than at the very
      // edge, where it would immediately underrun again.
      dropped += available - capacity_ / 2;
      read_pos = head_pos - capacity_ / 2;
      available = capacity_ / 2;
      ESP_LOGW(TAG, "client behind; dropped %u samples", (unsigned) dropped);
    }
    if (available == 0) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    const uint32_t n = std::min(available, kChunkSamples);
    uint32_t offset = read_pos % capacity_;
    uint32_t written = 0;
    for (uint32_t i = 0; i < n; i++) {
      int32_t s = ring_[offset];
      if (gain != 1.0f) {
        // Saturate rather than wrap: a wrapped sample is a full-scale click,
        // which is both unpleasant and looks like an event to a detector.
        const float amplified = (float) s * gain;
        s = amplified > 8388607.0f    ? 8388607
            : amplified < -8388608.0f ? -8388608
                                      : (int32_t) amplified;
      }
      if (bits_ == 16) {
        const int16_t v = (int16_t) (s >> 8);  // 24-bit payload down to 16
        out[written++] = (uint8_t) (v & 0xFF);
        out[written++] = (uint8_t) ((v >> 8) & 0xFF);
      } else {
        out[written++] = (uint8_t) (s & 0xFF);
        out[written++] = (uint8_t) ((s >> 8) & 0xFF);
        out[written++] = (uint8_t) ((s >> 16) & 0xFF);
      }
      offset = offset + 1 == capacity_ ? 0 : offset + 1;
    }

    uint32_t sent = 0;
    uint8_t stalls = 0;
    while (sent < written) {
      const int got = ::send(fd, out + sent, written - sent, 0);
      if (got > 0) {
        sent += got;
        stalls = 0;
        continue;
      }
      // A send that cannot proceed right now is not a dead client. On WiFi the
      // window closes for a moment all the time, and SO_SNDTIMEO turns that
      // into EAGAIN. Treating it as fatal closed the stream mid-listen - the
      // symptom being a player reporting the stream ended, rather than a gap.
      // Keep the ring filling and try again; the reader catches up afterwards,
      // or falls far enough behind that the drop logic above skips it forward.
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        if (++stalls > kMaxStalls) {
          ESP_LOGW(TAG, "client stalled for %u s; dropping it",
                   (unsigned) (kMaxStalls * kStallWaitMs / 1000));
          ok = false;
          break;
        }
        vTaskDelay(pdMS_TO_TICKS(kStallWaitMs));
        continue;
      }
      ESP_LOGI(TAG, "client send failed: errno %d", errno);
      ok = false;
      break;
    }
    read_pos += n;
  }

  if (out != nullptr)
    free(out);
  ::close(fd);
  clients_--;
  ESP_LOGI(TAG, "client disconnected (%u samples dropped)", (unsigned) dropped);
}

}  // namespace audio_stream
}  // namespace esphome

#endif  // USE_ESP_IDF
