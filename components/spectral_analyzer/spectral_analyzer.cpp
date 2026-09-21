#include "spectral_analyzer.h"

#ifdef USE_ESP_IDF

#include "esphome/core/log.h"
#include <esp_heap_caps.h>
#include <esp_memory_utils.h>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace esphome {
namespace spectral_analyzer {

static const char *const TAG = "spectral_analyzer";

static constexpr float kTiny = 1e-20f;

// Large buffers go to PSRAM when present (the ESP32-P4 board has 32 MB), and
// fall back to internal RAM so smaller FFT sizes still work without it.
static void *big_alloc(size_t bytes) {
  void *p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (p == nullptr)
    p = heap_caps_malloc(bytes, MALLOC_CAP_8BIT);
  return p;
}

// The FFT's working buffers, which want internal RAM rather than PSRAM.
//
// re[] and im[] are read and written log2(n) times per frame in a scattered
// butterfly pattern - by far the heaviest memory traffic the node generates,
// and concentrated into a burst every fft_size samples. With them in PSRAM
// that burst was audible in the microphone: a hiss pulsing at exactly the FFT
// frame rate, 41 dB above the envelope floor in the 4-12 kHz band, which
// followed the frame rate when fft_size was halved. No samples were being
// lost - capture ran at 99.93% of real time throughout - so this is coupling,
// not dropout.
//
// Falls back to PSRAM, because a large fft_size will not fit internally and a
// working node with a hiss beats one that will not allocate.
static void *hot_alloc(size_t bytes) {
  void *p = heap_caps_malloc(bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (p != nullptr)
    return p;
  ESP_LOGW(TAG, "%u bytes would not fit in internal RAM; using PSRAM", (unsigned) bytes);
  return heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

// Sub-bin peak position by parabolic fit on the log-magnitude, which is what
// gets 700.00 Hz out of 1.46 Hz bins.
static float interpolate_peak(const float *psd, uint32_t half, uint32_t k) {
  float k_interp = (float) k;
  if (k > 0 && k + 1 < half) {
    const float y1 = 10.0f * log10f(psd[k - 1] + kTiny);
    const float y2 = 10.0f * log10f(psd[k] + kTiny);
    const float y3 = 10.0f * log10f(psd[k + 1] + kTiny);
    const float denom = y1 - 2.0f * y2 + y3;
    if (fabsf(denom) > 1e-6f) {
      const float d = 0.5f * (y1 - y3) / denom;
      if (d > -1.0f && d < 1.0f)
        k_interp += d;
    }
  }
  return k_interp;
}

// ---------------------------------------------------------------- FFT ----

// In-place iterative radix-2 FFT. Deliberately dependency-free; esp-dsp could
// replace it, but at these frame rates the P4 has ample headroom.
static void fft(float *re, float *im, uint32_t n) {
  for (uint32_t i = 1, j = 0; i < n; i++) {
    uint32_t bit = n >> 1;
    for (; j & bit; bit >>= 1)
      j ^= bit;
    j ^= bit;
    if (i < j) {
      std::swap(re[i], re[j]);
      std::swap(im[i], im[j]);
    }
  }
  for (uint32_t len = 2; len <= n; len <<= 1) {
    const float ang = -2.0f * (float) M_PI / (float) len;
    const float wr = cosf(ang), wi = sinf(ang);
    const uint32_t h = len / 2;
    for (uint32_t i = 0; i < n; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (uint32_t k = 0; k < h; k++) {
        const float ur = re[i + k], ui = im[i + k];
        const float vr = re[i + k + h] * cr - im[i + k + h] * ci;
        const float vi = re[i + k + h] * ci + im[i + k + h] * cr;
        re[i + k] = ur + vr;
        im[i + k] = ui + vi;
        re[i + k + h] = ur - vr;
        im[i + k + h] = ui - vi;
        const float nr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = nr;
      }
    }
  }
}

// Median of one-sided bin powers over [lo,hi] Hz, excluding [ex_lo,ex_hi].
static float median_bin_power(const float *psd, uint32_t half, float bin_hz, float lo, float hi,
                              float ex_lo, float ex_hi) {
  std::vector<float> v;
  for (uint32_t k = 1; k < half; k++) {
    const float f = k * bin_hz;
    if (f < lo || f > hi)
      continue;
    if (f >= ex_lo && f <= ex_hi)
      continue;
    v.push_back(psd[k]);
  }
  if (v.empty())
    return kTiny;
  std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end());
  return v[v.size() / 2];
}

// ---------------------------------------------------- SpectrumChannel ----

bool SpectrumChannel::allocate() {
  const uint32_t n = fft_size;
  re = static_cast<float *>(hot_alloc(sizeof(float) * n));
  im = static_cast<float *>(hot_alloc(sizeof(float) * n));
  window = static_cast<float *>(big_alloc(sizeof(float) * n));
  psd_accum = static_cast<float *>(big_alloc(sizeof(float) * half()));
  psd_snapshot = static_cast<float *>(big_alloc(sizeof(float) * half()));
  if (!re || !im || !window || !psd_accum || !psd_snapshot)
    return false;

  ESP_LOGI(TAG, "fft buffers: re %s, im %s; internal heap free %u",
           esp_ptr_internal(re) ? "internal" : "psram",
           esp_ptr_internal(im) ? "internal" : "psram",
           (unsigned) heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

  double sum = 0.0;
  for (uint32_t i = 0; i < n; i++) {
    window[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * i / (float) (n - 1)));
    sum += window[i];
  }
  window_cg = (float) (sum / n);

  for (uint32_t k = 0; k < half(); k++) {
    psd_accum[k] = 0.0f;
    psd_snapshot[k] = 0.0f;
  }

  lock = xSemaphoreCreateMutex();
  return lock != nullptr;
}

void SpectrumChannel::accumulate_frame() {
  const uint32_t n = fft_size;
  // Normalisation: a full-scale sine reads -3.01 dB, the RMS convention.
  const float norm = 2.0f / ((float) n * (float) n * window_cg * window_cg);

  for (uint32_t i = 0; i < n; i++) {
    re[i] *= window[i];
    im[i] = 0.0f;
  }
  fft(re, im, n);

  if (xSemaphoreTake(lock, pdMS_TO_TICKS(50)) != pdTRUE)
    return;
  // im[] is finished with, so it doubles as scratch for this frame's power
  // spectrum: the accumulator holds the average, and the bands need each
  // frame on its own to see whether the peak is holding still.
  for (uint32_t k = 0; k < half(); k++) {
    im[k] = (re[k] * re[k] + im[k] * im[k]) * norm;
    psd_accum[k] += im[k];
  }
  track_frame_peaks(im);
  if (frames < UINT32_MAX)
    frames++;
  xSemaphoreGive(lock);
}

// Peak frequency of this one frame, per band, folded into running sums.
void SpectrumChannel::track_frame_peaks(const float *frame_psd) {
  const float bin_hz = (float) sample_rate / (float) fft_size;
  for (auto &b : bands) {
    if (b.stability == nullptr)
      continue;
    const uint32_t k_lo = std::max<uint32_t>(1, (uint32_t) ceilf(b.f_low / bin_hz));
    const uint32_t k_hi = std::min<uint32_t>(half() - 1, (uint32_t) floorf(b.f_high / bin_hz));
    if (k_lo > k_hi)
      continue;
    uint32_t k_peak = k_lo;
    for (uint32_t k = k_lo; k <= k_hi; k++)
      if (frame_psd[k] > frame_psd[k_peak])
        k_peak = k;
    const double f = (double) interpolate_peak(frame_psd, half(), k_peak) * bin_hz;
    b.peak_sum += f;
    b.peak_sq += f * f;
    b.peak_n++;
  }
}

uint32_t SpectrumChannel::drain() {
  if (xSemaphoreTake(lock, pdMS_TO_TICKS(200)) != pdTRUE)
    return 0;
  const uint32_t f = frames;
  // Spread of the per-frame peaks, then reset for the next interval. Needs two
  // frames for a standard deviation to mean anything.
  for (auto &b : bands) {
    if (b.peak_n >= 2) {
      const double mean = b.peak_sum / (double) b.peak_n;
      const double var = (b.peak_sq - b.peak_sum * mean) / (double) (b.peak_n - 1);
      b.stability_hz = var > 0.0 ? (float) sqrt(var) : 0.0f;
    } else {
      b.stability_hz = NAN;
    }
    b.peak_sum = b.peak_sq = 0.0;
    b.peak_n = 0;
  }
  for (uint32_t k = 0; k < half(); k++) {
    psd_snapshot[k] = f > 0 ? psd_accum[k] / (float) f : 0.0f;
    psd_accum[k] = 0.0f;
  }
  frames = 0;
  xSemaphoreGive(lock);
  return f;
}

// ------------------------------------------------------------ capture ----

void SpectralAnalyzer::setup() {
  if (!this->air_.allocate()) {
    ESP_LOGE(TAG, "out of memory for fft_size %u (%u bytes) - is psram: configured?",
             (unsigned) this->air_.fft_size, (unsigned) this->air_.bytes());
    this->mark_failed();
    return;
  }
  if (this->source_ == nullptr) {
    ESP_LOGE(TAG, "no audio source");
    this->mark_failed();
    return;
  }
  this->air_.sample_rate = this->source_->sample_rate();

  // Warn rather than fail: the node keeps working, it just loses audio in a
  // way that is invisible from outside. The FFT runs in the source's capture
  // task, so the DMA ring has to cover however long it takes.
  const uint32_t fft_ms =
      (uint32_t) (3.5e-6f * this->air_.fft_size * log2f((float) this->air_.fft_size));
  if (this->source_->dma_duration_ms() < 2 * fft_ms)
    ESP_LOGW(TAG,
             "DMA holds %u ms but one FFT takes about %u ms - audio will be lost silently. "
             "Raise dma_buffers on the audio source.",
             (unsigned) this->source_->dma_duration_ms(), (unsigned) fft_ms);

  this->band_defaults_.clear();
  for (auto &b : this->air_.bands)
    this->band_defaults_.push_back({b.f_low, b.f_high});
  this->restore_bands_();

  this->source_->add_consumer([this](const int32_t *s, uint32_t n) { this->on_audio(s, n); });
}

// Called from the source's capture task. Accumulates into the FFT buffer and
// processes a frame whenever it fills.
void SpectralAnalyzer::on_audio(const int32_t *samples, uint32_t count) {
  SpectrumChannel &ch = this->air_;
  for (uint32_t i = 0; i < count; i++) {
    ch.re[this->fill_] = (float) samples[i] / 8388608.0f;  // 2^23
    if (++this->fill_ >= ch.fft_size) {
      this->fill_ = 0;
      ch.accumulate_frame();
    }
  }
}

// ------------------------------------------------------ runtime bands ----

bool SpectralAnalyzer::set_band_range(const std::string &name, float f_low, float f_high) {
  if (f_low >= f_high || f_low < 0.0f) {
    ESP_LOGW(TAG, "ignoring band range %.1f-%.1f Hz", f_low, f_high);
    return false;
  }
  const float nyquist = 0.5f * (float) this->air_.sample_rate;
  if (f_high >= nyquist) {
    ESP_LOGW(TAG, "band '%s' f_high %.1f Hz is at or above Nyquist (%.0f Hz)", name.c_str(),
             f_high, nyquist);
    return false;
  }
  for (auto &b : this->air_.bands) {
    if (b.name != name)
      continue;
    b.f_low = f_low;
    b.f_high = f_high;
    // The peak statistics describe the old range; keeping them would blend two
    // different bands into one stability figure.
    b.peak_sum = b.peak_sq = 0.0;
    b.peak_n = 0;
    b.stability_hz = NAN;
    ESP_LOGI(TAG, "band '%s' retuned to %.1f-%.1f Hz", name.c_str(), f_low, f_high);
    this->save_bands_();
    return true;
  }
  ESP_LOGW(TAG, "no band named '%s'", name.c_str());
  return false;
}

bool SpectralAnalyzer::reset_band(const std::string &name) {
  for (size_t i = 0; i < this->air_.bands.size(); i++)
    if (this->air_.bands[i].name == name)
      return this->set_band_range(name, this->band_defaults_[i].f_low,
                                  this->band_defaults_[i].f_high);
  ESP_LOGW(TAG, "no band named '%s'", name.c_str());
  return false;
}

void SpectralAnalyzer::reset_all_bands() {
  for (size_t i = 0; i < this->air_.bands.size(); i++) {
    this->air_.bands[i].f_low = this->band_defaults_[i].f_low;
    this->air_.bands[i].f_high = this->band_defaults_[i].f_high;
  }
  ESP_LOGI(TAG, "all bands back to their configured ranges");
  this->save_bands_();
}

void SpectralAnalyzer::save_bands_() {
  if (this->air_.bands.size() > MAX_SAVED_BANDS)
    return;  // too many to store; runtime changes simply will not persist
  SavedBands blob{};
  blob.count = (uint8_t) this->air_.bands.size();
  for (uint8_t i = 0; i < blob.count; i++) {
    blob.range[i][0] = this->air_.bands[i].f_low;
    blob.range[i][1] = this->air_.bands[i].f_high;
  }
  this->band_pref_.save(&blob);
}

void SpectralAnalyzer::restore_bands_() {
  this->band_pref_ =
      global_preferences->make_preference<SavedBands>(fnv1_hash("spectral_analyzer_bands"));
  SavedBands blob{};
  if (!this->band_pref_.load(&blob))
    return;
  if (blob.count != this->air_.bands.size()) {
    ESP_LOGW(TAG, "saved band ranges are for a different configuration; ignoring them");
    return;
  }
  uint8_t moved = 0;
  for (uint8_t i = 0; i < blob.count; i++) {
    if (blob.range[i][0] >= blob.range[i][1])
      continue;
    if (blob.range[i][0] != this->air_.bands[i].f_low ||
        blob.range[i][1] != this->air_.bands[i].f_high)
      moved++;
    this->air_.bands[i].f_low = blob.range[i][0];
    this->air_.bands[i].f_high = blob.range[i][1];
  }
  if (moved > 0)
    ESP_LOGI(TAG, "restored %u band range(s) changed at runtime", (unsigned) moved);
}

// ----------------------------------------------------------- publish ----

void SpectralAnalyzer::update() {
  SpectrumChannel &ch = this->air_;
  if (ch.psd_snapshot == nullptr)
    return;

  const uint32_t frames = ch.drain();
  if (frames == 0) {
    // At 48 kHz / 32768 a frame takes 0.68 s; update_interval must exceed that.
    ESP_LOGW(TAG, "no audio frames this interval (check wiring, or lengthen update_interval)");
    return;
  }

  const float *psd = ch.psd_snapshot;
  const uint32_t half = ch.half();
  const float bin_hz = (float) ch.sample_rate / (float) ch.fft_size;

  if (ch.rms != nullptr) {
    float total = 0.0f;
    for (uint32_t k = 1; k < half; k++)
      total += psd[k];
    ch.rms->publish_state(10.0f * log10f(total + kTiny));
  }

  if (ch.floor != nullptr) {
    const float ref_floor = median_bin_power(psd, half, bin_hz, 300.0f, 2000.0f, 0.0f, 0.0f);
    ch.floor->publish_state(10.0f * log10f(ref_floor + kTiny));
  }

  for (auto &b : ch.bands) {
    const uint32_t k_lo = std::max<uint32_t>(1, (uint32_t) ceilf(b.f_low / bin_hz));
    const uint32_t k_hi = std::min<uint32_t>(half - 1, (uint32_t) floorf(b.f_high / bin_hz));
    if (k_lo > k_hi)
      continue;

    float band_power = 0.0f;
    uint32_t k_peak = k_lo;
    for (uint32_t k = k_lo; k <= k_hi; k++) {
      band_power += psd[k];
      if (psd[k] > psd[k_peak])
        k_peak = k;
    }

    if (b.level != nullptr)
      b.level->publish_state(10.0f * log10f(band_power + kTiny));

    if (b.snr != nullptr) {
      // Floor measured beside the band, with a one-bandwidth guard either side.
      const float guard = b.f_high - b.f_low;
      const float floor_bin =
          median_bin_power(psd, half, bin_hz, std::max(20.0f, b.f_low - 6.0f * guard),
                           b.f_high + 6.0f * guard, b.f_low - guard, b.f_high + guard);
      const float expected = floor_bin * (float) (k_hi - k_lo + 1);
      b.snr->publish_state(10.0f * log10f((band_power + kTiny) / (expected + kTiny)));
    }

    if (b.peak != nullptr)
      b.peak->publish_state(interpolate_peak(psd, half, k_peak) * bin_hz);

    if (b.stability != nullptr && !std::isnan(b.stability_hz))
      b.stability->publish_state(b.stability_hz);

    if (b.prominence != nullptr) {
      // Median of the band's own bins: one narrow tone cannot move it, so the
      // peak measures itself against the noise it is sitting in. Falls back to
      // the guarded neighbourhood when the band is too narrow to have a
      // meaningful interior median.
      const float local = (k_hi - k_lo >= 16)
                              ? median_bin_power(psd, half, bin_hz, b.f_low, b.f_high, 0.0f, 0.0f)
                              : median_bin_power(psd, half, bin_hz,
                                                 std::max(20.0f, b.f_low - 3.0f * (b.f_high - b.f_low)),
                                                 b.f_high + 3.0f * (b.f_high - b.f_low), b.f_low,
                                                 b.f_high);
      b.prominence->publish_state(10.0f * log10f((psd[k_peak] + kTiny) / (local + kTiny)));
    }
  }
  ESP_LOGD(TAG, "published from %u frames", (unsigned) frames);

  if (this->scan_interval_s_ > 0) {
    const uint32_t now = millis();
    // Runs on the same averaged spectrum the bands just used, so a scan costs
    // one pass over the bins and no extra capture.
    if (this->last_scan_ms_ == 0 || now - this->last_scan_ms_ >= this->scan_interval_s_ * 1000UL) {
      this->last_scan_ms_ = now;
      this->scan_spectrum_(psd, half, bin_hz);

      ESP_LOGI(TAG, "spectrum scan: %u peaks above %.0f dB prominence (%u frames averaged)",
               (unsigned) this->peaks_.size(), this->scan_min_prom_db_, (unsigned) frames);
      std::string summary;
      for (size_t i = 0; i < this->peaks_.size(); i++) {
        const SpectrumPeak &p = this->peaks_[i];
        const bool steady = p.seen >= this->scan_persist_;
        ESP_LOGI(TAG, "  %2u) %9.2f Hz  %7.1f dB  prominence %5.1f dB  %s(%u)", (unsigned) (i + 1),
                 p.freq_hz, p.level_db, p.prominence_db, steady ? "steady " : "new ",
                 (unsigned) p.seen);
        if (!summary.empty())
          summary += " ";
        char buf[28];
        // "697.1@10*" - the star marks a peak that has survived enough scans.
        snprintf(buf, sizeof(buf), "%.1f@%.0f%s", p.freq_hz, p.prominence_db, steady ? "*" : "");
        summary += buf;
      }
#ifdef USE_TEXT_SENSOR
      if (this->scan_text_ != nullptr)
        this->scan_text_->publish_state(summary);
#endif
    }
  }
}

// ------------------------------------------------------ spectrum scan ----

// Walk the whole spectrum and report the strongest tonal peaks.
//
// "Tonal" is judged by prominence above a LOCAL floor, not by absolute level:
// a 40 dB-down whine in a quiet band is a discovery, a broadband hiss 20 dB
// louder is not. The floor is the median of each block of bins, which one
// narrow tone cannot shift, so the tone measures its own prominence honestly.
void SpectralAnalyzer::scan_spectrum_(const float *psd, uint32_t half, float bin_hz) {
  // ~150 Hz of context at 1.46 Hz bins: wide enough that the median is noise,
  // narrow enough to follow the slope of real outdoor spectra.
  constexpr uint32_t kBlock = 101;
  float block[kBlock];

  const uint32_t k_lo = std::max<uint32_t>(1, (uint32_t) ceilf(this->scan_f_low_ / bin_hz));
  const float f_high = this->scan_f_high_ > 0.0f ? this->scan_f_high_ : bin_hz * (half - 1);
  const uint32_t k_hi = std::min<uint32_t>(half - 2, (uint32_t) floorf(f_high / bin_hz));

  this->peaks_.clear();
  if (k_lo + 2 >= k_hi)
    return;

  for (uint32_t base = k_lo; base < k_hi; base += kBlock) {
    const uint32_t end = std::min(base + kBlock, k_hi);
    const uint32_t n = end - base;
    std::copy(psd + base, psd + end, block);
    std::nth_element(block, block + n / 2, block + n);
    const float floor_pw = block[n / 2];

    for (uint32_t k = std::max(base, k_lo + 1); k < end; k++) {
      // A local maximum, strictly above one neighbour so a flat run of equal
      // bins reports once rather than at every bin.
      if (psd[k] < psd[k - 1] || psd[k] <= psd[k + 1])
        continue;
      const float prom = 10.0f * log10f((psd[k] + kTiny) / (floor_pw + kTiny));
      if (prom < this->scan_min_prom_db_)
        continue;
      SpectrumPeak p;
      p.freq_hz = interpolate_peak(psd, half, k) * bin_hz;
      p.level_db = 10.0f * log10f(psd[k] + kTiny);
      p.prominence_db = prom;
      this->peaks_.push_back(p);
    }
    // A pathological spectrum (or a very low threshold) could otherwise grow
    // this without bound; keep the strongest and carry on.
    if (this->peaks_.size() > 256) {
      std::partial_sort(this->peaks_.begin(), this->peaks_.begin() + 64, this->peaks_.end(),
                        [](const SpectrumPeak &a, const SpectrumPeak &b) {
                          return a.prominence_db > b.prominence_db;
                        });
      this->peaks_.resize(64);
    }
  }

  std::sort(this->peaks_.begin(), this->peaks_.end(),
            [](const SpectrumPeak &a, const SpectrumPeak &b) {
              return a.prominence_db > b.prominence_db;
            });

  // Spectral leakage puts skirts either side of a strong tone; keep only the
  // strongest peak within a few bins so one source reports as one line.
  const float min_sep_hz = 4.0f * bin_hz;
  std::vector<SpectrumPeak> kept;
  kept.reserve(this->scan_top_n_);
  for (const auto &p : this->peaks_) {
    bool clash = false;
    for (const auto &q : kept)
      if (fabsf(p.freq_hz - q.freq_hz) < min_sep_hz) {
        clash = true;
        break;
      }
    if (clash)
      continue;
    kept.push_back(p);
    if (kept.size() >= this->scan_top_n_)
      break;
  }
  this->peaks_.swap(kept);
  this->age_peaks_(bin_hz);
}

// Match this scan's peaks against the last one and carry the counts forward.
// A tolerance of a few bins keeps a drifting source (this one wanders about
// 5 Hz) matched to itself rather than reported as a new find every scan.
void SpectralAnalyzer::age_peaks_(float bin_hz) {
  const float tol_hz = std::max(4.0f * bin_hz, 6.0f);
  for (auto &p : this->peaks_) {
    for (const auto &q : this->previous_peaks_) {
      if (fabsf(p.freq_hz - q.freq_hz) <= tol_hz) {
        // Saturate rather than wrap: a tone running for days stays "steady".
        p.seen = q.seen < 65000 ? q.seen + 1 : q.seen;
        break;
      }
    }
  }
  this->previous_peaks_ = this->peaks_;
}

void SpectralAnalyzer::dump_config() {
  ESP_LOGCONFIG(TAG, "Spectral analyzer:");
  ESP_LOGCONFIG(TAG, "  %u Hz, FFT %u (%.2f Hz bins, %.2f s frames), %u bytes",
                (unsigned) this->air_.sample_rate, (unsigned) this->air_.fft_size,
                (float) this->air_.sample_rate / this->air_.fft_size,
                (float) this->air_.fft_size / this->air_.sample_rate,
                (unsigned) this->air_.bytes());
  for (size_t i = 0; i < this->air_.bands.size(); i++) {
    const Band &b = this->air_.bands[i];
    const bool moved = i < this->band_defaults_.size() &&
                       (b.f_low != this->band_defaults_[i].f_low ||
                        b.f_high != this->band_defaults_[i].f_high);
    ESP_LOGCONFIG(TAG, "    Band '%s': %.0f-%.0f Hz%s", b.name.c_str(), b.f_low, b.f_high,
                  moved ? "  (retuned at runtime)" : "");
  }
  if (this->scan_interval_s_ > 0)
    ESP_LOGCONFIG(TAG,
                  "  Spectrum scan: every %u s, top %u peaks, >= %.0f dB prominence, %.0f-%.0f Hz, "
                  "steady after %u scans",
                  (unsigned) this->scan_interval_s_, (unsigned) this->scan_top_n_,
                  this->scan_min_prom_db_, this->scan_f_low_,
                  this->scan_f_high_ > 0.0f ? this->scan_f_high_
                                            : 0.5f * (float) this->air_.sample_rate,
                  (unsigned) this->scan_persist_);
}

}  // namespace spectral_analyzer
}  // namespace esphome

#endif  // USE_ESP_IDF
