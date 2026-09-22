#pragma once

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/core/preferences.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/audio_source/audio_source.h"
#include "dsp.h"
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

#ifdef USE_ESP_IDF

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <vector>

namespace esphome {
namespace spectral_analyzer {

// One harmonic of a band's tone, measured at order x the band's peak.
struct Harmonic {
  uint8_t order;
  sensor::Sensor *relative{nullptr};    // dB, power relative to the fundamental
  sensor::Sensor *prominence{nullptr};  // dB over the local floor at n x f0
  sensor::Sensor *frequency{nullptr};   // Hz, where it actually is
};

// One configurable frequency band.
struct Band {
  std::string name;
  float f_low;
  float f_high;
  sensor::Sensor *level{nullptr};  // dB, band power sum
  sensor::Sensor *snr{nullptr};    // dB the whole band exceeds the floor
  sensor::Sensor *peak{nullptr};   // Hz, parabolic-interpolated peak in band
  // dB the single strongest bin stands above the local median floor. For a
  // narrow tone this is the honest detection metric: `snr` sums the band's
  // whole width, so one 1.46 Hz tone in a 110 Hz band is averaged away.
  sensor::Sensor *prominence{nullptr};
  // Hz, standard deviation of the per-frame peak across the interval. A tone
  // holds its frequency while its level fades; noise does not. Measured
  // indoors: 1.96 Hz for this source, against ~30 Hz for the band's width.
  sensor::Sensor *stability{nullptr};

  // Per-frame peak statistics, written by the capture task under the channel
  // lock and drained by update(). Welford would be tidier; sums are enough at
  // these counts and cost less in the audio path.
  double peak_sum{0.0}, peak_sq{0.0};
  uint32_t peak_n{0};
  // Drained result, computed under the lock so update() never reads the sums
  // while the capture task is adding to them. NaN until enough frames.
  float stability_hz{NAN};

  // Harmonics of the band's peak. A motor, transformer or pump has a series;
  // a whistle or an acoustic resonance mostly does not.
  std::vector<Harmonic> harmonics;
  float harmonic_tol_hz{2.0f};   // search half-width per order: +/- n x this
  float harmonic_min_prom{6.0f}; // fundamental must clear this to be measured

  // Zoom FFT over this band's range, for resolution the main FFT cannot
  // afford. Fed by the capture task under zoom_lock; see dsp::ZoomFFT.
  dsp::ZoomFFT *zoom{nullptr};
  uint32_t zoom_fft_size{0};
  sensor::Sensor *zoom_peak{nullptr};
  sensor::Sensor *zoom_prominence{nullptr};
  dsp::ZoomFFT::Result zoom_latest;  // copied out under zoom_mux_
  uint32_t zoom_published{0};  // last Result::seq sent, so a frame publishes once
};

// One peak found by a full-spectrum scan.
struct SpectrumPeak {
  float freq_hz;
  float level_db;       // bin power, same reference as the band levels
  float prominence_db;  // above the local median floor: what makes it a tone
  uint16_t seen{1};     // consecutive scans this peak has appeared in
};

// A spectrum path: buffers, a power accumulator, and the bands read off it.
// Kept as its own type so a second microphone can be added as a second
// channel later - two mics a known distance apart give bearing from the
// cross-spectrum phase.
//
// Sizes are 32-bit throughout: at fft_size 65536 a 16-bit loop counter in the
// FFT would wrap to zero and never terminate.
struct SpectrumChannel {
  uint32_t sample_rate{48000};
  uint32_t fft_size{32768};

  float *re{nullptr}, *im{nullptr}, *window{nullptr};
  float *psd_accum{nullptr};
  float *psd_snapshot{nullptr};  // drained copy, reused every update
  float window_cg{0.5f};         // coherent gain, so levels are calibrated

  SemaphoreHandle_t lock{nullptr};
  uint32_t frames{0};

  sensor::Sensor *rms{nullptr};
  sensor::Sensor *floor{nullptr};
  std::vector<Band> bands;

  uint32_t half() const { return fft_size / 2 + 1; }
  size_t bytes() const { return sizeof(float) * (3u * fft_size + 2u * half()); }
  bool allocate();
  // Window + FFT the samples already in re[], then fold into psd_accum.
  void accumulate_frame();
  // Copy the averaged spectrum into psd_snapshot and reset. Returns frames.
  uint32_t drain();
  // Per-frame band peak tracking, called from accumulate_frame() with the
  // frame's own spectrum still in re[]/im[].
  void track_frame_peaks(const float *frame_psd);
};

class SpectralAnalyzer : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_sample_rate(uint32_t sr) { air_.sample_rate = sr; }
  void set_fft_size(uint32_t n) { air_.fft_size = n; }
  void set_rms_sensor(sensor::Sensor *s) { air_.rms = s; }
  void set_floor_sensor(sensor::Sensor *s) { air_.floor = s; }
  void add_band(const std::string &name, float lo, float hi, sensor::Sensor *level,
                sensor::Sensor *snr, sensor::Sensor *peak, sensor::Sensor *prominence,
                sensor::Sensor *stability) {
    Band b;
    b.name = name;
    b.f_low = lo;
    b.f_high = hi;
    b.level = level;
    b.snr = snr;
    b.peak = peak;
    b.prominence = prominence;
    b.stability = stability;
    air_.bands.push_back(b);
  }
  // These act on the band added last, so codegen calls them straight after
  // add_band().
  void set_harmonic_params(float tol_hz, float min_prom) {
    air_.bands.back().harmonic_tol_hz = tol_hz;
    air_.bands.back().harmonic_min_prom = min_prom;
  }
  void add_harmonic(uint8_t order, sensor::Sensor *relative, sensor::Sensor *prominence,
                    sensor::Sensor *frequency) {
    air_.bands.back().harmonics.push_back({order, relative, prominence, frequency});
  }
  void set_zoom(uint32_t fft_size, sensor::Sensor *peak, sensor::Sensor *prominence) {
    Band &b = air_.bands.back();
    b.zoom_fft_size = fft_size;
    b.zoom_peak = peak;
    b.zoom_prominence = prominence;
  }

  // ----------------------------------------------------- spectrum scan
  // Periodic sweep of the whole spectrum, reporting the strongest tonal
  // peaks. This is the discovery half of the node: the bands above answer
  // "how loud is the thing I already know about", the scan answers "what
  // else is out there".
  void set_scan_interval(uint32_t seconds) { scan_interval_s_ = seconds; }
  void set_scan_top_n(uint8_t n) { scan_top_n_ = n; }
  void set_scan_min_prominence(float db) { scan_min_prom_db_ = db; }
  void set_scan_persistence(uint8_t scans) { scan_persist_ = scans; }
  void set_scan_range(float lo, float hi) {
    scan_f_low_ = lo;
    scan_f_high_ = hi;
  }
#ifdef USE_TEXT_SENSOR
  void set_scan_text_sensor(text_sensor::TextSensor *s) { scan_text_ = s; }
#endif

  // ------------------------------------------------------ runtime bands
  // Retune a band that was declared in YAML, and remember it across reboots.
  //
  // ESPHome builds its entity list at compile time, so a band cannot be
  // *added* at runtime - there would be no sensors to publish it on. What can
  // be done, and is what discovery work actually needs, is to point an
  // existing band somewhere else: declare a few spare bands in YAML, then aim
  // them at whatever the scan turns up, without a reflash.
  //
  // Returns false if no band has that name.
  bool set_band_range(const std::string &name, float f_low, float f_high);
  // Put a band back to the range it was given in YAML.
  bool reset_band(const std::string &name);
  void reset_all_bands();

  // Consumer callback, called from the audio source's capture task.
  void on_audio(const int32_t *samples, uint32_t count);

  void set_source(audio_source::AudioSource *src) { source_ = src; }

 protected:
  // Bands as YAML declared them, so reset_band() has something to go back to
  // and so a saved override can be told apart from a default.
  struct BandDefault {
    float f_low, f_high;
  };
  std::vector<BandDefault> band_defaults_;

  // Saved band ranges. ESPHome's preferences store a fixed-size, trivially
  // copyable blob, so this is an array rather than a vector, and the count is
  // stored with it: a saved blob is only meaningful for the same band list, so
  // if YAML has changed it is dropped rather than applied to the wrong bands.
  static constexpr uint8_t MAX_SAVED_BANDS = 16;
  struct SavedBands {
    uint8_t count;
    float range[MAX_SAVED_BANDS][2];
  };
  ESPPreferenceObject band_pref_;
  void save_bands_();
  void restore_bands_();

  audio_source::AudioSource *source_{nullptr};
  uint32_t fill_{0};  // samples accumulated into re[] so far

  // Zoom FFTs are fed per sample by the capture task and reconfigured by the
  // main loop when a band is retuned; this keeps the two apart. The capture
  // task never waits on it: if it is held, that block is dropped and the zoom
  // restarts rather than splicing across the gap. Results come out separately,
  // through zoom_mux_, so reading them never costs audio.
  SemaphoreHandle_t zoom_lock_{nullptr};
  portMUX_TYPE zoom_mux_ = portMUX_INITIALIZER_UNLOCKED;
  bool has_zoom_{false};
  bool zoom_gap_{false};  // capture task only
  void configure_zoom_(Band &b);
  void apply_range_(Band &b, float f_low, float f_high);
  void publish_harmonics_(const Band &b, const float *psd, uint32_t half, float bin_hz,
                          float f0, float prominence_db);

  // Fills peaks_ from the drained spectrum; returns how many it found.
  void scan_spectrum_(const float *psd, uint32_t half, float bin_hz);

  uint32_t scan_interval_s_{0};  // 0 disables the scan entirely
  uint8_t scan_top_n_{8};
  float scan_min_prom_db_{6.0f};
  float scan_f_low_{20.0f}, scan_f_high_{0.0f};  // 0 means up to Nyquist
  // Carries seen-counts between scans so a source that runs for an hour is
  // distinguishable from a door slam that happened to land in one scan.
  void age_peaks_(float bin_hz);

  uint32_t last_scan_ms_{0};
  uint8_t scan_persist_{2};  // scans a peak must survive before it is "steady"
  std::vector<SpectrumPeak> peaks_;
  std::vector<SpectrumPeak> previous_peaks_;
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *scan_text_{nullptr};
#endif

  SpectrumChannel air_;
};

// ------------------------------------------------------------ actions

template<typename... Ts> class SetBandAction : public Action<Ts...>, public Parented<SpectralAnalyzer> {
 public:
  TEMPLATABLE_VALUE(std::string, band_name)
  TEMPLATABLE_VALUE(float, f_low)
  TEMPLATABLE_VALUE(float, f_high)
  void play(const Ts &...x) override {
    this->parent_->set_band_range(this->band_name_.value(x...), this->f_low_.value(x...),
                                  this->f_high_.value(x...));
  }
};

template<typename... Ts> class ResetBandAction : public Action<Ts...>, public Parented<SpectralAnalyzer> {
 public:
  TEMPLATABLE_VALUE(std::string, band_name)
  void play(const Ts &...x) override { this->parent_->reset_band(this->band_name_.value(x...)); }
};

template<typename... Ts>
class ResetAllBandsAction : public Action<Ts...>, public Parented<SpectralAnalyzer> {
 public:
  void play(const Ts &...x) override { this->parent_->reset_all_bands(); }
};

}  // namespace spectral_analyzer
}  // namespace esphome

#endif  // USE_ESP_IDF
