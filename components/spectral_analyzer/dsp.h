#pragma once

// The signal processing, with no ESPHome or ESP-IDF in it, so it can be built
// and tested on a host against synthetic signals and real captures. See
// tests/test_dsp.cpp.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <vector>

namespace esphome {
namespace spectral_analyzer {
namespace dsp {

static constexpr float kTiny = 1e-20f;

inline float to_db(float power) { return 10.0f * log10f(power + kTiny); }

// In-place iterative radix-2 FFT. Deliberately dependency-free; esp-dsp could
// replace it, but at these frame rates the P4 has ample headroom.
inline void fft(float *re, float *im, uint32_t n) {
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

// Sub-bin peak position by parabolic fit on the log-magnitude, which is what
// gets 700.00 Hz out of 1.46 Hz bins. Returns a fractional bin index.
inline float interpolate_peak(const float *psd, uint32_t n, uint32_t k) {
  float k_interp = (float) k;
  if (k > 0 && k + 1 < n) {
    const float y1 = to_db(psd[k - 1]);
    const float y2 = to_db(psd[k]);
    const float y3 = to_db(psd[k + 1]);
    const float denom = y1 - 2.0f * y2 + y3;
    if (fabsf(denom) > 1e-6f) {
      const float d = 0.5f * (y1 - y3) / denom;
      if (d > -1.0f && d < 1.0f)
        k_interp += d;
    }
  }
  return k_interp;
}

// Median of one-sided bin powers over [lo,hi] Hz, excluding [ex_lo,ex_hi].
inline float median_bin_power(const float *psd, uint32_t half, float bin_hz, float lo, float hi,
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

// ------------------------------------------------------ robust spread ----

// Median of a small sample. Sorts in place; the caller owns the scratch.
inline float median_inplace(float *v, uint32_t n) {
  if (n == 0)
    return NAN;
  std::nth_element(v, v + n / 2, v + n);
  float m = v[n / 2];
  if ((n & 1u) == 0) {
    // Even count: average the two middle values, the lower of which is the
    // largest element of the first half.
    std::nth_element(v, v + n / 2 - 1, v + n / 2);
    m = 0.5f * (m + v[n / 2 - 1]);
  }
  return m;
}

// Spread of per-frame values that is not destroyed by one bad frame.
//
// The standard deviation was the first choice and fails exactly where it is
// needed: a marginal tone loses its band peak to noise in one frame out of
// seven, that one frame lands 40 Hz away, and the standard deviation reads
// ~15 Hz for a tone that held still in the other six. The median absolute
// deviation ignores that frame. It is scaled by 1.4826 so that for ordinary
// Gaussian scatter it reads the same as the standard deviation would, and the
// thresholds already tuned against it still apply.
struct Spread {
  float median{NAN};
  float spread{NAN};     // 1.4826 x MAD, in the values' own unit
  float agreement{NAN};  // fraction of values within +/- tol of the median
};

// `scratch` must hold n floats; the input is not modified.
inline Spread robust_spread(const float *values, uint32_t n, float tol, float *scratch) {
  Spread r;
  if (n < 2)
    return r;
  std::copy(values, values + n, scratch);
  r.median = median_inplace(scratch, n);
  uint32_t within = 0;
  for (uint32_t i = 0; i < n; i++) {
    scratch[i] = fabsf(values[i] - r.median);
    if (scratch[i] <= tol)
      within++;
  }
  r.spread = 1.4826f * median_inplace(scratch, n);
  r.agreement = (float) within / (float) n;
  return r;
}

// ----------------------------------------------------------- harmonics ----

// A tone near `f_center`, measured against the noise around it.
struct TonalMeasure {
  bool valid{false};
  float freq_hz{NAN};
  float prominence_db{NAN};  // strongest bin in the window over the local median
  // Power above the local floor, summed across the window. Summed rather than
  // read off the peak bin because a drifting fundamental smears its nth
  // harmonic n times wider: the peak bin of H4 would read low for a reason
  // that has nothing to do with the source.
  float excess_power{0.0f};
};

// Measure whatever tone sits within +/-tol_hz of f_center. The floor is the
// median of a neighborhood several windows wide, excluding the window itself.
inline TonalMeasure measure_tone(const float *psd, uint32_t half, float bin_hz, float f_center,
                                 float tol_hz) {
  TonalMeasure m;
  tol_hz = std::max(tol_hz, 3.0f * bin_hz);
  const float nyquist = bin_hz * (half - 1);
  if (f_center - tol_hz < bin_hz || f_center + tol_hz >= nyquist)
    return m;
  const uint32_t k_lo = (uint32_t) ceilf((f_center - tol_hz) / bin_hz);
  const uint32_t k_hi = std::min<uint32_t>(half - 2, (uint32_t) floorf((f_center + tol_hz) / bin_hz));
  if (k_lo > k_hi)
    return m;

  const float ctx = std::max(8.0f * tol_hz, 40.0f * bin_hz);
  const float floor_pw = median_bin_power(psd, half, bin_hz, std::max(bin_hz, f_center - ctx),
                                          f_center + ctx, f_center - tol_hz, f_center + tol_hz);
  uint32_t k_peak = k_lo;
  float excess = 0.0f;
  for (uint32_t k = k_lo; k <= k_hi; k++) {
    if (psd[k] > psd[k_peak])
      k_peak = k;
    if (psd[k] > floor_pw)
      excess += psd[k] - floor_pw;
  }
  m.valid = true;
  m.freq_hz = interpolate_peak(psd, half, k_peak) * bin_hz;
  m.prominence_db = to_db(psd[k_peak]) - to_db(floor_pw);
  m.excess_power = excess;
  return m;
}

// ------------------------------------------------------------ zoom FFT ----

// A high-resolution spectrum of one narrow band, at a fraction of the cost of
// a longer full-spectrum FFT.
//
// Frequency resolution is 1 / (frame duration), whatever the method. A zoom
// FFT buys it cheaply instead of with memory: mix the band down to 0 Hz,
// low-pass and decimate by D, then run a small complex FFT on the slow
// signal. A 2048-point zoom over a 110 Hz band gives 0.077 Hz bins from about
// 70 kB, where the full-spectrum FFT would need 2^20 points and 12 MB.
//
// The price is time: those 0.077 Hz bins take a 13 s frame, and a tone that
// wanders more than a bin inside one frame smears across several. Frames
// overlap by half, so a result arrives every ~6.5 s.
class ZoomFFT {
 public:
  struct Result {
    uint32_t seq{0};  // increments with every frame; 0 means none yet
    float peak_hz{NAN};
    float prominence_db{NAN};  // peak bin over the median of the band's bins
    float level_db{NAN};       // peak bin, RMS convention like the main FFT
    // The second strongest separate line in the band, at least four bins
    // from the first. Two machines running almost alike put two lines here;
    // one machine puts noise here, at a random frequency every frame.
    float second_hz{NAN};
    float second_prominence_db{NAN};
    // Periodic modulation of the band's envelope, from the spectrum of the
    // decimated baseband's power. Two tones df apart beat at exactly df, so
    // this and (second_hz - peak_hz) agreeing is the beating test. A single
    // tone with noise reads a random frequency at a few dB prominence.
    float modulation_hz{NAN};
    float modulation_prominence_db{NAN};  // over the median of the search range
    float modulation_depth{NAN};          // 0..1, sinusoidal power modulation index
  };

  // Where to look for envelope modulation, in Hz. The frame length limits
  // the resolution: 1024 points over a 110 Hz band give 0.15 Hz bins.
  void set_modulation_range(float lo, float hi) {
    mod_lo_ = lo;
    mod_hi_ = hi;
  }
  float modulation_bin_hz() const { return bin_hz(); }  // same frame, same bins

  // Decimation that puts the band in the middle 70% of the decimated rate.
  // The remaining 30% is the anti-alias filter's transition band. Shared with
  // the config validator, which has to agree on the frame length.
  static uint32_t decimation_for(float sample_rate, float bandwidth) {
    const float d = sample_rate * 0.7f / std::max(bandwidth, 0.1f);
    return std::max<uint32_t>(1, (uint32_t) floorf(d));
  }

  bool configure(float sample_rate, float f_low, float f_high, uint32_t fft_size) {
    fs_ = sample_rate;
    f_low_ = f_low;
    f_high_ = f_high;
    fc_ = 0.5f * (f_low + f_high);
    m_ = fft_size;
    d_ = decimation_for(sample_rate, f_high - f_low);
    const float fs_d = fs_ / (float) d_;

    // Windowed-sinc low-pass, Blackman window (74 dB stopband). Anything that
    // would alias into the band starts at fs_d - bw/2 and the band ends at
    // bw/2, so the filter must fall from pass to stop across fs_d - bw =
    // 0.3 fs_d. Blackman's transition is ~5.5 fs / taps, hence ~18 D taps.
    uint32_t taps = (uint32_t) ceilf(5.5f * (float) d_ / 0.3f);
    taps |= 1u;  // odd, so the filter has a center tap and linear phase
    h_.assign(taps, 0.0f);
    const double cutoff = 0.5 * fs_d / fs_;  // cycles per input sample
    const double mid = 0.5 * (taps - 1);
    double sum = 0.0;
    for (uint32_t i = 0; i < taps; i++) {
      const double t = i - mid;
      const double sinc = t == 0.0 ? 2.0 * cutoff : sin(2.0 * M_PI * cutoff * t) / (M_PI * t);
      const double w = 0.42 - 0.5 * cos(2.0 * M_PI * i / (taps - 1)) +
                       0.08 * cos(4.0 * M_PI * i / (taps - 1));
      h_[i] = (float) (sinc * w);
      sum += h_[i];
    }
    for (auto &v : h_)
      v = (float) (v / sum);  // unity gain at the band center

    // One accumulator per output whose window covers the current input. Each
    // input is added into all of them as it arrives, so no input history is
    // kept at all: the filter's memory is these few sums, not taps samples.
    slots_ = (taps + d_ - 1) / d_ + 1;
    acc_re_.assign(slots_, 0.0f);
    acc_im_.assign(slots_, 0.0f);
    acc_tap_.assign(slots_, 0u);
    acc_live_.assign(slots_, 0u);

    ring_re_.assign(m_, 0.0f);
    ring_im_.assign(m_, 0.0f);
    re_.assign(m_, 0.0f);
    im_.assign(m_, 0.0f);
    psd_.assign(m_, 0.0f);
    window_.assign(m_, 0.0f);
    double cg = 0.0;
    for (uint32_t i = 0; i < m_; i++) {
      window_[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * i / (float) (m_ - 1)));
      cg += window_[i];
    }
    window_cg_ = (float) (cg / m_);

    phase_step_ = 2.0 * M_PI * (double) fc_ / (double) fs_;
    step_re_ = (float) cos(phase_step_);
    step_im_ = (float) sin(phase_step_);
    reset();
    return true;
  }

  // Drop all state, keeping the configuration. After a gap in the input the
  // filter and the frame would otherwise splice two different moments.
  void reset() {
    std::fill(acc_live_.begin(), acc_live_.end(), 0u);
    phase_ = 0.0;
    osc_count_ = 0;
    rot_re_ = 1.0f;
    rot_im_ = 0.0f;
    since_start_ = 0;
    next_slot_ = 0;
    ring_pos_ = 0;
    ring_fill_ = 0;
    since_frame_ = 0;
  }

  // One input sample, full scale 1.0. Returns true when it completed a frame.
  bool push(float x) {
    // Mix down: multiply by e^(-j 2 pi fc t), so the band center lands at 0 Hz.
    const float xr = x * rot_re_, xi = -x * rot_im_;
    advance_oscillator_();

    if (since_start_ == 0) {
      const uint32_t s = next_slot_;
      next_slot_ = (next_slot_ + 1) % slots_;
      acc_re_[s] = acc_im_[s] = 0.0f;
      acc_tap_[s] = 0;
      acc_live_[s] = 1;
    }
    if (++since_start_ >= d_)
      since_start_ = 0;

    bool frame = false;
    const uint32_t taps = (uint32_t) h_.size();
    for (uint32_t s = 0; s < slots_; s++) {
      if (!acc_live_[s])
        continue;
      const float h = h_[acc_tap_[s]];
      acc_re_[s] += h * xr;
      acc_im_[s] += h * xi;
      if (++acc_tap_[s] == taps) {
        acc_live_[s] = 0;
        frame |= emit_(acc_re_[s], acc_im_[s]);
      }
    }
    return frame;
  }

  const Result &result() const { return result_; }
  uint32_t decimation() const { return d_; }
  uint32_t taps() const { return (uint32_t) h_.size(); }
  float bin_hz() const { return fs_ / (float) d_ / (float) m_; }
  float frame_s() const { return (float) m_ * (float) d_ / fs_; }
  size_t bytes() const {
    return sizeof(float) * (h_.size() + 6u * m_ + 2u * slots_) + 2u * slots_ * sizeof(uint32_t);
  }

 protected:
  void advance_oscillator_() {
    // The phasor is rotated in single precision and re-seeded from a double
    // phase every 1024 samples, so neither its magnitude nor its frequency can
    // drift. A frequency error here would be a frequency error in every result.
    phase_ += phase_step_;
    if (phase_ >= 2.0 * M_PI)
      phase_ -= 2.0 * M_PI;
    if (++osc_count_ >= 1024) {
      osc_count_ = 0;
      rot_re_ = (float) cos(phase_);
      rot_im_ = (float) sin(phase_);
    } else {
      const float nr = rot_re_ * step_re_ - rot_im_ * step_im_;
      rot_im_ = rot_re_ * step_im_ + rot_im_ * step_re_;
      rot_re_ = nr;
    }
  }

  // A decimated sample into the frame ring; runs a frame every half ring.
  bool emit_(float yr, float yi) {
    ring_re_[ring_pos_] = yr;
    ring_im_[ring_pos_] = yi;
    ring_pos_ = (ring_pos_ + 1) % m_;
    if (ring_fill_ < m_)
      ring_fill_++;
    if (++since_frame_ < m_ / 2 || ring_fill_ < m_)
      return false;
    since_frame_ = 0;
    analyze_();
    return true;
  }

  void analyze_() {
    // Oldest sample first, windowed.
    for (uint32_t i = 0; i < m_; i++) {
      const uint32_t j = (ring_pos_ + i) % m_;
      re_[i] = ring_re_[j] * window_[i];
      im_[i] = ring_im_[j] * window_[i];
    }
    fft(re_.data(), im_.data(), m_);
    // A real sine of RMS a appears as one complex line of amplitude a/sqrt(2)
    // after mixing, so the factor 2 keeps the RMS convention of the main FFT.
    const float norm = 2.0f / ((float) m_ * (float) m_ * window_cg_ * window_cg_);
    // Reorder so index 0 is the most negative frequency: fftshift.
    for (uint32_t i = 0; i < m_; i++) {
      const uint32_t k = (i + m_ / 2) % m_;
      psd_[i] = (re_[k] * re_[k] + im_[k] * im_[k]) * norm;
    }

    const float bin = bin_hz();
    const float base = fc_ - 0.5f * (float) m_ * bin;  // frequency of psd_[0]
    const uint32_t i_lo = std::max<uint32_t>(1, (uint32_t) ceilf((f_low_ - base) / bin));
    const uint32_t i_hi = std::min<uint32_t>(m_ - 2, (uint32_t) floorf((f_high_ - base) / bin));
    if (i_lo >= i_hi)
      return;
    uint32_t i_peak = i_lo;
    std::vector<float> in_band;
    in_band.reserve(i_hi - i_lo + 1);
    for (uint32_t i = i_lo; i <= i_hi; i++) {
      if (psd_[i] > psd_[i_peak])
        i_peak = i;
      in_band.push_back(psd_[i]);
    }
    std::nth_element(in_band.begin(), in_band.begin() + in_band.size() / 2, in_band.end());
    const float median = in_band[in_band.size() / 2];

    result_.seq++;
    result_.peak_hz = base + interpolate_peak(psd_.data(), m_, i_peak) * bin;
    result_.prominence_db = to_db(psd_[i_peak]) - to_db(median);
    result_.level_db = to_db(psd_[i_peak]);

    // Second line: the strongest local maximum at least four bins from the
    // first, so the first line's own Hann skirts are not reported as another.
    result_.second_hz = NAN;
    result_.second_prominence_db = NAN;
    uint32_t i_second = 0;
    for (uint32_t i = i_lo + 1; i < i_hi; i++) {
      if (i + 4 > i_peak && i < i_peak + 4)
        continue;
      if (psd_[i] < psd_[i - 1] || psd_[i] <= psd_[i + 1])
        continue;
      if (i_second == 0 || psd_[i] > psd_[i_second])
        i_second = i;
    }
    if (i_second != 0) {
      result_.second_hz = base + interpolate_peak(psd_.data(), m_, i_second) * bin;
      result_.second_prominence_db = to_db(psd_[i_second]) - to_db(median);
    }

    analyze_envelope_();
  }

  // Spectrum of the baseband's power over the same frame. The decimated
  // signal is the band's analytic signal, so |y|^2 is its envelope power;
  // two tones df apart make it a clean sinusoid at df, and a single tone
  // makes it flat plus noise. Reuses re_/im_, which the line analysis has
  // finished with.
  void analyze_envelope_() {
    result_.modulation_hz = NAN;
    result_.modulation_prominence_db = NAN;
    result_.modulation_depth = NAN;
    const float bin = bin_hz();
    const uint32_t k_lo = std::max<uint32_t>(1, (uint32_t) ceilf(mod_lo_ / bin));
    const uint32_t k_hi = std::min<uint32_t>(m_ / 2 - 1, (uint32_t) floorf(mod_hi_ / bin));
    if (k_lo + 2 > k_hi)
      return;
    double mean = 0.0;
    for (uint32_t i = 0; i < m_; i++) {
      const uint32_t j = (ring_pos_ + i) % m_;
      re_[i] = ring_re_[j] * ring_re_[j] + ring_im_[j] * ring_im_[j];
      mean += re_[i];
    }
    mean /= m_;
    if (mean <= 0.0)
      return;
    for (uint32_t i = 0; i < m_; i++) {
      re_[i] = (float) (re_[i] - mean) * window_[i];
      im_[i] = 0.0f;
    }
    fft(re_.data(), im_.data(), m_);
    // Power per bin, in place; each bin reads only its own index.
    for (uint32_t k = 0; k <= k_hi; k++)
      re_[k] = re_[k] * re_[k] + im_[k] * im_[k];
    uint32_t k_peak = k_lo;
    std::vector<float> range;
    range.reserve(k_hi - k_lo + 1);
    for (uint32_t k = k_lo; k <= k_hi; k++) {
      if (re_[k] > re_[k_peak])
        k_peak = k;
      range.push_back(re_[k]);
    }
    std::nth_element(range.begin(), range.begin() + range.size() / 2, range.end());
    result_.modulation_hz = interpolate_peak(re_.data(), k_hi + 1, k_peak) * bin;
    result_.modulation_prominence_db = to_db(re_[k_peak]) - to_db(range[range.size() / 2]);
    // A sinusoid of amplitude A in the windowed frame has a bin magnitude of
    // A x m x cg / 2; for power P(t) = P0 (1 + d cos), A = d x P0.
    const float amp = 2.0f * sqrtf(re_[k_peak]) / ((float) m_ * window_cg_);
    result_.modulation_depth = std::min(1.0f, amp / (float) mean);
  }

  float fs_{48000.0f}, f_low_{0.0f}, f_high_{0.0f}, fc_{0.0f};
  float mod_lo_{0.5f}, mod_hi_{5.0f};
  uint32_t m_{0}, d_{1};
  std::vector<float> h_;

  uint32_t slots_{0}, next_slot_{0}, since_start_{0};
  std::vector<float> acc_re_, acc_im_;
  std::vector<uint32_t> acc_tap_, acc_live_;

  double phase_{0.0}, phase_step_{0.0};
  float rot_re_{1.0f}, rot_im_{0.0f}, step_re_{1.0f}, step_im_{0.0f};
  uint32_t osc_count_{0};

  std::vector<float> ring_re_, ring_im_, re_, im_, psd_, window_;
  float window_cg_{0.5f};
  uint32_t ring_pos_{0}, ring_fill_{0}, since_frame_{0};

  Result result_;
};

}  // namespace dsp
}  // namespace spectral_analyzer
}  // namespace esphome
