#pragma once

// Doppler pass detection: a tonal source moving past in a straight line,
// heard as a line that glides down an S-shaped curve. No ESPHome or ESP-IDF in
// here, so it builds and tests on a host; see tests/test_doppler.cpp.
//
// The reference implementation is sensor-node/tools/doppler.py in the audio
// project, which was tuned on real captures first. Keep the two in step: the
// constants, the order of the acceptance tests and the fit's starting point
// are the same on purpose.
//
// Pipeline, per 1024 samples at 12 kHz (85 ms):
//   48 kHz -> 96-tap FIR, decimate by 4 -> 8192-point Hann FFT (1.46 Hz bins)
//   -> tonal peaks against a +-88 Hz running median -> peaks linked into
//   ridges -> ridges stitched into chains across gaps -> each chain fitted
//   with the moving-source model.
//
// The fine bins are for aircraft: one km away glides under 1 Hz/s, which
// 5.86 Hz bins cannot follow. A car at 8 m still fits to a few percent.
//
// The model. A source emitting f0 moves at v along a line whose closest point
// to the mic is d away, reached at emission time t0. Sound emitted at tau
// arrives at tau + r(tau)/c, heard at
//
//   f = f0 / (1 + (v/c) x / r),   x = v (tau - t0),  r = sqrt(d^2 + x^2)
//
// Fitting f0, beta = v/c, T = d/v and t0 gives v = beta c and d = v T.
//
// What is rejected. Engines changing RPM and machines spinning down move every
// harmonic together, so harmonics prove nothing. A pass glides on both sides
// of closest approach, fits the model to a fraction of a bin, is loudest at
// closest approach and fades again; those are the tests.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <new>
#include <vector>

namespace esphome {
namespace doppler {
namespace dsp {

static constexpr uint32_t kFsIn = 48000;
static constexpr uint32_t kDecim = 4;
static constexpr uint32_t kFs = kFsIn / kDecim;  // 12 kHz
static constexpr uint32_t kNfft = 8192;
static constexpr uint32_t kHop = 1024;
static constexpr uint32_t kBins = kNfft / 2 + 1;
static constexpr float kBinHz = (float) kFs / (float) kNfft;
static constexpr double kDt = (double) kHop / (double) kFs;
static constexpr uint32_t kTaps = 96;
static constexpr double kCutoffHz = 5200.0;
static constexpr uint32_t kMedHalf = 60;
static constexpr uint32_t kMaxPeaks = 32;  // hard ceiling; Params::max_peaks is the setting
static constexpr uint32_t kShortPts = 48;  // ridge points (4 s) before a long buffer is needed

inline float sound_speed(float temp_c) { return 331.3f * sqrtf(1.0f + temp_c / 273.15f); }

// Everything sizeable is allocated once, through one of these, so the node
// can put it in PSRAM. Nothing is ever freed: the detector lives as long as
// the firmware does.
using AllocFn = void *(*) (size_t);

struct Params {
  float f_lo{40.0f}, f_hi{5000.0f};
  float peak_db{8.0f};
  uint32_t max_peaks{24};
  uint32_t miss_max{3};
  float track_bins{1.5f};   // a peak extends a ridge within this many bins of the prediction
  float track_rel{0.004f};  // or this fraction of the frequency, if larger
  // Stitching. A car going by masks a distant aircraft for seconds, which
  // ends its ridge, and noise can pull a ridge off its line so the line
  // carries on as a new ridge overlapping the old one's tail. A finished ridge
  // continues a finished chain if it starts between overlap_s before the
  // chain's end and a gap after it, at the frequency the chain's glide
  // extrapolates to.
  float seg_min_s{1.0f};    // a ridge must be this long to stitch or be stitched
  float gap_min_s{5.0f};    // the gap allowed: gap_fraction of the chain's length,
  float gap_fraction{0.3f}; // at least gap_min_s
  float gap_cap_s{15.0f};   // and at most gap_cap_s
  float overlap_s{5.0f};
  float slope_s{2.0f};      // glides are extrapolated from this much of the chain's end
  float stitch_bins{2.0f};
  float stitch_rel{0.006f};
  float min_track_s{2.0f};
  float max_track_s{120.0f};
  uint32_t fit_points{512};
  // The chain spans 2 * cover_t * T in all. Each side of closest approach
  // shows cover_side_max_t * T, or cover_min_s if that is less, but never
  // under cover_side_t * T. An aircraft km away has T near 40 s, and traffic
  // rarely leaves a minute and a half of its line clean; a short pass can
  // always show its plateaus, and birds and revving engines make short
  // glides that lack them.
  float cover_t{1.0f};
  float cover_side_t{0.6f};
  float cover_side_max_t{1.2f};
  float cover_min_s{15.0f};
  float max_res_hz{1.0f};
  float min_swing_bins{1.0f};
  float swing_over_res{30.0f};
  float max_lag_t{1.0f};
  float min_rise_db{2.0f};  // louder near closest approach than at +-rise_t T
  float rise_t{1.2f};
  // A source that passes goes away again: beyond fade_t T it is over three
  // times as far. A steady tone that steps a few Hz fits a slow pass a few
  // metres off, but its level stays put.
  float fade_t{3.0f};
  float fade_min_s{1.0f};   // needs this much chain out there to judge
  float min_fade_db{6.0f};
  float beta_min{0.004f}, beta_max{0.65f};
  // T = d/v, the glide's time scale. A sigmoid over a second or two is any
  // pitch step - a bird's glide, an engine shifting - and its shape cannot
  // tell them apart. Every pass ADS-B confirmed had T 33-45 s; every fit no
  // aircraft explained, under 2 s. 5 s is a car 50 m off or a helicopter
  // 250 m off; 0 lets close vehicles back in with those false alarms.
  float min_t_s{5.0f};
  float sound_speed{340.3f};  // 15 C
  // Accepted lines within this long of each other are held and published as
  // one pass with harmonics.
  float group_hold_s{5.0f};
  uint32_t ridges{128};     // ridge slots, each holding kShortPts points
  uint32_t long_ridges{128}; // buffers for ridges past kShortPts points, and for chains
};

// In-place iterative radix-2. tw_re/tw_im hold exp(-2 pi i k / n) for
// k < n / 2; a recurrence drifts too far over 4096 steps in float.
inline void fft(float *re, float *im, uint32_t n, const float *tw_re, const float *tw_im) {
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
    const uint32_t h = len / 2, stride = n / len;
    for (uint32_t i = 0; i < n; i += len) {
      for (uint32_t k = 0; k < h; k++) {
        const float cr = tw_re[k * stride], ci = tw_im[k * stride];
        const float ur = re[i + k], ui = im[i + k];
        const float vr = re[i + k + h] * cr - im[i + k + h] * ci;
        const float vi = re[i + k + h] * ci + im[i + k + h] * cr;
        re[i + k] = ur + vr;
        im[i + k] = ui + vi;
        re[i + k + h] = ur - vr;
        im[i + k + h] = ui - vi;
      }
    }
  }
}

// scipy.signal.firwin(96, 5200, fs=48000): windowed sinc, Hamming, unit gain
// at DC.
inline void lowpass_taps(float *h) {
  const double fc = kCutoffHz / (kFsIn / 2.0);
  const double m = (kTaps - 1) / 2.0;
  double tmp[kTaps], sum = 0.0;
  for (uint32_t n = 0; n < kTaps; n++) {
    const double x = fc * (n - m);
    const double sinc = x == 0.0 ? 1.0 : sin(M_PI * x) / (M_PI * x);
    const double w = 0.54 - 0.46 * cos(2.0 * M_PI * n / (kTaps - 1));
    tmp[n] = fc * sinc * w;
    sum += tmp[n];
  }
  for (uint32_t n = 0; n < kTaps; n++)
    h[n] = (float) (tmp[n] / sum);
}

// FIR then keep every 4th output, starting with the first: y[0], y[4], ...
class Decimator {
 public:
  Decimator() {
    lowpass_taps(h_);
    memset(hist_, 0, sizeof(hist_));
  }
  // Returns how many outputs were written to out (at most n / 4 + 1).
  uint32_t process(const float *x, uint32_t n, float *out) {
    uint32_t m = 0;
    for (uint32_t i = 0; i < n; i++) {
      hist_[pos_] = x[i];
      if (phase_ == 0) {
        float acc = 0.0f;
        for (uint32_t k = 0; k < kTaps; k++)
          acc += h_[k] * hist_[(pos_ - k) & (kHist - 1)];
        out[m++] = acc;
      }
      phase_ = (phase_ + 1) % kDecim;
      pos_ = (pos_ + 1) & (kHist - 1);
    }
    return m;
  }

 protected:
  static constexpr uint32_t kHist = 128;
  float h_[kTaps];
  float hist_[kHist];
  uint32_t pos_{0};
  uint32_t phase_{0};
};

struct Peak {
  float f, db;
};

// One frame: Hann window, power spectrum in dBFS (full-scale sine -3.01 dB),
// tonal peaks against a running median over frequency.
class Spectrum {
 public:
  Spectrum() {
    double s = 0.0;
    for (uint32_t i = 0; i < kNfft; i++) {
      win_[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * i / (float) (kNfft - 1)));
      s += win_[i];
    }
    norm_ = (float) (2.0 / (s * s));
    for (uint32_t k = 0; k < kNfft / 2; k++) {
      tw_re_[k] = (float) cos(-2.0 * M_PI * k / kNfft);
      tw_im_[k] = (float) sin(-2.0 * M_PI * k / kNfft);
    }
  }

  // x: kNfft samples at 12 kHz, oldest first. Returns the number of peaks.
  uint32_t analyze(const float *x, const Params &p, Peak *out) {
    for (uint32_t i = 0; i < kNfft; i++) {
      re_[i] = x[i] * win_[i];
      im_[i] = 0.0f;
    }
    fft(re_, im_, kNfft, tw_re_, tw_im_);
    for (uint32_t k = 0; k < kBins; k++)
      db_[k] = 10.0f * log10f(norm_ * (re_[k] * re_[k] + im_[k] * im_[k]) + 1e-20f);

    const uint32_t k_lo = std::max<uint32_t>(1, (uint32_t) ceilf(p.f_lo / kBinHz));
    const uint32_t k_hi = std::min<uint32_t>(kBins - 1, (uint32_t) (p.f_hi / kBinHz));
    uint32_t n = 0;
    float win[2 * kMedHalf + 1];
    for (uint32_t k = k_lo; k < k_hi; k++) {
      const float c = db_[k];
      if (!(c > db_[k - 1] && c >= db_[k + 1]))
        continue;
      // Median of the +-kMedHalf neighbourhood, ends replicated (scipy's
      // median_filter, mode="nearest").
      for (int j = -(int) kMedHalf; j <= (int) kMedHalf; j++) {
        const int kk = std::min<int>(std::max<int>((int) k + j, 0), (int) kBins - 1);
        win[j + kMedHalf] = db_[kk];
      }
      std::nth_element(win, win + kMedHalf, win + 2 * kMedHalf + 1);
      const float ex = c - win[kMedHalf];
      if (ex < p.peak_db)
        continue;
      cand_k_[n] = k;
      cand_ex_[n] = ex;
      n++;
      if (n == kCandMax)
        break;
    }
    // Keep the strongest, in ascending order of excess, as numpy's argsort
    // slice does; otherwise in frequency order.
    uint32_t *order = order_;
    for (uint32_t i = 0; i < n; i++)
      order[i] = i;
    uint32_t keep = n, first = 0;
    const uint32_t maxp = std::min(p.max_peaks, kMaxPeaks);
    if (n > maxp) {
      std::stable_sort(order, order + n, [&](uint32_t a, uint32_t b) { return cand_ex_[a] < cand_ex_[b]; });
      first = n - maxp;
      keep = maxp;
    }
    for (uint32_t i = 0; i < keep; i++) {
      const uint32_t k = cand_k_[order[first + i]];
      const float a = db_[k - 1], b = db_[k], c = db_[k + 1];
      const float den = a - 2.0f * b + c;
      const float delta = den < 0.0f ? 0.5f * (a - c) / den : 0.0f;
      out[i].f = ((float) k + delta) * kBinHz;
      out[i].db = b - 0.25f * (a - c) * delta;
    }
    return keep;
  }

 protected:
  static constexpr uint32_t kCandMax = 1024;
  float win_[kNfft];
  float norm_;
  float re_[kNfft], im_[kNfft];
  float tw_re_[kNfft / 2], tw_im_[kNfft / 2];
  float db_[kBins];
  uint32_t cand_k_[kCandMax];
  uint32_t order_[kCandMax];
  float cand_ex_[kCandMax];
};

// ------------------------------------------------------------------ fitting

// Heard frequency at times t (relative seconds). p = {f0, beta, T, t0}.
inline void model(const float *p, const float *t, uint32_t n, float c, float *out) {
  const float f0 = p[0], beta = p[1], T = p[2], t0 = p[3];
  const float v = beta * c, d = v * T, dd = d * d;
  float lag = d / c;  // warm start: the previous point's propagation time
  for (uint32_t i = 0; i < n; i++) {
    float tau = t[i] - lag;
    for (int it = 0; it < 30; it++) {  // contraction with factor <= beta
      const float x = v * (tau - t0);
      const float nt = t[i] - sqrtf(dd + x * x) / c;
      const bool done = fabsf(nt - tau) < 1e-6f;
      tau = nt;
      if (done)
        break;
    }
    lag = t[i] - tau;
    const float x = v * (tau - t0);
    out[i] = f0 / (1.0f + beta * x / sqrtf(dd + x * x));
  }
}

struct Fit {
  bool accepted{false};
  const char *why{""};     // the first test failed; empty if accepted
  double t_heard{0};       // reception time of closest approach, stream seconds
  float f0{0}, speed{0}, dist{0}, T{0}, swing{0}, res{0};
  float level{0}, level_lag{0}, rise{0};
  float fade{99.0f};       // dB down beyond fade_t T; 99 if the chain ends sooner
  float beta{0};
  double t_first{0}, t_last{0};
  uint32_t n{0};
  float params[4]{};       // f0, beta, T, t0 (t0 relative to t_first)
};

// Scratch for one fit, sized for Params::fit_points.
struct FitScratch {
  float *t{nullptr}, *f{nullptr}, *db{nullptr}, *m{nullptr}, *mn{nullptr}, *r{nullptr}, *rn{nullptr},
      *jac{nullptr}, *smooth{nullptr};
  bool init(uint32_t n, AllocFn alloc = std::malloc) {
    float **one[] = {&t, &f, &db, &m, &mn, &r, &rn, &smooth};
    for (float **p : one)
      if (!(*p = (float *) alloc(sizeof(float) * n)))
        return false;
    return (jac = (float *) alloc(sizeof(float) * 4 * n)) != nullptr;
  }
};

namespace detail {

inline double median(const float *x, uint32_t n) {
  float v[128];
  n = std::min<uint32_t>(n, 128);
  std::copy(x, x + n, v);
  std::sort(v, v + n);
  return n % 2 ? v[n / 2] : 0.5 * ((double) v[n / 2 - 1] + v[n / 2]);
}

// Moving average padded with the end values (zero padding would drag dB
// figures toward 0 dB at both ends).
inline void smooth(const float *y, uint32_t n, uint32_t w, float *out) {
  const int left = (int) w / 2;
  for (uint32_t i = 0; i < n; i++) {
    double s = 0.0;
    for (int j = 0; j < (int) w; j++) {
      int k = (int) i - left + j;
      k = std::min<int>(std::max<int>(k, 0), (int) n - 1);
      s += y[k];
    }
    out[i] = (float) (s / w);
  }
}

inline bool solve4(double A[4][4], double b[4], double x[4]) {
  double M[4][5];
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++)
      M[i][j] = A[i][j];
    M[i][4] = b[i];
  }
  for (int c = 0; c < 4; c++) {
    int piv = c;
    for (int r = c + 1; r < 4; r++)
      if (fabs(M[r][c]) > fabs(M[piv][c]))
        piv = r;
    if (fabs(M[piv][c]) < 1e-300)
      return false;
    if (piv != c)
      for (int j = 0; j < 5; j++)
        std::swap(M[c][j], M[piv][j]);
    for (int r = c + 1; r < 4; r++) {
      const double f = M[r][c] / M[c][c];
      for (int j = c; j < 5; j++)
        M[r][j] -= f * M[c][j];
    }
  }
  for (int i = 3; i >= 0; i--) {
    double s = M[i][4];
    for (int j = i + 1; j < 4; j++)
      s -= M[i][j] * x[j];
    x[i] = s / M[i][i];
  }
  return true;
}

// Levenberg-Marquardt with a soft-L1 loss by reweighting, in scaled
// variables, with bounds by clamping.
inline void lm(float p[4], const float lo[4], const float hi[4], const float sc[4], const float *t,
               const float *f, uint32_t n, float c, float fscale, FitScratch &s) {
  auto cost = [&](const float *pp, float *m, float *r) {
    model(pp, t, n, c, m);
    double sum = 0.0;
    for (uint32_t i = 0; i < n; i++) {
      r[i] = m[i] - f[i];
      const double z = (double) r[i] * r[i] / ((double) fscale * fscale);
      sum += 2.0 * (sqrt(1.0 + z) - 1.0);
    }
    return sum;
  };
  float *m = s.m, *mn = s.mn, *r = s.r, *rn = s.rn, *J = s.jac;
  double C = cost(p, m, r);
  double lambda = 1e-3;
  for (int it = 0; it < 100; it++) {
    // Jacobian in scaled variables by forward differences.
    for (int j = 0; j < 4; j++) {
      float pp[4] = {p[0], p[1], p[2], p[3]};
      float h = 1e-3f * sc[j];
      if (pp[j] + h > hi[j])
        h = -h;
      pp[j] += h;
      model(pp, t, n, c, mn);
      const float hs = h / sc[j];
      for (uint32_t i = 0; i < n; i++)
        J[j * n + i] = (mn[i] - m[i]) / hs;
    }
    double A[4][4] = {}, g[4] = {};
    for (uint32_t i = 0; i < n; i++) {
      const double z = (double) r[i] * r[i] / ((double) fscale * fscale);
      const double w = 1.0 / sqrt(1.0 + z);
      for (int a = 0; a < 4; a++) {
        const double ja = J[a * n + i];
        g[a] += w * ja * r[i];
        for (int b = a; b < 4; b++)
          A[a][b] += w * ja * J[b * n + i];
      }
    }
    for (int a = 0; a < 4; a++)
      for (int b = 0; b < a; b++)
        A[a][b] = A[b][a];
    bool accepted = false;
    double Cn = C;
    for (int tries = 0; tries < 12; tries++) {
      double Al[4][4], rhs[4], dx[4];
      for (int a = 0; a < 4; a++) {
        for (int b = 0; b < 4; b++)
          Al[a][b] = A[a][b];
        Al[a][a] += lambda * (A[a][a] + 1e-12);
        rhs[a] = -g[a];
      }
      if (!solve4(Al, rhs, dx)) {
        lambda *= 4;
        continue;
      }
      float pn[4];
      for (int a = 0; a < 4; a++) {
        const float v = p[a] + (float) dx[a] * sc[a];
        const float l = lo[a] + 1e-6f * fabsf(lo[a]) + 1e-9f, h = hi[a] - 1e-6f * fabsf(hi[a]) - 1e-9f;
        pn[a] = std::min(std::max(v, l), h);
      }
      Cn = cost(pn, mn, rn);
      if (Cn < C) {
        memcpy(p, pn, sizeof(pn));
        memcpy(m, mn, n * sizeof(float));
        memcpy(r, rn, n * sizeof(float));
        lambda = std::max(lambda / 3.0, 1e-9);
        accepted = true;
        break;
      }
      lambda *= 4;
    }
    if (!accepted)
      break;
    const double gain = C - Cn;
    C = Cn;
    if (gain < 1e-9 * C + 1e-12)
      break;
  }
}

}  // namespace detail

// Fits one ridge. t is relative to the ridge's first point (t[0] == 0),
// t_first its absolute time. Returns false if the ridge is not worth a fit
// (too short, or not gliding down at all) - most ridges stop there.
inline bool fit_ridge(const float *t_in, const float *f_in, const float *db_in, uint32_t n_in, double t_first,
                      const Params &P, FitScratch &s, Fit &out) {
  const uint32_t step = n_in > P.fit_points ? (n_in + P.fit_points - 1) / P.fit_points : 1;
  uint32_t n = 0;
  for (uint32_t i = 0; i < n_in; i += step, n++) {
    s.t[n] = t_in[i];
    s.f[n] = f_in[i];
    s.db[n] = db_in[i];
  }
  if (n < 20)
    return false;
  const float *t = s.t, *f = s.f, *db = s.db;
  const uint32_t k = std::max<uint32_t>(3, n / 8);
  const double f_hi = detail::median(f, k), f_lo = detail::median(f + n - k, k);
  if (f_hi - f_lo < P.min_swing_bins * kBinHz)
    return false;

  // Steepest descent of the smoothed ridge seeds closest approach.
  float *fs = s.smooth;
  detail::smooth(f, n, 5, fs);
  uint32_t i0 = 2;
  double best = 1e300;
  for (uint32_t i = 2; i + 2 < n; i++) {  // numpy.gradient, interior points
    const double hd = (double) t[i] - t[i - 1], hs = (double) t[i + 1] - t[i];
    const double gr = (hd * hd * fs[i + 1] - hs * hs * fs[i - 1] + (hs * hs - hd * hd) * fs[i]) /
                      (hd * hs * (hd + hs));
    if (gr < best) {
      best = gr;
      i0 = i;
    }
  }
  const double beta0 = (f_hi - f_lo) / (f_hi + f_lo);
  const double f00 = 2.0 * f_hi * f_lo / (f_hi + f_lo);
  const double T0 = std::min(std::max(f00 * beta0 / std::max(-best, 1e-3), 0.2), 60.0);
  const float c = P.sound_speed;
  float lo[4] = {(float) (f_lo * 0.8), P.beta_min * 0.5f, 0.05f, t[0] - 60.0f};
  float hi[4] = {(float) (f_hi * 1.2), P.beta_max, 120.0f, t[n - 1] + 60.0f};
  float p[4] = {(float) f00, (float) std::min(std::max(beta0, (double) P.beta_min), (double) P.beta_max),
                (float) T0, (float) (t[i0] - beta0 * T0)};
  for (int a = 0; a < 4; a++)
    p[a] = std::min(std::max(p[a], lo[a] + 1e-6f), hi[a] - 1e-6f);
  const float sc[4] = {(float) f00, 0.05f, (float) T0, (float) T0};
  detail::lm(p, lo, hi, sc, t, f, n, c, 0.5f * kBinHz, s);

  const float f0 = p[0], beta = p[1], T = p[2], t0 = p[3];
  model(p, t, n, c, s.m);
  double ss = 0.0;
  uint32_t core = 0;
  for (uint32_t i = 0; i < n; i++) {
    const float r = s.m[i] - f[i];
    if (fabsf(r) < 3.0f * kBinHz) {
      ss += (double) r * r;
      core++;
    }
  }
  const float res = core > 10 ? (float) sqrt(ss / core) : 1e9f;
  const float v = beta * c, d = v * T;
  const float th = t0 + d / c;
  const float swing = f0 / (1.0f - beta) - f0 / (1.0f + beta);
  float *dbs = s.smooth;
  detail::smooth(db, n, 5, dbs);  // 0.43 s
  uint32_t il = 0;
  for (uint32_t i = 1; i < n; i++)
    if (dbs[i] > dbs[il])
      il = i;
  float near_max = -1e30f;
  double far_sum = 0.0;
  uint32_t near_n = 0, far_n = 0;
  for (uint32_t i = 0; i < n; i++) {
    const float a = fabsf(t[i] - th);
    if (a <= 0.5f * T) {
      near_max = std::max(near_max, dbs[i]);
      near_n++;
    }
    if (fabsf(a - P.rise_t * T) <= 0.25f * T) {
      far_sum += dbs[i];
      far_n++;
    }
  }
  const float rise = near_n && far_n ? near_max - (float) (far_sum / far_n) : -99.0f;
  // Median level beyond fade_t T, as numpy.median: mean of the middle two.
  uint32_t out_n = 0;
  for (uint32_t i = 0; i < n; i++)
    if (fabsf(t[i] - th) >= P.fade_t * T)
      s.rn[out_n++] = dbs[i];
  float fade = 99.0f;
  if (near_n && out_n * step * kDt >= P.fade_min_s) {
    float *v = s.rn;
    std::nth_element(v, v + out_n / 2, v + out_n);
    double med = v[out_n / 2];
    if (out_n % 2 == 0)
      med = 0.5 * (med + *std::max_element(v, v + out_n / 2));
    fade = near_max - (float) med;
  }

  out = Fit{};
  out.t_heard = t_first + th;
  out.f0 = f0;
  out.beta = beta;
  out.speed = v;
  out.dist = d;
  out.T = T;
  out.swing = swing;
  out.res = res;
  out.n = n;
  out.t_first = t_first;
  out.t_last = t_first + t[n - 1];
  out.level = dbs[il];
  out.level_lag = (t[il] - th) / T;
  out.rise = rise;
  out.fade = fade;
  memcpy(out.params, p, sizeof(p));
  const float side = std::min(P.cover_side_max_t * T, std::max(P.cover_side_t * T, P.cover_min_s));
  if (!(P.beta_min <= beta && beta < P.beta_max * 0.98f))
    out.why = "speed at a bound";
  else if (T < P.min_t_s)
    out.why = "too quick";
  else if (th - t[0] < side || t[n - 1] - th < side ||
           t[n - 1] - t[0] < 2.0f * P.cover_t * T)
    out.why = "no plateaus";
  else if (res > P.max_res_hz)
    out.why = "residual";
  else if (swing < P.min_swing_bins * kBinHz || swing < P.swing_over_res * res)
    out.why = "swing too small";
  else if (fabsf(out.level_lag) > P.max_lag_t || rise < P.min_rise_db)
    out.why = "not loudest at closest approach";
  else if (fade < P.min_fade_db)
    out.why = "does not fade";
  out.accepted = out.why[0] == '\0';
  return true;
}

// ------------------------------------------------------------------ passes

// One source: the accepted lines that share closest approach and speed.
struct Pass {
  Fit first;             // the line that opened the group; others are matched to it
  Fit best;              // the line with the lowest residual relative to swing
  float f0s[8];          // every line's f0, ascending
  uint32_t lines{0};
  double last_added{0};  // stream seconds
};

// ------------------------------------------------------------------ detector

class Detector {
 public:
  // Allocates everything up front. Ridge storage and fit buffers come from
  // alloc; the spectrum and decimator, touched every frame, from fast.
  // Returns false on allocation failure.
  bool init(const Params &p, AllocFn alloc = std::malloc, AllocFn fast = std::malloc) {
    P_ = p;
    P_.fit_points = std::min<uint32_t>(std::max<uint32_t>(P_.fit_points, 64), 1024);
    long_cap_ = (uint32_t) floor(P_.max_track_s / kDt) + 2;
    slots_.assign(P_.ridges, Slot{});
    for (auto &s : slots_) {
      s.frame = (uint32_t *) alloc(sizeof(uint32_t) * kShortPts);
      s.f = (float *) alloc(sizeof(float) * kShortPts);
      s.db = (float *) alloc(sizeof(float) * kShortPts);
      if (!s.frame || !s.f || !s.db)
        return false;
    }
    chains_.assign(P_.long_ridges, Slot{});
    pool_.reserve(P_.long_ridges);
    longs_.assign(P_.long_ridges, Long{});
    for (auto &l : longs_) {
      l.frame = (uint32_t *) alloc(sizeof(uint32_t) * long_cap_);
      l.f = (float *) alloc(sizeof(float) * long_cap_);
      l.db = (float *) alloc(sizeof(float) * long_cap_);
      if (!l.frame || !l.f || !l.db)
        return false;
    }
    spec_ = (Spectrum *) fast(sizeof(Spectrum));
    dec_ = (Decimator *) fast(sizeof(Decimator));
    if (!spec_ || !dec_)
      return false;
    new (spec_) Spectrum();
    new (dec_) Decimator();
    frame_buf_.assign(kNfft, 0.0f);
    dec_out_.assign(1024 / kDecim + 2, 0.0f);
    fit_t_ = (float *) alloc(sizeof(float) * long_cap_);
    fit_f_ = (float *) alloc(sizeof(float) * long_cap_);
    fit_db_ = (float *) alloc(sizeof(float) * long_cap_);
    if (!fit_t_ || !fit_f_ || !fit_db_ || !scratch_.init(P_.fit_points + 1, alloc))
      return false;
    order_.assign(P_.ridges, 0);
    by_order_.assign(P_.ridges, 0);
    seq_.assign(P_.ridges, 0);
    tmp_.reserve(P_.ridges);
    took_.assign(P_.ridges, false);
    cands_.reserve(P_.ridges * 4);
    pending_.reserve(16);
    done_.reserve(16);
    return true;
  }

  void set_sound_speed(float c) { P_.sound_speed = c; }
  const Params &params() const { return P_; }

  // 48 kHz samples, full scale +-1. Feed in blocks of at most 1024.
  void feed(const float *x, uint32_t n) {
    while (n > 0) {
      const uint32_t take = std::min<uint32_t>(n, 1024);
      const uint32_t m = dec_->process(x, take, dec_out_.data());
      for (uint32_t i = 0; i < m; i++) {
        frame_buf_[fill_++] = dec_out_[i];
        decimated_++;
        if (fill_ == kNfft) {
          frame_();
          memmove(frame_buf_.data(), frame_buf_.data() + kHop, sizeof(float) * (kNfft - kHop));
          fill_ = kNfft - kHop;
        }
      }
      x += take;
      n -= take;
    }
  }

  // Ends every live ridge, as at the end of a file. The node never calls it.
  void flush() {
    renumber_();
    const uint32_t n = (uint32_t) tmp_.size();
    for (uint32_t r = 0; r < n; r++)
      finish_(by_order_[r]);
    for (uint32_t ci : pool_)
      close_chain_(ci);
    pool_.clear();
    for (auto &g : pending_)
      done_.push_back(g);
    pending_.clear();
  }

  // Passes ready to publish, oldest first.
  bool pop(Pass &out) {
    if (done_.empty())
      return false;
    out = done_.front();
    done_.erase(done_.begin());
    return true;
  }

  // Every fit, accepted or not, for tests and the reference comparison.
  void set_fit_log(std::vector<Fit> *log) { fit_log_ = log; }

  double now() const { return (double) decimated_ / kFs; }
  uint32_t frames() const { return frame_idx_; }
  uint32_t live_ridges() const { return live_; }
  uint32_t dropped_ridges() const { return dropped_; }  // no free slot
  uint32_t truncated_ridges() const { return truncated_; }  // no long buffer
  uint32_t fits() const { return fits_; }

 protected:
  struct Cand {
    float d;
    uint32_t i, j;
  };
  struct Long {
    uint32_t *frame{nullptr};
    float *f{nullptr}, *db{nullptr};
    bool used{false};
  };
  struct Slot {
    uint32_t *frame{nullptr};
    float *f{nullptr}, *db{nullptr};
    int32_t lng{-1};    // index into longs_ once past kShortPts
    uint32_t head{0};   // ring start, long buffers only
    uint32_t count{0};
    uint32_t miss{0};
    bool live{false};
    bool full{false};   // ran out of room; stops taking points
  };

  uint32_t cap_(const Slot &s) const { return s.lng >= 0 ? long_cap_ : kShortPts; }
  uint32_t idx_(const Slot &s, uint32_t i) const { return s.lng >= 0 ? (s.head + i) % long_cap_ : i; }
  uint32_t *F_(Slot &s) { return s.lng >= 0 ? longs_[s.lng].frame : s.frame; }
  float *Fr_(Slot &s) { return s.lng >= 0 ? longs_[s.lng].f : s.f; }
  float *Db_(Slot &s) { return s.lng >= 0 ? longs_[s.lng].db : s.db; }

  static double t_of(uint32_t frame) { return ((double) frame * kHop + kNfft / 2.0) / kFs; }

  float predict_(Slot &s, double t) {
    uint32_t *fr = F_(s);
    float *f = Fr_(s);
    const float last = f[idx_(s, s.count - 1)];
    if (s.count >= 4) {
      const uint32_t a = idx_(s, s.count - 4), b = idx_(s, s.count - 1);
      const double slope = (last - f[a]) / (t_of(fr[b]) - t_of(fr[a]));
      return (float) (last + slope * (t - t_of(fr[b])));
    }
    return last;
  }

  int32_t take_long_() {
    for (uint32_t i = 0; i < longs_.size(); i++)
      if (!longs_[i].used) {
        longs_[i].used = true;
        return (int32_t) i;
      }
    return -1;
  }

  void add_(Slot &s, uint32_t frame, float f, float db) {
    s.miss = 0;
    if (s.full)
      return;
    if (s.lng < 0 && s.count == kShortPts) {
      const int32_t got = take_long_();
      if (got < 0) {
        s.full = true;
        truncated_++;
        return;
      }
      memcpy(longs_[got].frame, s.frame, sizeof(uint32_t) * kShortPts);
      memcpy(longs_[got].f, s.f, sizeof(float) * kShortPts);
      memcpy(longs_[got].db, s.db, sizeof(float) * kShortPts);
      s.lng = got;
      s.head = 0;
    }
    uint32_t *fr = F_(s);
    const uint32_t i = idx_(s, s.count);
    fr[i] = frame;
    Fr_(s)[i] = f;
    Db_(s)[i] = db;
    s.count++;
    // Drop points older than max_track_s, as the reference does.
    const double tl = t_of(frame);
    while (s.lng >= 0 && s.count > 1 && tl - t_of(fr[idx_(s, 0)]) > P_.max_track_s) {
      s.head = (s.head + 1) % long_cap_;
      s.count--;
    }
  }

  void release_(Slot &s) {
    if (s.lng >= 0)
      longs_[s.lng].used = false;
    s.lng = -1;
    s.head = s.count = s.miss = 0;
    s.live = s.full = false;
  }

  double first_t_(Slot &s) { return t_of(F_(s)[idx_(s, 0)]); }
  double last_t_(Slot &s) { return t_of(F_(s)[idx_(s, s.count - 1)]); }

  void fit_slot_(Slot &s) {
    uint32_t *fr = F_(s);
    float *f = Fr_(s), *db = Db_(s);
    const double t_first = first_t_(s);
    for (uint32_t i = 0; i < s.count; i++) {
      const uint32_t j = idx_(s, i);
      fit_t_[i] = (float) (t_of(fr[j]) - t_first);
      fit_f_[i] = f[j];
      fit_db_[i] = db[j];
    }
    Fit fit;
    if (fit_ridge(fit_t_, fit_f_, fit_db_, s.count, t_first, P_, scratch_, fit)) {
      fits_++;
      if (fit_log_)
        fit_log_->push_back(fit);
      if (fit.accepted)
        group_(fit);
    }
  }

  double gap_max_(Slot &c) {
    const double g = P_.gap_fraction * (last_t_(c) - first_t_(c));
    return std::min<double>(P_.gap_cap_s, std::max<double>(P_.gap_min_s, g));
  }

  // A ridge has ended. Long enough, it continues the pool chain whose glide
  // it carries on, or becomes a chain in the pool itself.
  void finish_(uint32_t si) {
    Slot &s = slots_[si];
    const double r0 = first_t_(s), r1 = last_t_(s);
    if (r1 - r0 >= P_.seg_min_s) {
      double st = 0, sf = 0;
      uint32_t k = 0;
      while (k < s.count && t_of(F_(s)[idx_(s, k)]) <= r0 + P_.slope_s) {
        st += t_of(F_(s)[idx_(s, k)]);
        sf += Fr_(s)[idx_(s, k)];
        k++;
      }
      const double tm = st / k, fm = sf / k;
      const double tol = std::max((double) P_.stitch_bins * kBinHz, (double) P_.stitch_rel * fm);
      int best = -1;
      double bd = 0;
      for (uint32_t q = 0; q < pool_.size(); q++) {
        Slot &c = chains_[pool_[q]];
        const double gap = r0 - last_t_(c);
        if (first_t_(c) < r0 && last_t_(c) < r1 && gap >= -P_.overlap_s && gap <= gap_max_(c)) {
          const double d = fabs(extrapolate_(c, tm) - fm);
          if (d <= tol && (best < 0 || d < bd)) {
            best = (int) q;
            bd = d;
          }
        }
      }
      if (best >= 0) {
        Slot &c = chains_[pool_[best]];
        while (c.count > 0 && t_of(F_(c)[idx_(c, c.count - 1)]) >= r0)  // the tail under r goes
          c.count--;
        uint32_t *fr = F_(s);
        float *f = Fr_(s), *db = Db_(s);
        for (uint32_t i = 0; i < s.count; i++) {
          const uint32_t j = idx_(s, i);
          add_(c, fr[j], f[j], db[j]);
        }
      } else {
        int32_t ci = -1;
        for (uint32_t i = 0; i < chains_.size(); i++)
          if (!chains_[i].live) {
            ci = (int32_t) i;
            break;
          }
        if (ci >= 0) {
          Slot &c = chains_[ci];
          if (s.lng >= 0) {  // hand the long buffer over
            c.lng = s.lng;
            c.head = s.head;
            c.count = s.count;
            s.lng = -1;
          } else if ((c.lng = take_long_()) >= 0) {
            memcpy(longs_[c.lng].frame, s.frame, sizeof(uint32_t) * s.count);
            memcpy(longs_[c.lng].f, s.f, sizeof(float) * s.count);
            memcpy(longs_[c.lng].db, s.db, sizeof(float) * s.count);
            c.head = 0;
            c.count = s.count;
          } else {
            ci = -1;
          }
        }
        if (ci >= 0) {
          chains_[ci].live = true;
          pool_.push_back((uint32_t) ci);
        } else {
          truncated_++;
        }
      }
    }
    release_(s);
    live_--;
  }

  // Least-squares line through the last slope_s of a chain, evaluated at t.
  double extrapolate_(Slot &c, double t) {
    const double tl = last_t_(c);
    uint32_t k = 0;
    while (k < c.count && t_of(F_(c)[idx_(c, k)]) < tl - P_.slope_s)
      k++;
    double sx = 0, sy = 0;
    const uint32_t n = c.count - k;
    for (uint32_t i = k; i < c.count; i++) {
      sx += t_of(F_(c)[idx_(c, i)]);
      sy += Fr_(c)[idx_(c, i)];
    }
    const double tm = sx / n, fm = sy / n;
    double num = 0, den = 0;
    for (uint32_t i = k; i < c.count; i++) {
      const double dt = t_of(F_(c)[idx_(c, i)]) - tm;
      num += dt * (Fr_(c)[idx_(c, i)] - fm);
      den += dt * dt;
    }
    return fm + (den > 0 ? num / den : 0.0) * (t - tm);
  }

  void close_chain_(uint32_t ci) {
    Slot &c = chains_[ci];
    if (last_t_(c) - first_t_(c) >= P_.min_track_s)
      fit_slot_(c);
    release_(c);
  }

  void group_(const Fit &f) {
    for (auto &g : pending_) {
      const Fit &q = g.first;
      if (fabs(f.t_heard - q.t_heard) < 0.3 * q.T + 0.2 && fabsf(f.speed - q.speed) < 0.15f * q.speed + 1.0f) {
        if (g.lines < 8)
          g.f0s[g.lines] = f.f0;
        g.lines++;
        std::sort(g.f0s, g.f0s + std::min<uint32_t>(g.lines, 8));
        if (f.res / f.swing < q.res / q.swing)
          g.best = f;
        g.last_added = now();
        return;
      }
    }
    Pass g;
    g.first = f;
    g.best = f;
    g.f0s[0] = f.f0;
    g.lines = 1;
    g.last_added = now();
    pending_.push_back(g);
  }

  void frame_() {
    Peak pk[kMaxPeaks];
    const uint32_t np = spec_->analyze(frame_buf_.data(), P_, pk);
    const uint32_t fi = frame_idx_++;
    const double t = t_of(fi);

    // Greedy association, closest first; ties in ridge age then peak order.
    cands_.clear();
    for (uint32_t i = 0; i < slots_.size(); i++) {
      Slot &s = slots_[i];
      if (!s.live)
        continue;
      const float pf = predict_(s, t);
      const float tol = std::max(P_.track_bins * kBinHz, P_.track_rel * pf) * (1.0f + 0.5f * s.miss);
      for (uint32_t j = 0; j < np; j++) {
        const float d = fabsf(pk[j].f - pf);
        if (d <= tol)
          cands_.push_back({d, order_[i], j});
      }
    }
    std::sort(cands_.begin(), cands_.end(), [](const Cand &a, const Cand &b) {
      if (a.d != b.d)
        return a.d < b.d;
      if (a.i != b.i)
        return a.i < b.i;
      return a.j < b.j;
    });
    bool used[kMaxPeaks] = {};
    std::fill(took_.begin(), took_.end(), false);
    for (const Cand &c : cands_) {
      const uint32_t si = by_order_[c.i];
      if (took_[si] || used[c.j])
        continue;
      add_(slots_[si], fi, pk[c.j].f, pk[c.j].db);
      took_[si] = true;
      used[c.j] = true;
    }
    const uint32_t was_live = (uint32_t) tmp_.size();
    for (uint32_t r = 0; r < was_live; r++) {
      const uint32_t si = by_order_[r];
      Slot &s = slots_[si];
      if (!took_[si] && ++s.miss > P_.miss_max)
        finish_(si);
    }
    // New ridges for unclaimed peaks. Order numbers keep the reference's
    // list order (older ridges first) for the tie-break above.
    for (uint32_t j = 0; j < np; j++) {
      if (used[j])
        continue;
      uint32_t si = 0;
      while (si < slots_.size() && slots_[si].live)
        si++;
      if (si == slots_.size()) {
        dropped_++;
        continue;
      }
      Slot &s = slots_[si];
      s.live = true;
      add_(s, fi, pk[j].f, pk[j].db);
      live_++;
      seq_[si] = next_seq_++;
    }
    renumber_();

    // A chain is done once no ridge can start in its window any more and
    // none that started there is still running.
    for (size_t q = 0; q < pool_.size();) {
      const uint32_t ci = pool_[q];
      Slot &c = chains_[ci];
      const double lo = last_t_(c) - P_.overlap_s, hi = last_t_(c) + gap_max_(c);
      bool open = t <= hi;
      for (uint32_t r = 0; !open && r < tmp_.size(); r++) {
        const double r0 = first_t_(slots_[by_order_[r]]);
        open = r0 >= lo && r0 <= hi;
      }
      if (open) {
        q++;
      } else {
        pool_.erase(pool_.begin() + q);
        close_chain_(ci);
      }
    }

    // Publish groups nobody has joined for a while.
    const double tn = now();
    for (size_t i = 0; i < pending_.size();) {
      if (tn - pending_[i].last_added >= P_.group_hold_s) {
        done_.push_back(pending_[i]);
        pending_.erase(pending_.begin() + i);
      } else {
        i++;
      }
    }
  }

  // order_[slot] = rank of the slot by creation sequence among live slots.
  void renumber_() {
    const uint32_t n = slots_.size();
    tmp_.clear();
    for (uint32_t i = 0; i < n; i++)
      if (slots_[i].live)
        tmp_.push_back(i);
    std::sort(tmp_.begin(), tmp_.end(), [&](uint32_t a, uint32_t b) { return seq_[a] < seq_[b]; });
    for (uint32_t r = 0; r < tmp_.size(); r++) {
      order_[tmp_[r]] = r;
      by_order_[r] = tmp_[r];
    }
  }

  Params P_;
  uint32_t long_cap_{0};
  std::vector<Slot> slots_;
  std::vector<Slot> chains_;     // finished ridges and their continuations; always long buffers
  std::vector<uint32_t> pool_;   // chains a ridge may still continue, oldest first
  std::vector<Long> longs_;
  Spectrum *spec_{nullptr};
  Decimator *dec_{nullptr};
  std::vector<float> frame_buf_, dec_out_;
  uint32_t fill_{0};
  uint64_t decimated_{0};
  uint32_t frame_idx_{0};
  uint32_t live_{0}, dropped_{0}, truncated_{0}, fits_{0};
  float *fit_t_{nullptr}, *fit_f_{nullptr}, *fit_db_{nullptr};
  FitScratch scratch_;
  std::vector<Pass> pending_, done_;
  std::vector<Fit> *fit_log_{nullptr};
  std::vector<uint32_t> order_, by_order_, seq_, tmp_;
  uint32_t next_seq_{0};
  std::vector<bool> took_;
  std::vector<Cand> cands_;
};

}  // namespace dsp
}  // namespace doppler
}  // namespace esphome
