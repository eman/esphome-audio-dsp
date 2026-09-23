// Host tests for dsp.h: synthetic signals with known answers, plus an optional
// real capture. Build and run:
//
//   c++ -std=c++17 -O2 -I components tests/test_dsp.cpp -o /tmp/test_dsp && /tmp/test_dsp [clip.wav]
//
// The clip must be 24-bit mono or stereo PCM WAV, as `audio_stream` serves it.

#include "spectral_analyzer/dsp.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <random>
#include <string>

using namespace esphome::spectral_analyzer::dsp;

static int failures = 0;
#define CHECK(cond, ...)                 \
  do {                                   \
    if (!(cond)) {                       \
      printf("  FAIL: " __VA_ARGS__);    \
      printf("\n");                      \
      failures++;                        \
    }                                    \
  } while (0)

// The main analyzer's averaged spectrum, the same way spectral_analyzer.cpp
// builds it: Hann window, RMS normalization, frames averaged.
static std::vector<float> averaged_psd(const std::vector<float> &x, uint32_t n) {
  std::vector<float> re(n), im(n), win(n), acc(n / 2 + 1, 0.0f);
  double cg = 0;
  for (uint32_t i = 0; i < n; i++) {
    win[i] = 0.5f * (1.0f - cosf(2.0f * (float) M_PI * i / (float) (n - 1)));
    cg += win[i];
  }
  const float wcg = (float) (cg / n);
  const float norm = 2.0f / ((float) n * n * wcg * wcg);
  uint32_t frames = 0;
  for (size_t off = 0; off + n <= x.size(); off += n) {
    for (uint32_t i = 0; i < n; i++) {
      re[i] = x[off + i] * win[i];
      im[i] = 0;
    }
    fft(re.data(), im.data(), n);
    for (uint32_t k = 0; k <= n / 2; k++)
      acc[k] += (re[k] * re[k] + im[k] * im[k]) * norm;
    frames++;
  }
  for (auto &v : acc)
    v /= (float) std::max<uint32_t>(frames, 1);
  return acc;
}

struct Tone {
  float f, rms_db, drift_hz = 0, drift_period_s = 1;
};

static std::vector<float> synth(float fs, float seconds, const std::vector<Tone> &tones,
                                float noise_db, uint32_t seed = 1) {
  std::mt19937 rng(seed);
  std::normal_distribution<float> g(0.0f, 1.0f);
  const size_t n = (size_t) (fs * seconds);
  std::vector<float> x(n, 0.0f);
  const float noise = powf(10.0f, noise_db / 20.0f);
  for (auto &v : x)
    v = noise * g(rng);
  for (const auto &t : tones) {
    const float amp = sqrtf(2.0f) * powf(10.0f, t.rms_db / 20.0f);
    double ph = 0;
    for (size_t i = 0; i < n; i++) {
      const double f = t.f + t.drift_hz * sin(2 * M_PI * i / (fs * t.drift_period_s));
      ph += 2 * M_PI * f / fs;
      x[i] += amp * (float) sin(ph);
    }
  }
  return x;
}

static ZoomFFT::Result run_zoom(const std::vector<float> &x, float fs, float lo, float hi,
                                uint32_t m, ZoomFFT *out = nullptr) {
  ZoomFFT z;
  z.configure(fs, lo, hi, m);
  for (float v : x)
    z.push(v);
  if (out)
    *out = z;
  return z.result();
}

static void test_zoom_accuracy() {
  printf("zoom: frequency and level of a steady tone\n");
  const float fs = 48000;
  for (float f : {651.3f, 700.37f, 703.912f, 758.8f}) {
    auto x = synth(fs, 30, {{f, -90}}, -110);
    ZoomFFT z;
    auto r = run_zoom(x, fs, 650, 760, 2048, &z);
    printf("  %.3f Hz -> %.4f Hz (err %+.4f, bin %.4f Hz, D=%u, %u taps, frame %.1f s, %.0f kB)"
           "  level %.2f dB  prom %.1f dB\n",
           f, r.peak_hz, r.peak_hz - f, z.bin_hz(), z.decimation(), z.taps(), z.frame_s(),
           z.bytes() / 1024.0, r.level_db, r.prominence_db);
    CHECK(r.seq > 0, "no frame produced");
    CHECK(fabsf(r.peak_hz - f) < 0.2f * z.bin_hz(), "frequency error %.4f Hz", r.peak_hz - f);
    // Hann scalloping is up to 1.4 dB; allow it.
    CHECK(fabsf(r.level_db - (-90.0f)) < 1.6f, "level %.2f dB, expected -90", r.level_db);
  }
}

static void test_zoom_vs_main_resolution() {
  printf("zoom: locks onto the stronger of two tones 0.3 Hz apart, inside one 1.46 Hz main bin\n");
  const float fs = 48000;
  auto x = synth(fs, 40, {{700.0f, -90}, {700.3f, -96}}, -115);
  ZoomFFT z;
  z.configure(fs, 650, 760, 4096);
  // Look at the last frame's spectrum directly: two separate local maxima.
  for (float v : x)
    z.push(v);
  printf("  bin %.4f Hz, strongest %.4f Hz\n", z.bin_hz(), z.result().peak_hz);
  CHECK(fabsf(z.result().peak_hz - 700.0f) < 0.03f, "strongest should be 700.0");
}

static void test_zoom_alias_rejection() {
  printf("zoom: loud tones outside the band do not alias into it\n");
  const float fs = 48000;
  ZoomFFT probe;
  probe.configure(fs, 650, 760, 2048);
  const float fs_d = fs / probe.decimation();
  // The worst case for a decimator: a tone exactly one output rate away from
  // a frequency in the band, so it folds right onto it. Plus mains rumble.
  const float bait = 705.0f + fs_d;
  auto x = synth(fs, 30, {{bait, -50}, {60.0f, -40}, {120.0f, -50}, {701.2f, -95}}, -115);
  auto r = run_zoom(x, fs, 650, 760, 2048);
  printf("  bait at %.1f Hz (-50 dB, folds to 705.0), 60 Hz at -40 dB; weak tone at 701.2 (-95 dB)"
         " -> peak %.3f Hz prom %.1f dB\n",
         bait, r.peak_hz, r.prominence_db);
  CHECK(fabsf(r.peak_hz - 701.2f) < 0.05f, "the alias won: peak at %.3f", r.peak_hz);
}

static void test_zoom_retune() {
  printf("zoom: reconfigure mid-stream\n");
  const float fs = 48000;
  auto x = synth(fs, 20, {{1203.4f, -90}}, -110);
  ZoomFFT z;
  z.configure(fs, 650, 760, 1024);
  for (size_t i = 0; i < x.size() / 4; i++)
    z.push(x[i]);
  z.configure(fs, 1150, 1250, 1024);
  for (size_t i = x.size() / 4; i < x.size(); i++)
    z.push(x[i]);
  printf("  -> %.3f Hz\n", z.result().peak_hz);
  CHECK(fabsf(z.result().peak_hz - 1203.4f) < 0.05f, "after retune %.3f", z.result().peak_hz);
}

static void test_harmonics() {
  printf("harmonics: relative levels of a drifting tone and its series\n");
  const float fs = 48000;
  const uint32_t n = 32768;
  const float bin = fs / n;
  // Fundamental drifting +/-1.5 Hz: H4 then wanders +/-6 Hz and smears.
  const float f0 = 700.4f;
  auto x = synth(fs, 5.5f,
                 {{f0, -90, 1.5f, 3.0f},
                  {2 * f0, -102, 3.0f, 3.0f},
                  {3 * f0, -110, 4.5f, 3.0f},
                  {4 * f0, -118, 6.0f, 3.0f}},
                 -120);
  // Every harmonic must drift in lock with the fundamental for this to be a
  // real harmonic series; synth() gives each its own phase but the same drift
  // period, which is what matters for the smeared power.
  auto psd = averaged_psd(x, n);
  const uint32_t half = n / 2 + 1;
  auto fund = measure_tone(psd.data(), half, bin, f0, 2.0f);
  printf("  H1 %.2f Hz prom %.1f dB\n", fund.freq_hz, fund.prominence_db);
  const float want[] = {0, 0, -12, -20, -28};
  for (uint32_t h = 2; h <= 4; h++) {
    auto m = measure_tone(psd.data(), half, bin, h * fund.freq_hz, h * 2.0f);
    const float rel = to_db(m.excess_power) - to_db(fund.excess_power);
    printf("  H%u %.2f Hz prom %5.1f dB  relative %6.1f dB (true %.0f)\n", h, m.freq_hz,
           m.prominence_db, rel, want[h]);
    CHECK(m.valid, "H%u not measured", h);
    CHECK(fabsf(rel - want[h]) < 2.0f, "H%u relative %.1f, want %.0f", h, rel, want[h]);
  }
  // And an absent harmonic must read as absent, not as a noise peak.
  auto h5 = measure_tone(psd.data(), half, bin, 5 * fund.freq_hz, 10.0f);
  printf("  H5 (absent) prom %.1f dB relative %.1f dB\n", h5.prominence_db,
         to_db(h5.excess_power) - to_db(fund.excess_power));
  CHECK(to_db(h5.excess_power) - to_db(fund.excess_power) < -30.0f, "H5 should be absent");
}

static void test_robust_spread() {
  printf("robust spread: one frame that lost the tone does not ruin the interval\n");
  // Six frames holding 700 Hz to within a hertz, one frame off in the noise.
  const float peaks[7] = {700.1f, 700.4f, 699.8f, 700.2f, 700.0f, 699.7f, 741.3f};
  float scratch[7];
  auto r = robust_spread(peaks, 7, 3.0f, scratch);
  // The standard deviation of the same seven, for comparison.
  double mean = 0, sq = 0;
  for (float p : peaks)
    mean += p;
  mean /= 7;
  for (float p : peaks)
    sq += (p - mean) * (p - mean);
  printf("  median %.2f Hz  robust spread %.2f Hz  agreement %.0f%%  (standard deviation %.1f Hz)\n",
         r.median, r.spread, 100 * r.agreement, sqrt(sq / 6));
  CHECK(fabsf(r.median - 700.1f) < 0.01f, "median %.2f", r.median);
  CHECK(r.spread < 1.0f, "robust spread %.2f should ignore the outlier", r.spread);
  CHECK(fabsf(r.agreement - 6.0f / 7.0f) < 1e-4f, "agreement %.3f", r.agreement);
  CHECK(sqrt(sq / 6) > 10.0, "the standard deviation should have been ruined, else no point");

  // Gaussian scatter: the scaled MAD reads as the standard deviation would.
  std::mt19937 rng(3);
  std::normal_distribution<float> g(700.0f, 2.0f);
  std::vector<float> many(2000), sc(2000);
  for (auto &v : many)
    v = g(rng);
  auto big = robust_spread(many.data(), many.size(), 3.0f, sc.data());
  printf("  2000 Gaussian peaks with sd 2.0: robust spread %.2f Hz\n", big.spread);
  CHECK(fabsf(big.spread - 2.0f) < 0.15f, "scaled MAD %.2f should read ~2.0", big.spread);
  auto two = robust_spread(peaks, 1, 3.0f, scratch);
  CHECK(std::isnan(two.spread), "one value has no spread");
  auto even = robust_spread(peaks, 4, 3.0f, scratch);
  CHECK(fabsf(even.median - 700.15f) < 0.01f, "even-count median %.3f", even.median);
}

static void test_zoom_second_line_and_beat() {
  printf("zoom: a beating pair shows two lines and a modulation at their separation\n");
  const float fs = 48000;
  // Noise at a level that gives the main FFT about 14 dB of prominence, the
  // node's overnight operating point, with the pair drifting independently.
  auto pair = synth(fs, 30, {{700.0f, -90, 1.2f, 25.0f}, {701.6f, -91, 1.0f, 31.0f}}, -65);
  ZoomFFT z;
  z.set_modulation_range(0.5f, 5.0f);
  z.configure(fs, 650, 760, 1024);
  int frames = 0, two_lines = 0, beat_at_sep = 0;
  float min_mod_prom = 1e9f;
  uint32_t seq = 0;
  for (float v : pair)
    if (z.push(v) && z.result().seq != seq) {
      seq = z.result().seq;
      const auto &r = z.result();
      const float sep = fabsf(r.second_hz - r.peak_hz);
      if (frames < 3)
        printf("  pair:   lines %.2f / %.2f Hz (sep %.2f)  modulation %.2f Hz  %.0f dB  depth %.0f%%\n",
               r.peak_hz, r.second_hz, sep, r.modulation_hz, r.modulation_prominence_db,
               100 * r.modulation_depth);
      frames++;
      if (sep > 0.8f && sep < 2.4f)
        two_lines++;
      if (fabsf(r.modulation_hz - sep) < 0.5f)
        beat_at_sep++;
      min_mod_prom = std::min(min_mod_prom, r.modulation_prominence_db);
    }
  printf("  %d frames: two lines 0.8-2.4 Hz apart in %d, modulation at their separation in %d, "
         "modulation prominence >= %.1f dB\n",
         frames, two_lines, beat_at_sep, min_mod_prom);
  CHECK(frames >= 6, "too few frames");
  CHECK(two_lines >= frames - 1, "second line not found reliably");
  CHECK(beat_at_sep >= frames - 1, "modulation frequency does not match the line separation");
  CHECK(min_mod_prom > 10.0f, "beat should be clearly prominent");

  printf("zoom: a single tone at the same SNR does not\n");
  auto single = synth(fs, 30, {{700.0f, -87, 1.2f, 25.0f}}, -65);
  ZoomFFT s;
  s.set_modulation_range(0.5f, 5.0f);
  s.configure(fs, 650, 760, 1024);
  frames = 0;
  float max_mod_prom = -1e9f, max_second = -1e9f;
  seq = 0;
  for (float v : single)
    if (s.push(v) && s.result().seq != seq) {
      seq = s.result().seq;
      const auto &r = s.result();
      if (frames < 3)
        printf("  single: lines %.2f / %.2f Hz  modulation %.2f Hz  %.0f dB  depth %.0f%%\n", r.peak_hz,
               r.second_hz, r.modulation_hz, r.modulation_prominence_db, 100 * r.modulation_depth);
      frames++;
      max_mod_prom = std::max(max_mod_prom, r.modulation_prominence_db);
      max_second = std::max(max_second, r.second_prominence_db - r.prominence_db);
    }
  printf("  %d frames: modulation prominence <= %.1f dB, second line at most %.1f dB relative to the first\n",
         frames, max_mod_prom, max_second);
  CHECK(max_mod_prom < 10.0f, "single tone should show no clear modulation");
  // The strongest of ~700 noise bins sits 10-13 dB over the median on its
  // own; the tone at this SNR sits ~25 dB over it.
  CHECK(max_second < -10.0f, "single tone's second line should be far below the first");
}

// ------------------------------------------------------------ real clip ----

static bool read_wav24(const char *path, std::vector<float> &x, float &fs) {
  FILE *f = fopen(path, "rb");
  if (!f)
    return false;
  std::vector<uint8_t> d;
  uint8_t buf[65536];
  size_t r;
  while ((r = fread(buf, 1, sizeof(buf), f)) > 0)
    d.insert(d.end(), buf, buf + r);
  fclose(f);
  if (d.size() < 44 || memcmp(d.data(), "RIFF", 4) != 0)
    return false;
  size_t p = 12;
  uint16_t ch = 1, bits = 0;
  uint32_t rate = 0;
  while (p + 8 <= d.size()) {
    uint32_t len;
    memcpy(&len, &d[p + 4], 4);
    if (memcmp(&d[p], "fmt ", 4) == 0) {
      memcpy(&ch, &d[p + 10], 2);
      memcpy(&rate, &d[p + 12], 4);
      memcpy(&bits, &d[p + 22], 2);
    } else if (memcmp(&d[p], "data", 4) == 0) {
      if (bits != 24)
        return false;
      const size_t end = std::min(d.size(), p + 8 + (size_t) len);  // endless streams lie
      for (size_t i = p + 8; i + 3 * ch <= end; i += 3 * ch) {
        int32_t v = d[i] | (d[i + 1] << 8) | (d[i + 2] << 16);
        if (v & 0x800000)
          v -= 0x1000000;
        x.push_back(v / 8388608.0f);
      }
      fs = (float) rate;
      return true;
    }
    p += 8 + len + (len & 1);
  }
  return false;
}

static void real_clip(const char *path) {
  std::vector<float> x;
  float fs = 0;
  if (!read_wav24(path, x, fs)) {
    printf("could not read %s as 24-bit WAV\n", path);
    failures++;
    return;
  }
  printf("real clip %s: %.1f s at %.0f Hz\n", path, x.size() / fs, fs);
  const uint32_t n = 32768;
  auto psd = averaged_psd(x, n);
  const float bin = fs / n;
  const uint32_t half = n / 2 + 1;
  for (auto band : {std::pair<float, float>{650, 760}, {1145, 1255}, {150, 210}}) {
    uint32_t kp = (uint32_t) ceilf(band.first / bin);
    for (uint32_t k = kp; k * bin <= band.second; k++)
      if (psd[k] > psd[kp])
        kp = k;
    const float f0 = interpolate_peak(psd.data(), half, kp) * bin;
    auto fund = measure_tone(psd.data(), half, bin, f0, 2.0f);
    printf("  band %.0f-%.0f: main FFT peak %.2f Hz prom %.1f dB\n", band.first, band.second, f0,
           fund.prominence_db);
    for (uint32_t h = 2; h <= 4; h++) {
      auto m = measure_tone(psd.data(), half, bin, h * f0, h * 2.0f);
      if (m.valid)
        printf("    H%u %.2f Hz prom %5.1f dB relative %6.1f dB\n", h, m.freq_hz, m.prominence_db,
               to_db(m.excess_power) - to_db(fund.excess_power));
    }
    ZoomFFT z;
    z.configure(fs, band.first, band.second, 1024);
    uint32_t seq = 0;
    for (float v : x)
      if (z.push(v) && z.result().seq != seq) {
        seq = z.result().seq;
        printf("    zoom frame %u: %.3f Hz prom %.1f dB (bin %.3f Hz)\n", seq, z.result().peak_hz,
               z.result().prominence_db, z.bin_hz());
      }
  }
}

int main(int argc, char **argv) {
  test_zoom_accuracy();
  test_zoom_vs_main_resolution();
  test_zoom_alias_rejection();
  test_zoom_retune();
  test_harmonics();
  test_robust_spread();
  test_zoom_second_line_and_beat();
  if (argc > 1)
    real_clip(argv[1]);
  printf(failures ? "\n%d FAILED\n" : "\nall passed\n", failures);
  return failures ? 1 : 0;
}
