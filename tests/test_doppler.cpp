// Host tests for doppler_dsp.h: synthetic passes with known speed and
// distance, signals that must be rejected, and optionally real captures.
//
//   c++ -std=c++17 -O2 -Wall -Wextra -I components tests/test_doppler.cpp -o /tmp/test_doppler
//   /tmp/test_doppler                      # synthetic cases
//   /tmp/test_doppler clip.wav [...]       # also every pass in each clip
//   /tmp/test_doppler --fits clip.wav      # every fit as TSV, to diff against doppler.py --fits
//
// Clips must be 24-bit PCM WAV at 48 kHz, as audio_stream serves them.

#include "doppler/doppler_dsp.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>

using namespace esphome::doppler::dsp;

static int failures = 0;
#define CHECK(cond, ...)              \
  do {                                \
    if (!(cond)) {                    \
      printf("  FAIL: " __VA_ARGS__); \
      printf("\n");                   \
      failures++;                     \
    }                                 \
  } while (0)

// Exact received signal from a moving source: each harmonic's phase is the
// source phase at the emission time, amplitude falls as 1/r.
static std::vector<float> synth_pass(double dur, const std::vector<double> &f0s, double amp_db, double v, double d,
                                     double t0, double c, std::mt19937 &rng) {
  const size_t n = (size_t) (dur * kFsIn);
  std::vector<float> y(n);
  std::uniform_real_distribution<double> ph(0, 2 * M_PI);
  std::vector<double> p0(f0s.size());
  for (auto &p : p0)
    p = ph(rng);
  const double a = pow(10.0, amp_db / 20.0);
  double lag = d / c;  // propagation time of the previous sample, as a warm start
  for (size_t i = 0; i < n; i++) {
    const double t = (double) i / kFsIn;
    double tau = t - lag;
    for (int it = 0; it < 60; it++) {
      const double x = v * (tau - t0);
      const double nt = t - sqrt(d * d + x * x) / c;
      if (fabs(nt - tau) < 1e-12) {
        tau = nt;
        break;
      }
      tau = nt;
    }
    lag = t - tau;
    const double r = sqrt(d * d + v * v * (tau - t0) * (tau - t0));
    double s = 0;
    for (size_t k = 0; k < f0s.size(); k++)
      s += a * (d / r) * sin(2 * M_PI * f0s[k] * tau + p0[k]);
    y[i] = (float) s;
  }
  return y;
}

static void add_noise(std::vector<float> &y, double db, std::mt19937 &rng) {
  std::normal_distribution<double> g(0, pow(10.0, db / 20.0));
  for (auto &v : y)
    v += (float) g(rng);
}

static std::vector<Pass> run(const std::vector<float> &x, const Params &p, std::vector<Fit> *log = nullptr,
                             Detector *keep = nullptr) {
  Detector local;
  Detector &d = keep ? *keep : local;
  if (!d.init(p)) {
    printf("  init failed\n");
    failures++;
    return {};
  }
  d.set_fit_log(log);
  d.feed(x.data(), (uint32_t) x.size());
  d.flush();
  std::vector<Pass> out;
  Pass ps;
  while (d.pop(ps))
    out.push_back(ps);
  return out;
}

static void print_pass(const Pass &g) {
  const Fit &b = g.best;
  printf("    PASS at %8.2f s  speed %5.1f m/s (%5.1f mph)  closest %6.1f m  f0", b.t_heard, b.speed,
         b.speed * 2.237, b.dist);
  for (uint32_t i = 0; i < std::min<uint32_t>(g.lines, 8); i++)
    printf(" %.1f", g.f0s[i]);
  printf(" Hz  swing %5.1f Hz  res %.2f Hz  level %6.1f dBFS  lag %+.2f T  rise %4.1f dB  [%u]\n", b.swing,
         b.res, b.level, b.level_lag, b.rise, g.lines);
}

static void synthetic() {
  Params p;
  const double c = p.sound_speed;
  std::mt19937 rng(1);
  struct Case {
    const char *name;
    std::vector<double> f0s;
    double amp_db, v, d, dur, min_t;
    bool expect;
  } cases[] = {
      // The cars are under min_t_s: found with the gate off, rejected with it on.
      {"car 25 mph @ 8 m, min_t_s 0", {96, 192, 288}, -60, 11.2, 8, 12, 0, true},
      {"car 40 mph @ 15 m, min_t_s 0", {130, 260, 650}, -62, 17.9, 15, 12, 0, true},
      {"car 40 mph @ 15 m", {130, 260, 650}, -62, 17.9, 15, 12, p.min_t_s, false},
      {"plane 60 m/s @ 400 m", {110, 220, 330, 440}, -65, 60, 400, 50, p.min_t_s, true},
      {"heli 45 m/s @ 300 m", {84, 168, 1100}, -62, 45, 300, 50, p.min_t_s, true},
  };
  for (const auto &cs : cases) {
    printf("--- %s\n", cs.name);
    auto y = synth_pass(cs.dur, cs.f0s, cs.amp_db, cs.v, cs.d, cs.dur / 2, c, rng);
    add_noise(y, -73, rng);
    Params pc = p;
    pc.min_t_s = (float) cs.min_t;
    auto passes = run(y, pc);
    for (const auto &g : passes)
      print_pass(g);
    if (!cs.expect) {
      CHECK(passes.empty(), "%s: %zu passes, expected none", cs.name, passes.size());
      continue;
    }
    CHECK(passes.size() == 1, "%s: %zu passes, expected 1", cs.name, passes.size());
    if (!passes.empty()) {
      const Fit &b = passes[0].best;
      const double ev = fabs(b.speed - cs.v) / cs.v, ed = fabs(b.dist - cs.d) / cs.d;
      printf("    speed error %.1f%%, distance error %.1f%%, lines %u of %zu\n", 100 * ev, 100 * ed,
             passes[0].lines, cs.f0s.size());
      CHECK(ev < 0.05, "%s: speed %.2f, expected %.2f", cs.name, b.speed, cs.v);
      CHECK(ed < 0.15, "%s: distance %.1f, expected %.1f", cs.name, b.dist, cs.d);
      CHECK(fabs(b.t_heard - (cs.dur / 2 + cs.d / c)) < 0.2, "%s: closest approach heard at %.2f, expected %.2f",
            cs.name, b.t_heard, cs.dur / 2 + cs.d / c);
    }
  }

  // Must be rejected, with the time-scale gate off: the other gates must catch them.
  Params q = p;
  q.min_t_s = 0;
  const size_t n = 12 * kFsIn;
  {
    printf("--- engine RPM sigmoid, constant level\n");
    std::vector<float> y(n);
    double ph[3] = {0, 0, 0};
    for (size_t i = 0; i < n; i++) {
      const double t = (double) i / kFsIn;
      const double f = 300 - 60 / (1 + exp(-(t - 6) / 0.8));
      double s = 0;
      for (int h = 1; h <= 3; h++) {
        ph[h - 1] += 2 * M_PI * h * f / kFsIn;
        s += pow(10.0, -60 / 20.0) * sin(ph[h - 1]);
      }
      y[i] = (float) s;
    }
    add_noise(y, -73, rng);
    auto passes = run(y, q);
    CHECK(passes.empty(), "RPM sigmoid accepted as %zu passes", passes.size());
  }
  {
    printf("--- spin-down from 250 Hz\n");
    std::vector<float> y(n);
    double ph[3] = {0, 0, 0};
    const int hs[3] = {1, 2, 4};
    for (size_t i = 0; i < n; i++) {
      const double t = (double) i / kFsIn;
      const double f = t < 4 ? 250 : 250 * exp(-(t - 4) / 1.0);
      double s = 0;
      for (int k = 0; k < 3; k++) {
        ph[k] += 2 * M_PI * hs[k] * f / kFsIn;
        s += pow(10.0, -55 / 20.0) * sin(ph[k]);
      }
      y[i] = (float) s;
    }
    add_noise(y, -73, rng);
    auto passes = run(y, q);
    CHECK(passes.empty(), "spin-down accepted as %zu passes", passes.size());
  }
  {
    printf("--- steady 700 Hz tone\n");
    std::vector<float> y(n);
    for (size_t i = 0; i < n; i++)
      y[i] = (float) (pow(10.0, -90 / 20.0) * sin(2 * M_PI * 700 * i / kFsIn));
    add_noise(y, -73, rng);
    auto passes = run(y, q);
    CHECK(passes.empty(), "steady tone accepted as %zu passes", passes.size());
  }
  {
    // The fit's model must invert the synthesis: a noiseless curve from the
    // model, fitted from the reference's starting point, gives back the
    // parameters it was drawn with.
    printf("--- model round trip\n");
    const float truth[4] = {440.0f, 50.0f / (float) c, 300.0f / 50.0f, 20.0f};
    std::vector<float> t(800), f(800), db(800, -60.0f);
    for (uint32_t i = 0; i < t.size(); i++)
      t[i] = (float) (i * kDt);
    model(truth, t.data(), (uint32_t) t.size(), (float) c, f.data());
    for (uint32_t i = 0; i < t.size(); i++) {
      const float s = (t[i] - 20.0f - 6.0f * 50.0f / (float) c) / 6.0f;
      db[i] = -60.0f - 10.0f * log10f(1.0f + s * s);  // 1/r^2, loudest at closest approach
    }
    FitScratch s;
    s.init(p.fit_points + 1);
    Fit fit;
    const bool ok = fit_ridge(t.data(), f.data(), db.data(), (uint32_t) t.size(), 0.0, p, s, fit);
    CHECK(ok, "round trip: no fit");
    CHECK(fit.accepted, "round trip: rejected (%s)", fit.why);
    CHECK(fabsf(fit.speed - 50.0f) < 0.2f, "round trip: speed %.3f", fit.speed);
    CHECK(fabsf(fit.dist - 300.0f) < 2.0f, "round trip: distance %.2f", fit.dist);
    printf("    speed %.3f m/s, distance %.2f m, f0 %.3f Hz, res %.4f Hz\n", fit.speed, fit.dist, fit.f0, fit.res);
  }
}

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

static void real_clip(const char *path, bool fits_tsv, float temp_c) {
  std::vector<float> x;
  float fs = 0;
  if (!read_wav24(path, x, fs) || fs != (float) kFsIn) {
    printf("could not read %s as 24-bit 48 kHz WAV\n", path);
    failures++;
    return;
  }
  Params p;
  p.sound_speed = sound_speed(temp_c);
  std::vector<Fit> log;
  Detector d;
  const auto t0 = std::chrono::steady_clock::now();
  auto passes = run(x, p, &log, &d);
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  if (fits_tsv) {
    printf("t_first\tt_last\tf0\tspeed\tdist\tres\tswing\tlag\trise\twhy\n");
    for (const auto &f : log)
      printf("%.3f\t%.3f\t%.2f\t%.2f\t%.1f\t%.3f\t%.2f\t%.2f\t%.2f\t%s\n", f.t_first, f.t_last, f.f0, f.speed,
             f.dist, f.res, f.swing, f.level_lag, f.rise, f.accepted ? "ok" : f.why);
    return;
  }
  printf("=== %s: %.0f s, processed in %.2f s on this host (%.0fx real time)\n", path, x.size() / fs, secs,
         x.size() / fs / secs);
  printf("    fits %u, ridges dropped (no slot) %u, truncated (no long buffer) %u\n", d.fits(), d.dropped_ridges(),
         d.truncated_ridges());
  for (const auto &g : passes)
    print_pass(g);
}

int main(int argc, char **argv) {
  bool tsv = false;
  float temp = 15.0f;
  std::vector<const char *> files;
  for (int i = 1; i < argc; i++) {
    if (!strcmp(argv[i], "--fits"))
      tsv = true;
    else if (!strcmp(argv[i], "--temp") && i + 1 < argc)
      temp = (float) atof(argv[++i]);
    else
      files.push_back(argv[i]);
  }
  if (!tsv)
    synthetic();
  for (const char *f : files)
    real_clip(f, tsv, temp);
  if (!tsv)
    printf("%s (%d failure%s)\n", failures ? "FAILED" : "OK", failures, failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}
