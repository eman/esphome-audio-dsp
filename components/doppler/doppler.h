#pragma once

#include "esphome/core/component.h"
#include "esphome/components/audio_source/audio_source.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#ifdef USE_ESP_IDF

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <atomic>

#include "doppler_dsp.h"

namespace esphome {
namespace doppler {

// Detects tonal sources passing the microphone - aircraft, helicopters,
// vehicles with a whine - and reports each pass's speed, closest distance and
// frequency. The signal processing is doppler_dsp.h; this is the plumbing.
//
// The capture task only copies samples into a ring. Everything else runs in
// this component's own task on core 1, below the capture task's priority, so
// a slow fit delays the detector and never the audio. Passes are handed to
// the main loop through a queue and published there.
class Doppler : public PollingComponent {
 public:
  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  // After the audio source, which must exist before a consumer registers.
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_source(audio_source::AudioSource *s) { source_ = s; }
  void set_frequency_range(float lo, float hi) {
    params_.f_lo = lo;
    params_.f_hi = hi;
  }
  void set_min_prominence(float db) { params_.peak_db = db; }
  void set_max_residual(float hz) { params_.max_res_hz = hz; }
  void set_min_duration(float s) { params_.min_track_s = s; }
  void set_min_rise(float db) { params_.min_rise_db = db; }
  void set_min_time_scale(float s) { params_.min_t_s = s; }
  void set_temperature_sensor(sensor::Sensor *s, bool fahrenheit) {
    temp_ = s;
    temp_f_ = fahrenheit;
  }

  void set_speed_sensor(sensor::Sensor *s) { speed_ = s; }
  void set_distance_sensor(sensor::Sensor *s) { distance_ = s; }
  void set_frequency_sensor(sensor::Sensor *s) { frequency_ = s; }
  void set_swing_sensor(sensor::Sensor *s) { swing_ = s; }
  void set_level_sensor(sensor::Sensor *s) { level_ = s; }
  void set_count_sensor(sensor::Sensor *s) { count_ = s; }
  void set_load_sensor(sensor::Sensor *s) { load_ = s; }
  void set_last_pass_text_sensor(text_sensor::TextSensor *s) { last_ = s; }

  void task_loop();  // public so the trampoline can reach it

 protected:
  // What the task hands the main loop. How long ago closest approach was heard
  // is worked out in the task, which owns the detector's clock.
  struct Item {
    dsp::Pass pass;
    float ago_s;
  };
  void push_(const int32_t *s, uint32_t n);
  void publish_(const dsp::Pass &p, float ago_s);

  audio_source::AudioSource *source_{nullptr};
  dsp::Params params_;
  dsp::Detector *det_{nullptr};

  // Raw samples from the capture task. 2^17 at 48 kHz is 2.7 s, the most a
  // fit may hold the detector up before audio is skipped.
  static constexpr uint32_t kRingBits = 17;
  static constexpr uint32_t kRing = 1u << kRingBits;
  int32_t *ring_{nullptr};
  std::atomic<uint32_t> write_pos_{0};
  uint32_t read_pos_{0};
  TaskHandle_t task_{nullptr};
  QueueHandle_t passes_{nullptr};

  sensor::Sensor *temp_{nullptr};
  bool temp_f_{false};
  std::atomic<float> sound_speed_{340.3f};

  // Busy time of the detector task against wall time, for the load sensor.
  std::atomic<uint32_t> busy_us_{0};
  int64_t load_since_us_{0};
  std::atomic<uint32_t> skipped_{0};  // samples lost to a full ring
  uint32_t count_total_{0};
  uint32_t updates_{0};
  bool low_heap_warned_{false};

  sensor::Sensor *speed_{nullptr}, *distance_{nullptr}, *frequency_{nullptr}, *swing_{nullptr}, *level_{nullptr},
      *count_{nullptr}, *load_{nullptr};
  text_sensor::TextSensor *last_{nullptr};
};

}  // namespace doppler
}  // namespace esphome

#endif  // USE_ESP_IDF
