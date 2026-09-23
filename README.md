# esphome-audio-dsp

Signal-processing components for ESP32 microphones in ESPHome: frequency-domain
analysis, tonal detection, full-spectrum discovery, and live uncompressed audio
over HTTP.

Built for finding and characterizing persistent sounds (a machine whining
somewhere in the neighborhood, a transformer hum, a pump that starts at 5 a.m.)
rather than for visualizers or voice assistants.

```yaml
external_components:
  - source: github://eman/esphome-audio-dsp

audio_source:
  id: mic
  bclk_pin: 20
  ws_pin: 21
  din_pin: 22

spectral_analyzer:
  source: mic
  broadband_rms:
    name: "Broadband level"
  bands:
    - name: tone700
      f_low: 645
      f_high: 755
      peak_frequency:
        name: "Tone 700 peak"
      peak_prominence:
        name: "Tone 700 prominence"
      peak_stability:
        name: "Tone 700 stability"

audio_stream:
  source: mic
```

## Three components

| | |
|---|---|
| `audio_source` | Owns the I2S peripheral, hands blocks of samples to everything that wants them |
| `spectral_analyzer` | FFT, configurable bands, periodic discovery scan |
| `audio_stream` | Endless WAV over HTTP |

The source exists so the analyzer and the stream can share one microphone. Use
either consumer on its own, or both.

## Why not build on ESPHome's `microphone`?

`i2s_audio` hardcodes `dma_desc_num = 4` and `dma_frame_num = 256`, which is
21 ms of DMA buffering at 48 kHz. A consumer that blocks overruns it and the audio is
lost *silently*: no error, no log line, just a sample rate quietly below real
time. A 32768-point FFT takes about 114 ms, so this bites immediately. One node
ran at 85.7% of real time for a week before anyone noticed.

`audio_source` makes the DMA depth configurable and defaults it to 341 ms.
`spectral_analyzer` warns at boot if the ring cannot cover its own FFT.

## Detection: prominence and stability

Band level and SNR answer "is this band loud". Neither finds a quiet tone in a
noisy band, because both average the whole band width, so one 1.46 Hz tone in a
110 Hz band is washed out.

- **`peak_prominence`**: the strongest bin against the band's *median*, so a
  narrow tone measures itself against the noise it sits in. Typically 10 dB
  more sensitive than SNR on the same signal.
- **`peak_stability`**: the spread of the per-frame peak frequency. This is
  the one that settles arguments. A tone holds its frequency while its level
  fades; noise does not. Measured on real sources: 0.2 to 1 Hz for a machine,
  13 to 48 Hz for a lump in the noise floor. Two candidates that looked
  convincing on prominence alone were disqualified this way.

  It is a robust spread, 1.4826 times the median absolute deviation, which
  reads the same as a standard deviation for ordinary scatter and ignores the
  odd frame. It was a standard deviation first, and that failed exactly where
  it was needed: a marginal tone at 8-10 dB loses its band peak to noise in
  one frame out of seven, that frame lands 40 Hz away, and the six frames
  that held still read 15 Hz. In a night of 375 intervals the weak-tone
  branch of the detector below never fired once. Simulated at 8 dB per
  frame, the standard deviation stays under 4 Hz 9% of the time; the robust
  spread, 79%.
- **`peak_agreement`**: percent of frames whose peak sits within
  `agreement_tolerance` (default 3 Hz) of the interval's median peak. The
  blunt companion: 100 is a tone, noise reads about 2 x tolerance / band
  width.
- **`peak_level_spread`**: robust spread of the per-frame peak level, in dB.
  A steady source reads about a decibel; one that fades or throbs reads more.

A detector that uses both survives a source that fades:

```yaml
lambda: |-
  return prom > 8.0f || (prom > 4.0f && sd < 4.0f);
```

## Discovery

`spectrum_scan` walks the whole spectrum periodically and reports the strongest
tonal peaks as `freq@prominence`, with `*` marking peaks that have persisted
across scans:

```
697.7@12  219.3@12  211.2@11  553.5@10*  1203.7@9
```

Prominence is measured against a *local* median computed in ~150 Hz blocks, so
a faint whine in a quiet region outranks loud broadband hiss. This is how you
find what to point a band at.

Persistence counts are carried on the full candidate list (up to 64 peaks
over `min_prominence`), not just the `top_n` that get reported, so a steady
source that a passing car pushed out of the top eight for one scan is still
`*` when it comes back. Peaks are matched to the previous scan within 1.5% of
their frequency (at least 6 Hz), which covers a drifting source; a fixed
few-bin tolerance undercounted a 700 Hz line that wanders more than that.

## Harmonics

A machine rarely makes one line. A motor, transformer, pump or fan has a
series at 2x, 3x, 4x its fundamental; a whistle, a resonance or a tuned
alarm mostly does not. `harmonics` measures that series at order x the band's
own peak. Orders below 1 look under it: a fan's blade-pass tone sits over a
shaft line at f0 / blades, so `order: 0.2` on a 700 Hz line tests for a
5-blade fan at 140 Hz, and a gated search there is more sensitive than the
discovery scan is down in the rumble.

```yaml
    - name: tone700
      f_low: 645
      f_high: 755
      harmonics:
        tolerance: 2.0        # search +/- order x 2 Hz around order x f0
        min_prominence: 6     # fundamental must be a tone first
        orders:
          - order: 2
            relative_level:
              name: "Tone 700 H2 relative"   # dB, H2 power over H1's
            prominence:
              name: "Tone 700 H2 prominence" # dB over the floor at 2 x f0
            frequency:
              name: "Tone 700 H2 frequency"
```

Two details carry the measurement:

- **Power is summed across the window, above the local floor**, rather than
  read off the peak bin. A fundamental wandering 1 Hz inside an interval
  smears its 4th harmonic across 4 Hz; the peak bin would read that as the
  harmonic being weaker, which is a fact about the drift, not the source.
- **Nothing is published unless the fundamental clears `min_prominence`**:
  the sensors go unknown instead. Harmonics of a noise peak are noise, and a
  graph of them would look like a harmonic series whenever the band is empty.
  A relative level of -99 dB means the fundamental was there and that
  harmonic was not.

## Zoom FFT

Frequency resolution is 1 / frame length, whatever the method. The main FFT
pays for it in memory: 0.077 Hz bins across the whole spectrum would need
2^20 points and 12 MB. A `zoom` pays for one band only: mix the band down to
0 Hz, low-pass and decimate, then run a small complex FFT on the slow signal.

```yaml
    - name: tone1200
      f_low: 1145
      f_high: 1255
      zoom:
        fft_size: 2048         # 0.077 Hz bins over this 110 Hz band
        peak_frequency:
          name: "Tone 1200 zoom peak"
        peak_prominence:
          name: "Tone 1200 zoom prominence"
```

| `fft_size` over a 110 Hz band at 48 kHz | bins | frame | result every | memory |
|---|---|---|---|---|
| 1024 | 0.154 Hz | 6.5 s | 3.3 s | ~62 kB |
| 2048 | 0.077 Hz | 13.0 s | 6.5 s | ~70 kB |
| 4096 | 0.038 Hz | 26.0 s | 13.0 s | ~86 kB |

The band sits in the middle 70% of the decimated rate; the rest is the
anti-alias filter's transition, a Blackman windowed sinc of ~18 x the
decimation. Tested on a tone 50 dB louder than the band's signal, placed
exactly where the decimator folds onto it: it did not come through.
Frequency error on a steady synthetic tone is about 0.001 Hz.

What it costs is time, and that is the thing to decide on:

- **A tone that moves more than a bin inside one frame smears.** The source
  this was written for drifts about 0.2 Hz/s, so 0.15 Hz bins are already as
  fine as it supports. A steady source (another one in the same capture held
  1176.93 Hz to +/-0.01 Hz) can use far more.
- **Single-frame prominence has a high noise baseline.** It is one frame, not
  an average, over hundreds of bins, so the largest noise bin alone reads
  roughly 10-13 dB above the median. Judge a tone by a peak that repeats from
  frame to frame, not by one frame's prominence.
- **Frames are longer than `update_interval`**, so a zoom publishes once per
  frame, when it lands, and most intervals publish nothing. Frames overlap by
  half. The config validator refuses a frame over 60 s.

### Two lines, and the beat between them

A zoom frame also reports the **second strongest separate line** in the band
(`second_peak_frequency`, `second_peak_prominence`, at least four bins from
the first) and the **periodic modulation of the band's envelope**
(`modulation_frequency`, `modulation_prominence`, `modulation_depth`,
searched over `modulation_f_low`..`modulation_f_high`, default 0.5-5 Hz).

The decimated baseband is the band's analytic signal, so its power is the
envelope, and the spectrum of that envelope over the frame is where a throb
shows up. This is the test for two similar machines running side by side:
two tones df apart beat at exactly df, so two lines whose separation equals
the modulation frequency, frame after frame, is beating. One source gives one
line, a second line in the noise at a random frequency each frame, and a
modulation peak of a few dB at a random frequency.

Simulated at a main-FFT prominence of 14 dB, a real node's overnight
operating point, with the pair drifting independently:

| | second line | modulation |
|---|---|---|
| pair 1.6 Hz apart | 1.4-1.8 Hz from the first, every frame | at the separation, 11-17 dB |
| one tone | 50 Hz away, then 36, then 27 | 4-7 dB, random frequency |

`modulation_depth` is the modulation index of a sinusoidal power modulation,
in percent: 100 for two equal tones. It reads low when the beat rate itself
changes inside a frame, which independently drifting tones do, so treat it
as a floor.

```yaml
      zoom:
        fft_size: 1024
        second_peak_frequency:
          name: "Tone 700 zoom second peak"
        modulation_frequency:
          name: "Tone 700 modulation"
        modulation_prominence:
          name: "Tone 700 modulation prominence"
```

Zooms follow their band when it is retuned at runtime. The filter runs in the
capture task at about 19 multiply-adds per input sample for a 110 Hz band,
and needs no input history: each sample is added into the few outputs whose
windows it falls in.

## Testing on a host

`components/spectral_analyzer/dsp.h` holds the signal processing with no
ESPHome in it. `tests/test_dsp.cpp` checks it against synthetic signals with
known answers, and optionally a real capture:

```sh
c++ -std=c++17 -O2 -I components tests/test_dsp.cpp -o /tmp/test_dsp
/tmp/test_dsp                 # synthetic
/tmp/test_dsp clip.wav        # plus a 24-bit capture from audio_stream
```

## Retuning bands at runtime

ESPHome fixes its entity list at compile time, so a band cannot be **added**
while running: there would be no sensors to publish it on. What works, and is
what discovery actually needs, is declaring spare bands in YAML and aiming them
wherever the scan points. Changes are saved to flash and survive a reboot.

```yaml
api:
  actions:
    - action: retune_band
      variables: {band: string, f_low: float, f_high: float}
      then:
        - spectral_analyzer.set_band:
            band: !lambda "return band;"
            f_low: !lambda "return f_low;"
            f_high: !lambda "return f_high;"
```

Also `spectral_analyzer.reset_band` and `spectral_analyzer.reset_all_bands`.
Saved ranges are dropped if the band list in YAML changes, rather than being
applied to the wrong bands.

## Streaming

`audio_stream` serves an endless uncompressed WAV. ffmpeg, sox, VLC, Audacity
and go2rtc all open the URL directly.

```sh
ffmpeg -i http://node.local:8080/audio.wav -t 30 -c copy clip.wav   # analyze
ffplay 'http://node.local:8080/audio.wav?gain=45'                   # listen
```

**`?gain=` is required for listening and wrong for analysis.** A measurement
microphone streams what it measures: an ICS-43434 puts 94 dB SPL at -26 dBFS,
so ordinary outdoor ambient arrives near -73 dBFS, using 14 of 24 bits. That is
the correct number and it is inaudible in every player, so the audio sounds
broken when it is not. `?gain=<dB>` scales on the way out, saturating rather
than wrapping so a loud event clips instead of becoming a full-scale click.
Leave it off for anything you intend to measure.

The capture path never waits on a socket: samples go to a ring buffer, and a
client that cannot keep up loses audio and logs how much.

The ring holds the WAV payload itself, 24-bit little-endian, packed once per
block in the capture task. A client that takes the stream as-is (no `?gain=`,
24-bit) sends straight out of the ring, at most two `send()` calls per chunk
and no conversion; only a client that asks for gain or a 16-bit stream walks
the samples, into a buffer in internal RAM. Clients sleep on a task
notification that each block gives, so nothing polls.

## Streaming over WiFi needs a bigger TCP window

lwIP's default send buffer in ESPHome is 5744 bytes. Throughput over TCP is
bounded by window divided by round trip time, so on a link with 37 ms RTT that
is a hard ceiling of about 1.2 Mbit/s no matter how good the radio is - and a
48 kHz 24-bit stream wants 1.15 Mbit/s. Measured on an ESP32-S3 over WiFi,
raising the window took delivery from 65% of real time to 81%:

```yaml
esp32:
  framework:
    type: esp-idf
    sdkconfig_options:
      CONFIG_LWIP_TCP_SND_BUF_DEFAULT: "65534"
      CONFIG_LWIP_TCP_WND_DEFAULT: "65534"
      CONFIG_LWIP_TCP_MSS: "1440"
```

Raising the WiFi driver's own TX buffers and block-ack windows on top of that
made no further difference on the same link, so reach for the TCP settings
first.

Two other things that cost real time on a constrained node, both measured:

- **A heavy analyzer starves the stream.** With `fft_size: 32768` and the FFT
  buffers in PSRAM, an S3 delivered 86% of real time; with the analyzer off,
  97%. If a node's job is to stream, give it no analyzer, or a small one.
- **Bitrate is not always the problem.** On that link, cutting 1.15 Mbit/s to
  0.26 changed delivery from 73% to 86% - nothing like proportional, because
  the limit was elsewhere. Measure before trading away sample rate or bits.

## `use_apll` defaults to true here

ESPHome's `i2s_audio` offers the same option and defaults it to false. For
measurement that default is wrong, and the symptom is memorable: on the node
this was written for, the microphone could **hear the FFT**. A hiss pulsing at
exactly the analysis frame rate, 41 dB above the envelope floor in 4 to 12 kHz,
which followed the frame rate when `fft_size` was halved.

No audio was being lost; capture ran at 99.93% of real time. A delta-sigma
microphone puts bit-clock jitter straight into its noise floor, and the
general-purpose PLL is perturbed by whatever else the chip is doing. APLL is
the dedicated audio PLL and divides to 3.072 MHz exactly for 48 kHz x 64.

| | 1.46 Hz modulation |
|---|---|
| default PLL, FFT buffers in PSRAM | 41.2 dB |
| default PLL, FFT buffers in internal SRAM | 32.9 dB |
| **APLL** | **14.9 dB** |

The noise floor improved 2 to 3 dB as well.

## Conventions

- A full-scale sine reads **-3.01 dBFS**, the RMS convention.
- Levels are dBFS unless `level_offset` is set. It is added to the broadband,
  floor and band levels only; ratios are unchanged. An ICS-43434 reads
  -26 dBFS at 94 dB SPL, so `level_offset: 120` publishes dB SPL through the
  microphone's nominal sensitivity, which is within a couple of dB of a
  calibrated reference and nowhere near it above 2 kHz behind a port.
- Bands of equal width give comparable stability figures; a peak needs room to
  wander before the band edges clip it.

## Hardware

Any I2S MEMS microphone that emits 24 bits left-justified in a 32-bit slot:
ICS-43434, INMP441, SPH0645. Developed on an ESP32-P4 with an ICS-43434.

`fft_size: 32768` needs about 384 kB; PSRAM is recommended, and the FFT working
buffers are placed in internal SRAM when they fit, because the memory traffic
is itself a noise source.

## License

MIT.
