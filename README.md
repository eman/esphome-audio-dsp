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
- **`peak_stability`**: the standard deviation of the per-frame peak
  frequency. This is the one that settles arguments. A tone holds its frequency
  while its level fades; noise does not. Measured on real sources: 0.2 to 1 Hz
  for a machine, 13 to 48 Hz for a lump in the noise floor. Two candidates that looked
  convincing on prominence alone were disqualified this way.

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
- Levels are dBFS, not dB SPL. Absolute SPL needs a calibrated reference.
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
