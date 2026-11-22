# forehead-sensor

A DIY positional sleep trainer: a **LilyGO T-Watch S3** with its strap off, taped to the forehead. When your head stays on its back (or half way onto it), it nudges you until you turn onto your side. It also logs the whole night, so in the morning you can see what happened.

> **Not a medical device.** This is a personal hobby project, shared as is. It does not diagnose or treat anything. If you have (or think you have) sleep apnea or another sleep disorder, see a doctor. Use it at your own risk.

**How to use it: [GUIDE.md](GUIDE.md).** Setting up a new watch: [docs/WATCH.md](docs/WATCH.md).

## Why the forehead?

Positional devices usually sit on the chest or the neck. But it's the **head** that matters for the airway, and the head can lie flat while the chest is still on its side (or the other way round). A watch on the forehead measures the head itself.

## What it does

- **Every 0.5 s** it measures the head angle, head movement, mic loudness and snoring-band loudness, and logs them to its flash (15 MB: about 134 h, two weeks of nights; when full, the oldest goes).
- **Nudges that escalate:** head flat for 10 s → gentle vibration → strong vibration → vibration + soft sound → vibration + loud sound, every 10 s, repeating until the head turns. Every step plays a randomly picked pattern and sound (16 variants) at a slightly random time, so you don't get used to one buzz.
- **"Flat" includes the in-between:** by default it nudges whenever the head is within 60° of face up (`angle`), not only fully on the back. It stops once the head is past 65°.
- **Waits:** no nudge for the first 15 min (falling asleep) or for 5 min after you sit up; 60 s flat on the back ends that pause early.
- **Starts and stops by itself:** off the charger and lying down, a night session starts after 1 min (two soft buzzes). Plugged in, it neither logs nor nudges. Time of day plays no part, so naps work too.
- **Spoken zero:** a double press of the button and the watch talks you through setting "flat" for your head (lie flat, it counts down, measures 3 s of stillness, says the result).
- **Two modes:** *shadow* (logs what it would do, nudges nothing; the default) and *therapy* (it nudges).
- **No Bluetooth, no app, no cloud.** In the morning, plug it into a laptop and `fhs dump --usb` copies the nights off the flash.

## What you need

- A **LilyGO T-Watch S3** (ESP32-S3; BMA423 accelerometer, DRV2605 vibration motor, MAX98357 speaker, PDM mic, PCF8563 clock, AXP2101 power chip).
- Skin-friendly tape (e.g. a strip of micropore with sports tape over it). Keep the speaker and mic holes clear.
- A Micro USB **data** cable (not a charge-only one) and a Linux laptop (macOS may work, untested).

Nothing to solder.

## Layout

| Path | What |
| --- | --- |
| `firmware/core/` | Everything the device does, in plain C++: sensing, nudge rules and patterns, flash ring log, commands. Builds on the watch and on the laptop. |
| `firmware/twatch/` | Hardware glue for the T-Watch S3: accelerometer (FIFO), vibration, speaker, mic, clock, power + button, screen, the 15 MB log partition, USB. Light sleep between reads. `prompts.h` holds the spoken zero's phrases. |
| `firmware/host/` | Laptop build of the core: `test_core` (unit tests) and `fhsim` (the core on a simulated head, behind pseudo-terminals). |
| `firmware/build.sh` | Builds the watch firmware (`firmware/out/twatch/`). With a port it also flashes. |
| `tool/fhs/` | `fhs`, the laptop tool: `dump`, `status`, `get`/`set`, `mode`, `zero`, `buzz-test`, `nudge-test`, `positions` (a guided test: a voice says the positions, the watch logs, then a report), `voice` (makes the spoken phrases), `watch`, `bench`… |
| `docs/PROTOCOL.md` | Record and flash formats, every command and setting, the nudge variants. |
| `docs/WATCH.md` | From the box to the first night: flash, check every part, which way it sits, tests. |

## Setup (on the laptop)

- [`arduino-cli`](https://arduino.github.io/arduino-cli/) with the esp32 core (3.3.x) and the XPowersLib library (0.3.x):
  ```
  arduino-cli core install esp32:esp32
  arduino-cli lib install XPowersLib
  ```
- Python 3.10+ and a virtual env with the tool:
  ```
  python3 -m venv .venv
  .venv/bin/pip install -e tool pytest
  ```
- Optional, for the spoken position test and for changing the phrases: [Piper](https://github.com/rhasspy/piper) with the `en_GB-jenny_dioco-medium` voice in `~/.local/share/forehead-sensor/voices/`. The firmware already contains the phrases.

```
make test                          # all tests: C++ core (sanitizers) and the tool against the simulator
firmware/build.sh                  # build
firmware/build.sh /dev/ttyACM0     # build and flash over USB
```

Or skip the build: each [release](../../releases) has a ready file to flash (see its notes).

## How a night works

1. At bedtime, unplug the watch and tape it on, button toward the hairline.
2. Lie down. 1 min later, two soft buzzes: it's on duty. Three long buzzes instead: battery under ~40 %, charge it first.
3. In therapy mode it nudges as above when your head stays flat.
4. In the morning: plug it in, `fhs dump --usb`. The nights land in `~/sleep-data/<date>_night/work/sensor.csv` (one row per 0.5 s: angles, movement, sound, every nudge with its step), and the watch's clock is set from the laptop.

What you do with the CSV is up to you: a spreadsheet, a notebook, a plot of the head angle over the night.

## Decisions

- **Angle zero:** any position is taken as the new zero, so do it flat on your back. The voice makes an unnoticed zero impossible. No nudge for 5 min after it.
- **Sitting up** counts only after 3 s upright in a row: a fast roll can read "head up" for a moment.
- **Record format 2:** the old motor-level byte is the nudge byte (step + variant). Format 1 blocks still decode.

## What is checked

| Point | State |
| --- | --- |
| Core logic (angles, sensing, nudge rules, ring log, protocol, sessions, zero, power loss, cutoff) | 937 unit-test checks, under address and undefined-behaviour sanitizers |
| Laptop tool against the simulated watch (dump, live, positions, holes, power loss) | 10 tests pass |
| On a real watch | In nightly use by the author since late October 2026 |

## Known limits

- **On the nightstand it nudges:** lying face up off USB looks like a flat head, so it vibrates and beeps until turned over. Keep it on the charger, or switch it off (hold the button 10 s). `max_buzz` sets a limit.
- **No gyro:** movement comes from how fast gravity's direction turns. A twist of the watch in the forehead plane shows as a few degrees of cross-talk between roll and pitch: tape it straight.
- **A hang restarts the watch:** a watchdog restarts it after 5 s without a loop, and the night carries on at once.
- **The clock drifts a little** between dumps; `fhs dump` prints how far off it was, then sets it.
- **Leftovers:** the tool still has Bluetooth commands (`pair`, `scan`) from an earlier board. The watch has no Bluetooth, so they print one line and do nothing.

## Credits and licences

- This project: MIT, see [LICENSE](LICENSE).
- The firmware is built with the [Arduino core for the ESP32](https://github.com/espressif/arduino-esp32) and [XPowersLib](https://github.com/lewisxhe/XPowersLib), under their own licences.
- The spoken phrases in `firmware/twatch/prompts.h` were made with [Piper](https://github.com/rhasspy/piper) and the `en_GB-jenny_dioco-medium` voice, trained on the "Jenny (Dioco)" dataset, under its own licence (it asks to credit the voice as "Jenny (Dioco)").
