# User guide: the forehead watch

> **Not a medical device.** A hobby project, shared as is. If you have a sleep disorder, see a doctor.

## What it does

The watch sits on your forehead at night. When your head turns onto your back, or half way onto it, for 10 seconds, it wakes you. It starts gentle and gets louder until you turn onto your side:

1. a gentle vibration
2. a strong vibration
3. vibration and a soft sound
4. vibration and a loud sound, again and again until you turn

There are 10 seconds between steps. Every step uses a different pattern, so you don't get used to it. On your side it stays quiet.

It also records the whole night (head angle, movement, sound). In the morning the laptop reads it.

## Every day

| When | What you do |
| --- | --- |
| Day | Keep it **on the charger**. On the charger it never records or nudges. |
| Bedtime | **Unplug it**, tape it on, lie down on your side. |
| 1 min later | **Two soft buzzes** = it's on duty. Go to sleep. |
| Morning | **Plug it in**, and run `fhs dump --usb` on the laptop. |

You don't need to switch it on or off. It starts by itself when you lie down off the charger, and stops when you get up or plug it in. The time of day doesn't matter, so naps work the same way.

For the first 15 minutes it doesn't nudge, so you can fall asleep.

## What the buzzes mean

| You feel | Meaning |
| --- | --- |
| 2 short soft buzzes, 1 min after lying down | On duty: it's recording and watching |
| 3 long buzzes, 1 min after lying down | Battery low (under about 40 %): charge it before you sleep |
| Nothing, 1 min after lying down | Something is wrong: still on the charger, switched off, or the battery is empty |
| Vibration, getting stronger, then sound | You're on your back or half way: turn onto your side |
| A voice after a double press | It's setting "flat": do what it says (see below) |

## The button

| Press | What it does |
| --- | --- |
| Short | Lights up the screen (battery, mode, angle). Also switches it on if it was off. |
| Double press (two quick presses), lying flat on your back | A short buzz, then sets "flat" with a voice (see below). Double press again while it talks: cancels |
| Hold 10 s | Switches it off: at 5 s the screen says SWITCHING OFF / KEEP HOLDING, at 10 s it goes dark. Let go before 10 s: STAYING ON |

**Tape it with the button pointing up, toward your hairline.** Then the pillow can't press it.

## Setting "flat"

The watch needs to know once what "flat on your back" looks like for you. If you wear a positional vest or belt, take it off first: with it you can't lie flat, and the watch would learn the wrong position.

1. Lie flat on your back, head straight.
2. Double press the button (two quick presses). It buzzes once, and says: **"Zeroing. Lie flat on your back, and keep still."**
3. It counts **"Three. Two. One."**, then is silent for 3 s while it measures. Keep still.
4. **"Zero set."**: done. The watch stays quiet for 5 min while you settle in.

What else it can say:

| It says | Meaning |
| --- | --- |
| "You moved. Let's try again." | It counts down again by itself (3 tries) |
| "Zero not set." | 3 tries with movement: the old setting stays. Double press to start again |
| "Cancelled." | You double pressed again while it talked |

Do it again only when the watch sits differently on your head (taped on in another place, or it slipped). A new pillow doesn't need it. It stays saved, also when the watch is off.

## First time

The full steps are in [docs/WATCH.md](docs/WATCH.md). In short:

1. Flash it: `firmware/build.sh /dev/ttyACM0` (or the file from a release)
2. Check it: `.venv/bin/fhs status --usb`. Everything should say `ok`.
3. Feel the nudges: `.venv/bin/fhs nudge-test 1 --usb` (up to `4`, which is LOUD)
4. Do the position test (below).
5. Switch on the nudging: `.venv/bin/fhs mode therapy --usb`. Until then it only records what it would do.

All commands run in the `forehead-sensor` folder.

## The position test (about 12 min, in bed)

```
.venv/bin/fhs positions
```

The laptop's voice tells you what to do: sit up, unplug, lie flat and set "flat", then about 20 positions, 30 s each. Then plug the watch back in, and it prints a report: for every position, whether the watch got it right. It must nudge when you're flat or half way, and stay quiet on your side.

Then once more with real nudges, to feel them and see how fast you turn:

```
.venv/bin/fhs positions --therapy
```

## In the morning

```
.venv/bin/fhs dump --usb
```

This copies the nights to the laptop (`~/sleep-data/<date>_night/work/sensor.csv`, or `--root <folder>`) and sets the watch's clock. It shows how many hours it read.

## Battery

- Charge it every day. A full charge lasts a night with room to spare.
- Below about 40 % at bedtime it warns you with 3 long buzzes.
- If the battery runs empty in the night, it stops cleanly. The night so far is kept.

## Away from home

The watch can't tell your head from a table or a bag: lying face up, it looks like you on your back, and it would nudge and beep. So:

| Where the watch is | What you do |
| --- | --- |
| On your head | nothing, it works |
| Next to a charger | on the charger |
| In a bag, pocket, or on a table without a charger | **switch it off** (hold the button 10 s) |

- Bedtime: short press to switch it on, tape it on, lie down. 1 min later, two soft buzzes. **No buzz: you probably forgot to switch it on.**
- Nothing is lost when it's off: settings, "flat" and the nights stay saved.
- Time zones don't matter. It stores about 2 weeks of nights; when full, it overwrites the oldest.
- Back home, read it with the laptop set to the time zone you slept in, then set the clock back:

```
TZ=Asia/Bangkok .venv/bin/fhs dump --usb
.venv/bin/fhs time --usb
```

## From an Android phone (no laptop)

You can talk to the watch from a phone. **Try this at home first.**

You need a USB-C to Micro USB OTG cable, and a serial terminal app, e.g. **Serial USB Terminal** (by Kai Morich).

1. Plug the watch into the phone. It stops recording, like on a charger.
2. Open the app and pick the watch from the device list. If it asks for a driver type, choose **CDC**.
3. Type a command and send it. The watch answers `OK ...`.

| Command | What it does |
| --- | --- |
| `status` | battery, angles, mode |
| `get` | all settings |
| `set angle 55` | nudge only when you're closer to flat (if it nudges you on your side) |
| `mode therapy` | nudging on |
| `mode shadow` | nudging off (it only records) |

## If something is wrong

| Problem | What to do |
| --- | --- |
| No buzz 1 min after lying down | Short press: does the screen light? If not, it's off or empty: charge it. |
| 3 long buzzes at bedtime | Charge it before sleeping. |
| It nudges while you're on your side | The pillow is probably too high: try a lower one, or `fhs set angle 55 --usb`. |
| It beeps on the table | Off the charger, lying face up, it thinks you're flat. Keep it on the charger. |
| "Flat" set in the wrong position (it takes whatever position you're in) | Lie flat on your back and set it again. |
| `status` says `MISSING`, or the screen stays dark | A part doesn't answer: open an issue with the `fhs status --usb` output. |

## Quick tests: no waiting

Normally it waits 15 min after you lie down, and 5 min after you sit up, before it nudges. For your own tests, switch both waits off:

```
.venv/bin/fhs set start_wait 0 --usb
.venv/bin/fhs set upright_pause 0 --usb
```

Then it nudges 10 s after you lie flat (recording still starts 1 min after lying down). **Before the night, put them back:**

```
.venv/bin/fhs set start_wait 900 --usb
.venv/bin/fhs set upright_pause 300 --usb
```

From the phone: the same without `.venv/bin/fhs` and `--usb`, e.g. `set start_wait 0`. The position test doesn't need this: it does it by itself.

## Settings worth knowing (laptop, watch plugged in)

| Setting | Command |
| --- | --- |
| How far from flat still nudges (60) | `fhs set angle 60 --usb` |
| Seconds flat before the first nudge (10) | `fhs set delay 10 --usb` |
| Minutes to fall asleep (15) | `fhs set start_wait 900 --usb` (in seconds) |
| Nudging on / off | `fhs mode therapy --usb` / `fhs mode shadow --usb` |
| See everything | `fhs status --usb` and `fhs get --usb` |

All the details: [docs/WATCH.md](docs/WATCH.md) and [docs/PROTOCOL.md](docs/PROTOCOL.md).
