# The T-Watch S3: from the box to the first night

The strap comes off and the watch is taped to the forehead, back flat on the skin. Nothing to solder.

What it does:

- **Nudges escalate:** step 1 vibrates gently, step 2 hard, step 3 adds a soft sound, step 4 a loud one. Step 4 repeats until the head turns. Each step plays a different pattern and sound, at a slightly random time.
- **No Bluetooth:** the nights stay in its flash (15 MB, about 134 h: two weeks of nights; when full, the oldest goes). In the morning, plug it in and run `fhs dump --usb`.
- **Clock chip:** it keeps the time across reboots.
- **Button:** a short press shows the status screen (after the double-press window); a double press (two presses of at most 0.6 s, the second within 0.8 s of letting go of the first): a short buzz and the spoken zero (below); holding does nothing until 5 s, then the screen says SWITCHING OFF / KEEP HOLDING and at 10 s the watch is off (let go before: STAYING ON).

Have a Micro USB **data** cable ready (not a charge-only one).

## 1. Flash it

1. Plug the watch in. `ls /dev/ttyACM*` shows a new port (usually `/dev/ttyACM0`).
2. `firmware/build.sh /dev/ttyACM0` builds and flashes it. The screen shows the status for 5 s, then goes dark.

Or flash a release file: unpack it with `gunzip` and write it at address 0x0, e.g. `esptool --chip esp32s3 --port /dev/ttyACM0 write_flash 0x0 forehead-sensor-<version>-merged.bin`. It writes the whole flash: settings and stored nights are wiped.

## 2. Check every part

```
.venv/bin/fhs status --usb
```

The `hardware:` line must say `imu=ok`, `drv=ok`, `rtc=ok`, `pmu=ok` and `flash=15296K`. `mic=ok` shows up within a second. Anything `MISSING`: open an issue with the output.

```
.venv/bin/fhs time --usb                      # the clock (it stays set from now on)
.venv/bin/fhs buzz-test 100 1000 --usb        # vibration, full strength, 1 s
.venv/bin/fhs nudge-test 3 --usb              # step 3: vibration + soft sound (3 s)
.venv/bin/fhs nudge-test 4 --usb              # step 4: LOUD
.venv/bin/fhs nudge-test 4 3 --usb            # step 4, variant 3: another pattern and sound (variants 0-15)
.venv/bin/fhs nudge-test 4 6 --usb            # step 4, variant 6: the low 520 Hz buzz
```

**If you sleep with earplugs:** put them in, tape the watch on, lie down and run step 4 a few times (variants 0-15). The low buzz (6) usually gets through best. Vibration is the main channel either way.

Press the button briefly: the screen shows battery, mode, angle, zero, roll, hours logged and the clock.

## 3. Which way it sits on the forehead

Tape it on (or hold it) the way it will sit at night.

1. **Sitting up straight:** `fhs status --usb`. The pitch should be about +90. If it isn't, try `fhs axis 0 --usb`, `1`, `2`, `3` until sitting up gives about +90.
2. **Lying flat on the back** (positional vest or belt off): double press the button (a short buzz, then the voice). The watch talks you through it: "Zeroing. Lie flat on your back, and keep still.", "Three. Two. One.", then 3 s of silence while it measures, then "Zero set." (screen: ZERO SET). If you moved: "You moved. Let's try again." and it counts down again (3 tries, then "Zero not set."). Whatever position you are in becomes the zero. Another double press while it runs: "Cancelled." It works any time of the night; after it, no nudge for 5 min. Roll and pitch now read about 0.
   Re-zero only when the watch sits differently on the head (taped on again in another place, slipped), not for a new pillow.
3. **Lying on the left side:** roll is about +90. On the right side it's about −90.
4. **The angle:** it nudges whenever the roll is under 60 (face up, and every half-back position in between), and stops past 65. Lie on each side the way you really sleep and read the roll on the screen (short press). It must be at least 70 on both sides, or it would nudge you on your side: then lower the angle a little (`fhs set angle 55 --usb`) or try a lower pillow.

## 4. The guided position test (in bed, about 11 min)

The laptop speaks one position after the other (Piper voice, offline; without it the robotic espeak speaks); the watch, unplugged and taped on, logs them. The positions: flat with the head tipped back and an arm over it, half on the back, the head turned a little and half way to each side, each side the way you sleep (face into the pillow too), the head tipped up over the shoulder, a hand to the forehead, a quick roll over the back, sitting up. Laptop by the bed, watch on the cable.

```
.venv/bin/fhs positions
```

1. It reads any nights still on the watch first (so no test record lands in a night).
2. "Sit up straight and keep still": it finds which way the watch sits (the head axis) by itself.
3. "Unplug the watch and lie down": then it says each position; hold it until the next one (30 s).
4. The first one sets the zero: flat on the back, double press the button and follow the watch's voice until "Zero set." Then 60 s to put on a vest and eye mask if you wear them (`--vest 0` skips that), and on to the positions.
5. "Plug the watch back in": it reads the test, puts the night settings back, and prints the report: for every position the angle it read and whether it would wake you (flat and in between: must; on the side: must not). Plus whether the angle (60) sits right between your in-between positions and your side. Saved in `~/sleep-data/tests/<date>_positions/`.

Then the same with real nudges, to feel them and to measure how fast you turn (about 5 min):

```
.venv/bin/fhs positions --therapy
```

If a test is cut short, `fhs positions --restore <the folder it printed>` puts the night settings back.

## 5. Sofa test

Tape it on for an evening (e.g. sports tape over a strip of micropore; keep the speaker and mic holes clear). Check that it stays put and the skin is fine.

To feel the nudges without the night's waits (15 min to fall asleep, 5 min after sitting up):

```
.venv/bin/fhs set start_wait 0 --usb
.venv/bin/fhs set upright_pause 0 --usb
.venv/bin/fhs mode therapy --usb
```

Unplug, lie flat on the back. The night log starts after 1 min lying; 10 s later it vibrates, then harder, then it beeps, then loud. Turn to the side: it stops at once. Afterwards plug it in and put the waits back: `fhs set start_wait 900 --usb`, `fhs set upright_pause 300 --usb`.

## 6. The first nights

Start in **shadow mode** (the default) if you like: it nudges nothing and logs what it would have done, so you can check the rule against your nights first. Then:

```
.venv/bin/fhs mode therapy --usb
```

Unplug it at bedtime (on the charger it doesn't log or nudge). 1 min after lying down it buzzes twice, softly: it's logging. Three long buzzes instead: the battery is too low for a night, charge it. In the morning:

```
.venv/bin/fhs status --usb       # battery left: above ~40 % means a full night is safe
.venv/bin/fhs dump --usb         # the nights into ~/sleep-data/<date>_night/work/sensor.csv
```

Each row of `sensor.csv` is 0.5 s: time, roll and pitch (head angle), movement, mic and snoring-band loudness, and the nudge byte (which step, which variant). The columns are described at the top of `tool/fhs/store.py`.

## Settings worth knowing

| What | Command |
| --- | --- |
| how far from face up still nudges (default 60) | `fhs set angle 60 --usb` |
| seconds flat before the first nudge | `fhs set delay 10 --usb` |
| seconds between steps | `fhs set step 10 --usb` |
| vibration % per step | `fhs set levels 50,100,100,100 --usb` |
| sound volume % per step | `fhs set volumes 0,0,30,100 --usb` |
| the same pattern every time | `fhs set vary 0 --usb` |

All settings: `fhs get --usb`, and [PROTOCOL.md](PROTOCOL.md).
