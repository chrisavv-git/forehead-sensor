# Protocol and formats

The source of truth is `firmware/core/format.h`; `tool/fhs/proto.py` mirrors it. Everything is little-endian.

## Record (16 bytes, one per 0.5 s window)

| Offset | Type | Field | Unit / meaning |
| --- | --- | --- | --- |
| 0 | int16 | roll | 0.01°. 0 = face up; + = left ear down (lying on the left); ±180 = face down |
| 2 | int16 | pitch | 0.01°. + = top of the head up; 90 = upright |
| 4 | uint16 | move | 0.1 °/s: how fast gravity's direction turns, RMS over 50 ms steps (the watch has no gyro; with a gyro it would be the gyro's RMS) |
| 6 | uint16 | buzz | 0.1 mg: accel high-passed above 80 Hz, magnitude RMS |
| 8 | uint16 | mic | mic RMS, raw PCM (DC removed) |
| 10 | uint16 | snore | mic 100–600 Hz band RMS, raw PCM |
| 12 | uint8 | nudge | bits 4–6: step (0 none, 1–2 vibration, 3–4 vibration + sound, 5 a test, 6 the bedtime buzz when a session starts: variant 0 on duty, 1 battery low; 7 the spoken zero talking: variant = the phrase, `zeroguide.h`); bits 0–3: variant 0–15 (the vibration pattern + sound it played). Bit 7 set: shadow (what the rules would do; nothing ran). Format 1: bits 0–6 were a motor level 0–100 % |
| 13 | uint8 | flags | 1 upright, 2 on USB, 4 low battery, 8 the button asked for the spoken zero (double press), 16 zero set, 32 clock set, 64 short (motion samples missing), 128 no mic |
| 14 | uint16 | seq | low 16 bits of the record number in this boot |

Record `seq` of a boot covers `[t0 + seq·0.5 s, t0 + (seq+1)·0.5 s)`, where `t0` is the unix time of record 0 of that boot. A stalled loop still advances `seq` (with empty, "short" windows), so `seq` is always time.

## Flash: 3824 blocks of 4 KB (15 MB), used as a ring

Each block has a 48-byte header, then 253 records at offset 48. Records are 16-byte aligned, so none crosses a 256-byte page.

| Offset | Type | Field |
| --- | --- | --- |
| 0 | uint32 | magic `FHS1` (0x31534846) |
| 4 | uint32 | block_no: counts up forever |
| 8 | uint32 | boot_id: counts up on every power-up; never reused, even after an erase |
| 12 | uint32 | first_seq |
| 16 | int64 | t_seq0_ms: unix ms of record 0 of this boot (0 = clock not set yet) |
| 24 | uint16 | battery mV |
| 26 | 3×int16 | angle zero: gravity in board axes, 1/10000 g |
| 32 | 6×uint8 | format version (3; 1 and 2 are still read, their roll sign flipped: before format 3 the firmware had + = right ear down), record size (16), records per block (253), mode, head axis, 0xFF |
| 38 | uint16 | CRC16-CCITT of bytes 0–37 |
| 40 | uint16 | record count, written when the block closes (0xFFFF = never closed: count the records whose seq matches) |

How a block is written:

1. When the block opens: erase the sector, then write the header.
2. While it fills: records are written a 256-byte page at a time (16 records = 8 s). Each page gets at most 3 writes.
3. When it closes: write the count.

A power loss loses at most the last 8 s.

**Time on the laptop.** A boot's `t0` comes from its newest block that has the clock set. A boot that never had its clock set is placed right after the boot before it (or right before the next one), and its rows get `src = flash-guess`.

## Commands (text lines, `\n`)

USB is trusted. Over Bluetooth (Nordic UART service), everything except `hello` and `auth` needs `auth <key>` first. `key` only works over USB. Replies end with `OK <command> …` or `ERR …`.

| Command | Reply / effect |
| --- | --- |
| `hello` | `OK hello fw=… boot=… auth=0/1` |
| `auth <16 hex>` | `OK auth` / `ERR auth` |
| `key` | (USB) `OK key <16 hex>`: the Bluetooth key, made on the first boot |
| `status` | `S fw= boot= seq= mode= batt= usb= chg= session= cutoff= clock= t0= tz= blocks= sectors= oldest= next= open= err=`, then `S hw imu= imu_fail= ovr= quiet= flash= mic= ...` (`quiet`: times the motion sensor was set up again after 3 s without a sample), then `S roll= pitch= flat_s= step= variant= imu= mic= zero= zero_set= axis= dropped= stalls=` (`stalls`: times this link was given up as not read, below), then `OK status` |
| `time <unix ms> [tz min]` | sets the clock (the watch's clock chip too, and keeps tz): `OK time t0=… seq=… boot=…` |
| `get` | `P angle=… hyst=… … mode=… axis=…`, then `OK get` |
| `set <key> <value>` | `OK set key=value` (saved), or `ERR set key` if out of range |
| `mode shadow\|therapy` | saved |
| `axis 0-3` | the board axis that points to the top of the head: 0 +X, 1 −X, 2 +Y, 3 −Y |
| `zero` | lying flat on the back and still for 2 s: this is 0° |
| `buzz <1-100> <1-5000 ms>` | our motor, now (works in shadow mode too) |
| `diag` | the vibration driver's self-test: `OK diag motor ok` (or `FAIL (open or shorted)`), `supply <V>, status 0x..` |
| `nudge <step 1-4> <variant 0-15> <1-10000 ms>` | one nudge step as therapy plays it, with its sound (works in shadow mode too) |
| `stop` | ends a buzz or nudge test, streams and raw mode on this link |
| (no command) | bytes waiting on a link and none taken for 5 s: nobody is reading (the port was closed; the watch cannot see that itself). The running `since`/`dump`/`blocks`, `live` and raw mode stop and the waiting bytes are dropped. Ask again after reconnecting. |
| `live on\|off` | `R <boot> <seq> <32 hex>` for each new record |
| `since <boot> <seq> [live]` | records from the flash from there on, then `C <boot> <next seq>` (and live, if asked) |
| `blocks` | `I <sector> <block_no> <boot> <first_seq> <n> <t0 ms> <open>` per block, oldest first, then `OK blocks n=` |
| `dump [from block_no] [count]` | `B <block_no> <offset> <512 hex>`, 16 lines per block, then `OK dump n=` |
| `raw imu\|mic\|off` | (USB) `X <seq> <hex>`: 20 samples of ax ay az gx gy gz int16 (0.977 mg per unit; the gyro values are 0). `M <seq> <hex>`: 128 mic samples at 16 kHz |
| `session` | restarts the start wait |
| `erase yes` | clears the log (the counters keep counting) |
| `reboot` | |

Event lines the device sends on its own: `N <text>`, for example `N zero set by tap`. The spoken zero sends `N zero: started`, `N zero: moved, try 2 of 3` (and 3 of 3), `N zero: not set (moved)`, `N zero: cancelled`, then `N zero set by tap` or `N zero rejected (...)`.

## Settings (`fhs get` / `fhs set`)

| Key | Default | Range | Meaning |
| --- | --- | --- | --- |
| angle | 60 | 5–90 | flat = \|roll\| under this (and not upright): face up and every half-back position in between (on the side is ~90) |
| hyst | 5 | 0–20 | stays flat until \|roll\| > angle + hyst |
| upright | 60 | 30–85 | pitch above this = sitting up |
| delay | 10 | 0–600 s | flat this long before step 1 |
| step | 10 | 5–600 s | the next step after this long; step 4 repeats (a new variant each time) until the head turns |
| start_wait | 900 | 0–7200 s | no nudge this long after a night session starts |
| upright_pause | 300 | 0–3600 s | no nudge this long after sitting up (3 s head up in a row), but never past 60 s flat on the back |
| levels | 50,100,100,100 | 0–100 each | vibration % in steps 1–4 |
| volumes | 0,0,30,100 | 0–100 each | sound volume % in steps 1–4 (0 = silent) |
| jitter | 3 | 0–30 s | the delay and every step move by up to this much, at random |
| vary | 1 | 0/1 | 1: a random variant every step, never the same twice in a row; 0: always variant 0 (a steady pulse) |
| max_buzz | 0 | 0–3600 s | give up after this much nudging in one flat stay (0 = never) |
| tap_zero | 1 | 0/1 | a double press of the button starts the spoken zero |
| still | 15 | 0.5–50 °/s | still enough for the zero (4 refused every bedtime zero on 30 Oct 2026) |
| log_usb | 0 | 0/1 | also log while on USB power |
| win_from / win_to | 0 / 0 | 0–23 h | with the clock set, only log between these hours (equal: always, the default) |
| lie_gate | 1 | 0/1 | only log while lying (session: starts after 2 min lying, ends after 20 min not lying) |
| low_mv / cutoff_mv | 3500 / 3300 | mV | low-battery flag; below the cutoff (off USB) logging and nudging stop, until 100 mV above it |
| night_mv | 3800 | 3300–4200 mV | when a session starts, two short soft buzzes (on duty), or three long ones if the battery is below this (~40 %: charge it first) |
| mic_duty | 25 | 10–100 % | how much of every 0.5 s window the mic listens (the chip sleeps the rest) |

## Nudge variants

A variant is a (vibration pattern, sound) pair: vibration `v % 8`, sound `(3v + v/8) % 8`, except that variants 5 and 10 (the name clip until 0.0.2) play the warble and the fast beeps, so all 16 differ. The patterns loop for as long as the step lasts (`firmware/core/therapy.cpp`).

| Pattern | Vibration (v % 8) | Sound ((3v + v/8) % 8) |
| --- | --- | --- |
| 0 | steady pulse 0.5 s on / off | double beep, 2 kHz |
| 1 | fast pulse 0.15 s | rising chirp 0.6 → 2.5 kHz |
| 2 | double knock | low buzz: 520 Hz, three 0.5 s beats (it wakes deep sleepers best) |
| 3 | triple knock | siren 0.9 / 1.3 kHz |
| 4 | ramp up 20 → 100 % | fast beeps, 3 kHz |
| 5 | long 1.2 s | warble 1.0 / 1.5 kHz |
| 6 | heartbeat | low-high 0.7 / 2.8 kHz |
| 7 | wobble 40 / 100 % | - |
