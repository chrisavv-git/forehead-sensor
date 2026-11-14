// Forehead sensor on the LilyGO T-Watch S3 (strap off, taped to the forehead). Logs head angle, movement,
// vibration and sound every 0.5 s to its flash, and nudges when the head stays flat: vibration first, then sound,
// a different pattern every step. The night is read in the morning over USB (fhs dump --usb).
//
// setup() and loop() are in hw.cpp; the device logic is in firmware/core (tested on the laptop: firmware/host).
// Build and flash: firmware/build.sh [/dev/ttyACM0]
