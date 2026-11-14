"""The watch's spoken zero (fhs voice): its phrases in the Piper voice the guided test speaks with. They become
firmware/twatch/prompts.h (16 kHz, 8 bit), made loud for the watch's small speaker: the lows it can't play are cut,
then they are compressed and normalised, so every syllable comes out near full scale. Build and flash after."""
import os
import shutil
import subprocess
import sys
import tempfile
import wave

import numpy as np

from .positions import VOICE, VOICES

RATE = 16000        # the speaker's sample rate (hw.cpp SPK_RATE)
PROMPTS_HEADER = os.path.join(os.path.dirname(__file__), '..', '..', 'firmware', 'twatch', 'prompts.h')
# the spoken zero's phrases, in the order of enum Prompt (firmware/core/zeroguide.h), with their longest length:
# the count must fit its 1 s steps
PROMPTS = [
    ('Zeroing. Lie flat on your back, and keep still.', 4.0),
    ('Three.', 0.9),
    ('Two.', 0.9),
    ('One.', 0.9),
    ('Zero set.', 2.0),
    ("You moved. Let's try again.", 3.0),
    ('Zero not set.', 2.0),
    ('Cancelled.', 1.5),
    ('Switching off.', 2.0),
]


def read_wav(path):
    """(samples -1..1 as float, rate) of a 16-bit PCM WAV, mixed to mono"""
    with wave.open(path) as w:
        if w.getsampwidth() != 2:
            raise ValueError('%s: only 16-bit WAV files' % path)
        x = np.frombuffer(w.readframes(w.getnframes()), dtype='<i2').astype(float) / 32768
        x = x.reshape(-1, w.getnchannels()).mean(axis=1)
        return x, w.getframerate()


def speak(text, length_scale=1.0):
    """the text in the Piper voice, as (samples, rate)"""
    model = os.path.join(VOICES, VOICE + '.onnx')
    piper = os.path.join(os.path.dirname(sys.executable), 'piper')
    if not (os.path.exists(model) and os.path.exists(piper)):
        raise FileNotFoundError('no Piper voice (%s): give a recording with --wav' % model)
    with tempfile.NamedTemporaryFile(suffix='.wav') as f:
        subprocess.run([piper, '-m', model, '-f', f.name, '--length-scale', str(length_scale)],
                       input=text.encode(), capture_output=True, check=True)
        return read_wav(f.name)


def loud(x, rate):
    """resampled to RATE, silence trimmed, lows cut, compressed and normalised to -1..1"""
    x = np.interp(np.arange(0, len(x) * RATE / rate) * rate / RATE, np.arange(len(x)), x)
    # cut below ~400 Hz: the speaker can't play them, and they would only take headroom from the rest
    spec = np.fft.rfft(x)
    f = np.fft.rfftfreq(len(x), 1 / RATE)
    spec *= np.clip((f - 250) / 250, 0, 1)
    x = np.fft.irfft(spec, len(x))
    # trim: from the first to the last 10 ms that has sound in it
    frames = np.abs(x[:len(x) // 160 * 160]).reshape(-1, 160).max(axis=1)
    on = np.nonzero(frames > frames.max() * 0.05)[0]
    x = x[on[0] * 160:(on[-1] + 1) * 160]
    # compress: even out the syllables (a slow gain from the 20 ms envelope), then soft-clip the peaks
    env = np.convolve(np.abs(x), np.ones(320) / 320, mode='same')
    x = x / np.maximum(env, env.max() * 0.1)
    x = np.tanh(2.5 * x / np.percentile(np.abs(x), 99))   # 2.5: hard enough to be loud, the words still clear
    return x / np.abs(x).max()


def write_prompts(clips, path=PROMPTS_HEADER):
    """clips: [(text, samples -1..1)] in the order of enum Prompt"""
    with open(path, 'w') as f:
        f.write('// The spoken zero\'s phrases (zeroguide.h enum Prompt), 16 kHz, 8 bit.\n')
        f.write('// Made by fhs voice (tool/fhs/voice.py): don\'t edit, make it again.\n')
        f.write('#pragma once\n#include <stdint.h>\n\n')
        for i, (text, x) in enumerate(clips):
            pcm = np.round(x * 127).astype(np.int8)
            rows = [', '.join(str(v) for v in pcm[k:k + 24]) for k in range(0, len(pcm), 24)]
            f.write('// %d: "%s", %.2f s\nstatic const int8_t PROMPT_%d[%d] = {\n  %s};\n' %
                    (i, text, len(pcm) / RATE, i, len(pcm), ',\n  '.join(rows)))
        f.write('\nstruct PromptClip { const int8_t* pcm; uint32_t n; };\n')
        f.write('static const PromptClip PROMPT_CLIPS[%d] = {\n%s};\n' %
                (len(clips), ''.join('  {PROMPT_%d, sizeof PROMPT_%d},\n' % (i, i) for i in range(len(clips)))))


def make_prompts(args):
    """every phrase of the spoken zero in the Piper voice, made loud; played on the laptop to
    check, then written to firmware/twatch/prompts.h"""
    clips = []
    for text, most in PROMPTS:
        x = loud(*speak(text, args.slow))
        secs = len(x) / RATE
        print('%5.2f s  %s' % (secs, text))
        if secs > most:
            print('fhs: "%s" is %.2f s, the most is %.1f: fewer words, or --slow 0.9' % (text, secs, most), file=sys.stderr)
            return 1
        if not args.no_play:
            play(x)
        clips.append((text, x))
    total = sum(len(x) for _, x in clips)
    if args.dry:
        print('%.1f s, %d KB (not saved)' % (total / RATE, total // 1024))
        return 0
    write_prompts(clips)
    print('%.1f s, %d KB -> %s\nnow build: firmware/build.sh' % (total / RATE, total // 1024,
                                                               os.path.relpath(os.path.abspath(PROMPTS_HEADER))))
    return 0


def play(x):
    """play it on the laptop, as the watch has it"""
    player = shutil.which('pw-play') or shutil.which('aplay')
    if not player:
        return
    with tempfile.NamedTemporaryFile(suffix='.wav') as f:
        with wave.open(f.name, 'wb') as w:
            w.setnchannels(1)
            w.setsampwidth(2)
            w.setframerate(RATE)
            w.writeframes((np.round(x * 127).astype(np.int8).astype('<i2') * 256).tobytes())
        subprocess.run([player, f.name], check=False)


def main(args):
    return make_prompts(args)
