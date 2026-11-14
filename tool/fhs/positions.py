"""The guided position test (fhs positions): the laptop speaks one head position after the other while the watch,
unplugged and taped on, logs; afterwards each position's angles are matched to what the watch did.

Each position has an expectation: 'nudge' (flat, or anywhere in between: the watch must wake you) or 'quiet' (the
head on its side on the pillow). The report says for each one whether the watch got it right, and whether the
angle setting has room between the in-between positions and the side."""
import os
import shutil
import subprocess
import sys
import tempfile
import time

# The positions are the ones seen on camera in real nights: rolls of 3-20 s, often through the back; the ones to catch
# are flat with the head tipped back and an arm over it, and half on the back against a positional vest's bump; on the
# side the head sometimes tips up over the shoulder, and a hand goes to the forehead before a roll.
# Done with the vest (if you wear one) and the eye mask on, as at night; but the zero comes first, without the vest: with it the back
# never lies flat, and a tilted zero would make one side read as half way (nudged on the side).
#   expect: zero (sets the angle zero), vest (time to put it on; not judged), nudge (must wake), quiet (must not),
#   measure (no right answer: we look at it)

ZERO = ('Lie flat on your back, head straight, the way it rests on the pillow. Hold the button, then do what the '
        'watch says until it says zero set.', 'zero', 20)   # the spoken zero takes ~12 s after the press
VEST = ('Now put on the vest and the eye mask, then lie down on your left side, the way you sleep.', 'vest', None)

SHADOW = [   # (what the laptop says, expect, seconds or None = the hold time)
    ('Stay flat on your back, chin up, as if asleep.', 'nudge', None),
    ('Flat on your back, head tipped back, right arm over your head.', 'nudge', None),
    ('Flat on your back, turn your head a little to the left.', 'nudge', None),
    ('Turn your head half way to the left.', 'nudge', None),
    ('Now on your left side, head on the pillow, the way you sleep.', 'quiet', None),
    ('Stay on your left side, face turned down into the pillow.', 'quiet', None),
    ('Stay on your left side, arm over your head, head tipped back.', 'measure', None),
    ('Half on your back against the bump, face turned to the left.', 'nudge', None),
    ('Flat on your back again.', 'nudge', None),
    ('Turn your head a little to the right.', 'nudge', None),
    ('Turn your head half way to the right.', 'nudge', None),
    ('Now on your right side, head on the pillow, the way you sleep.', 'quiet', None),
    ('Stay on your right side, arm over your head, head tipped up over your shoulder.', 'measure', None),
    ('Half on your back against the bump, face turned to the right.', 'nudge', None),
    ('On your left side. Put your hand on your forehead for a moment, as you do before a roll, then rest.', 'measure', None),
    ('Now roll slowly from your left side, over your back, onto your right side. Take about five seconds.', 'measure', 20),
    ('Now roll back over your back to the left, but stop flat on your back for a while.', 'nudge', None),
    ('Sit up on the edge of the bed.', 'quiet', 20),
    ('Lie back down on your left side, the way you sleep.', 'quiet', None),
]

THERAPY = [   # (what the laptop says, expect, seconds): now it really nudges
    ('Flat on your back. Stay flat until the sound is loud, then turn onto your left side.', 'nudge', 75),
    ('Stay on your left side.', 'quiet', 20),
    ('Flat on your back again. This time, turn onto your right side as soon as you feel the first buzz.', 'nudge', 40),
    ('Stay on your right side.', 'quiet', 20),
    ('Half on your back against the bump. Turn onto your side when you feel it.', 'nudge', 40),
    ('Stay on your side.', 'quiet', 15),
    ('Flat, head tipped back, arm over your head. Turn when you feel it.', 'nudge', 40),
    ('Stay on your side.', 'quiet', 15),
]

# the settings the test changes (put back afterwards): log at once, any time of day, nudge without the night's waits
TEST_SETTINGS = dict(start_wait='0', upright_pause='0', lie_gate='0', win_from='0', win_to='0')


VOICES = os.path.expanduser('~/.local/share/forehead-sensor/voices')   # Piper voice models (natural, offline)
VOICE = 'en_GB-jenny_dioco-medium'   # Piper voice used for the test and the spoken zero


def say(text, speak=True, voice=VOICE):
    """print it, and say it: a Piper voice if it's installed, else the robotic espeak (spd-say)"""
    print(time.strftime('%H:%M:%S'), text, flush=True)
    if not speak:
        return
    model = os.path.join(VOICES, voice + '.onnx')
    piper = os.path.join(os.path.dirname(sys.executable), 'piper')
    player = shutil.which('pw-play') or shutil.which('aplay')
    if os.path.exists(model) and os.path.exists(piper) and player:
        with tempfile.NamedTemporaryFile(suffix='.wav') as f:
            if subprocess.run([piper, '-m', model, '-f', f.name], input=text.encode(), capture_output=True).returncode == 0:
                subprocess.run([player, f.name], check=False)
                return
    if shutil.which('spd-say'):
        subprocess.run(['spd-say', '-w', '-r', '-15', text], check=False)


def plan(therapy, hold, vest=60):
    """[(text, expect, seconds)] of the run: the zero first (without the vest), then the vest goes on. The therapy run
    has neither: the zero is set by then, the vest is on from the start, and a flat zero would already be nudged"""
    if therapy:
        return list(THERAPY)
    return [ZERO, VEST[:2] + (vest,)] + [(t, e, sec or hold) for t, e, sec in SHADOW]


def analyze(rows, segments, angle, hyst, therapy):
    """per segment: the angles, and whether the watch did what it should. rows: sensor.csv dicts (t, roll, ...)"""
    out = []
    for s in segments:
        if s['expect'] == 'vest':
            continue
        r = [x for x in rows if s['start'] <= float(x['t']) < s['end']]
        seg = dict(s, n=len(r))
        if not r:
            seg.update(ok=False, note='no records (the watch was not logging?)')
            out.append(seg)
            continue
        roll = sorted(abs(float(x['roll'])) for x in r)
        pitch = sorted(float(x['pitch']) for x in r)
        med = lambda v: v[len(v) // 2]
        seg.update(roll=round(med(sorted(float(x['roll']) for x in r)), 1), abs_roll_min=round(roll[0], 1),
                   abs_roll_max=round(roll[-1], 1), pitch=round(med(pitch), 1))
        would = [float(x['t']) - s['start'] for x in r if int(x['would']) > 0]
        seg['nudge_after_s'] = round(would[0], 1) if would else None
        seg['max_step'] = max(int(x['would']) for x in r)
        if therapy and s['expect'] == 'nudge':   # how long from the first nudge until the head left
            ran = [float(x['t']) for x in r if int(x['motor']) > 0]
            off = [float(x['t']) for x in r if ran and float(x['t']) > ran[0] and abs(float(x['roll'])) > angle + hyst]
            seg['turned_after_nudge_s'] = round(off[0] - ran[0], 1) if off else None
        if s['expect'] == 'zero':
            seg['ok'] = abs(seg['roll']) < 5 and abs(seg['pitch']) < 5
            seg['note'] = 'zero set: roll and pitch about 0' if seg['ok'] else 'zero NOT set here (button not held?)'
        elif s['expect'] == 'nudge':
            seg['ok'] = seg['nudge_after_s'] is not None
            seg['note'] = ('nudged after %.0f s' % seg['nudge_after_s']) if seg['ok'] else \
                'NO NUDGE: |roll| %.0f-%.0f, the angle is %.0f' % (seg['abs_roll_min'], seg['abs_roll_max'], angle)
        elif s['expect'] == 'quiet':
            seg['ok'] = seg['max_step'] == 0
            seg['note'] = 'quiet' if seg['ok'] else 'NUDGED: |roll| down to %.0f' % seg['abs_roll_min']
        else:   # measure: what it read, and what it did
            seg['ok'] = None
            seg['note'] = '|roll| %.0f-%.0f: %s' % (seg['abs_roll_min'], seg['abs_roll_max'], (
                'would nudge after %.0f s' % seg['nudge_after_s']) if seg['nudge_after_s'] is not None else 'quiet')
        out.append(seg)
    return out


def angle_advice(res, angle, hyst):
    """does the angle sit between the in-between positions (must nudge) and the side (must stay quiet)?"""
    nudge = [s['abs_roll_max'] for s in res if s['expect'] == 'nudge' and 'abs_roll_max' in s]
    side = [s['abs_roll_min'] for s in res if s['expect'] == 'quiet' and 'abs_roll_min' in s]
    if not nudge or not side:
        return 'not enough positions to judge the angle'
    hi, lo = max(nudge), min(side)
    line = 'in-between positions reach |roll| %.0f; on the side it goes down to %.0f; the angle is %.0f (stops past %.0f)' % (
        hi, lo, angle, angle + hyst)
    if hi < angle and lo > angle + hyst:
        return line + ': the angle is right'
    if hi + 2 * hyst < lo:
        return line + ': set the angle to %.0f (fhs set angle %.0f --usb)' % ((hi + lo) / 2 - hyst / 2, (hi + lo) / 2 - hyst / 2)
    return line + ': the in-between and the side overlap, look at it together'


def report(res, advice, therapy):
    lines = ['%-75s %-7s %6s %6s  %s' % ('position', 'expect', 'roll', 'pitch', 'result')]
    for s in res:
        lines.append('%-75s %-7s %6s %6s  %s%s' % (
            s['text'][:75], s['expect'], s.get('roll', '-'), s.get('pitch', '-'), '!! ' if s['ok'] is False else '', s['note']))
        if therapy and s.get('expect') == 'nudge' and 'max_step' in s:
            lines.append('%-75s %-7s %6s %6s    reached step %d; turned %s s after the first nudge' % (
                '', '', '', '', s['max_step'], s.get('turned_after_nudge_s')))
    lines.append('')
    judged = [s for s in res if s['ok'] is not None]
    lines.append('%d of %d positions right (%d only measured)' % (sum(bool(s['ok']) for s in judged), len(judged),
                                                                len(res) - len(judged)))
    lines.append(advice)
    return '\n'.join(lines)
