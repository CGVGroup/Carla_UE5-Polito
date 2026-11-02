#!/usr/bin/env python3
"""
check_session_log.py: verifies a session log written by the session logger
(leaderboard/leaderboard/utils/session_logger.py) and prints a PASS / WARN / FAIL report.

Checks:
- files: frames.csv, events.jsonl, summary.json present and readable; recording.log and the recorder's
  analyses (if the CARLA recorder ran)
- frames.csv: expected columns; frames strictly increasing, without duplicates, gaps reported; sim_time
  increasing with a steady step; no position jumps (teleports); plausible speeds / accelerations;
  valid control values and control_source; hazards / emergency_stop consistent
- events.jsonl: valid JSON lines, in time order, route_start first and route_end last, takeover / scenario /
  emergency stop start-end pairs balanced
- summary.json vs frames / events: duration, distance, number of collisions / takeovers / hard brakes /
  emergency stops recomputed from the raw data
- gaze (optional, --gaze <csv>): frames overlapping the session, share of valid gaze samples

Usage:
    python tools/check_session_log.py logs/sessions/<session folder> [--gaze <gaze csv>] [--plot]
    python tools/check_session_log.py --latest              # the most recent session in logs/sessions
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

EXPECTED_COLUMNS = [
    'frame', 'sim_time', 'wall_time', 'x', 'y', 'z', 'yaw', 'speed_kmh', 'accel', 'road_id', 'lane_id',
    'is_junction', 'lateral_offset', 'throttle', 'steer', 'brake', 'hand_brake', 'reverse', 'cmd_throttle',
    'cmd_steer', 'cmd_brake', 'control_source', 'failure_behavior', 'hazards', 'emergency_stop',
    'target_speed', 'scenario', 'dashboard_mode']


class Report:
    def __init__(self):
        self.counts = {'PASS': 0, 'WARN': 0, 'FAIL': 0}

    def __call__(self, level, message):
        self.counts[level] += 1
        print('[{}] {}'.format(level, message))

    def check(self, condition, message, level_if_false='FAIL'):
        self('PASS' if condition else level_if_false, message)
        return condition


def fnum(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_frames(path):
    with open(path, newline='', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        return reader.fieldnames or [], list(reader)


def load_events(path, report):
    events, bad = [], 0
    with open(path, encoding='utf-8') as f:
        for number, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError:
                bad += 1
                if bad <= 3:
                    report('FAIL', 'events.jsonl line {} is not valid JSON'.format(number))
    return events, bad


def check_frames(columns, rows, report):
    missing = [c for c in EXPECTED_COLUMNS if c not in columns]
    report.check(not missing, 'frames.csv columns ({} columns{})'.format(
        len(columns), ', missing: ' + ', '.join(missing) if missing else ''))
    if not report.check(len(rows) > 0, 'frames.csv has {} rows'.format(len(rows))):
        return {}

    frames = [int(r['frame']) for r in rows]
    duplicates = len(frames) - len(set(frames))
    decreasing = sum(1 for a, b in zip(frames, frames[1:]) if b <= a)
    gaps = [(a, b) for a, b in zip(frames, frames[1:]) if b - a > 1]
    missing_frames = sum(b - a - 1 for a, b in gaps)
    report.check(duplicates == 0 and decreasing == 0, 'frames strictly increasing ({} duplicates, {} going back)'.format(
        duplicates, decreasing))
    ratio = missing_frames / max(1, frames[-1] - frames[0])
    report('PASS' if ratio < 0.01 else 'WARN' if ratio < 0.1 else 'FAIL',
           'missing frames: {} in {} gaps ({:.1%} of the span {}..{})'.format(
               missing_frames, len(gaps), ratio, frames[0], frames[-1]))

    times = [fnum(r['sim_time']) for r in rows]
    steps = [b - a for a, b in zip(times, times[1:]) if a is not None and b is not None]
    report.check(all(s > 0 for s in steps), 'sim_time increasing')
    if steps:
        steps_sorted = sorted(steps)
        median = steps_sorted[len(steps_sorted) // 2]
        irregular = sum(1 for s in steps if s > 3 * median)
        report('PASS' if irregular <= len(steps) * 0.01 else 'WARN',
               'time step: median {:.3f} s, {} steps over 3x the median'.format(median, irregular))

    # Positions / speeds
    jumps = []
    for a, b in zip(rows, rows[1:]):
        dt = (fnum(b['sim_time']) or 0) - (fnum(a['sim_time']) or 0)
        d = math.hypot(fnum(b['x']) - fnum(a['x']), fnum(b['y']) - fnum(a['y']))
        v = max(fnum(a['speed_kmh']) or 0, fnum(b['speed_kmh']) or 0) / 3.6
        if dt > 0 and d > max(2.0, 3.0 * v * dt + 1.0):
            jumps.append((b['frame'], round(d, 1)))
    report.check(not jumps, 'no position jumps{}'.format(': ' + str(jumps[:5]) if jumps else ''), 'WARN')
    speeds = [fnum(r['speed_kmh']) or 0 for r in rows]
    report.check(max(speeds) < 150, 'speeds plausible (max {:.1f} km/h)'.format(max(speeds)), 'WARN')
    accels = [fnum(r['accel']) or 0 for r in rows]
    extreme = sum(1 for a in accels if abs(a) > 15)
    report.check(extreme == 0, 'accelerations plausible ({} frames over 15 m/s2, min {:.1f}, max {:.1f})'.format(
        extreme, min(accels), max(accels)), 'WARN')

    # Controls
    out_of_range = 0
    for r in rows:
        t, s, b = fnum(r['throttle']), fnum(r['steer']), fnum(r['brake'])
        if t is None or s is None or b is None or not (0 <= t <= 1 and -1 <= s <= 1 and 0 <= b <= 1):
            out_of_range += 1
    report.check(out_of_range == 0, 'control values in range ({} frames out)'.format(out_of_range))
    sources = {r['control_source'] for r in rows}
    report.check(sources <= {'agent', 'driver'}, 'control_source values: {}'.format(sorted(sources)))
    driver_frames = sum(1 for r in rows if r['control_source'] == 'driver')
    report('PASS', 'driver in control during {} frames ({:.1%})'.format(driver_frames, driver_frames / len(rows)))
    inconsistent = sum(1 for r in rows if r['emergency_stop'] == '1' and not r['hazards'])
    report.check(inconsistent == 0, 'emergency stops have a hazard ({} without)'.format(inconsistent), 'WARN')
    modes = {r['dashboard_mode'] for r in rows}
    report('PASS', 'dashboard mode(s): {}'.format(sorted(modes)))

    distance = sum(math.hypot(fnum(b['x']) - fnum(a['x']), fnum(b['y']) - fnum(a['y'])) for a, b in zip(rows, rows[1:]))
    return {'duration': (times[-1] or 0) - (times[0] or 0), 'distance': distance, 'frames': frames,
            'driver_frames': driver_frames}


def check_events(events, report):
    if not report.check(len(events) > 0, 'events.jsonl has {} events'.format(len(events))):
        return {}
    kinds = [e.get('event') for e in events]
    report.check(kinds[0] == 'route_start', 'first event is route_start ({})'.format(kinds[0]))
    report.check(kinds[-1] == 'route_end', 'last event is route_end ({})'.format(kinds[-1]),
                 'WARN')  # missing when the route was interrupted
    times = [e.get('sim_time') for e in events if e.get('sim_time') is not None]
    report.check(all(b >= a for a, b in zip(times, times[1:])), 'events in time order', 'WARN')
    counts = {}
    for k in kinds:
        counts[k] = counts.get(k, 0) + 1
    for start, end in (('takeover_start', 'takeover_end'), ('scenario_start', 'scenario_end'),
                       ('emergency_stop_start', 'emergency_stop_end')):
        report.check(abs(counts.get(start, 0) - counts.get(end, 0)) <= 1,
                     '{} / {}: {} / {}'.format(start, end, counts.get(start, 0), counts.get(end, 0)), 'WARN')
    print('       events: ' + ', '.join('{} {}'.format(k, v) for k, v in sorted(counts.items())))
    return counts


def check_summary(summary, frame_stats, counts, report):
    route = summary.get('route', {})
    if frame_stats:
        for key, recomputed, tolerance in (('duration', frame_stats['duration'], 0.05),
                                           ('distance', frame_stats['distance'], 0.10)):
            value = route.get(key)
            ok = value is not None and abs(value - recomputed) <= max(1.0, tolerance * recomputed)
            report.check(ok, 'summary {} {} vs {:.1f} recomputed from the frames'.format(key, value, recomputed), 'WARN')
    for key, event in (('collisions', 'collision'), ('takeovers', 'takeover_start'), ('hard_brakes', 'hard_brake'),
                       ('emergency_stops', 'emergency_stop_start'), ('lane_invasions', 'lane_invasion')):
        report.check(route.get(key) == counts.get(event, 0), 'summary {} = {} ({} {} events)'.format(
            key, route.get(key), counts.get(event, 0), event))
    report('PASS', 'scenarios in the summary: {}'.format(', '.join(summary.get('scenarios', {})) or 'none'))


def check_recording(directory, summary, report):
    recording = os.path.join(directory, 'recording.log')
    if not os.path.exists(recording):
        report('WARN', 'no recording.log (CARLA recorder not run: RECORD_PATH set, or not started)')
        return
    size = os.path.getsize(recording)
    report.check(size > 10000, 'recording.log: {:.1f} MB'.format(size / 1e6))
    for name in ('info', 'collisions', 'blocked'):
        path = os.path.join(directory, 'recording_{}.txt'.format(name))
        report.check(os.path.exists(path) and os.path.getsize(path) > 0,
                     'recording_{}.txt {}'.format(name, 'present' if os.path.exists(path) else 'missing'), 'WARN')
    info = os.path.join(directory, 'recording_info.txt')
    if os.path.exists(info):
        text = open(info, encoding='utf-8', errors='replace').read()
        frames_line = [l for l in text.splitlines() if 'Frames' in l or 'Duration' in l]
        if frames_line:
            print('       recorder: ' + ' | '.join(l.strip() for l in frames_line[:2]))


def check_gaze(path, frame_stats, report):
    try:
        with open(path, newline='', encoding='utf-8') as f:
            rows = list(csv.DictReader(f))
    except OSError as e:
        report('FAIL', 'gaze file not readable ({})'.format(e))
        return
    if not report.check(rows, 'gaze file has {} samples'.format(len(rows))):
        return
    valid = sum(1 for r in rows if r.get('gaze_valid') == '1')
    connected = sum(1 for r in rows if r.get('eye_tracker_connected') == '1')
    report.check(connected > 0, 'eye tracker connected in {:.1%} of the samples'.format(connected / len(rows)))
    report('PASS' if valid / len(rows) > 0.8 else 'WARN' if valid else 'FAIL',
           'valid gaze in {:.1%} of the samples'.format(valid / len(rows)))
    if frame_stats:
        session = set(frame_stats['frames'])
        gaze_frames = {int(r['frame']) for r in rows if r.get('frame', '').isdigit()}
        overlap = len(session & gaze_frames) / max(1, len(session))
        report('PASS' if overlap > 0.9 else 'WARN', 'session frames with a gaze sample: {:.1%}'.format(overlap))
        hits = sum(1 for r in rows if r.get('hit') == '1')
        print('       gaze hits something in {:.1%} of the samples'.format(hits / len(rows)))


def plot(directory, rows):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print('(matplotlib not installed: no plot)')
        return
    t = [fnum(r['sim_time']) for r in rows]
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    colors = ['tab:red' if r['control_source'] == 'driver' else 'tab:blue' for r in rows]
    axes[0].scatter([fnum(r['x']) for r in rows], [fnum(r['y']) for r in rows], c=colors, s=2)
    axes[0].set_title('Trajectory (blue: agent, red: driver)')
    axes[0].axis('equal')
    axes[0].invert_yaxis()
    axes[1].plot(t, [fnum(r['speed_kmh']) for r in rows], label='speed [km/h]')
    axes[1].plot(t, [fnum(r['target_speed']) for r in rows], label='target speed [km/h]', alpha=0.6)
    axes[1].fill_between(t, 0, [5 if r['emergency_stop'] == '1' else 0 for r in rows], color='tab:red',
                         alpha=0.3, label='emergency stop')
    axes[1].legend()
    axes[1].set_xlabel('sim time [s]')
    out = os.path.join(directory, 'check_plot.png')
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print('Plot saved to', out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('directory', nargs='?', help='session folder')
    ap.add_argument('--latest', action='store_true', help='check the most recent session in logs/sessions')
    ap.add_argument('--gaze', help='gaze CSV of the session (<CARLA Saved dir>/GazeLogs/...)')
    ap.add_argument('--plot', action='store_true', help='save a trajectory / speed plot (needs matplotlib)')
    args = ap.parse_args()

    directory = args.directory
    if args.latest or not directory:
        root = os.environ.get('SESSION_LOG_DIR') or os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'logs', 'sessions')
        sessions = sorted(glob.glob(os.path.join(root, '*')), key=os.path.getmtime)
        if not sessions:
            print('No session in', root)
            return 2
        directory = sessions[-1]
    print('Session:', directory, '\n')
    report = Report()

    paths = {name: os.path.join(directory, name) for name in ('frames.csv', 'events.jsonl', 'summary.json')}
    for name, path in paths.items():
        report.check(os.path.exists(path), '{} present'.format(name))

    columns, rows = ([], [])
    frame_stats, counts, summary = {}, {}, {}
    if os.path.exists(paths['frames.csv']):
        columns, rows = load_frames(paths['frames.csv'])
        frame_stats = check_frames(columns, rows, report)
    if os.path.exists(paths['events.jsonl']):
        events, _ = load_events(paths['events.jsonl'], report)
        counts = check_events(events, report)
    if os.path.exists(paths['summary.json']):
        try:
            summary = json.load(open(paths['summary.json'], encoding='utf-8'))
            check_summary(summary, frame_stats, counts, report)
        except json.JSONDecodeError:
            report('FAIL', 'summary.json is not valid JSON (route interrupted before the end?)')
    else:
        report('WARN', 'no summary.json: the route was interrupted before its end (frames / events are still usable)')
    check_recording(directory, summary, report)
    if args.gaze:
        check_gaze(args.gaze, frame_stats, report)
    if args.plot and rows:
        plot(directory, rows)

    print('\n{PASS} passed, {WARN} warnings, {FAIL} failures'.format(**report.counts))
    return 1 if report.counts['FAIL'] else 0


if __name__ == '__main__':
    sys.exit(main())
