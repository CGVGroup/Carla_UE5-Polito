#!/usr/bin/env python3
"""
visualize_gaze.py: visualizes a driver's gaze recording (<CARLA Saved dir>/GazeLogs/<session>_<date>.csv;
packaged CARLA: %LOCALAPPDATA%/CarlaUnreal/Saved/GazeLogs), optionally with its session log
(leaderboard/logs/sessions/<session>_<date>/, for the scenarios and takeovers).

Outputs, next to the gaze file (or in --out):
- <name>_map.png: top-down view of the drive (grey: agent driving, red: driver) and of the points the gaze
  hit, colored by area of interest (AOI)
- <name>_timeline.png: AOI looked at over time, gaze direction relative to the head (yaw / pitch), speed;
  takeovers and scenarios shaded
- <name>_heatmap.png: where the eyes looked in the field of view (gaze direction relative to the head)
- <name>_aoi.csv: per AOI, for the whole drive and each scenario: dwell time, share of the valid gaze
  time, number of fixations, mean fixation duration (also printed)

AOIs, from what the gaze hits: vehicle, pedestrian, prop (cones, signs of the roadworks, blockers...),
traffic sign / light, road, sidewalk, building, vegetation, other, nothing (sky, far away). Fixations:
velocity threshold (I-VT): gaze angular velocity under 30 deg/s for at least 100 ms.

Usage:
    python tools/visualize_gaze.py <gaze csv> [--session <session folder>] [--out <folder>]
    python tools/visualize_gaze.py --latest                  # latest gaze file (and its session, if found)
"""
import argparse
import csv
import datetime
import glob
import math
import os
import sys

import numpy as np

AOI_COLORS = {
    'vehicle': '#d62728', 'pedestrian': '#ff7f0e', 'prop': '#bcbd22', 'traffic sign/light': '#9467bd',
    'road': '#7f7f7f', 'sidewalk': '#c7c7c7', 'building': '#8c564b', 'vegetation': '#2ca02c',
    'other': '#17becf', 'nothing': '#1f77b4',
}
FIXATION_MAX_VELOCITY = 30.0  # [deg/s]
FIXATION_MIN_DURATION = 0.1  # [s]


def fnum(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def wall_seconds(text):
    """Seconds of a wall_time (ISO 8601), NaN if missing"""
    try:
        return datetime.datetime.fromisoformat(text.strip().rstrip('Z')).timestamp()
    except (AttributeError, ValueError):
        return float('nan')


def aoi_of(row):
    """Area of interest of a gaze sample, from what it hits"""
    if row.get('gaze_valid') != '1':
        return None
    if row.get('hit') != '1':
        return 'nothing'
    actor_type = (row.get('hit_actor_type') or '').lower()
    semantic = (row.get('hit_semantic') or '').lower()
    if actor_type.startswith('vehicle.'):
        return 'vehicle'
    if actor_type.startswith('walker.'):
        return 'pedestrian'
    if actor_type.startswith('traffic.') or semantic in ('trafficsigns', 'trafficlight', 'traffic sign', 'traffic light'):
        return 'traffic sign/light'
    if actor_type.startswith('static.prop'):
        return 'prop'
    if semantic in ('roads', 'roadlines', 'roadline', 'road', 'ground', 'terrain'):
        return 'road'
    if semantic in ('sidewalks', 'sidewalk'):
        return 'sidewalk'
    if semantic in ('buildings', 'walls', 'fences', 'bridge', 'building', 'wall', 'fence', 'guardrail', 'static'):
        return 'building'
    if semantic in ('vegetation',):
        return 'vegetation'
    if semantic in ('car', 'truck', 'bus', 'motorcycle', 'bicycle', 'vehicles'):
        return 'vehicle'
    if semantic in ('pedestrians', 'pedestrian', 'rider'):
        return 'pedestrian'
    return 'other'


def gaze_angles(row):
    """Gaze yaw / pitch [deg], and relative to the head's"""
    dx, dy, dz = fnum(row['gaze_dir_x']), fnum(row['gaze_dir_y']), fnum(row['gaze_dir_z'])
    norm = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, dz / norm))))
    rel_yaw = (yaw - fnum(row['head_yaw']) + 180.0) % 360.0 - 180.0
    rel_pitch = pitch - fnum(row['head_pitch'])
    return (dx / norm, dy / norm, dz / norm), rel_yaw, rel_pitch


def load_session(directory):
    """frame -> (scenario, control_source, speed) and the takeover / scenario spans from the session log"""
    by_frame, spans = {}, {'takeover': [], 'scenario': []}
    frames_csv = os.path.join(directory, 'frames.csv')
    if os.path.exists(frames_csv):
        with open(frames_csv, newline='', encoding='utf-8') as f:
            for row in csv.DictReader(f):
                by_frame[int(row['frame'])] = (row.get('scenario', ''), row.get('control_source', 'agent'),
                                               fnum(row.get('speed_kmh')))
    return by_frame, spans


def find_session_for(gaze_path):
    """The session folder with the same name as the gaze file (both <session name>_<date>)"""
    stem = os.path.splitext(os.path.basename(gaze_path))[0]
    name = stem.rsplit('_', 2)[0] if stem.count('_') >= 2 else stem
    root = os.environ.get('SESSION_LOG_DIR') or os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'logs', 'sessions')
    candidates = sorted(glob.glob(os.path.join(root, name + '_*')), key=os.path.getmtime)
    if not candidates:
        return None
    gaze_time = os.path.getmtime(gaze_path)
    return min(candidates, key=lambda c: abs(os.path.getmtime(c) - gaze_time))


def spans_of(times, labels):
    """Consecutive runs of the same label: [(label, start, end)]"""
    spans, start = [], 0
    for i in range(1, len(labels) + 1):
        if i == len(labels) or labels[i] != labels[start]:
            spans.append((labels[start], times[start], times[min(i, len(times) - 1)]))
            start = i
    return spans


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('gaze', nargs='?', help='gaze CSV')
    ap.add_argument('--latest', action='store_true', help='the latest gaze file')
    ap.add_argument('--session', help='session folder of the same drive (default: found by name and date)')
    ap.add_argument('--out', help='output folder (default: next to the gaze file)')
    args = ap.parse_args()

    gaze_path = args.gaze
    if args.latest or not gaze_path:
        folder = os.path.join(os.environ.get('LOCALAPPDATA', ''), 'CarlaUnreal', 'Saved', 'GazeLogs')
        files = sorted(glob.glob(os.path.join(folder, '*.csv')), key=os.path.getmtime)
        if not files:
            print('No gaze file in', folder)
            return 2
        gaze_path = files[-1]
    print('Gaze file:', gaze_path)
    with open(gaze_path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print('Empty gaze file')
        return 2

    session = args.session or find_session_for(gaze_path)
    by_frame = {}
    if session:
        print('Session log:', session)
        by_frame, _ = load_session(session)
    else:
        print('No session log found (scenarios and takeovers not shown)')

    out = args.out or os.path.dirname(os.path.abspath(gaze_path))
    os.makedirs(out, exist_ok=True)
    base = os.path.join(out, os.path.splitext(os.path.basename(gaze_path))[0])

    # Per sample. Real time (wall_time): the eyes move in real time, and in VR CARLA samples the gaze on
    # every rendered frame, several per simulation step (same sim_time). Older files: sim_time
    t = np.array([wall_seconds(r.get('wall_time')) for r in rows])
    if np.isnan(t).any():
        t = np.array([fnum(r['sim_time']) for r in rows])
    t = t - t[0]
    valid = np.array([r.get('gaze_valid') == '1' for r in rows])
    print('{} samples, {:.0f} s, valid gaze in {:.1%}'.format(len(rows), t[-1], valid.mean()))
    if not valid.any():
        print('No valid gaze sample: nothing to visualize (is eye tracking enabled for PC VR in VIVE Hub?)')
        return 1
    aois = [aoi_of(r) for r in rows]
    directions, rel_yaw, rel_pitch = [], [], []
    for r in rows:
        d, y, p = gaze_angles(r)
        directions.append(d)
        rel_yaw.append(y)
        rel_pitch.append(p)
    directions, rel_yaw, rel_pitch = np.array(directions), np.array(rel_yaw), np.array(rel_pitch)
    frames = [int(r['frame']) for r in rows]
    scenario = [by_frame.get(f, ('', 'agent', 0.0))[0] for f in frames]
    source = [by_frame.get(f, ('', 'agent', 0.0))[1] for f in frames]
    speed = np.array([fnum(r['ego_speed_kmh']) for r in rows])

    # Fixations (I-VT)
    dt = np.diff(t, prepend=t[0])
    dt[dt <= 0] = np.median(dt[dt > 0]) if (dt > 0).any() else 0.05
    cosines = np.clip(np.sum(directions[1:] * directions[:-1], axis=1), -1.0, 1.0)
    velocity = np.concatenate([[0.0], np.degrees(np.arccos(cosines)) / dt[1:]])
    slow = valid & (velocity < FIXATION_MAX_VELOCITY)
    fixations = []  # (start index, end index)
    i = 0
    while i < len(rows):
        if slow[i]:
            j = i
            while j + 1 < len(rows) and slow[j + 1]:
                j += 1
            if t[j] - t[i] >= FIXATION_MIN_DURATION:
                fixations.append((i, j))
            i = j + 1
        else:
            i += 1

    # AOI statistics: whole drive and per scenario
    groups = {'(whole drive)': np.ones(len(rows), dtype=bool)}
    for name in sorted({s for s in scenario if s}):
        groups[name] = np.array([s == name for s in scenario])
    stats_rows = []
    for group, mask in groups.items():
        valid_time = float(np.sum(dt[mask & valid]))
        for aoi in AOI_COLORS:
            in_aoi = mask & valid & np.array([a == aoi for a in aois])
            dwell = float(np.sum(dt[in_aoi]))
            group_fix = [(a, b) for a, b in fixations if mask[a] and aois[a] == aoi]
            durations = [t[b] - t[a] for a, b in group_fix]
            if dwell > 0 or group_fix:
                stats_rows.append({'group': group, 'aoi': aoi, 'dwell_s': round(dwell, 2),
                                   'share': round(dwell / valid_time, 3) if valid_time else 0.0,
                                   'fixations': len(group_fix),
                                   'mean_fixation_s': round(float(np.mean(durations)), 3) if durations else 0.0})
    with open(base + '_aoi.csv', 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['group', 'aoi', 'dwell_s', 'share', 'fixations', 'mean_fixation_s'])
        writer.writeheader()
        writer.writerows(stats_rows)
    print('\n{:<34} {:<20} {:>8} {:>7} {:>9} {:>10}'.format('group', 'aoi', 'dwell s', 'share', 'fixations', 'mean fix s'))
    for s in stats_rows:
        print('{:<34} {:<20} {:>8} {:>7.1%} {:>9} {:>10}'.format(s['group'][:34], s['aoi'], s['dwell_s'], s['share'],
                                                                 s['fixations'], s['mean_fixation_s']))

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Patch

    # Map
    fig, ax = plt.subplots(figsize=(11, 10))
    ex = np.array([fnum(r['ego_x']) for r in rows])
    ey = np.array([fnum(r['ego_y']) for r in rows])
    driver = np.array([s == 'driver' for s in source])
    ax.plot(ex, ey, color='#bbbbbb', lw=2, zorder=1)
    if driver.any():
        ax.scatter(ex[driver], ey[driver], color='red', s=3, zorder=2, label='driver driving')
    hit = valid & np.array([r.get('hit') == '1' for r in rows])
    for aoi, color in AOI_COLORS.items():
        mask = hit & np.array([a == aoi for a in aois])
        if mask.any():
            ax.scatter([fnum(rows[i]['hit_x']) for i in np.where(mask)[0]],
                       [fnum(rows[i]['hit_y']) for i in np.where(mask)[0]], s=4, color=color, alpha=0.5, label=aoi, zorder=3)
    ax.set_aspect('equal')
    ax.invert_yaxis()  # CARLA's y axis points down on a map seen from above
    ax.set_title('Drive and gaze hit points')
    ax.legend(markerscale=4, loc='best', fontsize=8)
    fig.tight_layout()
    fig.savefig(base + '_map.png', dpi=120)
    plt.close(fig)

    # Timeline
    fig, axes = plt.subplots(3, 1, figsize=(15, 8), sharex=True, gridspec_kw={'height_ratios': [1, 2, 1.3]})
    for label, start, end in spans_of(list(t), [a or 'invalid' for a in aois]):
        axes[0].axvspan(start, end, color=AOI_COLORS.get(label, 'white'), lw=0)
    axes[0].set_yticks([])
    axes[0].set_title('Looked at')
    axes[0].legend(handles=[Patch(color=c, label=a) for a, c in AOI_COLORS.items()], ncol=5, fontsize=7,
                   loc='upper center', bbox_to_anchor=(0.5, -0.05))
    axes[1].plot(t[valid], rel_yaw[valid], lw=0.6, label='gaze yaw vs head [deg] (+: right)')
    axes[1].plot(t[valid], rel_pitch[valid], lw=0.6, label='gaze pitch vs head [deg] (+: up)')
    for a, b in fixations:
        axes[1].axvspan(t[a], t[b], color='#cccccc', alpha=0.3, lw=0)
    axes[1].legend(fontsize=8, loc='upper right')
    axes[1].set_ylim(-60, 60)
    axes[2].plot(t, speed, color='black', lw=0.8, label='speed [km/h]')
    axes[2].set_xlabel('time [s]')
    axes[2].legend(fontsize=8, loc='upper right')
    for label, start, end in spans_of(list(t), source):
        if label == 'driver':
            for a in axes[1:]:
                a.axvspan(start, end, color='red', alpha=0.12, lw=0)
    for label, start, end in spans_of(list(t), scenario):
        if label:
            axes[2].axvspan(start, end, color='gold', alpha=0.25, lw=0)
            axes[2].text((start + end) / 2, axes[2].get_ylim()[1] * 0.9, label, ha='center', fontsize=7)
    fig.tight_layout()
    fig.savefig(base + '_timeline.png', dpi=120)
    plt.close(fig)

    # Heatmap of the gaze in the field of view
    fig, ax = plt.subplots(figsize=(8, 5))
    h = ax.hist2d(rel_yaw[valid], rel_pitch[valid], bins=[72, 40], range=[[-60, 60], [-35, 35]], cmap='inferno')
    fig.colorbar(h[3], ax=ax, label='samples')
    ax.set_xlabel('gaze yaw vs head [deg] (+: right)')
    ax.set_ylabel('gaze pitch vs head [deg] (+: up)')
    ax.set_title('Where the eyes looked in the field of view')
    fig.tight_layout()
    fig.savefig(base + '_heatmap.png', dpi=120)
    plt.close(fig)

    print('\nSaved: {0}_map.png, {0}_timeline.png, {0}_heatmap.png, {0}_aoi.csv'.format(base))
    return 0


if __name__ == '__main__':
    sys.exit(main())
