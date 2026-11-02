#!/usr/bin/env python3
"""
make_session_gaze_video.py: after a route, makes the session's gaze outputs (run by run_leaderboard_xai.ps1
at the end): finds the session folder, its gaze file (<CARLA Saved dir>/GazeLogs) and the OBS video of the
headset view recorded during it, waits for the video to be complete, then writes into the session folder:
    gaze_video.mp4                         the video with the gaze on top (tools/gaze_video_overlay.py)
    <gaze file>_map/_timeline/_heatmap.png, _aoi.csv   the gaze visualizations (tools/visualize_gaze.py)

Calibration of the gaze position on the video: measured from the SYNC cue in the video (see
gaze_video_overlay.py). GAZE_HFOV, GAZE_CX, GAZE_CY environment variables: the values used when the cue
can't be measured (GAZE_HFOV also when the recording has no "+" markers), or always with
GAZE_CALIBRATION=manual.

Usage:
    python tools/make_session_gaze_video.py --since <unix time the route started>
"""
import argparse
import configparser
import csv
import glob
import os
import subprocess
import sys
import time

TOOLS = os.path.dirname(os.path.abspath(__file__))
LEADERBOARD = os.path.dirname(TOOLS)


def newest(pattern, since):
    files = [f for f in glob.glob(pattern) if os.path.getmtime(f) >= since]
    return max(files, key=os.path.getmtime) if files else None


def obs_video_folder():
    """OBS's recording folder (from its settings), default: the user's Videos"""
    folders = []
    for ini in glob.glob(os.path.join(os.environ.get('APPDATA', ''), 'obs-studio', 'basic', 'profiles', '*', 'basic.ini')):
        parser = configparser.ConfigParser(strict=False, interpolation=None)
        try:
            parser.read(ini, encoding='utf-8-sig')
        except (configparser.Error, OSError):
            continue
        for section in ('SimpleOutput', 'AdvOut'):
            for key in ('FilePath', 'RecFilePath'):
                value = parser.get(section, key, fallback='')
                if value:
                    folders.append(value.replace('\\\\', '\\'))
    folders.append(os.path.join(os.environ.get('USERPROFILE', ''), 'Videos'))
    return [f for f in dict.fromkeys(folders) if os.path.isdir(f)]


def wait_complete(path, quiet_seconds=4.0, timeout=120.0):
    """Waits until the file stops growing (OBS finished writing it). False on timeout"""
    start, last_size, stable_since = time.monotonic(), -1, time.monotonic()
    while time.monotonic() - start < timeout:
        size = os.path.getsize(path)
        if size != last_size:
            last_size, stable_since = size, time.monotonic()
        elif time.monotonic() - stable_since >= quiet_seconds:
            return True
        time.sleep(1.0)
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--since', type=float, required=True, help='unix time the route started')
    args = ap.parse_args()
    since = args.since - 5.0

    sessions_root = os.environ.get('SESSION_LOG_DIR') or os.path.join(LEADERBOARD, 'logs', 'sessions')
    sessions = [d for d in glob.glob(os.path.join(sessions_root, '*')) if os.path.isdir(d) and os.path.getmtime(d) >= since]
    session = max(sessions, key=os.path.getmtime) if sessions else None
    # Packaged CARLA: %LOCALAPPDATA%/CarlaUnreal/Saved; CARLA run from the editor: the Unreal project's Saved
    unreal_project = os.environ.get('CARLA_UE_ROOT', r'C:\Users\USER\Documents\GitHub\Carla_UE5-Polito')
    gaze = newest(os.path.join(os.environ.get('LOCALAPPDATA', ''), 'CarlaUnreal', 'Saved', 'GazeLogs', '*.csv'), since) \
        or newest(os.path.join(unreal_project, 'Unreal', 'CarlaUnreal', 'Saved', 'GazeLogs', '*.csv'), since)
    print('Session folder:', session or 'none')
    print('Gaze file:     ', gaze or 'none')
    if not gaze:
        print('No gaze recording for this route: no gaze outputs')
        return 1
    out_dir = session or os.path.dirname(gaze)

    # No eye data in the recording (the headset sent none): nothing to draw, say why
    with open(gaze, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    if rows and not any(r.get('gaze_valid') == '1' for r in rows):
        connected = any(r.get('eye_tracker_connected') == '1' for r in rows)
        print('\nNO EYE DATA in this recording ({} samples, 0% valid gaze, eye tracker {}): no gaze video.\n'
              'The headset sent no eye tracking to CARLA. Before the next route, with CARLA closed and the headset\n'
              'on, run:  python tools\\check_eye_tracking.py\n'
              '  - headset connected through VIVE Business Streaming (not Steam Link / another streaming app)\n'
              '  - eye tracking enabled in VIVE Business Streaming, eye calibration done on the headset\n'
              '  - then restart CARLA'.format(len(rows), 'seen' if connected else 'not even seen'))
        return 1

    # Gaze visualizations (map, timeline, heatmap, AOI table)
    command = [sys.executable, os.path.join(TOOLS, 'visualize_gaze.py'), gaze, '--out', out_dir]
    if session:
        command += ['--session', session]
    subprocess.run(command)

    # Video: the OBS recording that started during this route
    videos = []
    for folder in obs_video_folder():
        for extension in ('*.mkv', '*.mp4', '*.mov', '*.flv'):
            videos += [v for v in glob.glob(os.path.join(folder, extension))
                       if os.path.getmtime(v) >= since and not v.endswith('_gaze.mp4') and '_preview_' not in v]
    if not videos:
        print('No OBS video recorded during this route: no gaze video')
        return 0
    video = max(videos, key=os.path.getmtime)
    print('OBS video:     ', video)
    if not wait_complete(video):
        print('The OBS video is still growing after 2 minutes: stop the OBS recording, then run\n'
              '  python tools\\gaze_video_overlay.py "{}" "{}" --out "{}"'.format(
                  video, gaze, os.path.join(out_dir, 'gaze_video.mp4')))
        return 1
    command = [sys.executable, os.path.join(TOOLS, 'gaze_video_overlay.py'), video, gaze,
               '--out', os.path.join(out_dir, 'gaze_video.mp4'),
               '--hfov', os.environ.get('GAZE_HFOV', '100'),
               '--cx', os.environ.get('GAZE_CX', '0.5'), '--cy', os.environ.get('GAZE_CY', '0.5')]
    if os.environ.get('GAZE_CALIBRATION', '').lower() == 'manual':
        command.append('--manual')
    return subprocess.run(command).returncode


if __name__ == '__main__':
    sys.exit(main())
