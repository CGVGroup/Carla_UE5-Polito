#!/usr/bin/env python3
"""
replay_session.py: replays a session's CARLA recording (<session folder>/recording.log) in the running CARLA
server, the camera following the ego vehicle. For the interview's video-cued replays and for re-deriving
metrics (protocol XAI-HUD-EXP-01, Section 6: "CARLA recorder ... replayable .log").

The recording holds the world state only (actors, traffic lights, collisions), not the HUD / text display nor
the headset view: those are in the OBS video (paper/gaze_video.mp4 with the gaze).

Usage (CARLA running, e.g. .\\launch_carla.ps1 -Desktop):
    python tools/replay_session.py <session folder>                 the whole route
    python tools/replay_session.py <session folder> --event 3       from 5 s before event 3's t0 (the events of
                                                                    paper/event_coding_sheet.md: run
                                                                    analyze_session.py first)
    python tools/replay_session.py <session folder> --start 40 --duration 20   route time [s], as in the report
    python tools/replay_session.py <session folder> --info          what the recording holds (frames, collisions)
    options: --speed 0.5 (slow motion), --free-camera (no following), --host / --port
"""
import argparse
import datetime
import json
import os
import struct
import sys
import time

import carla

TOOLS = os.path.dirname(os.path.abspath(__file__))


def iso_seconds(text):
    try:
        return datetime.datetime.fromisoformat(str(text).strip().rstrip('Z')).timestamp()
    except (TypeError, ValueError):
        return None


def route_start_wall(session):
    """Wall time of the session's first logged frame (route time 0 of the reports)"""
    frames = os.path.join(session, 'frames.csv')
    if os.path.exists(frames):
        with open(frames, encoding='utf-8') as f:
            header = f.readline().strip().split(',')
            first = f.readline().strip().split(',')
        if 'wall_time' in header and len(first) == len(header):
            return iso_seconds(first[header.index('wall_time')])
    return None


def recording_start_wall(recording):
    """When the recording started [s, 1 s resolution]: the date of its header (CarlaRecorderInfo: uint16
    version, FString magic, time_t date, ...; FString = uint16 length + UTF-8)"""
    try:
        with open(recording, 'rb') as f:
            header = f.read(64)
        length = struct.unpack_from('<H', header, 2)[0]
        if header[4:4 + length] != b'CARLA_RECORDER':
            return None
        return float(struct.unpack_from('<q', header, 4 + length)[0])
    except (OSError, struct.error):
        return None


def recording_offset(session, recording):
    """Seconds from the start of the recording to route time 0 (None: unknown)"""
    start, recorded = route_start_wall(session), recording_start_wall(recording)
    return None if start is None or recorded is None else max(0.0, start - recorded)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', help='session folder (logs/sessions/<session>) or a .log file')
    ap.add_argument('--event', type=int, help='start 5 s before this event\'s t0 (paper/metrics.json)')
    ap.add_argument('--start', type=float, help='route time [s] to start from')
    ap.add_argument('--duration', type=float, default=0.0, help='seconds to replay (0: to the end)')
    ap.add_argument('--speed', type=float, default=1.0, help='replay speed (0.5: slow motion)')
    ap.add_argument('--free-camera', action='store_true', help="don't follow the ego vehicle")
    ap.add_argument('--info', action='store_true', help='print what the recording holds, and the collisions')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=2000)
    args = ap.parse_args()

    if args.session.lower().endswith('.log'):
        recording, session = os.path.abspath(args.session), os.path.dirname(os.path.abspath(args.session))
    else:
        session = os.path.abspath(args.session)
        recording = os.path.join(session, 'recording.log')
    if not os.path.exists(recording):
        print('No recording:', recording, '(RECORD_PATH set in run_leaderboard_xai.ps1? analyze_session.py also '
              'copies it from logs/recordings)')
        return 2

    client = carla.Client(args.host, args.port)
    client.set_timeout(60.0)
    if args.info:
        print(client.show_recorder_info(recording, False))
        print(client.show_recorder_collisions(recording, 'a', 'a'))
        return 0

    ego_id = 0
    summary_path = os.path.join(session, 'summary.json')
    if os.path.exists(summary_path) and not args.free_camera:
        with open(summary_path, encoding='utf-8') as f:
            ego_id = int(json.load(f).get('ego_id') or 0)

    offset = recording_offset(session, recording)
    start = args.start
    duration = args.duration
    if args.event is not None:
        metrics_path = os.path.join(session, 'paper', 'metrics.json')
        if not os.path.exists(metrics_path):
            print('No paper/metrics.json: run tools/analyze_session.py on the session first')
            return 2
        with open(metrics_path, encoding='utf-8') as f:
            events = (json.load(f).get('protocol') or {}).get('events') or []
        event = next((e for e in events if e.get('event') == args.event), None)
        if event is None:
            print('No event', args.event, '- events:', ', '.join('{} {}'.format(e['event'], e['scenario']) for e in events))
            return 2
        start = event['phase_f_start']
        duration = duration or (event['phase_f_end'] - event['phase_f_start'])
        print('Event {} {}{}: t0 {} s, replay from {} s for {:.0f} s'.format(
            event['event'], event['scenario'], ' (omission)' if event.get('omission') else '', event['t0'],
            start, duration))
    if start is not None and offset is None:
        print('Recording start unknown: times taken from the start of the recording')
    recording_start = max(0.0, (start or 0.0) + (offset or 0.0)) if start is not None else 0.0

    # The replayer plays in asynchronous mode (nobody ticks a synchronous server)
    world = client.get_world()
    settings = world.get_settings()
    if settings.synchronous_mode:
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)
    client.set_replayer_time_factor(args.speed)
    print(client.replay_file(recording, recording_start, duration, ego_id, False))
    if duration:
        time.sleep(duration / max(args.speed, 0.05) + 2.0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
