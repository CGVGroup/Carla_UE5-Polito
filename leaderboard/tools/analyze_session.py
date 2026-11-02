#!/usr/bin/env python3
"""
analyze_session.py: after a run, gathers everything about one route session for the paper, into
<session folder>/paper/ (run_leaderboard_xai.ps1 prints the exact command at its end; it can be run any time
later, again and again: the outputs are overwritten).

Inputs, found by themselves (or given):
    the session log         logs/sessions/<session>/ (frames.csv, events.jsonl, summary.json)
    the gaze recording      <CARLA Saved dir>/GazeLogs/<session name>_<date>.csv, the one written during the
                            session (packaged CARLA: %LOCALAPPDATA%/CarlaUnreal/Saved)
    the OBS video           the recording of the headset view made during the session (OBS's folder)

Outputs (paper/):
    report.md               readable summary of the run: condition, route, per scenario, per hazard, takeovers,
                            gaze. What to quote in the paper
    metrics.json            all the numbers below, machine readable (one file per run: merge runs from it)
    scenarios.csv           one row per scenario: duration, speeds, braking, collisions, takeovers, min TTC,
                            hazard visibility onset t0, takeover / glance reaction times, pupil diameter
    hazards.csv             one row per scenario road user (vehicle, bike, pedestrian of a scenario): when it
                            came in range / became visible (t0), when the system detected it and when its box
                            was drawn, distance and TTC at t0, min gap / TTC, takeover reaction time from t0,
                            first glance at it from t0, dwell time on it
    takeovers.csv           every takeover: start, duration, cause, scenario, speed, time from the scenario's
                            hazard onset (session log), and the server's own takeover state changes (gaze file)
    check_report.txt        consistency checks of the logs (tools/check_session_log.py)
    gaze_*.png, gaze_aoi.csv   gaze visualizations, dwell / fixations per area of interest (tools/visualize_gaze.py)
    gaze_video.mp4          the OBS video with the gaze on top (tools/gaze_video_overlay.py; slow: --no-video)
    session_video.mp4       instead, without eye data: the same video without the gaze marker
    inputs.txt              the files used
    recording.log           CARLA's recording of the route (replayable: tools/replay_session.py <session>), moved
                            here by run_leaderboard_xai.ps1 (or found in logs/recordings by time)

Protocol XAI-HUD-EXP-01 v2.0, Part III (tools/protocol_analysis.py for the definitions):
    event_coding_sheet.csv / .md   Appendix B: per event, time, road users, detected by the system, driver
                            reaction, collision (filled from the logs)
    protocol_events.csv     Section 10, per event: omission, t0, detection, reaction time, gaze-to-hazard latency,
                            min TTC, peak decel. / lat. acc., collision, lane exceedance, unnecessary intervention
    figures/fig_drive_timeline.png   the drive: speed, brake (automation / driver), TTC, Phase F windows, takeovers
    figures/fig_event_<n>_<scenario>.png   per event, from t0: speed, brake, steer, TTC, gaze on the road user
    metrics.json 'protocol'  the above and Phase E gaze: monitoring ratio, glances > 2 s on the display
    Merge the drives of the study: tools/aggregate_study.py (Section 12 tables and figures)

Times: simulation seconds (sim_time), except gaze dwell / fixation durations (real time: the eyes move in
real time). Gaze samples are matched to the session log by CARLA frame.

Usage:
    python tools/analyze_session.py <session folder> [--participant P1 --drive 1] [--no-video]
    python tools/analyze_session.py --latest
    options: --gaze <csv>  --video <file>  --hfov 100 --cx 0.72 --cy 0.5 [--manual]  (gaze video calibration,
             default: GAZE_HFOV / GAZE_CX / GAZE_CY / GAZE_CALIBRATION, or measured from the SYNC cue)
"""
import argparse
import bisect
import csv
import datetime
import glob
import json
import os
import shutil
import subprocess
import re
import sys

TOOLS = os.path.dirname(os.path.abspath(__file__))
LEADERBOARD = os.path.dirname(TOOLS)
sys.path.insert(0, TOOLS)
from make_session_gaze_video import obs_video_folder, wait_complete  # noqa: E402
import protocol_analysis  # noqa: E402

# Visualization mode of the routes XML -> condition of the protocol (XAI-HUD-EXP-01)
CONDITIONS = {'none': 'N', 'full': 'A', 'text': 'T'}


def fnum(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def iso_seconds(text):
    """Seconds of an ISO 8601 time (local time; Unreal's trailing 'Z' is ignored), None if missing"""
    try:
        return datetime.datetime.fromisoformat(str(text).strip().rstrip('Z')).timestamp()
    except (TypeError, ValueError):
        return None


def r(value, digits=3):
    return None if value is None else round(value, digits)


def read_csv(path):
    with open(path, newline='', encoding='utf-8') as f:
        return list(csv.DictReader(f))


def write_csv(path, rows, columns=None):
    columns = columns or (list(rows[0].keys()) if rows else ['none'])
    with open(path, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)


# ---------------------------------------------------------------------------------------------- inputs

def latest_session():
    root = os.environ.get('SESSION_LOG_DIR') or os.path.join(LEADERBOARD, 'logs', 'sessions')
    sessions = [d for d in glob.glob(os.path.join(root, '*')) if os.path.isfile(os.path.join(d, 'frames.csv'))]
    return max(sessions, key=os.path.getmtime) if sessions else None


def find_gaze(name, start, end):
    """The gaze file of the session: <name>_<date>.csv, written between the session's start and end"""
    unreal_project = os.environ.get('CARLA_UE_ROOT', r'C:\Users\USER\Documents\GitHub\Carla_UE5-Polito')
    folders = [os.path.join(os.environ.get('LOCALAPPDATA', ''), 'CarlaUnreal', 'Saved', 'GazeLogs'),
               os.path.join(unreal_project, 'Unreal', 'CarlaUnreal', 'Saved', 'GazeLogs')]
    candidates = []
    for folder in folders:
        for path in glob.glob(os.path.join(folder, name + '_*.csv')):
            modified = os.path.getmtime(path)
            if start is None or end is None or start - 60 <= modified <= end + 600:
                candidates.append(path)
    if not candidates:
        return None
    return min(candidates, key=lambda p: abs(os.path.getmtime(p) - (end or os.path.getmtime(p))))


def find_recording(directory, start, end):
    """CARLA's recording of the session: recording.log in the session folder, else the one of logs/recordings
    written during the session (copied into the session folder)"""
    own = os.path.join(directory, 'recording.log')
    if os.path.exists(own):
        return own
    if start is None or end is None:
        return None
    folder = os.environ.get('RECORD_PATH') or os.path.join(LEADERBOARD, 'logs', 'recordings')
    candidates = [p for p in glob.glob(os.path.join(folder, '*.log')) if start - 5 <= os.path.getmtime(p) <= end + 120]
    if not candidates:
        return None
    source = min(candidates, key=lambda p: abs(os.path.getmtime(p) - end))
    shutil.copy2(source, own)
    return own


def route_code(routes_file):
    """R1 / R2 / R3 of the protocol from the routes file name (..._long<n>...), else the file name"""
    name = os.path.splitext(os.path.basename(routes_file or ''))[0]
    match = re.search(r'long(\d+)', name)
    return 'R{}'.format(match.group(1)) if match else name


def find_video(start, end):
    """The OBS recording running during the session: started before its end, written to after its start"""
    if start is None or end is None:
        return None
    videos = []
    for folder in obs_video_folder():
        for extension in ('*.mkv', '*.mp4', '*.mov', '*.flv'):
            for video in glob.glob(os.path.join(folder, extension)):
                if video.endswith('_gaze.mp4') or '_preview_' in video:
                    continue
                if os.path.getctime(video) <= end + 5 and os.path.getmtime(video) >= start:
                    videos.append(video)
    # The one started closest before the session (OBS is started before the route)
    return min(videos, key=lambda v: abs(os.path.getctime(v) - start)) if videos else None


# ---------------------------------------------------------------------------------------------- analysis

class Session:
    def __init__(self, directory):
        self.directory = directory
        self.summary = {}
        summary_path = os.path.join(directory, 'summary.json')
        if os.path.exists(summary_path):
            with open(summary_path, encoding='utf-8') as f:
                self.summary = json.load(f)
        self.frames = read_csv(os.path.join(directory, 'frames.csv'))
        self.events = []
        events_path = os.path.join(directory, 'events.jsonl')
        if os.path.exists(events_path):
            with open(events_path, encoding='utf-8') as f:
                for line in f:
                    try:
                        self.events.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
        self.frame_numbers = [int(row['frame']) for row in self.frames]
        # Times from the start of the route (the first frame logged), not the simulator's own clock
        self.origin = fnum(self.frames[0]['sim_time'], 0.0) if self.frames else 0.0
        self.frame_times = [self.rel(row['sim_time']) for row in self.frames]
        for event in self.events:
            event['sim_time'] = self.rel(event.get('sim_time'))
        self.start = iso_seconds(self.summary.get('start')) or iso_seconds(self.frames[0]['wall_time'] if self.frames else None)
        self.end = iso_seconds(self.summary.get('end')) or iso_seconds(self.frames[-1]['wall_time'] if self.frames else None)

    def rel(self, sim_time):
        """Simulator time -> seconds from the start of the route (None stays None)"""
        value = fnum(sim_time)
        return None if value is None else round(value - self.origin, 3)

    def sim_time_of_frame(self, frame):
        """Session sim time of a CARLA frame (the closest frame logged before it)"""
        if not self.frame_numbers:
            return None
        i = max(0, bisect.bisect_right(self.frame_numbers, frame) - 1)
        return self.frame_times[i]

    def scenario_windows(self):
        """[(name, type, start, end)] from the scenario start / end events (sim time)"""
        windows, open_ = [], {}
        last_time = self.frame_times[-1] if self.frame_times else None
        for event in self.events:
            if event.get('event') == 'scenario_start':
                open_[event.get('name')] = (event.get('type', ''), event.get('sim_time'))
            elif event.get('event') == 'scenario_end' and event.get('name') in open_:
                kind, start = open_.pop(event.get('name'))
                windows.append((event.get('name'), kind, start, event.get('sim_time')))
        for name, (kind, start) in open_.items():
            windows.append((name, kind, start, last_time))
        return sorted(windows, key=lambda w: w[2] if w[2] is not None else 0.0)


def first_frame_time(session, column, actor_id):
    """Sim time of the first frame whose 'id:...|id:...' column lists the actor"""
    prefix = '{}:'.format(actor_id)
    for row in session.frames:
        value = row.get(column) or ''
        if any(item.startswith(prefix) for item in value.split('|')):
            return session.rel(row['sim_time'])
    return None


def analyze_gaze(session, gaze_rows):
    """Per sample: (sim time, wall seconds, valid, hit actor id, left / right pupil); server takeovers"""
    samples = []
    for row in gaze_rows:
        frame = int(fnum(row.get('frame'), -1))
        samples.append({
            'sim_time': session.sim_time_of_frame(frame) if frame >= 0 else None,
            'wall': iso_seconds(row.get('wall_time')),
            'valid': row.get('gaze_valid') == '1',
            'actor': int(fnum(row.get('hit_actor_id'), -1)) if row.get('hit') == '1' else -1,
            'pupil': [p for p in (fnum(row.get('left_pupil')), fnum(row.get('right_pupil'))) if p and p > 0],
            'state': row.get('automation_state', ''),
            'cause': row.get('takeover_cause', ''),
        })
    # The server's own takeover state changes (exact, every rendered frame)
    server_takeovers, previous = [], 'engaged'
    for s in samples:
        if s['state'] and s['state'] != previous:
            if s['state'] == 'driver':
                server_takeovers.append({'sim_time': r(s['sim_time']), 'cause': s['cause'], 'end_sim_time': None})
            elif server_takeovers and server_takeovers[-1]['end_sim_time'] is None:
                server_takeovers[-1]['end_sim_time'] = r(s['sim_time'])
            previous = s['state']
    return samples, server_takeovers


def glance_stats(samples, actor_id, t0, until):
    """First glance at the actor from t0 (sim time, latency) and dwell on it [real s], number of glances"""
    first, dwell, glances, on = None, 0.0, 0, False
    for i, s in enumerate(samples):
        if s['sim_time'] is None or s['sim_time'] < t0 or (until is not None and s['sim_time'] > until):
            on = False
            continue
        hit = s['valid'] and s['actor'] == actor_id
        if hit:
            if first is None:
                first = s['sim_time']
            if i + 1 < len(samples) and s['wall'] is not None and samples[i + 1]['wall'] is not None:
                dwell += max(0.0, min(0.2, samples[i + 1]['wall'] - s['wall']))
            if not on:
                glances += 1
        on = hit
    return first, dwell, glances


def mean(values):
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else None


def build_metrics(session, gaze_rows, participant, drive_number=None, route_label=None):
    summary = session.summary
    mode = summary.get('visualization') or (session.frames[0].get('dashboard_mode') if session.frames else '')
    events = session.events
    takeover_events = [e for e in events if e.get('event') == 'takeover_start']
    windows = session.scenario_windows()
    samples, server_takeovers = analyze_gaze(session, gaze_rows) if gaze_rows else ([], [])
    hazard_events = {e.get('actor_id'): e for e in events if e.get('event') == 'hazard_visible'}

    def window_of(time):
        """The scenario a road user belongs to: the one running when it came in range, else the next one to
        start (road users come in range, up to 100 m ahead, before the ego reaches their scenario's trigger)"""
        if time is None:
            return ''
        for name, kind, start, end in windows:
            if start is not None and start <= time <= (end if end is not None else time):
                return name
        upcoming = [w for w in windows if w[2] is not None and w[2] > time]
        return min(upcoming, key=lambda w: w[2])[0] if upcoming else ''

    # Per scenario road user
    hazards = []
    for actor_id, stats in (summary.get('scenario_actors') or {}).items():
        actor_id = int(actor_id)
        stats = dict(stats)
        for key in ('first_in_range', 'visible_at', 'min_ttc_time'):
            stats[key] = session.rel(stats.get(key))
        scenario = window_of(stats.get('first_in_range'))
        end = next((w[3] for w in windows if w[0] == scenario), None)
        t0 = stats.get('visible_at')
        detected_at = first_frame_time(session, 'detections', actor_id)
        box_at = first_frame_time(session, 'rendered_boxes', actor_id)
        takeover = next((e for e in takeover_events if t0 is not None and e.get('sim_time') is not None
                         and t0 <= e['sim_time'] <= (end if end is not None else e['sim_time'])), None)
        first_glance, dwell, glances = glance_stats(samples, actor_id, t0, end) if samples and t0 is not None \
            else (None, None, None)
        hazards.append({
            'actor_id': actor_id, 'type': stats.get('type'), 'scenario': scenario,
            'first_in_range': stats.get('first_in_range'), 'visible_at_t0': t0,
            'distance_at_t0': stats.get('distance_at_visible'), 'ttc_at_t0': stats.get('ttc_at_visible'),
            'detected_at_t0': bool((hazard_events.get(actor_id) or {}).get('detected')),
            'detected_by_system': detected_at is not None,
            'system_detected_at': detected_at,
            'detection_latency_from_t0': r(detected_at - t0) if detected_at is not None and t0 is not None else None,
            'box_drawn_at': box_at,
            'min_gap': stats.get('min_gap'), 'min_ttc': stats.get('min_ttc'), 'min_ttc_time': stats.get('min_ttc_time'),
            'takeover_after_t0': takeover is not None,
            'takeover_reaction_time': r(takeover['sim_time'] - t0) if takeover else None,
            'takeover_cause': takeover.get('cause') if takeover else None,
            'first_glance_at': r(first_glance),
            'glance_latency_from_t0': r(first_glance - t0) if first_glance is not None else None,
            'dwell_on_actor_s': r(dwell), 'glances': glances,
        })
    hazards.sort(key=lambda h: (h['first_in_range'] is None, h['first_in_range']))

    # Per scenario
    scenario_stats = summary.get('scenarios') or {}
    scenarios = []
    for name, kind, start, end in windows:
        stats = scenario_stats.get(name, {})
        in_scenario = [h for h in hazards if h['scenario'] == name]
        onsets = [h['visible_at_t0'] for h in in_scenario if h['visible_at_t0'] is not None]
        t0 = min(onsets) if onsets else None
        takeover = next((e for e in takeover_events if e.get('scenario') == name), None)
        pupils = [sum(s['pupil']) / len(s['pupil']) for s in samples
                  if s['valid'] and s['pupil'] and s['sim_time'] is not None and start is not None
                  and start <= s['sim_time'] <= (end if end is not None else s['sim_time'])]
        glance_latencies = [h['glance_latency_from_t0'] for h in in_scenario if h['glance_latency_from_t0'] is not None]
        scenarios.append({
            'scenario': name, 'type': kind, 'start': r(start), 'end': r(end),
            'duration': stats.get('duration'), 'distance': stats.get('distance'),
            'mean_speed_kmh': stats.get('mean_speed_kmh'), 'min_speed_kmh': stats.get('min_speed_kmh'),
            'max_decel': stats.get('max_decel'), 'hard_brakes': stats.get('hard_brakes'),
            'emergency_stops': stats.get('emergency_stops'), 'collisions': stats.get('collisions'),
            'lane_invasions': stats.get('lane_invasions'), 'min_scenario_ttc': stats.get('min_scenario_ttc'),
            'min_vehicle_dist': stats.get('min_vehicle_dist'), 'min_pedestrian_dist': stats.get('min_pedestrian_dist'),
            'mean_abs_lateral_offset': stats.get('mean_abs_lateral_offset'),
            'scenario_road_users': len(in_scenario),
            'hazard_onset_t0': r(t0),
            'takeovers': stats.get('takeovers'), 'takeover_time': stats.get('takeover_time'),
            'first_takeover_at': takeover.get('sim_time') if takeover else None,
            'first_takeover_cause': takeover.get('cause') if takeover else None,
            'takeover_reaction_time_from_t0': r(takeover['sim_time'] - t0) if takeover and t0 is not None
            and takeover.get('sim_time') is not None and takeover['sim_time'] >= t0 else None,
            'first_glance_latency_from_t0': r(min(glance_latencies)) if glance_latencies else None,
            'mean_pupil_mm': r(mean(pupils)),
        })

    # Takeovers
    takeovers = []
    for e in takeover_events:
        end = next((x for x in events if x.get('event') == 'takeover_end' and x.get('sim_time') is not None
                    and e.get('sim_time') is not None and x['sim_time'] >= e['sim_time']), None)
        scenario_row = next((s for s in scenarios if s['scenario'] == e.get('scenario')), None)
        t0 = scenario_row['hazard_onset_t0'] if scenario_row else None
        takeovers.append({
            'start': e.get('sim_time'), 'duration': end.get('duration') if end else None,
            'until_route_end': bool(end and end.get('until_end')), 'cause': e.get('cause'),
            'speed_kmh': e.get('speed_kmh'), 'scenario': e.get('scenario'),
            'time_from_hazard_onset_t0': r(e['sim_time'] - t0) if t0 is not None and e.get('sim_time') is not None else None,
        })

    # Gaze, whole route
    gaze = None
    if gaze_rows:
        valid = [s for s in samples if s['valid']]
        walls = [s['wall'] for s in samples if s['wall'] is not None]
        span = (walls[-1] - walls[0]) if len(walls) > 1 else None
        gaze = {'samples': len(samples), 'valid_ratio': r(len(valid) / len(samples)) if samples else None,
                'sample_rate_hz': r(len(samples) / span, 1) if span else None,
                'mean_pupil_mm': r(mean([sum(s['pupil']) / len(s['pupil']) for s in valid if s['pupil']])),
                'server_takeovers': server_takeovers}

    route = summary.get('route') or {}
    return {
        'participant': participant, 'drive': drive_number, 'session': os.path.basename(session.directory),
        'route_code': route_label or route_code(summary.get('routes_file')),
        'condition': CONDITIONS.get((mode or '').lower(), mode), 'visualization_mode': mode,
        'routes_file': summary.get('routes_file'), 'start': summary.get('start'), 'end': summary.get('end'),
        'ego_type': summary.get('ego_type'),
        'route': {k: route.get(k) for k in (
            'duration', 'distance', 'mean_speed_kmh', 'max_speed_kmh', 'max_decel', 'hard_brakes', 'collisions',
            'lane_invasions', 'emergency_stops', 'takeovers', 'takeover_time', 'takeover_ratio', 'stopped_time',
            'min_vehicle_dist', 'min_pedestrian_dist', 'min_lead_ttc', 'min_scenario_ttc', 'mean_abs_lateral_offset')},
        'collisions': summary.get('collisions') or [],
        'scenarios': scenarios, 'hazards': hazards, 'takeovers': takeovers, 'gaze': gaze,
    }


# ---------------------------------------------------------------------------------------------- report

def fmt(value, unit='', digits=2):
    if value is None or value == '':
        return '-'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    if isinstance(value, float):
        return '{:.{}f}{}'.format(value, digits, unit)
    return '{}{}'.format(value, unit)


def table(columns, rows):
    lines = ['| ' + ' | '.join(title for title, _ in columns) + ' |', '|' + '---|' * len(columns)]
    for row in rows:
        lines.append('| ' + ' | '.join(fmt(row.get(key)) for _, key in columns) + ' |')
    return '\n'.join(lines)


def write_report(path, m, outputs):
    route = m['route']
    lines = [
        '# Run {}'.format(m['session']), '',
        '- Participant: {}'.format(m['participant'] or '-'),
        '- Condition: **{}** (visualization mode `{}`)'.format(m['condition'], m['visualization_mode']),
        '- Route file: {}'.format(m['routes_file']),
        '- Start / end: {} / {}'.format(m['start'], m['end']),
        '- Ego vehicle: {}'.format(m['ego_type']),
        '- Drive: {} · route {}'.format(m.get('drive') or '-', m.get('route_code') or '-'), '',
        '## Route', '',
        '- Duration {} s, distance {} m, mean speed {} km/h (max {})'.format(
            fmt(route['duration'], digits=1), fmt(route['distance'], digits=0), fmt(route['mean_speed_kmh'], digits=1),
            fmt(route['max_speed_kmh'], digits=1)),
        '- Takeovers {} ({} s under driver control, {} of the frames)'.format(
            fmt(route['takeovers']), fmt(route['takeover_time'], digits=1), fmt(route['takeover_ratio'])),
        '- Collisions {}, lane invasions {}, hard brakes {}, emergency stops {}'.format(
            fmt(route['collisions']), fmt(route['lane_invasions']), fmt(route['hard_brakes']), fmt(route['emergency_stops'])),
        '- Max deceleration {} m/s2, min TTC with a scenario road user {} s, min distance to a pedestrian {} m'.format(
            fmt(route['max_decel']), fmt(route['min_scenario_ttc']), fmt(route['min_pedestrian_dist'])), '',
        '## Scenarios', '',
        table([('Scenario', 'scenario'), ('t0 [s]', 'hazard_onset_t0'), ('Takeover RT from t0 [s]', 'takeover_reaction_time_from_t0'),
               ('Cause', 'first_takeover_cause'), ('1st glance from t0 [s]', 'first_glance_latency_from_t0'),
               ('Min TTC [s]', 'min_scenario_ttc'), ('Max decel', 'max_decel'), ('Collisions', 'collisions'),
               ('Pupil [mm]', 'mean_pupil_mm')], m['scenarios']), '',
        '## Scenario road users (hazards)', '',
        table([('Id', 'actor_id'), ('Type', 'type'), ('Scenario', 'scenario'), ('t0 [s]', 'visible_at_t0'),
               ('Dist t0 [m]', 'distance_at_t0'), ('TTC t0 [s]', 'ttc_at_t0'), ('Detected', 'detected_by_system'),
               ('Detected at t0', 'detected_at_t0'),
               ('Detection - t0 [s]', 'detection_latency_from_t0'), ('Takeover RT [s]', 'takeover_reaction_time'),
               ('Glance - t0 [s]', 'glance_latency_from_t0'), ('Dwell [s]', 'dwell_on_actor_s'), ('Min TTC [s]', 'min_ttc')],
              m['hazards']), '',
        '## Takeovers', '',
        table([('Start [s]', 'start'), ('Duration [s]', 'duration'), ('Cause', 'cause'), ('Speed [km/h]', 'speed_kmh'),
               ('Scenario', 'scenario'), ('From t0 [s]', 'time_from_hazard_onset_t0')], m['takeovers']), '',
        '## Gaze', '',
    ]
    gaze = m['gaze']
    if gaze:
        lines += ['- {} samples ({} Hz), valid {}, mean pupil {}'.format(
            gaze['samples'], fmt(gaze['sample_rate_hz'], digits=0), fmt(gaze['valid_ratio']),
            fmt(gaze['mean_pupil_mm'], ' mm') if gaze['mean_pupil_mm'] is not None else 'not given by the headset'),
            '- Areas of interest (dwell, fixations): gaze_aoi.csv; plots: gaze_*_map / _timeline / _heatmap.png']
    else:
        lines.append('- No gaze recording for this run')
    protocol = m.get('protocol')
    if protocol:
        lines += ['', '## Protocol XAI-HUD-EXP-01 — event coding sheet (Appendix B)', '',
                  table([('#', 'event'), ('Time t0 [s]', 't0'), ('Event / actor', 'event_actor'),
                         ('Detected by system', 'detected_by_system'), ('Driver reaction', 'driver_reaction'),
                         ('Collision', 'collision'), ('Notes', 'notes')], protocol['coding_sheet']), '',
                  '## Protocol — Section 10 metrics per event', '',
                  table([('#', 'event'), ('Omission', 'omission'), ('t0 [s]', 't0'), ('Detection', 'driver_reaction'),
                         ('RT [s]', 'reaction_time'), ('Input', 'reaction_input'), ('Gaze lat. [s]', 'gaze_latency'),
                         ('Min TTC [s]', 'min_ttc'), ('Peak decel [m/s2]', 'peak_decel_after_reaction'),
                         ('Peak lat [m/s2]', 'peak_lat_acc_after_reaction'), ('Collision', 'collision'),
                         ('Lane exceed.', 'lane_exceedance'), ('Unnecessary', 'unnecessary_intervention')],
                        protocol['events']), '']
        summary = protocol['summary']
        lines += ['- {} events, {} omissions; omissions detected by the driver {}/{}; unnecessary interventions {}/{} '
                  'handled events; observed system reliability {}'.format(
                      summary['events'], summary['omissions'], summary['omissions_detected'], summary['omissions'],
                      summary['unnecessary_interventions'], summary['events'] - summary['omissions'],
                      fmt(summary['observed_reliability']))]
        for phase, label in (('E', 'Phase E (nominal driving)'), ('F', 'Phase F (event windows)')):
            g = (protocol.get('gaze') or {}).get(phase)
            if g:
                lines.append('- {}: monitoring ratio {} (road {} s, display {} s, in-vehicle {} s), display glances {} '
                             '(> 2 s: {}, mean {} s)'.format(
                                 label, fmt(g['monitoring_ratio']), fmt(g['road_s'], digits=1),
                                 fmt(g['display_s'], digits=1), fmt(g['in_vehicle_s'], digits=1),
                                 g['display_glances'], g['display_glances_over_2s'], fmt(g['display_glance_mean_s'])))
        if protocol.get('gaze'):
            lines.append('- Display AOI: {}'.format(protocol['gaze'].get('display_aoi')))
        lines += ['- Figures: figures/fig_drive_timeline.png, figures/fig_event_<n>_<scenario>.png']
    if m.get('recording'):
        lines += ['', '## CARLA recording', '', '- {}'.format(m['recording']),
                  '- Replay (CARLA running): `python tools\\replay_session.py "{}"` (--event <n>: from that event)'.format(
                      os.path.dirname(m['recording']))]
    lines += ['', '## Files', ''] + ['- {}: {}'.format(name, value) for name, value in outputs.items()]
    lines += ['', 'Times are simulation seconds from the start of the session log, except dwell times (real time). '
              't0: first frame the driver has a line of sight to the scenario road user (session logger).']
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')


# ---------------------------------------------------------------------------------------------- main

def run(command, log_path=None):
    print('>', ' '.join('"{}"'.format(c) if ' ' in c else c for c in command))
    result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8',
                            errors='replace')
    print(result.stdout)
    if log_path:
        with open(log_path, 'w', encoding='utf-8') as f:
            f.write(result.stdout)
    return result.returncode


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('session', nargs='?', help='session folder (logs/sessions/<session>)')
    ap.add_argument('--latest', action='store_true', help='the most recent session')
    ap.add_argument('--participant', default=os.environ.get('PARTICIPANT', ''), help='participant code (P1..P6)')
    ap.add_argument('--drive', default=os.environ.get('DRIVE', ''), help='drive number of the session (1..3)')
    ap.add_argument('--route', default='', help='route code (default: R<n> from the routes file ..._long<n>...)')
    ap.add_argument('--gaze', help='gaze CSV (default: found by name and time)')
    ap.add_argument('--video', help='OBS video (default: found by time)')
    ap.add_argument('--no-video', action='store_true', help="don't make the gaze video (the slow part)")
    ap.add_argument('--hfov', default=os.environ.get('GAZE_HFOV', '100'))
    ap.add_argument('--cx', default=os.environ.get('GAZE_CX', '0.5'))
    ap.add_argument('--cy', default=os.environ.get('GAZE_CY', '0.5'))
    ap.add_argument('--manual', action='store_true', default=os.environ.get('GAZE_CALIBRATION', '').lower() == 'manual',
                    help='gaze video calibration as given (not measured from the SYNC cue)')
    args = ap.parse_args()

    directory = args.session if args.session and not args.latest else latest_session()
    if not directory or not os.path.isfile(os.path.join(directory, 'frames.csv')):
        print('No session folder (with frames.csv):', directory)
        return 2
    directory = os.path.abspath(directory)
    out = os.path.join(directory, 'paper')
    os.makedirs(out, exist_ok=True)
    session = Session(directory)
    name = session.summary.get('name') or os.path.basename(directory).rsplit('_', 2)[0]
    print('Session:', directory)

    gaze_path = args.gaze or find_gaze(name, session.start, session.end)
    gaze_rows = read_csv(gaze_path) if gaze_path else []
    has_eye_data = any(row.get('gaze_valid') == '1' for row in gaze_rows)
    print('Gaze file:', gaze_path or 'none', '' if has_eye_data or not gaze_path else '(no valid eye data)')
    video = args.video or (None if args.no_video else find_video(session.start, session.end))
    print('OBS video:', video or 'none')

    outputs = {'session log': directory, 'gaze file': gaze_path or 'none', 'OBS video': video or 'none'}

    # 1. Consistency of the logs
    command = [sys.executable, os.path.join(TOOLS, 'check_session_log.py'), directory]
    if gaze_path:
        command += ['--gaze', gaze_path]
    run(command, os.path.join(out, 'check_report.txt'))
    outputs['checks'] = 'check_report.txt'

    # 2. Gaze plots and areas of interest
    if has_eye_data:
        run([sys.executable, os.path.join(TOOLS, 'visualize_gaze.py'), gaze_path, '--session', directory, '--out', out])
        stem = os.path.splitext(os.path.basename(gaze_path))[0]
        for suffix in ('_map.png', '_timeline.png', '_heatmap.png', '_aoi.csv'):
            source = os.path.join(out, stem + suffix)
            if os.path.exists(source):
                os.replace(source, os.path.join(out, 'gaze' + suffix))
        outputs['gaze plots'] = 'gaze_map.png, gaze_timeline.png, gaze_heatmap.png, gaze_aoi.csv'

    # 3. Metrics for the paper
    metrics = build_metrics(session, gaze_rows if has_eye_data else [], args.participant, args.drive or None,
                            args.route or None)
    recording = find_recording(directory, session.start, session.end)
    metrics['recording'] = recording
    outputs['CARLA recording'] = recording or 'none (RECORD_PATH empty?)'
    print('CARLA recording:', recording or 'none')

    # Protocol XAI-HUD-EXP-01, Part III (driver inputs from the gaze file even without eye data)
    gaze_for_protocol = gaze_rows if has_eye_data else []
    events, drive = protocol_analysis.build_events(session, metrics, gaze_rows)
    protocol_gaze = protocol_analysis.phase_e_gaze(drive, events, gaze_for_protocol, metrics['visualization_mode']) \
        if gaze_for_protocol else None
    coding_sheet = [{
        'event': e['event'], 't0': e['t0'],
        'event_actor': '{} ({})'.format(e['scenario'], e['actors'] or 'no road user'),
        'detected_by_system': 'N' if e['omission'] else 'Y',
        'driver_reaction': 'Y' if e['driver_reaction'] else 'N', 'collision': 'Y' if e['collision'] else 'N',
        'notes': '; '.join(n for n in (
            'OMISSION #{}'.format(e.get('omission_index')) if e['omission'] else '',
            'RT {} s ({})'.format(e['reaction_time'], e['reaction_input']) if e['reaction_time'] is not None else '',
            'unnecessary intervention' if e['unnecessary_intervention'] else '',
            'min TTC {} s'.format(e['min_ttc']) if e['min_ttc'] is not None else '',
            'omitted actor shown on the display!' if e['omission'] and e['shown_on_display'] else '',
            'handled, but its road users not in the detection list' if not e['omission'] and e['actors']
            and not e['in_detection_list'] else '',
            't0 = scenario start' if e['t0_source'] != 'line of sight' else '') if n),
    } for e in events]
    omissions = [e for e in events if e['omission']]
    handled = [e for e in events if not e['omission']]
    metrics['protocol'] = {
        'events': events, 'coding_sheet': coding_sheet, 'gaze': protocol_gaze,
        'summary': {
            'events': len(events), 'omissions': len(omissions),
            'omissions_detected': sum(e['driver_reaction'] for e in omissions),
            'unnecessary_interventions': sum(e['unnecessary_intervention'] for e in handled),
            'unnecessary_intervention_rate': r(sum(e['unnecessary_intervention'] for e in handled) / len(handled))
            if handled else None,
            'observed_reliability': r(len(handled) / len(events)) if events else None,
            'collisions': sum(e['collision'] for e in events),
        },
        'parameters': {k: getattr(protocol_analysis, k) for k in (
            'EVENT_PRE_S', 'EVENT_POST_S', 'BRAKE_THRESHOLD', 'STEER_THRESHOLD_DEG', 'INPUT_LAG_S', 'LOWPASS_S',
            'LANE_EXCEEDANCE_M', 'LONG_GLANCE_S', 'IN_VEHICLE_PITCH_DEG')},
    }
    sheet_columns = ['event', 't0', 'event_actor', 'detected_by_system', 'driver_reaction', 'collision', 'notes']
    write_csv(os.path.join(out, 'event_coding_sheet.csv'), coding_sheet, sheet_columns)
    event_columns = sorted({k for e in events for k in e}, key=lambda k: list(events[0]).index(k)
                           if k in events[0] else len(events[0])) if events else None
    write_csv(os.path.join(out, 'protocol_events.csv'), events, event_columns)
    with open(os.path.join(out, 'event_coding_sheet.md'), 'w', encoding='utf-8') as f:
        f.write('Participant {} · Drive {} · Interface {} · Route {}\n\n'.format(
            metrics['participant'] or '____', metrics['drive'] or '____', metrics['condition'], metrics['route_code']))
        f.write(table([('#', 'event'), ('Time [s]', 't0'), ('Event / actor', 'event_actor'),
                       ('Detected by system', 'detected_by_system'), ('Driver reaction', 'driver_reaction'),
                       ('Collision', 'collision'), ('Notes', 'notes')], coding_sheet) + '\n')
    figure_files = protocol_analysis.figures(drive, events, gaze_for_protocol, metrics, os.path.join(out, 'figures'))
    outputs['protocol'] = 'event_coding_sheet.csv/.md, protocol_events.csv, figures/ ({} figures)'.format(
        len(figure_files))
    with open(os.path.join(out, 'metrics.json'), 'w', encoding='utf-8') as f:
        json.dump(metrics, f, indent=2, default=str)
    write_csv(os.path.join(out, 'scenarios.csv'), metrics['scenarios'])
    write_csv(os.path.join(out, 'hazards.csv'), metrics['hazards'])
    write_csv(os.path.join(out, 'takeovers.csv'), metrics['takeovers'],
              ['start', 'duration', 'until_route_end', 'cause', 'speed_kmh', 'scenario', 'time_from_hazard_onset_t0'])
    outputs['metrics'] = 'metrics.json, scenarios.csv, hazards.csv, takeovers.csv'

    # The run's console log and leaderboard results, when written during the session
    for source, target in ((os.path.join(LEADERBOARD, 'run_log.txt'), 'run_log.txt'),
                           (os.path.join(LEADERBOARD, 'results.json'), 'leaderboard_results.json')):
        if os.path.exists(source) and session.start is not None and os.path.getmtime(source) >= session.start:
            shutil.copy2(source, os.path.join(out, target))
            outputs[target] = target

    # 4. Gaze video (slow)
    # Without eye data: the same video without the gaze marker (session_video.mp4); without a gaze file at all,
    # the OBS recording as is, converted to MP4
    video_name = 'gaze_video.mp4' if has_eye_data else 'session_video.mp4'
    if video and not args.no_video:
        if not wait_complete(video):
            print('The OBS video is still being written: stop the OBS recording, then run this again')
        elif not gaze_path:
            print('No gaze file: the OBS video copied as is to', video_name)
            try:
                subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', video, '-c', 'copy',
                                os.path.join(out, video_name)], check=True)
                outputs['session video'] = video_name
            except (OSError, subprocess.CalledProcessError) as e:
                print('(ffmpeg failed: {}; the OBS video is {})'.format(e, video))
        else:
            command = [sys.executable, os.path.join(TOOLS, 'gaze_video_overlay.py'), video, gaze_path,
                       '--out', os.path.join(out, video_name), '--hfov', str(args.hfov),
                       '--cx', str(args.cx), '--cy', str(args.cy)]
            # The SYNC cue shows when the route starts: that far into the video when OBS was already recording
            # before (e.g. not stopped after an interrupted run), a minute of margin
            gaze_start = None
            for row in gaze_rows:
                gaze_start = iso_seconds(row.get('wall_time'))
                if gaze_start is not None:
                    break
            lead = (gaze_start or session.start or 0.0) - os.path.getctime(video)
            if lead > 120.0:
                print('The OBS video started {:.0f} s before the route: SYNC cue looked for from there'.format(lead))
                command += ['--sync-search-from', '{:.0f}'.format(lead - 60.0)]
            if args.manual:
                command.append('--manual')
            if not has_eye_data:
                command.append('--sync-file-time')
            if run(command) == 0:
                outputs['gaze video' if has_eye_data else 'session video'] = video_name

    write_report(os.path.join(out, 'report.md'), metrics, outputs)
    with open(os.path.join(out, 'inputs.txt'), 'w', encoding='utf-8') as f:
        f.write('\n'.join('{}: {}'.format(k, v) for k, v in outputs.items()) + '\n')
    print('\nPaper outputs in', out)
    print('  report.md (summary), event_coding_sheet.md, protocol_events.csv, figures/, metrics.json, '
          'scenarios.csv, hazards.csv, takeovers.csv' +
          ''.join(', ' + outputs[k] for k in ('gaze video', 'session video') if k in outputs))
    return 0


if __name__ == '__main__':
    sys.exit(main())
