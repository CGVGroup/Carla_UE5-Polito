#!/usr/bin/env python3
"""
protocol_analysis.py: the per-drive measures of protocol XAI-HUD-EXP-01 v2.0 (Part III), used by
analyze_session.py (not run on its own).

Event = one scenario of the route (8 per drive in the protocol). For each:
    t0              first frame a scenario road user is in the driver's line of sight (session logger's ray
                    cast, 'visible_at'); the scenario's start when none was seen
    resolution      the scenario's end
    omission        the agent's failure behaviour (fail triggers of the routes file) was on during the event:
                    the system doesn't react to the road user, its boxes / text aren't shown
    Phase F         [t0 - 5 s, resolution + 10 s] (Section 1 note); Phase E: the rest of the drive

Section 10 metrics, per event:
    detection               driver reaction between t0 and the resolution (yes / no)
    reaction time           min(t brake > 10 %, t |steer - command| > 2 deg, t takeover) - t0. The pedals and the
                            wheel follow the automation (force feedback) with a short lag: a driver input counts
                            only when it differs from every automation command of the last INPUT_LAG_S
    gaze-to-hazard latency  first gaze sample on a road user of the event - t0 (the gaze ray hits the actor:
                            conservative, a look next to a small actor isn't counted)
    min TTC                 min over Phase F of the TTC to the scenario road users (closing speed > 0)
    peak decel. / lat. acc. max |ax|, |ay| (moving average over LOWPASS_S) from the reaction to the end of
                            Phase F; also over the whole Phase F (what the automation did)
    collision               collision with a road user of the event
    lane exceedance         after a takeover, |lateral offset| > LANE_EXCEEDANCE_M or a lane marking crossed
                            (yes / no, duration)
    unnecessary intervention  driver reaction on an event the system handled (not an omission)

Phase E gaze (monitoring ratio, glances > 2 s on the display):
    display     T: the text panel (placed by the server 3 m ahead of the head, TEXT_HEIGHT_CM above the view
                center, following the head's yaw), while a text is shown; A: a road user while its AR box is
                drawn; N: no display
    in-vehicle  gaze more than IN_VEHICLE_PITCH_DEG below the head's forward direction (instrument cluster)
    road        every other valid gaze sample
    monitoring ratio = road / (road + display + in-vehicle) time
"""
import bisect
import datetime
import math
import os

# Phase F around each event [s]
EVENT_PRE_S = 5.0
EVENT_POST_S = 10.0
# Reaction thresholds (Gold et al., 2013)
BRAKE_THRESHOLD = 0.10
STEER_THRESHOLD_DEG = 2.0
INPUT_LAG_S = 0.3
# Low-pass filter of the accelerations [s]
LOWPASS_S = 0.5
# Lane exceedance: lateral offset from the lane center [m] (half a 3.5 m lane minus half a car)
LANE_EXCEEDANCE_M = 0.8
# Display AOI of the text interface (see DashboardWriteVehicleText / vr_dashboard.TEXT_HEIGHT)
TEXT_DISTANCE_CM = 300.0
TEXT_HEIGHT_CM = 30.0
TEXT_HALF_WIDTH_DEG = 15.0
TEXT_BELOW_DEG = 4.0
TEXT_ABOVE_DEG = 8.0
IN_VEHICLE_PITCH_DEG = -20.0
# Display glances longer than this (NHTSA, 2013) [s]; samples further apart than the gap end a glance
LONG_GLANCE_S = 2.0
GLANCE_GAP_S = 0.15


def fnum(value, default=None):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def r(value, digits=3):
    return None if value is None else round(value, digits)


def ids_of(column):
    """Actor ids of an 'id:...|id:...' column"""
    out = set()
    for item in (column or '').split('|'):
        head = item.split(':', 1)[0]
        if head.strip().lstrip('-').isdigit():
            out.add(int(head))
    return out


def moving_average(values, n):
    out, total, window = [], 0.0, []
    for v in values:
        window.append(v)
        total += v
        if len(window) > n:
            total -= window.pop(0)
        out.append(total / len(window))
    return out


class Drive:
    """The session log as arrays (one entry per logged frame), with the driver's raw inputs of the gaze file"""

    def __init__(self, session, gaze_rows, max_steer_deg):
        frames = session.frames
        self.session = session
        self.t = list(session.frame_times)
        self.frame = list(session.frame_numbers)
        col = lambda name, default=0.0: [fnum(row.get(name), default) for row in frames]  # noqa: E731
        self.speed = col('speed_kmh')
        self.accel_long = col('accel_long')
        self.accel_lat = col('accel_lat')
        self.lateral_offset = col('lateral_offset')
        self.cmd_brake = col('cmd_brake')
        self.cmd_steer = col('cmd_steer_deg')
        self.brake = col('brake')
        self.steer = col('steer_deg')
        self.ttc = col('scenario_min_ttc', None)
        self.source = [row.get('control_source', '') for row in frames]
        self.failure = [row.get('failure_behavior') == '1' for row in frames]
        self.text = [bool((row.get('dashboard_text') or '').strip()) for row in frames]
        self.boxes = [ids_of(row.get('rendered_boxes')) for row in frames]
        self.detections = [ids_of(row.get('detections')) for row in frames]
        dt = [b - a for a, b in zip(self.t, self.t[1:]) if b > a]
        self.dt = sorted(dt)[len(dt) // 2] if dt else 0.05
        self.lag_frames = max(1, int(round(INPUT_LAG_S / self.dt)))
        # Driver's raw inputs (gaze file, every rendered frame), at the session's frames
        self.driver = []
        for row in gaze_rows:
            frame = int(fnum(row.get('frame'), -1))
            i = self.index_of_frame(frame)
            if i is None:
                continue
            self.driver.append((i, self.t[i], fnum(row.get('driver_brake'), 0.0),
                                fnum(row.get('driver_steer'), 0.0) * max_steer_deg))

    def index_of_frame(self, frame):
        i = bisect.bisect_right(self.frame, frame) - 1
        return i if i >= 0 else None

    def index_at(self, time):
        """Index of the last frame at or before that time"""
        return max(0, bisect.bisect_right(self.t, time) - 1)

    def span(self, start, end):
        a = bisect.bisect_left(self.t, start)
        b = bisect.bisect_right(self.t, end)
        return range(a, b)

    def reaction(self, start, end):
        """First driver input deviating from the automation in [start, end]: (time, kind)"""
        first = []
        lag = self.lag_frames
        # Raw pedal / wheel of the gaze file (exact); else the session log's applied control while the driver drives
        for i, time, brake, steer in self.driver:
            if time < start or time > end:
                continue
            window = range(max(0, i - lag), i + 1)
            if brake > BRAKE_THRESHOLD and all(brake > self.cmd_brake[j] + 0.05 for j in window):
                first.append((time, 'brake'))
                break
        for i, time, brake, steer in self.driver:
            if time < start or time > end:
                continue
            window = range(max(0, i - lag), min(len(self.t), i + 2))
            if min(abs(steer - self.cmd_steer[j]) for j in window) > STEER_THRESHOLD_DEG:
                first.append((time, 'steer'))
                break
        if not self.driver:
            for i in self.span(start, end):
                if self.source[i] not in ('', 'agent'):
                    if self.brake[i] > BRAKE_THRESHOLD:
                        first.append((self.t[i], 'brake'))
                        break
                    if abs(self.steer[i] - self.cmd_steer[i]) > STEER_THRESHOLD_DEG:
                        first.append((self.t[i], 'steer'))
                        break
        for e in self.session.events:
            if e.get('event') == 'takeover_start' and e.get('sim_time') is not None and start <= e['sim_time'] <= end:
                first.append((e['sim_time'], 'takeover ({})'.format(e.get('cause') or '?')))
                break
        return min(first) if first else (None, None)

    def peaks(self, start, end):
        """Peak deceleration and lateral acceleration (low-pass filtered) in [start, end] [m/s2]"""
        idx = list(self.span(start, end))
        if not idx:
            return None, None
        n = max(1, int(round(LOWPASS_S / self.dt)))
        lon = moving_average([self.accel_long[i] for i in idx], n)
        lat = moving_average([self.accel_lat[i] for i in idx], n)
        return r(max(0.0, -min(lon)), 2), r(max(abs(v) for v in lat), 2)

    def min_ttc(self, start, end):
        values = [self.ttc[i] for i in self.span(start, end) if self.ttc[i] is not None and self.ttc[i] > 0]
        return r(min(values), 2) if values else None


def build_events(session, metrics, gaze_rows):
    """Events (Appendix B / Section 10 rows) of the drive"""
    summary = session.summary
    drive = Drive(session, gaze_rows, fnum(summary.get('max_steer_angle_deg'), 35.0))
    hazards = metrics.get('hazards') or []
    collisions = [e for e in session.events if e.get('event') == 'collision']
    lane_invasions = [e.get('sim_time') for e in session.events if e.get('event') == 'lane_invasion']
    takeovers = [e for e in session.events if e.get('event') == 'takeover_start']
    takeover_ends = [e for e in session.events if e.get('event') == 'takeover_end']
    last = drive.t[-1] if drive.t else 0.0

    events = []
    for number, (name, kind, start, end) in enumerate(session.scenario_windows(), 1):
        end = end if end is not None else last
        start = start if start is not None else end
        users = [h for h in hazards if h.get('scenario') == name]
        ids = {h['actor_id'] for h in users}
        onsets = [h['visible_at_t0'] for h in users if h.get('visible_at_t0') is not None]
        t0 = min(onsets) if onsets else start
        f_start, f_end = t0 - EVENT_PRE_S, end + EVENT_POST_S
        span = drive.span(min(t0, start), end)
        omission = any(drive.failure[i] for i in span)
        in_list = any(drive.detections[i] & ids for i in span) if ids else False
        boxed = any(drive.boxes[i] & ids for i in span) if ids else False
        reaction_time, reaction_kind = drive.reaction(t0, end)
        glances = [h.get('glance_latency_from_t0') for h in users if h.get('glance_latency_from_t0') is not None]
        hit = [c for c in collisions if c.get('sim_time') is not None and f_start <= c['sim_time'] <= f_end
               and (not ids or c.get('other_id') in ids)]
        peak_decel, peak_lat = drive.peaks(reaction_time, f_end) if reaction_time is not None else (None, None)
        window_decel, window_lat = drive.peaks(f_start, f_end)
        # Lane exceedance after a takeover of the event
        takeover = next((e for e in takeovers if e.get('sim_time') is not None and t0 <= e['sim_time'] <= f_end), None)
        exceed, exceed_s = None, None
        if takeover is not None:
            stop = next((e['sim_time'] for e in takeover_ends if e.get('sim_time') is not None
                         and e['sim_time'] >= takeover['sim_time']), f_end)
            out = [i for i in drive.span(takeover['sim_time'], min(stop, f_end))
                   if abs(drive.lateral_offset[i]) > LANE_EXCEEDANCE_M]
            crossed = any(t is not None and takeover['sim_time'] <= t <= min(stop, f_end) for t in lane_invasions)
            exceed, exceed_s = bool(out) or crossed, r(len(out) * drive.dt, 2)
        events.append({
            'event': number, 'scenario': name, 'type': kind,
            'actors': ' '.join('{}:{}'.format(h['actor_id'], (h.get('type') or '').split('.')[0]) for h in users),
            'omission': omission,
            't0': r(t0), 't0_source': 'line of sight' if onsets else 'scenario start (no road user seen)',
            'resolution': r(end), 'phase_f_start': r(f_start), 'phase_f_end': r(f_end),
            'distance_at_t0': min((h['distance_at_t0'] for h in users if h.get('distance_at_t0') is not None), default=None),
            # Ground truth of the script: the system detects every road user except during an omission
            'detected_by_system': not omission, 'in_detection_list': in_list, 'shown_on_display': boxed,
            'driver_reaction': reaction_time is not None,
            'reaction_time': r(reaction_time - t0) if reaction_time is not None else None,
            'reaction_input': reaction_kind,
            'gaze_latency': r(min(glances)) if glances else None,
            'min_ttc': drive.min_ttc(f_start, f_end),
            'min_gap': min((h['min_gap'] for h in users if h.get('min_gap') is not None), default=None),
            'peak_decel_after_reaction': peak_decel, 'peak_lat_acc_after_reaction': peak_lat,
            'peak_decel_phase_f': window_decel, 'peak_lat_acc_phase_f': window_lat,
            'collision': bool(hit), 'collision_with': ' '.join(str(c.get('other_type')) for c in hit),
            'takeover': takeover is not None,
            'lane_exceedance': exceed, 'lane_exceedance_s': exceed_s,
            'unnecessary_intervention': (reaction_time is not None) and not omission,
        })
    omissions = [e for e in events if e['omission']]
    for k, e in enumerate(omissions, 1):
        e['omission_index'] = k
    return events, drive


# ---------------------------------------------------------------------------------------------- Phase E gaze

def gaze_category(row, mode, boxes):
    """'display', 'in-vehicle', 'road' (valid samples), None (invalid)"""
    if row.get('gaze_valid') != '1':
        return None
    dx, dy, dz = fnum(row.get('gaze_dir_x'), 0.0), fnum(row.get('gaze_dir_y'), 0.0), fnum(row.get('gaze_dir_z'), 0.0)
    norm = math.sqrt(dx * dx + dy * dy + dz * dz) or 1.0
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, dz / norm))))
    rel_yaw = (yaw - fnum(row.get('head_yaw'), 0.0) + 180.0) % 360.0 - 180.0
    rel_pitch = pitch - fnum(row.get('head_pitch'), 0.0)
    if rel_pitch < IN_VEHICLE_PITCH_DEG:
        return 'in-vehicle'
    if mode == 'text':
        # The text panel follows the head's yaw only: at a fixed elevation in the world
        elevation = math.degrees(math.atan2(TEXT_HEIGHT_CM, TEXT_DISTANCE_CM))
        if boxes == 'text' and abs(rel_yaw) <= TEXT_HALF_WIDTH_DEG and \
                elevation - TEXT_BELOW_DEG <= pitch <= elevation + TEXT_ABOVE_DEG:
            return 'display'
    elif mode == 'full' and isinstance(boxes, set) and row.get('hit') == '1':
        if int(fnum(row.get('hit_actor_id'), -1)) in boxes:
            return 'display'
    return 'road'


def phase_e_gaze(drive, events, gaze_rows, mode):
    """Monitoring ratio and display glances in Phase E (outside every event's Phase F)"""
    windows = [(e['phase_f_start'], e['phase_f_end']) for e in events]
    samples = []
    for row in gaze_rows:
        frame = int(fnum(row.get('frame'), -1))
        i = drive.index_of_frame(frame)
        try:
            wall = datetime.datetime.fromisoformat(str(row.get('wall_time')).rstrip('Z')).timestamp()
        except (TypeError, ValueError):
            wall = None
        if i is None or wall is None:
            continue
        time = drive.t[i]
        phase = 'F' if any(a <= time <= b for a, b in windows) else 'E'
        display = 'text' if drive.text[i] else None
        category = gaze_category(row, mode, display if mode == 'text' else drive.boxes[i])
        samples.append((wall, time, phase, category))
    out = {}
    for phase in ('E', 'F'):
        part = [s for s in samples if s[2] == phase]
        durations = {'road': 0.0, 'display': 0.0, 'in-vehicle': 0.0}
        glances, current = [], None
        for k, (wall, time, _, category) in enumerate(part):
            step = min(0.2, part[k + 1][0] - wall) if k + 1 < len(part) else 0.0
            if category in durations:
                durations[category] += max(0.0, step)
            # Display glances: consecutive display samples
            if category == 'display':
                if current is None or wall - current[1] > GLANCE_GAP_S:
                    if current is not None:
                        glances.append(current[1] - current[0])
                    current = [wall, wall + max(0.0, step)]
                else:
                    current[1] = wall + max(0.0, step)
            elif current is not None and wall - current[1] > GLANCE_GAP_S:
                glances.append(current[1] - current[0])
                current = None
        if current is not None:
            glances.append(current[1] - current[0])
        total = sum(durations.values())
        out[phase] = {
            'gaze_time_s': r(total, 2),
            'road_s': r(durations['road'], 2), 'display_s': r(durations['display'], 2),
            'in_vehicle_s': r(durations['in-vehicle'], 2),
            'monitoring_ratio': r(durations['road'] / total) if total else None,
            'display_ratio': r(durations['display'] / total) if total else None,
            'in_vehicle_ratio': r(durations['in-vehicle'] / total) if total else None,
            'display_glances': len(glances),
            'display_glance_mean_s': r(sum(glances) / len(glances), 2) if glances else None,
            'display_glances_over_2s': sum(g > LONG_GLANCE_S for g in glances),
            'display_glances_over_2s_ratio': r(sum(g > LONG_GLANCE_S for g in glances) / len(glances)) if glances else None,
        }
    out['display_aoi'] = {'none': 'no display (N)', 'full': 'road users while their AR box is drawn',
                          'text': 'text panel while a text is shown'}.get(mode, mode)
    return out


# ---------------------------------------------------------------------------------------------- figures

def figures(drive, events, gaze_rows, metrics, out_dir):
    """fig_drive_timeline.png and fig_event_<n>.png (one per event) in out_dir; the files written"""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib missing: no figures (pip install matplotlib)')
        return []
    os.makedirs(out_dir, exist_ok=True)
    written = []
    condition = metrics.get('condition') or ''
    title = '{} · condition {} · {}'.format(metrics.get('participant') or '-', condition, metrics.get('route_code') or '')

    # Whole drive: speed, brake (automation / driver), Phase F windows, events, takeovers
    fig, axes = plt.subplots(3, 1, figsize=(12, 7), sharex=True, gridspec_kw={'height_ratios': [2, 1, 1]})
    axes[0].plot(drive.t, drive.speed, color='#1f77b4', lw=1)
    axes[0].set_ylabel('speed [km/h]')
    axes[1].plot(drive.t, drive.cmd_brake, color='#7f7f7f', lw=1, label='automation brake')
    if drive.driver:
        axes[1].plot([d[1] for d in drive.driver], [d[2] for d in drive.driver], color='#d62728', lw=0.8,
                     label='driver brake pedal')
    axes[1].set_ylabel('brake')
    axes[1].legend(fontsize=7, loc='upper right')
    axes[2].plot(drive.t, [t if t is not None and t < 10 else None for t in drive.ttc], color='#9467bd', lw=1)
    axes[2].set_ylabel('TTC [s]')
    axes[2].set_xlabel('time from route start [s]')
    if drive.t:
        axes[2].set_xlim(drive.t[0], drive.t[-1])
    for ax in axes:
        for e in events:
            ax.axvspan(e['phase_f_start'], e['phase_f_end'], color='#d62728' if e['omission'] else '#ff7f0e',
                       alpha=0.12 if e['omission'] else 0.07, lw=0)
            ax.axvline(e['t0'], color='#d62728' if e['omission'] else '#ff7f0e', lw=0.8, ls='--')
        for x in metrics.get('takeovers') or []:
            if x.get('start') is not None:
                ax.axvline(x['start'], color='k', lw=1)
    for e in events:
        axes[0].text(e['t0'], axes[0].get_ylim()[1], ' {}{}'.format(e['event'], ' (omission)' if e['omission'] else ''),
                     fontsize=7, va='top', color='#d62728' if e['omission'] else '#a05000',
                     fontweight='bold' if e['omission'] else 'normal')
    axes[0].set_title('Drive: {} — shaded: Phase F (event windows, red = omission), dashed: t0, black: takeover'.format(title),
                      fontsize=9)
    fig.tight_layout()
    path = os.path.join(out_dir, 'fig_drive_timeline.png')
    fig.savefig(path, dpi=150)
    plt.close(fig)
    written.append(path)

    # Per event: time from t0
    hazards = {h['actor_id']: h for h in metrics.get('hazards') or []}
    for e in events:
        ids = {int(a.split(':')[0]) for a in e['actors'].split()} if e['actors'] else set()
        idx = list(drive.span(e['phase_f_start'], e['phase_f_end']))
        if not idx:
            continue
        t = [drive.t[i] - e['t0'] for i in idx]
        fig, axes = plt.subplots(4, 1, figsize=(8, 7.5), sharex=True)
        axes[0].plot(t, [drive.speed[i] for i in idx], color='#1f77b4')
        axes[0].set_ylabel('speed [km/h]')
        axes[1].plot(t, [drive.cmd_brake[i] for i in idx], color='#7f7f7f', label='automation')
        driver = [d for d in drive.driver if e['phase_f_start'] <= d[1] <= e['phase_f_end']]
        if driver:
            axes[1].plot([d[1] - e['t0'] for d in driver], [d[2] for d in driver], color='#d62728', label='driver pedal')
        axes[1].axhline(BRAKE_THRESHOLD, color='#d62728', lw=0.5, ls=':')
        axes[1].set_ylabel('brake')
        axes[1].legend(fontsize=7)
        axes[2].plot(t, [drive.cmd_steer[i] for i in idx], color='#7f7f7f', label='automation')
        if driver:
            axes[2].plot([d[1] - e['t0'] for d in driver], [d[3] for d in driver], color='#d62728', label='driver wheel')
        axes[2].set_ylabel('steer [deg]')
        axes[2].legend(fontsize=7)
        axes[3].plot(t, [drive.ttc[i] if drive.ttc[i] is not None and drive.ttc[i] < 10 else None for i in idx],
                     color='#9467bd')
        axes[3].set_ylabel('TTC [s]')
        axes[3].set_xlabel('time from t0 [s]')
        # Gaze on the event's road users
        on = []
        for row in gaze_rows:
            if row.get('gaze_valid') == '1' and row.get('hit') == '1' and int(fnum(row.get('hit_actor_id'), -1)) in ids:
                i = drive.index_of_frame(int(fnum(row.get('frame'), -1)))
                if i is not None and e['phase_f_start'] <= drive.t[i] <= e['phase_f_end']:
                    on.append(drive.t[i] - e['t0'])
        if on:
            axes[0].plot(on, [axes[0].get_ylim()[0]] * len(on), '|', color='#2ca02c', ms=12, label='gaze on road user')
            axes[0].legend(fontsize=7)
        for ax in axes:
            ax.axvline(0, color='k', lw=0.8, ls='--')
            ax.axvline(e['resolution'] - e['t0'], color='#7f7f7f', lw=0.8, ls=':')
            if e['reaction_time'] is not None:
                ax.axvline(e['reaction_time'], color='#d62728', lw=1)
        axes[0].set_title('Event {} {}{} — {}\nt0 (dashed), reaction (red: {}), resolution (dotted); min TTC {} s, collision {}'.format(
            e['event'], e['scenario'], ' — OMISSION' if e['omission'] else '', title,
            '{} s, {}'.format(e['reaction_time'], e['reaction_input']) if e['reaction_time'] is not None else 'none',
            e['min_ttc'] if e['min_ttc'] is not None else '-', 'yes' if e['collision'] else 'no'), fontsize=8)
        fig.tight_layout()
        path = os.path.join(out_dir, 'fig_event_{}_{}.png'.format(e['event'], e['scenario']))
        fig.savefig(path, dpi=150)
        plt.close(fig)
        written.append(path)
    return written
