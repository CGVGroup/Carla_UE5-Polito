#!/usr/bin/env python3
"""
gaze_video_overlay.py: overlays the driver's gaze on a video of what the headset showed (OBS recording
of the VR mirror window), from the gaze recording of the same session.

Recording a session:
  1. OBS: Window Capture of the CARLA window (the VR mirror), constant frame rate (Settings > Video), e.g. 60
     fps, MP4 or MKV. Start the OBS recording BEFORE the route starts.
  2. Run the route (or gaze_record.py): when the gaze recording starts, a big magenta "SYNC" shows in front
     of the view for 1 s (flagged in the gaze file's 'sync' column). Stop OBS after the route.

Synchronization: the SYNC cue is found in the video (magenta pixels), which gives the video time of the
gaze file's first 'sync' sample; then the gaze file's wall_time gives the time of every other sample.
--sync-video-time sets that video time by hand instead (seconds), e.g. when the cue isn't visible.

Projection: the gaze direction relative to the head is projected on the image with a pinhole camera of
horizontal field of view hfov centered at (cx, cy) (fractions of the width / height). The mirror window
shows one eye's view, cropped by Unreal, so these are calibrated from the SYNC cue itself: the "SYNC" text
is drawn straight in front of the view (its position in the video gives cx, cy), then two "+" at 20 deg
left and right of it (their distance gives hfov). Recordings made before the "+" markers existed only
calibrate cx, cy (hfov: --hfov). --manual uses --hfov / --cx / --cy as given; --preview saves a few frames
with the gaze and a 10 deg grid of the projection, to check it.

Usage:
    python tools/gaze_video_overlay.py <video> <gaze csv> [--out gaze_overlay.mp4] [--hfov 100 --cx 0.5 --cy 0.5]
    python tools/gaze_video_overlay.py <video> <gaze csv> --preview 10    # 10 calibration images
"""
import argparse
import csv
import datetime
import math
import os
import subprocess
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from visualize_gaze import aoi_of, AOI_COLORS  # noqa: E402


def fnum(value, default=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return default


def parse_time(text):
    """Seconds of an ISO 8601 time (Unreal writes a trailing 'Z', which Python 3.8 doesn't parse: only
    differences between these times are used, so the time zone doesn't matter)"""
    try:
        return datetime.datetime.fromisoformat(text.strip().rstrip('Z')).timestamp()
    except (AttributeError, TypeError, ValueError):
        return None


def head_axes(pitch, yaw, roll):
    """Forward / right / up unit vectors of an Unreal rotator (degrees)"""
    p, y, r = (math.radians(v) for v in (pitch, yaw, roll))
    sp, cp, sy, cy, sr, cr = math.sin(p), math.cos(p), math.sin(y), math.cos(y), math.sin(r), math.cos(r)
    forward = np.array([cp * cy, cp * sy, sp])
    right = np.array([sr * sp * cy - cr * sy, sr * sp * sy + cr * cy, -sr * cp])
    up = np.array([-(cr * sp * cy + sr * sy), cy * sr - cr * sp * sy, cr * cp])
    return forward, right, up


# The mirror window shows ONE eye's view (Unreal's spectator screen: the left eye), not the view from the
# middle of the head the gaze is given from. A point at distance d appears shifted sideways in that eye's
# image by atan(half IPD / d): nothing far away, but 6 deg for the SYNC cue, 30 cm in front of the face.
# So the gaze point (along the gaze, at the distance its ray hits something) is projected from that eye,
# and the calibration from the cue is corrected for it (see eye_corrected_cx)
MIRROR_EYE_OFFSET_CM = -3.15  # eye seen in the mirror, right of the head's center [cm] (left eye: half a 63 mm IPD)
SYNC_DISTANCE_CM = 30.0       # distance of the SYNC cue in front of the view (GazeRecorder SyncDistance)
FAR_DISTANCE_CM = 5000.0      # gaze point distance when the ray hits nothing


def eye_corrected_cx(cx, hfov, eye_offset_cm, cue_distance_cm=SYNC_DISTANCE_CM):
    """cx measured on the SYNC cue -> cx of the eye's forward direction (where far objects straight ahead show)"""
    focal_ratio = 0.5 / math.tan(math.radians(hfov) / 2.0)  # focal length / image width
    return cx + focal_ratio * eye_offset_cm / cue_distance_cm


def project(row, width, height, hfov, cx, cy, eye_offset_cm=MIRROR_EYE_OFFSET_CM):
    """Pixel of the gaze in the headset view, or None (no valid gaze, or behind)"""
    if row.get('gaze_valid') != '1':
        return None
    direction = np.array([fnum(row['gaze_dir_x']), fnum(row['gaze_dir_y']), fnum(row['gaze_dir_z'])])
    norm = np.linalg.norm(direction)
    if not norm:
        return None
    direction = direction / norm
    forward, right, up = head_axes(fnum(row['head_pitch']), fnum(row['head_yaw']), fnum(row['head_roll']))
    # The looked-at point in the head's frame [cm] (hit distance in m), then seen from the mirror's eye
    distance = fnum(row.get('hit_distance')) * 100.0 if row.get('hit') == '1' else 0.0
    distance = distance if distance > 20.0 else FAR_DISTANCE_CM
    x, y, z = distance * (direction @ forward), distance * (direction @ right), distance * (direction @ up)
    y -= eye_offset_cm
    if x <= 1.0:
        return None
    focal = (width / 2.0) / math.tan(math.radians(hfov) / 2.0)
    return int(round(cx * width + focal * y / x)), int(round(cy * height - focal * z / x))


def find_sync_cue(video_path, max_seconds=180.0, from_seconds=0.0):
    """Video time [s] when the magenta SYNC cue first shows (searched for max_seconds from from_seconds), or None"""
    capture = cv2.VideoCapture(video_path)
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    index = 0
    if from_seconds > 0:
        capture.set(cv2.CAP_PROP_POS_MSEC, from_seconds * 1000.0)
        index = int(from_seconds * fps)
        max_seconds += from_seconds
    found = None
    first, run = None, 0  # first frame of the current run of frames showing magenta, its length
    while index < max_seconds * fps:
        ok, frame = capture.read()
        if not ok:
            break
        small = cv2.resize(frame, (320, int(320 * frame.shape[0] / frame.shape[1])))
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        # Magenta: hue ~150 (OpenCV 0..180), saturated and bright. Even partly hidden (by the car's interior,
        # in recordings made before the cue was moved closer), it shows for a few frames in a row: a single
        # frame with some magenta is something else
        mask = cv2.inRange(hsv, (140, 120, 120), (165, 255, 255))
        if mask.mean() / 255.0 > 0.0015:
            first = index if run == 0 else first
            run += 1
            if run >= max(3, int(0.1 * fps)):
                found = first / fps
                break
        else:
            run = 0
        index += 1
    capture.release()
    return found


MARKER_ANGLE = 20.0  # [deg] of the SYNC cue's "+" markers from the view's forward direction (GazeRecorder)


def calibrate_from_cue(video_path, cue_time, duration=1.3):
    """
    (cx, cy, hfov or None) from the SYNC cue in the video, starting at cue_time: (cx, cy) where the "SYNC"
    text (straight ahead) is, hfov from the "+" markers at +/-MARKER_ANGLE. None if the cue isn't found
    """
    capture = cv2.VideoCapture(video_path)
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    capture.set(cv2.CAP_PROP_POS_MSEC, max(0.0, cue_time - 0.1) * 1000.0)
    centers, lefts, rights = [], [], []
    for _ in range(int(duration * fps)):
        ok, frame = capture.read()
        if not ok:
            break
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), (140, 120, 120), (165, 255, 255))
        ys, xs = np.nonzero(mask)
        if len(xs) < 30:
            continue
        # One cluster ("SYNC") or two far apart (the two "+"): split at the widest gap between columns
        columns = np.unique(xs)
        gaps = np.diff(columns)
        if len(gaps) and gaps.max() > 0.15 * width:
            split = columns[int(np.argmax(gaps))]
            left, right = xs <= split, xs > split
            lefts.append((xs[left].mean(), ys[left].mean()))
            rights.append((xs[right].mean(), ys[right].mean()))
        else:
            centers.append((xs.mean(), ys.mean()))
    capture.release()
    if not centers and not (lefts and rights):
        return None
    if centers:
        cx_px, cy_px = np.median(np.array(centers), axis=0)
    else:  # only the markers: forward is halfway between them
        cx_px = (np.median([p[0] for p in lefts]) + np.median([p[0] for p in rights])) / 2.0
        cy_px = (np.median([p[1] for p in lefts]) + np.median([p[1] for p in rights])) / 2.0
    hfov = None
    if lefts and rights:
        half_spread = (np.median([p[0] for p in rights]) - np.median([p[0] for p in lefts])) / 2.0
        if half_spread > 0:
            focal = half_spread / math.tan(math.radians(MARKER_ANGLE))
            hfov = math.degrees(2.0 * math.atan((width / 2.0) / focal))
    return cx_px / width, cy_px / height, hfov


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('video')
    ap.add_argument('gaze', help='gaze CSV of the same session')
    ap.add_argument('--out', help='output video (default: <video>_gaze.mp4)')
    ap.add_argument('--hfov', type=float, default=100.0, help='horizontal field of view of the video [deg]')
    ap.add_argument('--cx', type=float, default=0.5, help='image center x (fraction of the width)')
    ap.add_argument('--cy', type=float, default=0.5, help='image center y (fraction of the height)')
    ap.add_argument('--sync-video-time', type=float, help='video time [s] of the SYNC cue (default: found)')
    ap.add_argument('--sync-search-from', type=float, default=0.0,
                    help='video time [s] to look for the SYNC cue from (a recording started long before the route)')
    ap.add_argument('--eye-offset', type=float, default=MIRROR_EYE_OFFSET_CM,
                    help='eye shown by the mirror window, right of the head center [cm] (default -3.15: the left '
                         'eye; 0: no eye correction)')
    ap.add_argument('--trail', type=float, default=0.3, help='[s] of gaze trail drawn')
    ap.add_argument('--preview', type=int, help='only save that many calibration images')
    ap.add_argument('--manual', action='store_true', help="use --hfov / --cx / --cy, don't calibrate from the cue")
    ap.add_argument('--sync-file-time', action='store_true',
                    help="when the SYNC cue isn't found: synchronize from the video file's creation time (OBS's start, "
                         "about 1 s off) instead of stopping; for drives without eye data")
    args = ap.parse_args()

    with open(args.gaze, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    times = np.array([parse_time(r.get('wall_time')) or np.nan for r in rows])
    if np.isnan(times).all():
        print('No wall_time in the gaze file')
        return 2
    sync_rows = [i for i, r in enumerate(rows) if r.get('sync') == '1']
    if not sync_rows:
        print("No 'sync' sample in the gaze file (recorded before the SYNC cue was added?): use --sync-video-time "
              "with the video time matching the first gaze sample")
    gaze_sync_time = times[sync_rows[0]] if sync_rows else times[0]
    valid = sum(r.get('gaze_valid') == '1' for r in rows)
    print('{} gaze samples, {:.1%} valid'.format(len(rows), valid / max(1, len(rows))))

    video_sync = args.sync_video_time
    if video_sync is None:
        print('Looking for the SYNC cue in the video...')
        video_sync = find_sync_cue(args.video, from_seconds=args.sync_search_from)
        if video_sync is None and args.sync_file_time:
            video_sync = gaze_sync_time - os.path.getctime(args.video)
            print('SYNC cue not found: synchronized from the video file creation time (about 1 s off)')
        if video_sync is None:
            print('SYNC cue not found between {:.0f} s and {:.0f} s of the video: give --sync-video-time (or '
                  '--sync-search-from)'.format(args.sync_search_from, args.sync_search_from + 180.0))
            return 2
    print('SYNC cue at {:.3f} s in the video'.format(video_sync))

    if not valid:
        print('No valid gaze: the video is made without the gaze marker (frame numbers only)')
    elif not args.manual:
        calibration = calibrate_from_cue(args.video, video_sync)
        if calibration is None:
            print('Calibration: SYNC cue not measurable, using --hfov {} --cx {} --cy {}'.format(args.hfov, args.cx, args.cy))
        else:
            args.cx, args.cy = calibration[0], calibration[1]
            if calibration[2] is not None:
                args.hfov = calibration[2]
                origin = 'measured from the "+" markers'
            else:
                origin = 'given (no "+" markers in this recording: rebuild CARLA for them)'
            measured_cx = args.cx
            args.cx = eye_corrected_cx(measured_cx, args.hfov, args.eye_offset)
            print('Calibration from the cue: cx {:.3f} (cue seen at {:.3f}: {} eye of the mirror, cue at {:.0f} cm), '
                  'cy {:.3f}, hfov {:.1f} deg ({})'.format(
                      args.cx, measured_cx, 'left' if args.eye_offset < 0 else 'right' if args.eye_offset > 0 else 'no',
                      SYNC_DISTANCE_CM, args.cy, args.hfov, origin))

    capture = cv2.VideoCapture(args.video)
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    print('Video: {}x{} at {:.2f} fps, {} frames'.format(width, height, fps, total))

    out_path = args.out or os.path.splitext(args.video)[0] + '_gaze.mp4'
    writer = None
    preview_every = max(1, total // max(1, args.preview)) if args.preview else None
    if not args.preview:
        tmp_path = out_path + '.tmp.mp4'
        writer = cv2.VideoWriter(tmp_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (width, height))

    order = np.argsort(times)
    sorted_times = times[order]
    trail = []
    index = 0
    saved = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        video_time = index / fps
        wall = gaze_sync_time + (video_time - video_sync)
        k = int(np.searchsorted(sorted_times, wall))
        candidates = [order[j] for j in (k - 1, k) if 0 <= j < len(order)]
        row = None
        if candidates:
            nearest = min(candidates, key=lambda i: abs(times[i] - wall))
            if abs(times[nearest] - wall) < 0.1:
                row = rows[nearest]

        point = project(row, width, height, args.hfov, args.cx, args.cy, args.eye_offset) if row else None
        aoi = aoi_of(row) if row else None
        trail = [(t, p) for t, p in trail if video_time - t <= args.trail]
        if point is not None:
            trail.append((video_time, point))
        for (_, p0), (_, p1) in zip(trail, trail[1:]):
            cv2.line(frame, p0, p1, (255, 255, 255), 2, cv2.LINE_AA)
        if point is not None:
            color = tuple(int(AOI_COLORS.get(aoi, '#ffffff')[i:i + 2], 16) for i in (5, 3, 1))  # BGR
            cv2.circle(frame, point, 22, color, 3, cv2.LINE_AA)
            cv2.circle(frame, point, 4, color, -1, cv2.LINE_AA)
            cv2.putText(frame, aoi or '', (point[0] + 26, point[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
                        cv2.LINE_AA)
        status = 'frame {}  t={:.2f}s'.format(row['frame'], video_time) if row else 't={:.2f}s  (no gaze sample)'.format(video_time)
        if not valid:
            status += '  (no eye data)'
        elif row and row.get('gaze_valid') != '1':
            status += '  (gaze not valid)'
        cv2.putText(frame, status, (12, height - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2, cv2.LINE_AA)

        if args.preview:
            if index % preview_every == 0 and point is not None:
                grid = frame.copy()
                focal = (width / 2.0) / math.tan(math.radians(args.hfov) / 2.0)
                for angle in range(-40, 41, 10):  # projection grid every 10 deg
                    u = int(args.cx * width + focal * math.tan(math.radians(angle)))
                    v = int(args.cy * height - focal * math.tan(math.radians(angle)))
                    cv2.line(grid, (u, 0), (u, height), (0, 255, 255), 1)
                    cv2.line(grid, (0, v), (width, v), (0, 255, 255), 1)
                path = '{}_preview_{:03d}.png'.format(os.path.splitext(args.video)[0], saved)
                cv2.imwrite(path, grid)
                saved += 1
                if saved >= args.preview:
                    break
        else:
            writer.write(frame)
        index += 1
        if index % 600 == 0:
            print('  {}/{} frames'.format(index, total))
    capture.release()

    if args.preview:
        print('Saved {} calibration images ({}_preview_*.png): adjust --hfov / --cx / --cy until the gaze marker '
              'sits on what was looked at'.format(saved, os.path.splitext(args.video)[0]))
        return 0
    writer.release()
    # Re-encode in H.264 (playable everywhere), keeping the original audio if any
    try:
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-i', tmp_path, '-i', args.video, '-map', '0:v',
                        '-map', '1:a?', '-c:v', 'libx264', '-crf', '20', '-pix_fmt', 'yuv420p', '-c:a', 'copy',
                        out_path], check=True)
        os.remove(tmp_path)
    except (OSError, subprocess.CalledProcessError) as e:
        os.replace(tmp_path, out_path)
        print('(ffmpeg re-encoding failed: {}; MPEG-4 video kept)'.format(e))
    print('Saved', out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
