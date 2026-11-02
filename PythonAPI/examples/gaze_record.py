#!/usr/bin/env python3
"""
gaze_record.py: records the driver's gaze (VR headset eye tracking, e.g. VIVE Focus Vision) during a
driving session, for sessions not run through the leaderboard (which records each route by itself),
e.g. alongside generate_traffic.py.

The recording is done by the CARLA server (the eye tracker is read there): this script only starts it
and stops it on Ctrl+C. One CSV line per simulation frame, in
<CARLA Saved dir>/GazeLogs/<name>_<date>.csv (packaged CARLA: %LOCALAPPDATA%/CarlaUnreal/Saved,
from the editor: Unreal/CarlaUnreal/Saved):
    wall_time, sim_time, frame (same as world.get_snapshot().frame),
    eye_tracker_connected, gaze_valid, confidence,
    gaze_origin_x/y/z, gaze_dir_x/y/z, fixation_x/y/z (CARLA coordinates, meters),
    left_blink, right_blink, left_pupil, right_pupil (if the runtime provides them),
    head_x/y/z, head_pitch/yaw/roll (the headset),
    ego_id, ego_x/y/z, ego_yaw, ego_speed_kmh (vehicle with role_name 'hero'),
    hit, hit_x/y/z, hit_distance, hit_actor_id, hit_actor_type, hit_semantic, hit_object
        (first thing the gaze hits outside the ego vehicle: CARLA actor id / type id, or the semantic
        class of the map object, e.g. Roads, Buildings, TrafficSigns)

Usage:
    python gaze_record.py --name participant01_session1
"""
import argparse
import sys
import time

import carla


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=2000)
    ap.add_argument('--timeout', type=float, default=10.0)
    ap.add_argument('--name', default='session', help='name of the recording (file name prefix)')
    args = ap.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    world = client.get_world()
    if not hasattr(world, 'set_type_string_dashboard'):
        print('This CARLA PythonAPI has no dashboard bindings: rebuild it.')
        return 1

    world.set_type_string_dashboard('gaze:start:{}'.format(args.name))
    print('Recording the gaze as "{}" (file in %LOCALAPPDATA%/CarlaUnreal/Saved/GazeLogs for a packaged CARLA). '
          'Ctrl+C to stop.'.format(args.name))
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            world.set_type_string_dashboard('gaze:stop')
            print('Gaze recording stopped.')
        except RuntimeError as e:
            print('Could not stop the recording ({}): it stops when the map is reloaded.'.format(e))
    return 0


if __name__ == '__main__':
    sys.exit(main())
