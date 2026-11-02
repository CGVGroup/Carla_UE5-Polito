#!/usr/bin/env python3
"""
check_eye_tracking.py: checks, outside Unreal / CARLA, that the PC's OpenXR runtime (VIVE Hub / VIVE
Streaming for the VIVE Focus Vision) delivers the eye gaze the way Unreal reads it
(XR_EXT_eye_gaze_interaction), and prints it live.

It reports:
  1. the OpenXR runtime in use and whether it offers XR_EXT_eye_gaze_interaction (what Unreal uses) and
     XR_HTC_eye_tracker (HTC's own eye tracking extension)
  2. whether the runtime says the headset supports eye gaze interaction (supportsEyeGazeInteraction)
  3. the live gaze for --seconds seconds: share of the frames with a valid gaze pose, and the gaze direction

Close CARLA (and any other VR application) first: only one OpenXR application runs at a time. Put the
headset on, and look around during the test.

Install once:   pip install pyopenxr glfw PyOpenGL
Usage:          python tools/check_eye_tracking.py [--seconds 15]
"""
import argparse
import ctypes
import math
import sys
import time

try:
    import xr
except ImportError:
    print('pyopenxr is not installed: pip install pyopenxr glfw PyOpenGL')
    sys.exit(2)

try:  # the context helper moved between pyopenxr versions
    ContextObject = xr.ContextObject
except AttributeError:
    from xr.utils.gl import ContextObject  # noqa: E402

def _tolerate_unknown_structure_types():
    """
    The runtime may send events of types this pyopenxr version doesn't know (vendor extensions, e.g.
    1000301002 from SteamVR): pyopenxr then raises while polling the events. Unknown values become
    placeholder members instead, which pyopenxr ignores
    """
    def _missing_(cls, value):
        if not isinstance(value, int):
            return None
        member = int.__new__(cls, value)
        member._name_ = 'UNKNOWN_{}'.format(value)
        member._value_ = value
        return member
    xr.StructureType._missing_ = classmethod(_missing_)


_tolerate_unknown_structure_types()

GAZE_EXT = 'XR_EXT_eye_gaze_interaction'
HTC_EXT = 'XR_HTC_eye_tracker'


def quaternion_forward(q):
    """OpenXR's forward (-Z) direction rotated by the quaternion"""
    x, y, z, w = q.x, q.y, q.z, q.w
    return (-(2 * (x * z + w * y)), -(2 * (y * z - w * x)), -(1 - 2 * (x * x + y * y)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seconds', type=float, default=15.0, help='duration of the live test')
    args = ap.parse_args()

    # 1. Runtime extensions
    names = [e.extension_name.decode() if isinstance(e.extension_name, bytes) else e.extension_name
             for e in xr.enumerate_instance_extension_properties()]
    print('1. OpenXR runtime extensions:')
    print('   {:<32} {}'.format(GAZE_EXT, 'offered' if GAZE_EXT in names else 'NOT offered'))
    print('   {:<32} {}'.format(HTC_EXT, 'offered' if HTC_EXT in names else 'NOT offered'))
    if GAZE_EXT not in names:
        print('\n=> The runtime does not offer the eye gaze extension Unreal uses. Check that VIVE Hub / '
              'VIVE Streaming is the active OpenXR runtime, and that eye tracking is enabled for PC VR in it.')
        return 1

    create_info = xr.InstanceCreateInfo(enabled_extension_names=[xr.KHR_OPENGL_ENABLE_EXTENSION_NAME, GAZE_EXT])
    with ContextObject(instance_create_info=create_info) as context:
        instance, session = context.instance, context.session
        runtime = ''
        try:
            properties = xr.get_instance_properties(instance)
            runtime = properties.runtime_name.decode() if isinstance(properties.runtime_name, bytes) else properties.runtime_name
            print('   runtime: {} {}'.format(runtime, xr.Version(properties.runtime_version)))
        except Exception:  # noqa: BLE001 - informative only
            pass

        # 2. Does the headset support eye gaze, according to the runtime?
        # (the eye gaze properties are chained to the system properties: through the raw OpenXR call, the
        # pyopenxr wrapper doesn't take the chain)
        eye_properties = xr.SystemEyeGazeInteractionPropertiesEXT()
        system_properties = xr.SystemProperties(next=ctypes.cast(ctypes.pointer(eye_properties), ctypes.c_void_p))
        supported = None
        try:
            raw = getattr(xr, 'raw_functions', None)
            get_properties = getattr(raw, 'xrGetSystemProperties', None) if raw is not None else None
            if get_properties is None:
                get_properties = getattr(xr, 'xrGetSystemProperties')
            result = get_properties(instance, context.system_id, ctypes.byref(system_properties))
            if int(getattr(result, 'value', result)) == 0:
                supported = bool(eye_properties.supports_eye_gaze_interaction)
        except Exception as e:  # noqa: BLE001 - informative, the live test below is the real check
            print('   (could not read the eye gaze support: {})'.format(e))
        system_name = system_properties.system_name
        system_name = system_name.decode() if isinstance(system_name, bytes) else system_name
        print('\n2. Headset: {} - supportsEyeGazeInteraction = {}'.format(
            system_name or '?', 'unknown' if supported is None else supported))
        # VIVE Business Streaming runs through SteamVR: the eye gaze comes from its facial tracking API layer
        try:
            layers = [l.layer_name.decode() if isinstance(l.layer_name, bytes) else l.layer_name
                      for l in xr.enumerate_api_layer_properties()]
        except Exception:  # noqa: BLE001 - informative only
            layers = []
        vive_layer = 'XR_APILAYER_VIVE_facial_tracking'
        print('   VIVE eye / facial tracking layer ({}): {}'.format(
            vive_layer, 'present' if vive_layer in layers else 'NOT present'))
        if supported is False:
            print('   (the runtime reports no eye gaze support: some layers report it wrongly, the live test '
                  'below tells for sure)')

        # 3. Live gaze: an action bound to the eye gaze pose, located every frame
        gaze_action = xr.create_action(
            action_set=context.default_action_set,
            create_info=xr.ActionCreateInfo(action_type=xr.ActionType.POSE_INPUT, action_name='eye_gaze',
                                            localized_action_name='Eye gaze'))
        xr.suggest_interaction_profile_bindings(
            instance=instance,
            suggested_bindings=xr.InteractionProfileSuggestedBinding(
                interaction_profile=xr.string_to_path(instance, '/interaction_profiles/ext/eye_gaze_interaction'),
                suggested_bindings=(xr.ActionSuggestedBinding * 1)(xr.ActionSuggestedBinding(
                    action=gaze_action,
                    binding=xr.string_to_path(instance, '/user/eyes_ext/input/gaze_ext/pose')))))
        gaze_space = xr.create_action_space(session=session, create_info=xr.ActionSpaceCreateInfo(
            action=gaze_action, pose_in_action_space=xr.Posef()))

        print('\n3. Live gaze for {:.0f} s (look around):'.format(args.seconds))
        frames = valid = 0
        start = time.monotonic()
        last_print = 0.0
        for frame_state in context.frame_loop():
            if time.monotonic() - start > args.seconds:
                break
            if context.session_state != xr.SessionState.FOCUSED:
                continue
            active = xr.ActiveActionSet(action_set=context.default_action_set, subaction_path=xr.NULL_PATH)
            xr.sync_actions(session=session, sync_info=xr.ActionsSyncInfo(
                count_active_action_sets=1, active_action_sets=ctypes.pointer(active)))
            state = xr.get_action_state_pose(session, xr.ActionStateGetInfo(action=gaze_action))
            location = xr.locate_space(space=gaze_space, base_space=context.space,
                                       time=frame_state.predicted_display_time)
            frames += 1
            ok = bool(state.is_active) and bool(location.location_flags & xr.SPACE_LOCATION_ORIENTATION_VALID_BIT)
            if ok:
                valid += 1
            now = time.monotonic() - start
            if now - last_print > 0.5:
                last_print = now
                if ok:
                    fx, fy, fz = quaternion_forward(location.pose.orientation)
                    yaw = math.degrees(math.atan2(fx, -fz))
                    pitch = math.degrees(math.asin(max(-1.0, min(1.0, fy))))
                    print('   t={:4.1f}s  gaze OK   yaw {:+6.1f} deg  pitch {:+6.1f} deg'.format(now, yaw, pitch))
                else:
                    print('   t={:4.1f}s  no gaze   (action active: {}, location flags: {:#x})'.format(
                        now, bool(state.is_active), int(location.location_flags)))

        share = valid / frames if frames else 0.0
        print('\nResult: valid gaze in {} of {} focused frames ({:.0%})'.format(valid, frames, share))
        if frames == 0:
            print('=> The session never got focus: put the headset on (and close other VR applications).')
            return 1
        if share < 0.5:
            print('=> The runtime accepts the eye gaze extension but delivers (almost) no gaze: eye tracking is '
                  'not streamed to the PC. Check VIVE Hub / VIVE Streaming: eye tracking enabled for PC VR, eye '
                  'calibration done, VIVE Hub up to date; restart VIVE Streaming after changing it.')
            return 1
        print('=> Eye tracking works through OpenXR: Unreal / CARLA should receive it (restart CARLA after any '
              'VIVE setting change).')
        return 0


def print_runtime_help():
    print(r"""
With VIVE Business Streaming, the OpenXR runtime is SteamVR and the eye gaze comes from VIVE's
"XR_APILAYER_VIVE_facial_tracking" layer (VIVE Hub\VIVE Business Streaming\OpenXR\ViveVR_openxr). It only
gets eye data when:
  - in VIVE Business Streaming's settings (on the PC), eye tracking / "Eye & facial tracking" is enabled
  - the headset authorizes eye tracking for VIVE Business Streaming, and the eye calibration was done
  - the headset is connected through VIVE Business Streaming (not another streaming app), restarted after
    changing these settings
Then run this test again, then CARLA.""")


if __name__ == '__main__':
    code = main()
    if code:
        print_runtime_help()
    sys.exit(code)
