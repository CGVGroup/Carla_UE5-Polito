#!/usr/bin/env python3
"""
headset_connected.py: exit code 0 if a VR headset is available through OpenXR right now (connected and
ready, e.g. through VIVE Business Streaming or Steam Link), 1 if not, 2 if it can't be checked (pyopenxr
missing). Used by launch_carla.ps1 to start CARLA in VR or not.

Usage: python tools/headset_connected.py
"""
import sys

try:
    import xr
except ImportError:
    print('pyopenxr not installed (pip install pyopenxr): cannot check the headset')
    sys.exit(2)


def main():
    try:
        instance = xr.create_instance(xr.InstanceCreateInfo(
            application_info=xr.ApplicationInfo(application_name='headset_check', engine_name='none')))
    except Exception as e:  # noqa: BLE001 - no OpenXR runtime at all
        print('No OpenXR runtime: {}'.format(e))
        return 1
    try:
        system = xr.get_system(instance, xr.SystemGetInfo(form_factor=xr.FormFactor.HEAD_MOUNTED_DISPLAY))
        properties = xr.get_system_properties(instance, system)
        name = properties.system_name
        print('Headset available: {}'.format(name.decode() if isinstance(name, bytes) else name))
        return 0
    except Exception as e:  # noqa: BLE001 - XR_ERROR_FORM_FACTOR_UNAVAILABLE: no headset connected
        print('No headset available ({})'.format(type(e).__name__))
        return 1
    finally:
        try:
            xr.destroy_instance(instance)
        except Exception:  # noqa: BLE001
            pass


if __name__ == '__main__':
    sys.exit(main())
