"""
ue5_spawn_repsplines.py
-----------------------
Run this INSIDE the UE5 Editor Python console (Output Log → Cmd → Python).

Reads the JSON produced by xodr_parallel_splines.py and spawns one
BP_RepSpline actor per spline entry, setting its SplineComponent
control points from the offset point array.

Requirements:
  - BP_RepSpline must exist in your Content Browser (CARLA ships it)
  - Unreal Engine 5.x with Python Editor Script Plugin enabled

Usage (in UE5 Output Log):
    py "C:/path/to/ue5_spawn_repsplines.py"

Or set JSON_PATH and BP_PATH at the top and run via the Editor Python REPL.
"""

import unreal
import json

# ── Config ────────────────────────────────────────────────────────────────────
JSON_PATH = "C:/Dev/CarlaUE5/Unreal/CarlaUnreal/Source/CarlaUnreal/SplieGen/Json/splines.json"          # ← output of xodr_parallel_splines.py
BP_PATH   = "/Game/Carla/Maps/TownNurb2/Meshes/BP_RepSpline_Wall.BP_RepSpline_Wall"  # ← adjust to your content path
Z_OFFSET  = 10.0   # cm above road surface (UE5 uses cm)
SCALE     = 100.0  # metres → centimetres
# ─────────────────────────────────────────────────────────────────────────────


def get_spline_component(actor):
    """Return the first SplineComponent found on the actor."""
    for comp in actor.get_components_by_class(unreal.SplineComponent):
        return comp
    return None


def set_spline_points(spline_comp, points_m):
    """
    Replace all spline control points.
    """
    spline_comp.clear_spline_points(True)
    for i, p in enumerate(points_m):
        ue_loc = unreal.Vector(
             p['x'] * SCALE,
            -p['y'] * SCALE,    # xodr Y flips to UE5 Y
             p['z'] * SCALE + Z_OFFSET
        )
        spline_comp.add_spline_point(ue_loc, unreal.SplineCoordinateSpace.WORLD, False)

    spline_comp.set_closed_loop(False, True)
    spline_comp.update_spline()

    # --- FIX 1: Evita che la spline si resetti ---
    # Comunica all'Editor che questi punti sono definitivi e vanno salvati
    try:
        spline_comp.set_editor_property("bSplineHasBeenEdited", True)
    except Exception as e:
        unreal.log_warning(f"[RepSpline] Impossibile impostare bSplineHasBeenEdited: {e}")


def spawn_repsplines(json_path, bp_path):
    editor   = unreal.EditorLevelLibrary
    world    = editor.get_editor_world()
    bp_asset = unreal.load_asset(bp_path)

    if bp_asset is None:
        unreal.log_error(f"[RepSpline] Cannot load blueprint: {bp_path}")
        return

    with open(json_path, 'r') as f:
        data = json.load(f)

    splines = data.get('splines', [])
    unreal.log(f"[RepSpline] Spawning {len(splines)} splines from {json_path}")

    with unreal.ScopedEditorTransaction("Spawn RepSplines from xodr") as _:
        for entry in splines:
            name   = entry['ue_name']
            points = entry['points']

            if len(points) < 2:
                unreal.log_warning(f"  [SKIP] {name}: fewer than 2 points")
                continue

            # Spawn at centroid of first point
            origin = unreal.Vector(
                 points[0]['x'] * SCALE,
                -points[0]['y'] * SCALE,
                 points[0]['z'] * SCALE + Z_OFFSET
            )
            actor = unreal.EditorLevelLibrary.spawn_actor_from_class(
                bp_asset.generated_class(),
                origin,
                unreal.Rotator(0, 0, 0)
            )

            if actor is None:
                unreal.log_error(f"  [FAIL] Could not spawn actor for {name}")
                continue

            actor.set_actor_label(name)

            spline_comp = get_spline_component(actor)
            if spline_comp:
                set_spline_points(spline_comp, points)
                
                # --- FIX 2: Forza il ricalcolo del Blueprint ---
                # Eseguendo un micro-spostamento avanti e indietro, costringiamo Unreal 
                # a far ripartire il Construction Script generando le mesh sui punti appena creati
                loc = actor.get_actor_location()
                actor.set_actor_location(loc + unreal.Vector(0, 0, 0.01), False, False)
                actor.set_actor_location(loc, False, False)

                unreal.log(f"  [OK] {name}  ({len(points)} pts, offset {entry['offset_m']}m {entry['side']})")
            else:
                unreal.log_warning(f"  [WARN] {name}: no SplineComponent found on BP_RepSpline")

    unreal.EditorLevelLibrary.save_current_level()
    unreal.log("[RepSpline] Done.")

# ── Entry point ───────────────────────────────────────────────────────────────
spawn_repsplines(JSON_PATH, BP_PATH)
