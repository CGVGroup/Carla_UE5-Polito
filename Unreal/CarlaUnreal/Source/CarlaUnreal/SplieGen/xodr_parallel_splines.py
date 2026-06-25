"""
xodr_parallel_splines.py
------------------------
Parses an OpenDRIVE (.xodr) file, extracts road reference lines,
and generates offset splines at user-defined lateral distances.

Output: JSON consumable by a UE5 Editor Python script to spawn BP_RepSpline actors.

Geometry support:
  - line      ✓ exact
  - arc        ✓ exact
  - spiral     ✓ numerical (Fresnel integration, good to ~1mm)
  - poly3      ✓
  - paramPoly3 ✓

Usage:
    python xodr_parallel_splines.py \
        --xodr Town01.xodr \
        --offsets "6:guardrail" "10:bike_path" "15:furniture" \
        --step 2.0 \
        --output splines.json \
        --plot                      # show preview; add --save-plot out.png to write to disk
"""

import xml.etree.ElementTree as ET
import json
import argparse
import math
import numpy as np
from typing import Optional


# ─── Geometry samplers ────────────────────────────────────────────────────────

def sample_line(x0, y0, hdg, length, step):
    pts = []
    s = 0.0
    while s <= length + 1e-6:
        pts.append((x0 + s * math.cos(hdg), y0 + s * math.sin(hdg), hdg))
        s += step
    return pts


def sample_arc(x0, y0, hdg, length, curvature, step):
    """Exact circular arc sampling."""
    if abs(curvature) < 1e-10:
        return sample_line(x0, y0, hdg, length, step)
    R = 1.0 / curvature
    cx = x0 - R * math.sin(hdg)
    cy = y0 + R * math.cos(hdg)
    pts = []
    s = 0.0
    while s <= length + 1e-6:
        theta = hdg + curvature * s
        x = cx + R * math.sin(theta)
        y = cy - R * math.cos(theta)
        pts.append((x, y, theta))
        s += step
    return pts


def _fresnel_step(s, hdg0, curv0, curv_dot, ds=0.1):
    """Numerical integration for Euler spiral (clothoid)."""
    x, y, hdg = 0.0, 0.0, hdg0
    t = 0.0
    pts = [(x, y, hdg)]
    while t < s - 1e-9:
        dt = min(ds, s - t)
        k = curv0 + curv_dot * t
        x += dt * math.cos(hdg)
        y += dt * math.sin(hdg)
        hdg += k * dt
        t += dt
        pts.append((x, y, hdg))
    return pts


def sample_spiral(x0, y0, hdg0, length, curv_start, curv_end, step):
    curv_dot = (curv_end - curv_start) / length if length > 1e-9 else 0
    raw = _fresnel_step(length, 0.0, curv_start, curv_dot, ds=min(step * 0.25, 0.05))
    # Rotate + translate to world frame
    cos_h, sin_h = math.cos(hdg0), math.sin(hdg0)
    result = []
    for lx, ly, lhdg in raw:
        wx = x0 + lx * cos_h - ly * sin_h
        wy = y0 + lx * sin_h + ly * cos_h
        result.append((wx, wy, hdg0 + lhdg))
    # Subsample to requested step
    out = [result[0]]
    acc = 0.0
    for i in range(1, len(result)):
        dx = result[i][0] - result[i-1][0]
        dy = result[i][1] - result[i-1][1]
        acc += math.hypot(dx, dy)
        if acc >= step - 1e-6:
            out.append(result[i])
            acc = 0.0
    return out


def sample_poly3(x0, y0, hdg0, length, a, b, c, d, step):
    """Cubic polynomial in local Frenet frame."""
    cos_h, sin_h = math.cos(hdg0), math.sin(hdg0)
    pts = []
    s = 0.0
    while s <= length + 1e-6:
        u = s
        v = a + b * u + c * u**2 + d * u**3
        dv = b + 2*c*u + 3*d*u**2
        hdg_local = math.atan2(dv, 1.0)
        wx = x0 + u * cos_h - v * sin_h
        wy = y0 + u * sin_h + v * cos_h
        whdg = hdg0 + hdg_local
        pts.append((wx, wy, whdg))
        s += step
    return pts


def sample_param_poly3(x0, y0, hdg0, length, aU, bU, cU, dU,
                        aV, bV, cV, dV, p_range, step):
    """Parametric cubic polynomial."""
    cos_h, sin_h = math.cos(hdg0), math.sin(hdg0)
    pts = []
    # Estimate arc-length to step in parameter space
    n_steps = max(int(length / step), 2)
    p_max = 1.0 if p_range == 'normalized' else length
    for i in range(n_steps + 1):
        p = i / n_steps * p_max
        u = aU + bU*p + cU*p**2 + dU*p**3
        v = aV + bV*p + cV*p**2 + dV*p**3
        du = bU + 2*cU*p + 3*dU*p**2
        dv = bV + 2*cV*p + 3*dV*p**2
        hdg_local = math.atan2(dv, du)
        wx = x0 + u * cos_h - v * sin_h
        wy = y0 + u * sin_h + v * cos_h
        pts.append((wx, wy, hdg0 + hdg_local))
    return pts


# ─── xodr parser ──────────────────────────────────────────────────────────────

def parse_geometry(geom_el, step=2.0):
    """Return list of (x, y, heading_rad) world-space samples for one <geometry>."""
    x0   = float(geom_el.get('x', 0))
    y0   = float(geom_el.get('y', 0))
    hdg  = float(geom_el.get('hdg', 0))
    length = float(geom_el.get('length', 0))

    child = list(geom_el)
    tag = child[0].tag if child else 'line'
    el  = child[0] if child else None

    if tag == 'line' or el is None:
        return sample_line(x0, y0, hdg, length, step)

    elif tag == 'arc':
        curv = float(el.get('curvature', 0))
        return sample_arc(x0, y0, hdg, length, curv, step)

    elif tag == 'spiral':
        cs = float(el.get('curvStart', 0))
        ce = float(el.get('curvEnd', 0))
        return sample_spiral(x0, y0, hdg, length, cs, ce, step)

    elif tag == 'poly3':
        a = float(el.get('a', 0)); b = float(el.get('b', 0))
        c = float(el.get('c', 0)); d = float(el.get('d', 0))
        return sample_poly3(x0, y0, hdg, length, a, b, c, d, step)

    elif tag == 'paramPoly3':
        aU = float(el.get('aU', 0)); bU = float(el.get('bU', 0))
        cU = float(el.get('cU', 0)); dU = float(el.get('dU', 0))
        aV = float(el.get('aV', 0)); bV = float(el.get('bV', 0))
        cV = float(el.get('cV', 0)); dV = float(el.get('dV', 0))
        pRange = el.get('pRange', 'normalized')
        return sample_param_poly3(x0, y0, hdg, length,
                                  aU, bU, cU, dU, aV, bV, cV, dV, pRange, step)

    else:
        print(f"  [WARN] Unknown geometry type <{tag}>, falling back to line")
        return sample_line(x0, y0, hdg, length, step)


def get_road_elevation(road_el, s):
    """Sample elevation polynomial at arc-length s → z offset."""
    z = 0.0
    for elev in road_el.findall('.//elevationProfile/elevation'):
        s0 = float(elev.get('s', 0))
        if s0 > s:
            break
        ds = s - s0
        a = float(elev.get('a', 0)); b = float(elev.get('b', 0))
        c = float(elev.get('c', 0)); d = float(elev.get('d', 0))
        z = a + b*ds + c*ds**2 + d*ds**3
    return z


def extract_reference_lines(xodr_path, step=2.0):
    """
    Parse xodr and return dict:
      road_id -> list of (x, y, z, heading_rad) world-space samples
    """
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    roads = {}

    for road in root.findall('.//road'):
        road_id = road.get('id', '?')
        pts_all = []
        s_acc = 0.0  # running arc-length for elevation lookup

        for geom in road.findall('.//planView/geometry'):
            seg_pts = parse_geometry(geom, step=step)
            for x, y, hdg in seg_pts:
                z = get_road_elevation(road, s_acc)
                pts_all.append((x, y, z, hdg))
            length = float(geom.get('length', 0))
            s_acc += length

        if pts_all:
            roads[road_id] = pts_all

    return roads


# ─── Offset calculation ────────────────────────────────────────────────────────

def offset_polyline(pts, lateral_m, side='both'):
    def _single(pts, t):
        result = []
        for x, y, z, hdg in pts:
            rx =  math.cos(hdg + math.pi / 2)  # right normal x
            ry =  math.sin(hdg + math.pi / 2)  # right normal y
            ox = x + t * rx
            oy = y + t * ry
            result.append({'x': round(ox, 4),
                           'y': round(oy, 4),
                           'z': round(z,  4)})
        
        # --- FIX ROTAZIONE 180 GRADI ---
        # Se t è negativo (siamo sul lato sinistro), invertiamo l'array dei punti!
        if t > 0:
            result.reverse()
            
        return result

    if side == 'both':
        return {
            'right': _single(pts, +lateral_m),
            'left':  _single(pts, -lateral_m),
        }
    elif side == 'right':
        return {'right': _single(pts, +lateral_m)}
    else:
        return {'left': _single(pts, -lateral_m)}


# ─── Visualisation ────────────────────────────────────────────────────────────

# Distinct colours cycled per offset label so each type (guardrail, bike path…)
# gets its own hue.  All darker than the road blue so they read against white.
_OFFSET_PALETTE = [
    '#e63946',  # red
    '#f4a261',  # orange
    '#2a9d8f',  # teal
    '#9b5de5',  # violet
    '#f72585',  # pink
    '#06d6a0',  # mint
]


def plot_result(roads, result, save_path=None):
    """
    Plot road reference lines (blue) and all offset splines (coloured by label).
    Roads with fewer than 2 points are skipped silently.

    Args:
        roads      : dict road_id → [(x,y,z,hdg), ...]
        result     : output of build_output()
        save_path  : if set, save PNG there instead of showing interactively
    """
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as mpatches
        from matplotlib.lines import Line2D
    except ImportError:
        print("  [WARN] matplotlib not installed — skipping plot. pip install matplotlib")
        return

    fig, ax = plt.subplots(figsize=(14, 10))
    ax.set_aspect('equal')
    ax.set_facecolor('#1a1a2e')
    fig.patch.set_facecolor('#12122a')
    ax.tick_params(colors='#aaaacc')
    for spine in ax.spines.values():
        spine.set_edgecolor('#333355')
    ax.set_xlabel('X  (m)', color='#aaaacc', fontsize=9)
    ax.set_ylabel('Y  (m)', color='#aaaacc', fontsize=9)
    ax.set_title('xodr reference lines  ·  offset splines', color='#ddddff', fontsize=11, pad=12)
    ax.grid(True, color='#222244', linewidth=0.4, linestyle='--')

    # ── Road reference lines (blue) ──────────────────────────────────────────
    road_plotted = False
    for road_id, pts in roads.items():
        if len(pts) < 2:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        ax.plot(xs, ys, color='#4fc3f7', linewidth=1.2,
                alpha=0.85, solid_capstyle='round', zorder=2)
        road_plotted = True

    # ── Offset splines (one colour per label) ────────────────────────────────
    label_colour = {}   # label → hex colour
    colour_idx   = 0

    for entry in result['splines']:
        lbl = entry['label']
        if lbl not in label_colour:
            label_colour[lbl] = _OFFSET_PALETTE[colour_idx % len(_OFFSET_PALETTE)]
            colour_idx += 1
        colour = label_colour[lbl]

        pts = entry['points']
        if len(pts) < 2:
            continue
        xs = [p['x'] for p in pts]
        ys = [p['y'] for p in pts]
        ax.plot(xs, ys, color=colour, linewidth=0.9,
                alpha=0.75, solid_capstyle='round', zorder=3)

    # ── Legend ───────────────────────────────────────────────────────────────
    legend_handles = []
    if road_plotted:
        legend_handles.append(
            Line2D([0], [0], color='#4fc3f7', linewidth=2, label='road centre (xodr)')
        )
    for lbl, colour in label_colour.items():
        # Collect unique offsets for this label to show in legend
        offsets = sorted({e['offset_m'] for e in result['splines'] if e['label'] == lbl})
        off_str = ', '.join(f'{o}m' for o in offsets)
        legend_handles.append(
            Line2D([0], [0], color=colour, linewidth=2, label=f'{lbl}  ({off_str})')
        )

    legend = ax.legend(
        handles=legend_handles,
        loc='upper left',
        fontsize=8,
        framealpha=0.35,
        facecolor='#1a1a2e',
        edgecolor='#444466',
        labelcolor='#ddddff',
    )

    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight',
                    facecolor=fig.get_facecolor())
        print(f"      Plot saved → {save_path}")
    else:
        plt.show()

    plt.close(fig)


# ─── Main ──────────────────────────────────────────────────────────────────────

def parse_offset_spec(spec_list):
    """
    Parse CLI offsets like "6:guardrail" or "10:bike_path:left"
    Returns list of dicts: {distance, label, side}
    """
    result = []
    for s in spec_list:
        parts = s.split(':')
        dist  = float(parts[0])
        label = parts[1] if len(parts) > 1 else f'offset_{dist}m'
        side  = parts[2] if len(parts) > 2 else 'both'
        result.append({'distance': dist, 'label': label, 'side': side})
    return result


def build_output(roads, offset_specs, road_filter=None):
    out = {'splines': []}
    for road_id, pts in roads.items():
        if road_filter and road_id not in road_filter:
            continue
        for spec in offset_specs:
            sides = offset_polyline(pts, spec['distance'], spec['side'])
            for side_key, points in sides.items():
                out['splines'].append({
                    'road_id':  road_id,
                    'label':    spec['label'],
                    'side':     side_key,
                    'offset_m': spec['distance'],
                    'points':   points,
                    # Hint for UE5 Editor Python / BP_RepSpline
                    'ue_name':  f"RepSpline_{spec['label']}_{side_key}_road{road_id}",
                })
    return out


def main():
    parser = argparse.ArgumentParser(description='xodr → parallel offset splines for CARLA BP_RepSpline')
    parser.add_argument('--xodr',    required=True, help='Path to .xodr file')
    parser.add_argument('--offsets', nargs='+', default=['25:guardrail'],
                    help='List of "distance:label[:side]" e.g. "25:guardrail:both"')
    parser.add_argument('--step',    type=float, default=2.0,
                        help='Sampling step along road in metres (default 2.0)')
    parser.add_argument('--output',  default='C:/Dev/CarlaUE5/Unreal/CarlaUnreal/Source/CarlaUnreal/SplieGen/Json/splines.json',
                        help='Output JSON path')
    parser.add_argument('--roads',     nargs='*', default=None,
                        help='Filter to specific road IDs (default: all)')
    parser.add_argument('--plot',      action='store_true',
                        help='Show interactive matplotlib preview after generation')
    parser.add_argument('--save-plot', metavar='PATH', default=None,
                        help='Save plot to PNG path instead of (or in addition to) showing it')
    args = parser.parse_args()

    print(f"[1/3] Parsing {args.xodr} ...")
    roads = extract_reference_lines(args.xodr, step=args.step)
    print(f"      Found {len(roads)} roads")

    offset_specs = parse_offset_spec(args.offsets)
    print(f"[2/3] Computing offsets: {[s['label'] for s in offset_specs]} ...")
    result = build_output(roads, offset_specs, road_filter=args.roads)

    n = len(result['splines'])
    total_pts = sum(len(s['points']) for s in result['splines'])
    print(f"      Generated {n} splines, {total_pts} total control points")

    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)
    print(f"[3/3] Written → {args.output}")

    if args.plot or args.save_plot:
        print(f"[4/4] Rendering plot ...")
        # If --save-plot only (no --plot), write file without opening a window
        save_path = args.save_plot
        if not args.plot and save_path:
            import matplotlib
            matplotlib.use('Agg')   # non-interactive backend
        plot_result(roads, result, save_path=save_path)
        if args.plot and not save_path:
            pass   # plt.show() already called inside plot_result

    print()
    print("  Next: run ue5_spawn_repsplines.py inside the UE5 Editor Python console")
    print("  to consume this JSON and spawn BP_RepSpline actors in the level.")


if __name__ == '__main__':
    main()
