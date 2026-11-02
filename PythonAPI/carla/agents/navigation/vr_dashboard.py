"""
Bridges the ego agent's live driving state and detected hazards to the
in-VR AR-HUD dashboard.

This targets the World-level RPC bindings documented and exercised in
PythonAPI/examples/arhud_client1_vehicle.py (ego speed/steer),
arhud_client2_obstacles.py (obstacles, keyed by id) and
ARHUD_dashboard_test.py (full API reference):

    world.set_type_string_dashboard(type_string)
    world.set_control_value_dashboard(speed_kmh, steer)
    world.update_obstacle_dashboard(id, type, is_danger, position, rotation,
                                     obstacle_speed, obstacle_steer)
    world.remove_id_obstacle_dashboard(id)
    world.update_dashboard()

If a binding is missing (older server build) or no ADashboard actor was
registered server-side, every call silently no-ops, same as the reference
clients — this module is safe to use even when nothing is listening.
"""

import math
import time

import carla

from agents.tools.misc import get_speed

# The dashboard's blueprint (CarlaVR_Vehicles/ARHUD_TestCase/BP_DashboardManager) only
# draws a bounding box for 3 obstacle types: 'car' (BP_BB_Car), 'truck' (BP_BB_Truck) and
# 'pedestrian' (BP_BB_Pedestrian), fixed-size boxes, red or yellow depending on isDanger.
# Any other type (cone, sign, bike...) is kept in its list but never drawn, so every
# obstacle is sent with the box type closest to its shape.
_CATEGORY_TO_TYPE = {
    'vehicle': 'car',
    'pedestrian': 'pedestrian',
    'traffic_light': 'sign',  # no box for traffic lights
    'construction': 'pedestrian',
    'parked': 'car',  # parked cars near the ego ('full' mode), never dangers: shown only while listed
}

# Height [m] of a pedestrian box's pivot above the ground (a walker's location is the
# center of its body): small props drawn with that box are raised by it
_PEDESTRIAN_BOX_HEIGHT = 0.9

# Bounds [m] of how much car / truck boxes are raised (to the center of the vehicle's bounding box)
_VEHICLE_BOX_MIN_HEIGHT = 0.5
_VEHICLE_BOX_MAX_HEIGHT = 1.5

# Minimum time [s] a sentence stays shown in the 'text' mode before the next one replaces it (some
# messages are logged at every tick, e.g. in junctions, and would make the text flicker)
_TEXT_MIN_DURATION = 1.5
# Same, for an urgent sentence (the ego starting to brake for something): shown almost at once
_URGENT_TEXT_MIN_DURATION = 0.4

# Global path ('full' mode): the stretch of the route ahead of the ego, updated as it drives. Two styles:
# - 'arhud': drawn by the AR-HUD (the dashboard blueprint's global path planner / virtual carpet), fed
#   like arhud_client3_waypoints.py. It needs the ego vehicle's blueprint to register with the dashboard
#   blueprint when it spawns (BeginPlay -> BP_DashboardManager.E_VR_Vehicle_Spawn, as BP_XAI_vivace does).
#   Only PATH_ARHUD_POINTS waypoints are sent at a time (the whole route at once freezes the simulator)
# - 'ribbon' (default): a flat band laid on the road by the server (a mesh following the road surface, with
#   the AR-HUD's translucent projection material PATH_RIBBON_MATERIAL: soft, not glowing)
# - 'lines': 3D lines in the world (CARLA's debug line batcher: unlit, can be glaring)
PATH_STYLE = 'ribbon'
PATH_RIBBON_MATERIAL = '/CarlaVR_Vehicles/ARHUD_TestCase/ARHUD_Designs/M_Projection_LightBlue.M_Projection_LightBlue'
# (also M_Projection_White, M_Projection_LightGreen, M_Projection_LightRed in the same folder)
PATH_SPACING = 2.0  # [m] between path points (the ribbon joins them with straight pieces: closer follows curves better)
PATH_RIBBON_REFRESH = 3  # 'ribbon' style: sent again every that many path points driven (~6 m)
PATH_ARHUD_POINTS = 30  # waypoints sent to the AR-HUD at a time (~120 m)
PATH_ARHUD_REFRESH = 10  # sent again every that many waypoints driven (~40 m)
PATH_HEIGHT = 0.2  # [m] above the road ('arhud' style)
# 'lines' style: a soft band on the road along the route, covering PATH_WIDTH_RATIO of the lane. Debug
# lines are unlit (drawn at full brightness, can't be transparent): a dark, matte color, so that the band
# reads as a tint of the road and doesn't glow. It is made of PATH_STRIPS lines side by side (lying flat on
# the road) joined edge to edge, and each piece is drawn only once, for PATH_PIECE_LIFE seconds: lines
# drawn on top of each other (redrawing the same piece, overlapping strips) shimmer
PATH_LINE_HEIGHT = 0.08  # [m] above the road
PATH_START = 6.0  # [m] ahead of the ego where the path starts (not right under the hood)
PATH_AHEAD = 70.0  # [m] of path drawn ahead of the ego
PATH_WIDTH_RATIO = 0.5  # width of the band, relative to the lane width
PATH_DEFAULT_LANE_WIDTH = 3.5  # [m] when the route doesn't give it
PATH_STRIPS = 4  # lines making the band (each line is one call to the server)
PATH_COLOR = carla.Color(0, 0, 62)  # dark, desaturated blue-grey
PATH_PIECE_LIFE = 40.0  # [s] (simulation time) a piece of band lasts, redrawn just before it expires
PATH_REDRAW_PERIOD = 0.5  # [s]: the lines last a bit longer, so that the path never blinks

# Default height [cm] of the 'text' mode sentence above the center of the driver's view (negative: below)
TEXT_HEIGHT = 30.0

# Where the box of an obstacle whose actor was destroyed is sent: the blueprint never
# destroys boxes, it would stay floating where it was last drawn
_HIDDEN_DEPTH = -1000.0  # [m] below the obstacle

# A box is sent again only when its actor moved / turned more than this (or moves, or its danger changes)
_BOX_MOVE_TOLERANCE = 0.05  # [m]
_BOX_TURN_TOLERANCE = 1.0  # [deg]
# ... or when its speed / steer (the motion prediction the dashboard draws from them) changed more than this
_BOX_SPEED_TOLERANCE = 0.5  # [km/h]
_BOX_STEER_TOLERANCE = 0.02  # [-1, 1]
# Moving actors farther than this from the ego: their box is updated every _BOX_FAR_PERIOD ticks only
_BOX_FAR_DISTANCE = 80.0  # [m]
_BOX_FAR_PERIOD = 5

# Dashboard ids of the level's parked cars (LevelObject): above any actor id
PARKED_CAR_FIRST_ID = 1000000


class LevelObject:
    """
    An object that is part of the level (not an actor, e.g. a parked car of Town15), with what
    sync_obstacles reads from an actor, to give it a box. Static: never moves, never destroyed
    """

    def __init__(self, object_id, world_box, truck=False):
        self.id = object_id
        self.type_id = 'level.parked_truck' if truck else 'level.parked_car'
        self.attributes = {}
        self.is_alive = True
        extent = world_box.extent
        # Located at its base, like a vehicle, with its box centered above it
        self.location = carla.Location(world_box.location.x, world_box.location.y,
                                       world_box.location.z - extent.z)
        self.bounding_box = carla.BoundingBox(carla.Location(0.0, 0.0, extent.z), extent)
        self._transform = carla.Transform(self.location, world_box.rotation)

    def get_transform(self):
        return self._transform

    def get_location(self):
        return self.location

    def get_velocity(self):
        return carla.Vector3D()


def _motion(actor):
    """
    (speed [km/h], steer [-1, 1]) of an obstacle, from which the dashboard draws its motion prediction: the
    same units as the ego's (set_control_value_dashboard). Steer: the vehicle's last control (read from the
    snapshot, no call to the server); 0 for walkers, props and parked cars
    """
    speed = get_speed(actor)
    steer = 0.0
    if isinstance(actor, carla.Vehicle):
        steer = actor.get_control().steer
    return speed, steer


def _box_center_height(actor):
    """Height [m] of the center of an actor's bounding box above its location (its base for vehicles)"""
    bounding_box = actor.bounding_box
    height = bounding_box.location.z
    if height < 0.1:  # box not centered on the volume: half the height of the actor
        height = bounding_box.extent.z
    return min(max(height, _VEHICLE_BOX_MIN_HEIGHT), _VEHICLE_BOX_MAX_HEIGHT)


def is_parked_vehicle(actor):
    """True for a parked car: a static mesh prop of a vehicle (spawned along the route by the leaderboard)"""
    if actor.type_id != 'static.prop.mesh':
        return False
    mesh_path = actor.attributes.get('mesh_path', '').lower()
    if mesh_path:
        return 'parkedvehicles' in mesh_path or '/car/' in mesh_path
    extent = actor.bounding_box.extent
    return extent.x > 1.5 and extent.y > 0.7  # no mesh path given: car-sized


def _obstacle_type(category, actor):
    """Dashboard obstacle type (box) of a detected actor, from its category and blueprint."""
    if category == 'parked' or is_parked_vehicle(actor):
        # (also when the ego avoids it like a static obstacle)
        return 'truck' if 'truck' in actor.type_id else 'car'
    type_id = actor.type_id.lower()
    if category == 'vehicle':
        base_type = actor.attributes.get('base_type', '').lower()
        if base_type in ('truck', 'van', 'bus') or any(k in type_id for k in ('truck', 'bus', 'van', 'sprinter')):
            return 'truck'
        if base_type in ('bicycle', 'motorcycle') or any(k in type_id for k in ('bike', 'bicycle', 'motorcycle', 'vespa', 'yamaha', 'harley', 'kawasaki')):
            return 'pedestrian'  # narrow and upright
        return 'car'
    if category == 'construction':
        # Wide props (the warning panel, barriers): car box. Small ones (cones, signs): pedestrian box
        if any(k in type_id for k in ('trafficwarning', 'warningconstruction', 'barrier', 'barricade', 'fence')):
            return 'car'
        return 'pedestrian'
    return _CATEGORY_TO_TYPE.get(category, 'car')


# Visualization modes, chosen per route in the routes XML (<route ... visualization="...">):
# - 'full': ego speed / steer, the global path of the route, and bounding boxes on the obstacles the
#   ego takes into account
# - 'none': nothing (speed and steer streamed as 0, no boxes, no text)
# - 'text': only a sentence telling what the ego is doing and why (BasicAgent2._narrate), no boxes
VISUALIZATION_MODES = ('full', 'none', 'text')


class VRDashboard:
    """Live bridge from one ego agent's run_step() to the VR dashboard."""

    def __init__(self, world, scale=100.0, mode='full', text_height=TEXT_HEIGHT):
        # scale converts CARLA's metres to the dashboard's expected units
        # (the server does not auto-convert, see ARHUD_dashboard_test.py).
        self._world = world
        self._scale = scale
        self._text = None  # text last shown
        self._text_time = 0.0  # when it was shown
        self._text_height_sent = None  # its height
        self._pending_text = None  # (text, height, urgent) to show, once the current one was shown long enough
        self.text_height = text_height  # default height of the texts [cm above the center of the view]
        self._path_points = []  # the route resampled every PATH_SPACING m (carla.Transform)
        self._path_widths = []  # lane width at each of them [m]
        self._ribbon_index = None  # path point the ego was at when the ribbon was last sent
        self._path_index = 0  # path point the ego is at
        self._path_drawn_time = None  # when the path was last drawn (time.monotonic, 'lines' style)
        self._path_piece_expiry = {}  # path point index -> simulation time its piece of band expires
        self._path_sent_index = None  # path point the ego was at when the path was last sent ('arhud' style)
        self._followed = {}  # actor id -> (actor, category): obstacles shown, whose box follows the actor
        self._last_sent = {}  # id -> (type, position, rotation, is_danger, speed, steer) last sent, to hide its box
        self._tick = 0  # sync_obstacles calls
        self.mode = None
        self.set_mode(mode)

    def set_mode(self, mode):
        """Selects the visualization mode (see VISUALIZATION_MODES)"""
        mode = (mode or 'full').lower()
        if mode not in VISUALIZATION_MODES:
            print("WARNING: unknown visualization mode '{}', using 'full' (expected one of {})".format(
                mode, VISUALIZATION_MODES))
            mode = 'full'
        self.mode = mode
        # Tells the server the mode: outside 'full' it hides the AR-HUD driving aids that the dashboard
        # blueprints draw by themselves from the vehicle's own steering / speed (virtual carpet,
        # motion prediction arrows, navigation waypoints), which no dashboard value controls
        self._call('set_type_string_dashboard', 'mode:' + mode)
        # The dashboard's string is shown as text in the VR vehicle (WB_Text_XAI, through the server
        # forwarding it to the vehicle's Write_Text event): empty except in 'text' mode
        self._text = None  # always resend: the vehicle may still show a previous route's text
        self._pending_text = None
        self._show_text('')
        if mode != 'full':
            self._call('set_control_value_dashboard', 0.0, 0.0)
        self.push()

    def start_gaze_recording(self, name):
        """
        Starts recording the driver's gaze (VR headset eye tracking) on the server, to
        <CARLA Saved dir>/GazeLogs/<name>_<date>.csv (one line per frame: gaze, what it hits, head, ego)
        """
        self._call('set_type_string_dashboard', 'gaze:start:{}'.format(name))

    def stop_gaze_recording(self):
        """Stops the driver's gaze recording (the file is closed)"""
        self._call('set_type_string_dashboard', 'gaze:stop')

    def set_text(self, message, height=None, urgent=False):
        """
        In 'text' mode, shows a sentence telling what the ego is doing, until the next one.

        :param height: height of the text above the center of the driver's view [cm] (negative: below),
            self.text_height if None
        :param urgent: an urgent sentence (e.g. the ego starting to brake) replaces the current one
            after _URGENT_TEXT_MIN_DURATION seconds, others after _TEXT_MIN_DURATION seconds. Meanwhile
            only the latest sentence is kept (see push)
        """
        if self.mode != 'text':
            return
        message = str(message)
        height = self.text_height if height is None else float(height)
        if message == self._text and height == self._text_height_sent:
            self._pending_text = None
        elif time.monotonic() - self._text_time >= (_URGENT_TEXT_MIN_DURATION if urgent else _TEXT_MIN_DURATION):
            self._pending_text = None
            self._show_text(message, height)
        else:
            self._pending_text = (message, height, urgent)

    def _show_text(self, text, height=None):
        if text == self._text and (not text or height == self._text_height_sent):
            return
        self._text = text
        self._text_height_sent = height
        self._text_time = time.monotonic()
        if text and height is not None:
            text = '[height={:g}]{}'.format(height, text)
        self._call('set_type_string_dashboard', text)

    def _call(self, method, *args):
        fn = getattr(self._world, method, None)
        if fn is None:
            return
        try:
            fn(*args)
        except Exception:  # noqa: BLE001 - dashboard issues must never break driving
            pass

    def update_ego(self, speed_kmh, steer):
        """Streams the ego's current speed (km/h) and steer to the dashboard (0 except in 'full' mode)."""
        if self.mode != 'full':
            speed_kmh, steer = 0.0, 0.0
        self._call('set_control_value_dashboard', float(speed_kmh), float(steer))

    def sync_obstacles(self, detections, ego_location=None):
        """
        Updates the dashboard's obstacles, keyed by actor id (like arhud_client2_obstacles.py).
        The dashboard's blueprint never removes the box of an obstacle, so once shown, an
        obstacle's box keeps following its actor (every tick) until the end of the route: red
        while the ego reacts to it, yellow otherwise. The box of an actor that gets destroyed is
        moved out of sight.

        :param detections: list of (actor, category, is_danger) for every obstacle the ego
            currently takes into account (category: see _CATEGORY_TO_TYPE). An actor listed
            several times is a danger if any entry says so.
        :param ego_location: the boxes of actors farther than _BOX_FAR_DISTANCE from it are updated
            less often
        """
        if self.mode != 'full':
            return  # no boxes
        self._tick += 1
        danger = {}  # actor id -> is_danger, this tick
        for actor, category, is_danger in detections:
            if actor.id not in self._followed:
                self._followed[actor.id] = (actor, category)
            danger[actor.id] = danger.get(actor.id, False) or bool(is_danger)

        for actor_id, (actor, category) in list(self._followed.items()):
            try:
                transform = actor.get_transform()
                speed, steer = _motion(actor)
                obstacle_type = _obstacle_type(category, actor)
                alive = actor.is_alive
            except RuntimeError:
                alive = False
            # A parked car's box is hidden once it is no longer listed (out of range): there are hundreds
            # along a route; it comes back if listed again
            if not alive or (category == 'parked' and actor_id not in danger):
                self._hide(actor_id)
                del self._followed[actor_id]
                continue
            location = transform.location
            z = location.z
            if obstacle_type in ('car', 'truck'):
                # The car / truck boxes are centered on their pivot, while a vehicle (or prop) is located
                # at its base: raised to the center of its bounding box (~0.7-0.8 m for a car)
                z += _box_center_height(actor)
            elif obstacle_type == 'pedestrian' and category != 'pedestrian':
                z += _PEDESTRIAN_BOX_HEIGHT  # props / bikes are located at their base
            position = carla.Vector3D(location.x * self._scale,
                                       location.y * self._scale,
                                       z * self._scale)
            rotation = carla.Vector3D(0.0, 0.0, transform.rotation.yaw)
            is_danger = danger.get(actor_id, False)
            # Each update is a call to the server, waited for before the next simulation step: only sent
            # when the box or its motion prediction changes (not for the props and parked cars, every tick),
            # and every few ticks for the actors far from the ego
            last = self._last_sent.get(actor_id)
            if last is not None and last[3] == is_danger and last[0] == obstacle_type:
                moved = math.hypot(position.x - last[1].x, position.y - last[1].y) + abs(position.z - last[1].z)
                turned = abs((rotation.z - last[2].z + 180.0) % 360.0 - 180.0)
                motion_changed = (abs(speed - last[4]) >= _BOX_SPEED_TOLERANCE
                                  or abs(steer - last[5]) >= _BOX_STEER_TOLERANCE)
                if (moved < _BOX_MOVE_TOLERANCE * self._scale and turned < _BOX_TURN_TOLERANCE and speed < 0.1
                        and not motion_changed):
                    continue
                if (ego_location is not None and location.distance(ego_location) > _BOX_FAR_DISTANCE
                        and self._tick % _BOX_FAR_PERIOD != actor_id % _BOX_FAR_PERIOD):
                    continue
            self._call('update_obstacle_dashboard', actor_id, obstacle_type,
                       is_danger, position, rotation, float(speed), float(steer))
            self._last_sent[actor_id] = (obstacle_type, position, rotation, is_danger, speed, steer)

    def rendered_boxes(self):
        """The boxes drawn right now: [(actor id, box type, is_danger)] (empty outside the 'full' mode)"""
        return [(actor_id, last[0], last[3]) for actor_id, last in self._last_sent.items()]

    def _hide(self, actor_id):
        """Moves the box of an obstacle out of sight (where it stays: boxes are never destroyed)"""
        last = self._last_sent.pop(actor_id, None)
        if last is not None:
            obstacle_type, position, rotation = last[:3]
            hidden = carla.Vector3D(position.x, position.y, position.z + _HIDDEN_DEPTH * self._scale)
            self._call('update_obstacle_dashboard', actor_id, obstacle_type, False, hidden, rotation, 0.0, 0.0)

    def clear(self):
        """End of the route: moves the boxes of every obstacle shown out of sight, removes the path ribbon."""
        if PATH_STYLE == 'ribbon':
            self._call('set_type_string_dashboard', 'ribbon:')
        for actor_id in list(self._followed):
            self._hide(actor_id)
        self._followed = {}
        self.push()

    def set_path(self, plan, spacing=PATH_SPACING):
        """
        'full' mode: sets the global path of the route, drawn ahead of the ego as it drives (see update_path).

        :param plan: the route, list of (carla.Waypoint or carla.Transform, RoadOption)
        """
        self._path_points, self._path_index, self._path_drawn_time, self._path_sent_index = [], 0, None, None
        self._path_piece_expiry = {}
        if self.mode != 'full' or not plan:
            return
        transforms = [getattr(item[0], 'transform', item[0]) for item in plan]
        widths = [getattr(item[0], 'lane_width', None) or PATH_DEFAULT_LANE_WIDTH for item in plan]
        points, point_widths = [transforms[0]], [widths[0]]
        travelled = 0.0
        for index in range(1, len(transforms)):
            travelled += transforms[index - 1].location.distance(transforms[index].location)
            if travelled >= spacing:
                points.append(transforms[index])
                point_widths.append(widths[index])
                travelled = 0.0
        if points[-1] is not transforms[-1]:
            points.append(transforms[-1])
            point_widths.append(widths[-1])
        self._path_points = points
        self._path_widths = point_widths
        self._ribbon_index = None
        if PATH_STYLE == 'ribbon':
            self._call('set_type_string_dashboard', 'ribbon_material:' + PATH_RIBBON_MATERIAL)
        print("VR dashboard: global path of {} points every {:.0f} m ({} style)".format(
            len(points), spacing, PATH_STYLE))

    def update_path(self, ego_location, ahead=PATH_AHEAD):
        """
        'full' mode, every tick: follows the ego along the global path and shows the stretch ahead of it,
        in the PATH_STYLE style
        """
        if self.mode != 'full' or not self._path_points:
            return
        points = self._path_points
        # Where the ego is along the path: the closest of the next few points only (it's called every tick,
        # the ego drives less than a point per tick). Searching further would jump, where the route loops
        # over the same place again, to its second passage, skipping the loop
        last = min(len(points), self._path_index + 5)
        index = min(range(self._path_index, last), key=lambda i: points[i].location.distance(ego_location))
        if points[index].location.distance(ego_location) > 15.0:
            # Far from the path (e.g. the driver took over and left it): the first point ahead near the ego
            for i in range(self._path_index, min(len(points), self._path_index + 60)):
                if points[i].location.distance(ego_location) < 8.0:
                    index = i
                    break
        self._path_index = index
        if PATH_STYLE == 'arhud':
            self._send_arhud_path()
            return
        if PATH_STYLE == 'ribbon':
            self._send_ribbon(ahead)
            return

        now = time.monotonic()
        if self._path_drawn_time is not None and now - self._path_drawn_time < PATH_REDRAW_PERIOD:
            return
        self._path_drawn_time = now

        try:
            sim_now = self._world.get_snapshot().timestamp.elapsed_seconds
        except RuntimeError:
            return
        debug = self._world.debug
        lift = carla.Location(z=PATH_LINE_HEIGHT)
        first, last = self._path_index, min(len(points) - 1, self._path_index + int((PATH_START + ahead) / PATH_SPACING) + 1)

        def right_at(i):
            """Unit vector to the right of the path at point i (averaged over its two segments: the pieces
            of band on both sides of the point share the same edge, no gap nor overlap in curves)"""
            a = points[max(0, i - 1)].location
            b = points[min(len(points) - 1, i + 1)].location
            dx, dy = b.x - a.x, b.y - a.y
            norm = math.hypot(dx, dy) or 1.0
            return carla.Location(x=-dy / norm, y=dx / norm)

        try:
            distance = 0.0  # along the path, from the ego
            for i in range(first, last):
                begin, finish = points[i].location + lift, points[i + 1].location + lift
                distance += begin.distance(finish)
                if distance <= PATH_START:
                    continue
                expiry = self._path_piece_expiry.get(i)
                if expiry is not None and expiry - sim_now > 1.0:
                    continue  # still drawn
                # Piece of band from point i to point i + 1: PATH_STRIPS lines side by side
                half_begin = PATH_WIDTH_RATIO * self._path_widths[i] / 2.0
                half_finish = PATH_WIDTH_RATIO * self._path_widths[i + 1] / 2.0
                right_begin, right_finish = right_at(i), right_at(i + 1)
                strip = 2.0 * min(half_begin, half_finish) / PATH_STRIPS
                for k in range(PATH_STRIPS):
                    u = (k + 0.5) / PATH_STRIPS * 2.0 - 1.0  # -1 (left edge) .. 1 (right edge)
                    debug.draw_line(begin + right_begin * (u * half_begin),
                                    finish + right_finish * (u * half_finish),
                                    thickness=strip, color=PATH_COLOR, life_time=PATH_PIECE_LIFE)
                self._path_piece_expiry[i] = sim_now + PATH_PIECE_LIFE
        except RuntimeError:
            pass  # drawing must never break driving

    def _send_ribbon(self, ahead):
        """
        Sends the center line of the path ahead of the ego (from PATH_START to PATH_START + ahead meters)
        to the server, which lays the band on the road. Again every PATH_RIBBON_REFRESH path points (the
        server rebuilds the band each time: every point made frequent hitches in VR); the band starts
        closer to the ego meanwhile, and goes PATH_RIBBON_REFRESH points further so its far end doesn't pop
        """
        if self._ribbon_index is not None and 0 <= self._path_index - self._ribbon_index < PATH_RIBBON_REFRESH:
            return
        ahead += PATH_RIBBON_REFRESH * PATH_SPACING
        self._ribbon_index = self._path_index
        points, widths = self._path_points, self._path_widths
        items, distance = [], 0.0
        for i in range(self._path_index, len(points)):
            if i > self._path_index:
                distance += points[i - 1].location.distance(points[i].location)
            if distance < PATH_START:
                continue
            location = points[i].location
            items.append('{:.2f},{:.2f},{:.2f},{:.2f}'.format(
                location.x, location.y, location.z, PATH_WIDTH_RATIO * widths[i]))
            if distance > PATH_START + ahead:
                break
        self._call('set_type_string_dashboard', 'ribbon:' + ';'.join(items))

    def _send_arhud_path(self, window=PATH_ARHUD_POINTS, refresh=PATH_ARHUD_REFRESH):
        """
        Sends the stretch of path ahead of the ego to the AR-HUD, again each time the ego drove `refresh`
        waypoints (the dashboard's path is emptied first: the server does it on the 'mode:' command, as
        add_vector_pair_dashboard only appends)
        """
        if self._path_sent_index is not None and self._path_index - self._path_sent_index < refresh:
            return
        self._path_sent_index = self._path_index
        stretch = self._path_points[max(0, self._path_index - 1):self._path_index - 1 + window]
        self._call('set_type_string_dashboard', 'mode:full')  # empties the dashboard's path
        for index, transform in enumerate(stretch):
            location = transform.location
            # Heading along the path (towards the next point), the waypoint's own one at the end
            if index + 1 < len(stretch):
                following = stretch[index + 1].location
                yaw = math.degrees(math.atan2(following.y - location.y, following.x - location.x))
            else:
                yaw = transform.rotation.yaw
            self._call('add_vector_pair_dashboard',
                       carla.Vector3D(location.x * self._scale, location.y * self._scale,
                                      (location.z + PATH_HEIGHT) * self._scale),
                       carla.Vector3D(0.0, 0.0, yaw))
        self._call('trigger_path_dashboard')

    def push(self):
        """Flushes this tick's updates to the dashboard (and a waiting 'text' mode sentence, when due)."""
        if self._pending_text is not None:
            text, height, urgent = self._pending_text
            if time.monotonic() - self._text_time >= (_URGENT_TEXT_MIN_DURATION if urgent else _TEXT_MIN_DURATION):
                self._pending_text = None
                self._show_text(text, height)
        self._call('update_dashboard')


_dashboard = None  # type: VRDashboard | None


def get_dashboard(world, mode='full'):
    """Returns the shared VRDashboard instance (creating it on first call), in the given visualization mode."""
    global _dashboard
    try:
        same_world = _dashboard is None or _dashboard._world.id == world.id
    except RuntimeError:
        same_world = False
    if not same_world:
        # New route / world: drop what the previous one left in the dashboard
        _dashboard.clear()
        _dashboard = None
    if _dashboard is None:
        _dashboard = VRDashboard(world, mode=mode)
    else:
        _dashboard.clear()  # a new agent starts with an empty dashboard
        _dashboard.set_mode(mode)
    return _dashboard
