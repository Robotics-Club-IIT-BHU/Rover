#!/usr/bin/env python3
"""
ROS2 node that builds a persistent three-state occupancy map from a depth
point cloud while a simulated rover drives forward, curving around
confirmed obstacles.

The camera never rotates: body -> world is a pure translation.
    wx = x + rover_x
    wz = z + rover_z
The rover translates along a steered heading while the camera keeps
facing +z (crab motion). See the note in tick() on what that models.

Grid states
-----------
  UNKNOWN  (gray)   never observed.
  FREE     (white)  a ray passed through with nothing detected before it.
  OCCUPIED (black)  confirmed over `freeze_hits` frames. Permanent.

Path planning (enable_planning, default False)
----------------------------------------------
Off: the rover drives straight and none of the planning code runs.

On: A* over a coarse local window, replanned every `replan_period`.
Obstacles are COSTLY, not impassable:
    core cell   (a black fine cell)      -> core_cost
    margin cell (within the rover's      -> margin_cost
                 half-width of a core)
    unknown cell                         -> unknown_cost
Every cell stays traversable, so the search ALWAYS returns a complete
path to the horizon and never returns empty. The rover therefore never
stalls; it bends around obstacles because bending is cheaper than
cutting through. If it is genuinely walled in, the path degrades to
crossing the obstacle and the HUD says so, rather than deadlocking.

The path is then corner-cut (Chaikin) into a smooth curve, and followed
with a rate-limited heading controller so the rover eases onto it
instead of snapping sideways.

Controls
--------
  left-drag   pan
  wheel       zoom
  f           re-enable follow mode
  c           clear the map
  p           toggle planning at runtime
  q / ESC     quit
"""
import heapq
import math

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

UNKNOWN, FREE, OCC = 0, 1, 2
PALETTE = np.array([[170, 170, 170],   # UNKNOWN - gray
                    [255, 255, 255],   # FREE    - white
                    [0, 0, 0]],        # OCC     - black
                   dtype=np.uint8)

NEIGHBOURS = ((1, 0), (1, 1), (1, -1), (0, 1), (0, -1),
              (-1, 0), (-1, 1), (-1, -1))


class MapperNode(Node):
    def __init__(self):
        super().__init__('mapper_node')

        # --- Window ---
        self.declare_parameter('width', 900)
        self.declare_parameter('height', 900)
        self.declare_parameter('scale', 120.0)

        # --- Cloud filtering, height above floor ---
        self.declare_parameter('floor_y', 0.184)
        self.declare_parameter('h_min', 0.03)
        self.declare_parameter('h_max', 0.50)
        self.declare_parameter('z_min', 0.15)
        self.declare_parameter('z_max', 2.0)

        # --- Ray casting ---
        self.declare_parameter('horizontal_fov_deg', 85.0)
        self.declare_parameter('angular_res_deg', 0.5)

        # --- Map ---
        self.declare_parameter('cell_size', 0.02)
        self.declare_parameter('map_half_x', 6.0)
        self.declare_parameter('map_len_z', 30.0)
        self.declare_parameter('freeze_hits', 4)
        self.declare_parameter('neighbourhood', 1)

        # --- Rover ---
        self.declare_parameter('sim_speed', 0.3)
        self.declare_parameter('corridor_width', 0.6)
        self.declare_parameter('warn_distance', 0.4)
        self.declare_parameter('frame_period', 0.05)

        # --- Path planning ---
        self.declare_parameter('enable_planning', True)
        self.declare_parameter('plan_cell_size', 0.10)
        self.declare_parameter('plan_horizon', 2.5)
        self.declare_parameter('plan_half_width', 2.5)
        self.declare_parameter('replan_period', 0.4)
        self.declare_parameter('lookahead', 0.45)
        self.declare_parameter('smooth_iters', 2)
        # Traversal penalties, in multiples of one cell step.
        self.declare_parameter('unknown_cost', 0.6)
        self.declare_parameter('margin_cost', 8.0)
        self.declare_parameter('core_cost', 120.0)
        self.declare_parameter('lateral_cost', 0.15)
        # Steering limits.
        self.declare_parameter('max_turn_rate_deg', 90.0)   # deg/s
        self.declare_parameter('max_heading_deg', 55.0)     # from +z

        g = self.get_parameter
        self.width = g('width').value
        self.height = g('height').value
        self.scale = g('scale').value
        self.floor_y = g('floor_y').value
        self.h_min = g('h_min').value
        self.h_max = g('h_max').value
        self.z_min = g('z_min').value
        self.z_max = g('z_max').value
        self.cell = g('cell_size').value
        self.map_half_x = g('map_half_x').value
        self.map_len_z = g('map_len_z').value
        self.freeze_hits = g('freeze_hits').value
        self.nbhd = g('neighbourhood').value
        self.sim_speed = g('sim_speed').value
        self.corridor_width = g('corridor_width').value
        self.warn_distance = g('warn_distance').value
        self.frame_period = g('frame_period').value

        self.enable_planning = g('enable_planning').value
        self.plan_cell = g('plan_cell_size').value
        self.plan_horizon = g('plan_horizon').value
        self.plan_half_width = g('plan_half_width').value
        self.replan_period = g('replan_period').value
        self.lookahead = g('lookahead').value
        self.smooth_iters = g('smooth_iters').value
        self.unknown_cost = g('unknown_cost').value
        self.margin_cost = g('margin_cost').value
        self.core_cost = g('core_cost').value
        self.lateral_cost = g('lateral_cost').value
        self.max_turn_rate = math.radians(g('max_turn_rate_deg').value)
        self.max_heading = math.radians(g('max_heading_deg').value)

        # --- Ray-casting geometry ---
        self.half_fov = math.radians(g('horizontal_fov_deg').value) / 2.0
        req_res = math.radians(g('angular_res_deg').value)
        self.n_bins = max(1, int(round(2.0 * self.half_fov / req_res)))
        self.angular_res = 2.0 * self.half_fov / self.n_bins
        self.bin_centers = (-self.half_fov
                            + (np.arange(self.n_bins) + 0.5) * self.angular_res)
        self.ray_step = self.cell / 2.0
        self.free_margin = 1.5 * self.cell

        # --- Map storage ---
        self.nx = int(2.0 * self.map_half_x / self.cell)
        self.nz = int(self.map_len_z / self.cell)
        self.state = np.zeros((self.nz, self.nx), dtype=np.uint8)
        self.hits = np.zeros((self.nz, self.nx), dtype=np.uint8)
        self.occupied_count = 0

        # --- Rover state ---
        self.rover_x = 0.0
        self.rover_z = 0.0
        self.heading = 0.0            # motion direction, rad from +z
        self.live_xz = None
        self.trail = [(0.0, 0.0)]

        # --- Planner state ---
        self.path = []
        self.plan_age = 1e9           # force a plan on the first tick
        self.plan_status = 'idle'
        self.blocked_ahead = False
        self.path_crosses = 0

        # --- Viewport ---
        self.view_x = 0.0
        self.view_z = 0.0
        self.follow = True
        self.dragging = False
        self.drag_px = (0, 0)
        self.drag_view = (0.0, 0.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points_coordinates',
            self.cloud_callback,
            qos,
        )
        self.timer = self.create_timer(self.frame_period, self.tick)

        self.window_name = 'Rover Map'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.width, self.height)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        self.get_logger().info(
            f'MapperNode started. grid {self.nz}x{self.nx} @ {self.cell} m. '
            f'planning={"ON" if self.enable_planning else "OFF"} (press p).'
        )

    # ==================================================================
    # Perception + mapping
    # ==================================================================
    def cloud_callback(self, msg: PointCloud2):
        raw = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )
        if raw.shape[0] == 0:
            self.live_xz = None
            return

        x, y, z = raw['x'], raw['y'], raw['z']
        h = self.floor_y - y
        band = ((h > self.h_min) & (h < self.h_max)
                & (z > self.z_min) & (z < self.z_max))
        x, z = x[band], z[band]
        self.live_xz = (x, z)
        self.update_map(x, z)

    def update_map(self, x, z):
        if x.size:
            bearing = np.arctan2(x, z)
            rng = np.sqrt(x * x + z * z)
            in_fov = np.abs(bearing) <= self.half_fov
            bearing, rng = bearing[in_fov], rng[in_fov]
        else:
            bearing = np.empty(0)
            rng = np.empty(0)

        sentinel = self.z_max * 10.0
        min_range = np.full(self.n_bins, sentinel, dtype=np.float64)
        if bearing.size:
            idx = ((bearing + self.half_fov) / self.angular_res).astype(np.int64)
            idx = np.clip(idx, 0, self.n_bins - 1)
            np.minimum.at(min_range, idx, rng)

        has_obstacle = min_range < self.z_max
        march_limit = np.minimum(min_range, self.z_max)

        self.mark_obstacles(has_obstacle, min_range)
        self.mark_free(march_limit)

    def body_to_grid(self, bx, bz):
        wx = bx + self.rover_x
        wz = bz + self.rover_z
        gx = ((wx + self.map_half_x) / self.cell).astype(np.int32)
        gz = (wz / self.cell).astype(np.int32)
        ok = (gx >= 0) & (gx < self.nx) & (gz >= 0) & (gz < self.nz)
        return gz[ok], gx[ok]

    def mark_obstacles(self, has_obstacle, min_range):
        if not has_obstacle.any():
            return
        bins = np.nonzero(has_obstacle)[0]
        r = min_range[bins]
        b = self.bin_centers[bins]
        gz, gx = self.body_to_grid(r * np.sin(b), r * np.cos(b))
        if gx.size == 0:
            return

        fresh = self.state[gz, gx] != OCC
        gz, gx = gz[fresh], gx[fresh]
        if gx.size == 0:
            return

        current = self.hits[gz, gx].astype(np.int16)
        self.hits[gz, gx] = np.minimum(current + 1, 255).astype(np.uint8)
        self.confirm_region(gz, gx)

    def confirm_region(self, ugz, ugx):
        z0, z1 = int(ugz.min()), int(ugz.max()) + 1
        x0, x1 = int(ugx.min()), int(ugx.max()) + 1

        k = self.nbhd
        hits = self.hits
        state = self.state
        thresh = self.freeze_hits
        newly = 0

        for iz in range(z0, z1):
            for ix in range(x0, x1):
                if state[iz, ix] == OCC or hits[iz, ix] < thresh:
                    continue
                zlo, zhi = max(0, iz - k), min(self.nz, iz + k + 1)
                xlo, xhi = max(0, ix - k), min(self.nx, ix + k + 1)
                support = int((hits[zlo:zhi, xlo:xhi] > 0).sum()) - 1
                if support >= 1:
                    state[iz, ix] = OCC
                    newly += 1

        self.occupied_count += newly

    def mark_free(self, march_limit):
        n_steps = max(1, int(self.z_max / self.ray_step))
        steps = self.z_min + np.arange(n_steps) * self.ray_step
        limit = march_limit - self.free_margin

        mask = steps[None, :] < limit[:, None]
        if not mask.any():
            return

        bg = self.bin_centers[:, None]
        rg = steps[None, :]
        gz, gx = self.body_to_grid((rg * np.sin(bg))[mask],
                                   (rg * np.cos(bg))[mask])
        if gx.size == 0:
            return

        flat = np.unique(gz.astype(np.int64) * self.nx + gx)
        fgz, fgx = np.divmod(flat, self.nx)
        writable = self.state[fgz, fgx] != OCC
        self.state[fgz[writable], fgx[writable]] = FREE

    # ==================================================================
    # Planning
    # ==================================================================
    def corridor_blocked(self):
        """HUD indicator only. Planning runs continuously regardless -
        gating the planner on this flag is what used to cause the path to
        be discarded and re-created, giving a zigzag instead of a curve."""
        gz0 = int(self.rover_z / self.cell)
        gz1 = min(self.nz, gz0 + int(self.plan_horizon / self.cell))
        half = self.corridor_width / 2.0
        gx0 = max(0, int((self.rover_x - half + self.map_half_x) / self.cell))
        gx1 = min(self.nx, int((self.rover_x + half + self.map_half_x)
                               / self.cell) + 1)
        if gz1 <= gz0 or gx1 <= gx0:
            return False
        return bool((self.state[gz0:gz1, gx0:gx1] == OCC).any())

    def build_plan_grid(self):
        """
        Downsample a local window to the coarse planning resolution and
        split obstacles into `core` (actually occupied) and `margin`
        (within the rover's half-width of something occupied).

        Keeping these separate is what lets the planner shave the safety
        margin when it has to, before it ever considers cutting through a
        real obstacle - a graceful degradation that a single hard
        impassable mask cannot express.
        """
        gz0 = int(self.rover_z / self.cell)
        gz1 = min(self.nz, gz0 + int(self.plan_horizon / self.cell))
        gx_c = int((self.rover_x + self.map_half_x) / self.cell)
        gx_r = int(self.plan_half_width / self.cell)
        gx0 = max(0, gx_c - gx_r)
        gx1 = min(self.nx, gx_c + gx_r)
        if gz1 - gz0 < 3 or gx1 - gx0 < 3:
            return None

        sub = self.state[gz0:gz1, gx0:gx1]

        ratio = self.plan_cell / self.cell
        rows = max(3, int((gz1 - gz0) / ratio))
        cols = max(3, int((gx1 - gx0) / ratio))

        occ_f = (sub == OCC).astype(np.float32)
        unk_f = (sub == UNKNOWN).astype(np.float32)

        # Any black fine cell inside a coarse cell marks it as core.
        core = cv2.resize(occ_f, (cols, rows),
                          interpolation=cv2.INTER_AREA) > 1e-6
        unk = cv2.resize(unk_f, (cols, rows),
                         interpolation=cv2.INTER_AREA) > 0.5

        r_cells = int(math.ceil((self.corridor_width / 2.0) / self.plan_cell))
        if r_cells > 0:
            k = 2 * r_cells + 1
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
            infl = cv2.dilate(core.astype(np.uint8), kernel).astype(bool)
        else:
            infl = core
        margin = infl & ~core

        return {'core': core, 'margin': margin, 'unk': unk,
                'rows': rows, 'cols': cols,
                'wz0': gz0 * self.cell,
                'wx0': gx0 * self.cell - self.map_half_x}

    def plan_path(self):
        """
        A* to the far edge of the window. Every cell is traversable, so a
        complete path always exists and this never returns empty - that
        guarantee is what stops the rover stalling in front of obstacles.
        """
        info = self.build_plan_grid()
        if info is None:
            self.plan_status = 'no window'
            return []

        core, margin, unk = info['core'], info['margin'], info['unk']
        rows, cols = info['rows'], info['cols']
        wz0, wx0 = info['wz0'], info['wx0']

        start_c = int(np.clip(int((self.rover_x - wx0) / self.plan_cell),
                              0, cols - 1))
        start = (0, start_c)

        step_c = self.plan_cell
        diag_c = math.sqrt(2.0) * self.plan_cell

        g_score = {start: 0.0}
        came = {}
        open_heap = [(0.0, 0, start_c)]
        closed = set()
        goal = None

        while open_heap:
            _, r, c = heapq.heappop(open_heap)
            if (r, c) in closed:
                continue
            closed.add((r, c))

            if r >= rows - 1:
                goal = (r, c)
                break

            base = g_score[(r, c)]
            for dr, dc in NEIGHBOURS:
                nr, nc = r + dr, c + dc
                if not (0 <= nr < rows and 0 <= nc < cols):
                    continue
                if (nr, nc) in closed:
                    continue

                cost = diag_c if (dr and dc) else step_c
                if core[nr, nc]:
                    cost += self.core_cost * self.plan_cell
                elif margin[nr, nc]:
                    cost += self.margin_cost * self.plan_cell
                if unk[nr, nc]:
                    cost += self.unknown_cost * self.plan_cell
                if dc:
                    cost += self.lateral_cost * self.plan_cell

                tentative = base + cost
                if tentative < g_score.get((nr, nc), float('inf')):
                    g_score[(nr, nc)] = tentative
                    came[(nr, nc)] = (r, c)
                    h = (rows - 1 - nr) * self.plan_cell
                    heapq.heappush(open_heap, (tentative + h, nr, nc))

        if goal is None:
            self.plan_status = 'search failed'
            return []

        cells = [goal]
        while cells[-1] != start:
            cells.append(came[cells[-1]])
        cells.reverse()

        crossed = sum(1 for r, c in cells if core[r, c])
        self.path_crosses = crossed
        if crossed:
            self.plan_status = f'squeezing ({crossed} cells)'
        elif any(margin[r, c] for r, c in cells):
            self.plan_status = 'tight'
        else:
            self.plan_status = 'clear'

        pts = [(wx0 + (c + 0.5) * self.plan_cell,
                wz0 + (r + 0.5) * self.plan_cell) for r, c in cells]
        return self.smooth(pts)

    def smooth(self, pts):
        """
        Chaikin corner cutting. The raw A* result is an 8-connected
        staircase; cutting corners turns it into a curve the heading
        controller can actually track. Cuts stay inside the inflation
        margin, so smoothing does not eat the clearance.
        """
        for _ in range(self.smooth_iters):
            if len(pts) < 3:
                break
            out = [pts[0]]
            for i in range(len(pts) - 1):
                (px, pz), (qx, qz) = pts[i], pts[i + 1]
                out.append((0.75 * px + 0.25 * qx, 0.75 * pz + 0.25 * qz))
                out.append((0.25 * px + 0.75 * qx, 0.25 * pz + 0.75 * qz))
            out.append(pts[-1])
            pts = out
        return pts

    def lookahead_target(self):
        """First path point at least `lookahead` away and genuinely ahead.
        The 'ahead' test is what the old version lacked: without it the
        target could collapse onto a waypoint beside the rover and motion
        would grind to nothing."""
        best = None
        for wx, wz in self.path:
            if wz <= self.rover_z + 1e-3:
                continue
            if math.hypot(wx - self.rover_x, wz - self.rover_z) >= self.lookahead:
                return (wx, wz)
            best = (wx, wz)
        return best

    @staticmethod
    def wrap(a):
        return (a + math.pi) % (2.0 * math.pi) - math.pi

    def steer(self, dt):
        """
        Rate-limited heading control. Speed along the heading is constant,
        so forward progress never collapses; the heading clamp keeps the
        forward component at cos(max_heading) of full speed at worst.
        """
        target = self.lookahead_target()
        if target is None:
            desired = 0.0
        else:
            desired = math.atan2(target[0] - self.rover_x,
                                 target[1] - self.rover_z)

        desired = float(np.clip(desired, -self.max_heading, self.max_heading))
        err = self.wrap(desired - self.heading)
        max_step = self.max_turn_rate * dt
        self.heading = self.wrap(
            self.heading + float(np.clip(err, -max_step, max_step)))

        step = self.sim_speed * dt
        self.rover_x += step * math.sin(self.heading)
        self.rover_z += step * math.cos(self.heading)

    # ==================================================================
    # Tick
    # ==================================================================
    def nearest_ahead(self):
        if self.live_xz is None:
            return float('inf')
        x, z = self.live_xz
        zc = z[np.abs(x) <= self.corridor_width / 2.0]
        if zc.shape[0] < 10:
            return float('inf')
        return float(np.percentile(zc, 1.0))

    def tick(self):
        dt = self.frame_period

        if self.enable_planning:
            self.blocked_ahead = self.corridor_blocked()
            self.plan_age += dt
            # Replan on a timer, unconditionally. The previous version
            # only planned while blocked and threw the path away once
            # clear, which produced a zigzag instead of a smooth curve.
            if self.plan_age >= self.replan_period:
                self.path = self.plan_path()
                self.plan_age = 0.0
            self.steer(dt)
        else:
            self.heading = 0.0
            self.rover_z += self.sim_speed * dt

        # Lateral drift is bounded by the map, not by the motion model.
        self.rover_x = float(np.clip(self.rover_x,
                                     -self.map_half_x + 0.5,
                                     self.map_half_x - 0.5))

        if self.rover_z + self.z_max >= self.map_len_z:
            self.get_logger().warn('Reached end of map extent - wrapping.')
            self.rover_x = self.rover_z = self.heading = 0.0
            self.path = []
            self.trail = [(0.0, 0.0)]

        lx, lz = self.trail[-1]
        if (self.rover_x - lx) ** 2 + (self.rover_z - lz) ** 2 > 0.0025:
            self.trail.append((self.rover_x, self.rover_z))
            if len(self.trail) > 6000:
                self.trail.pop(0)

        clr = self.nearest_ahead()
        if clr <= self.warn_distance:
            self.get_logger().warn(
                f'Obstacle {clr:.2f} m ahead at odom {self.rover_z:.2f} m')

        if self.follow:
            self.view_x = self.rover_x
            self.view_z = self.rover_z + (self.height * 0.25) / self.scale

        self.draw(clr)

    # ==================================================================
    # Viewport
    # ==================================================================
    def on_mouse(self, event, mx, my, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.dragging = True
            self.follow = False
            self.drag_px = (mx, my)
            self.drag_view = (self.view_x, self.view_z)
        elif event == cv2.EVENT_MOUSEMOVE and self.dragging:
            dx = mx - self.drag_px[0]
            dy = my - self.drag_px[1]
            self.view_x = self.drag_view[0] - dx / self.scale
            self.view_z = self.drag_view[1] + dy / self.scale
        elif event == cv2.EVENT_LBUTTONUP:
            self.dragging = False
        elif event == cv2.EVENT_MOUSEWHEEL:
            try:
                delta = cv2.getMouseWheelDelta(flags)
            except AttributeError:
                delta = 1 if flags > 0 else -1
            factor = 1.15 if delta > 0 else 1.0 / 1.15
            self.scale = float(np.clip(self.scale * factor, 20.0, 1200.0))

    def wx_to_px(self, wx):
        return int(round((wx - self.view_x) * self.scale + self.width / 2.0))

    def wz_to_py(self, wz):
        return int(round(self.height / 2.0 - (wz - self.view_z) * self.scale))

    # ==================================================================
    # Rendering
    # ==================================================================
    def draw(self, clr):
        frame = np.full((self.height, self.width, 3),
                        PALETTE[UNKNOWN], dtype=np.uint8)

        self.blit_state(frame)
        self.draw_grid_lines(frame)
        self.draw_live(frame)
        self.draw_trail(frame)
        if self.enable_planning:
            self.draw_path(frame)
        self.draw_rover(frame)
        self.draw_hud(frame, clr)

        cv2.imshow(self.window_name, frame)
        self.handle_keys()

    def blit_state(self, frame):
        wz0 = self.view_z - (self.height / 2.0) / self.scale
        wz1 = self.view_z + (self.height / 2.0) / self.scale
        wx0 = self.view_x - (self.width / 2.0) / self.scale
        wx1 = self.view_x + (self.width / 2.0) / self.scale

        gz0 = max(0, int(math.floor(wz0 / self.cell)))
        gz1 = min(self.nz, int(math.ceil(wz1 / self.cell)))
        gx0 = max(0, int(math.floor((wx0 + self.map_half_x) / self.cell)))
        gx1 = min(self.nx, int(math.ceil((wx1 + self.map_half_x) / self.cell)))
        if gz1 <= gz0 or gx1 <= gx0:
            return

        tile_src = PALETTE[self.state[gz0:gz1, gx0:gx1]][::-1, :, :]

        left = self.wx_to_px(gx0 * self.cell - self.map_half_x)
        right = self.wx_to_px(gx1 * self.cell - self.map_half_x)
        bottom = self.wz_to_py(gz0 * self.cell)
        top = self.wz_to_py(gz1 * self.cell)

        tw, th = right - left, bottom - top
        if tw <= 0 or th <= 0:
            return

        tile = cv2.resize(tile_src, (tw, th), interpolation=cv2.INTER_NEAREST)
        dx0, dx1 = max(0, left), min(self.width, right)
        dy0, dy1 = max(0, top), min(self.height, bottom)
        if dx1 <= dx0 or dy1 <= dy0:
            return
        frame[dy0:dy1, dx0:dx1] = tile[dy0 - top:dy1 - top,
                                       dx0 - left:dx1 - left]

    def draw_grid_lines(self, frame):
        for step in (0.25, 0.5, 1.0, 2.0, 5.0):
            if step * self.scale >= 60:
                break
        wz0 = self.view_z - (self.height / 2.0) / self.scale
        wz1 = self.view_z + (self.height / 2.0) / self.scale
        n = int(math.floor(wz0 / step))
        while n * step <= wz1:
            wz = n * step
            py = self.wz_to_py(wz)
            if 0 <= py < self.height:
                cv2.line(frame, (0, py), (self.width, py), (150, 140, 140), 1)
                cv2.putText(frame, f'{wz:.2f} m', (6, py - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (90, 80, 80), 1)
            n += 1

    def draw_live(self, frame):
        if self.live_xz is None:
            return
        x, z = self.live_xz
        wx = x + self.rover_x
        wz = z + self.rover_z
        px = ((wx - self.view_x) * self.scale
              + self.width / 2.0).astype(np.int32)
        py = (self.height / 2.0
              - (wz - self.view_z) * self.scale).astype(np.int32)
        ok = (px >= 0) & (px < self.width) & (py >= 0) & (py < self.height)
        frame[py[ok], px[ok]] = (0, 0, 220)

    def draw_trail(self, frame):
        if len(self.trail) < 2:
            return
        pts = np.array([[self.wx_to_px(tx), self.wz_to_py(tz)]
                        for tx, tz in self.trail], dtype=np.int32)
        cv2.polylines(frame, [pts], False, (200, 170, 170), 2)

    def draw_path(self, frame):
        if len(self.path) < 2:
            return
        pts = np.array([[self.wx_to_px(wx), self.wz_to_py(wz)]
                        for wx, wz in self.path], dtype=np.int32)

        half_px = max(1, int((self.corridor_width / 2.0) * self.scale))
        overlay = frame.copy()
        band = (80, 160, 240) if self.path_crosses else (120, 220, 120)
        cv2.polylines(overlay, [pts], False, band, half_px * 2)
        cv2.addWeighted(overlay, 0.25, frame, 0.75, 0, frame)

        line = (0, 90, 200) if self.path_crosses else (0, 150, 0)
        cv2.polylines(frame, [pts], False, line, 2, cv2.LINE_AA)

        tgt = self.lookahead_target()
        if tgt is not None:
            cv2.circle(frame, (self.wx_to_px(tgt[0]), self.wz_to_py(tgt[1])),
                       5, (0, 120, 255), -1)

    def draw_rover(self, frame):
        px = self.wx_to_px(self.rover_x)
        py = self.wz_to_py(self.rover_z)
        if not (0 <= px < self.width and 0 <= py < self.height):
            return

        # FOV wedge stays aligned to +z: the camera does not rotate.
        for sign in (-1.0, 1.0):
            ex = self.rover_x + self.z_max * math.sin(sign * self.half_fov)
            ez = self.rover_z + self.z_max * math.cos(sign * self.half_fov)
            cv2.line(frame, (px, py),
                     (self.wx_to_px(ex), self.wz_to_py(ez)),
                     (0, 170, 170), 1, cv2.LINE_AA)

        half = self.corridor_width / 2.0
        col = (0, 0, 220) if self.blocked_ahead else (190, 190, 190)
        for sign in (-1.0, 1.0):
            cv2.line(frame,
                     (self.wx_to_px(self.rover_x + sign * half), py),
                     (self.wx_to_px(self.rover_x + sign * half),
                      self.wz_to_py(self.rover_z + self.plan_horizon)), col, 1)

        # Heading stub shows the steered direction of travel.
        hx = self.rover_x + 0.25 * math.sin(self.heading)
        hz = self.rover_z + 0.25 * math.cos(self.heading)
        cv2.line(frame, (px, py), (self.wx_to_px(hx), self.wz_to_py(hz)),
                 (0, 0, 200), 2)
        cv2.circle(frame, (px, py), 7, (0, 0, 200), -1)

    def draw_hud(self, frame, clr):
        clr_txt = 'inf' if math.isinf(clr) else f'{clr:.2f} m'
        mode = 'FOLLOW' if self.follow else 'FREE (f)'
        free_n = int((self.state == FREE).sum())
        cv2.rectangle(frame, (0, 0), (self.width, 78), (255, 255, 255), -1)
        cv2.putText(frame,
                    f'pos ({self.rover_x:+.2f}, {self.rover_z:.2f}) m   '
                    f'hdg {math.degrees(self.heading):+5.1f} deg   '
                    f'ahead {clr_txt:>8}   occ {self.occupied_count}   '
                    f'free {free_n}',
                    (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.46, (40, 40, 40), 1)
        cv2.putText(frame, f'{mode}   zoom {self.scale:.0f} px/m',
                    (10, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (120, 120, 120), 1)
        if self.enable_planning:
            ptxt = (f'PLANNING ON   {self.plan_status}   '
                    f'waypoints {len(self.path)}   '
                    f'{"corridor blocked" if self.blocked_ahead else "corridor clear"}')
            pcol = (0, 90, 200) if self.path_crosses else (0, 110, 0)
        else:
            ptxt = 'PLANNING OFF (press p)'
            pcol = (120, 120, 120)
        cv2.putText(frame, ptxt, (10, 66),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, pcol, 1)

    def handle_keys(self):
        k = cv2.waitKey(1) & 0xFF
        if k == 255:
            return
        if k == ord('f'):
            self.follow = True
        elif k == ord('c'):
            self.state[:] = UNKNOWN
            self.hits[:] = 0
            self.occupied_count = 0
            self.path = []
            self.get_logger().info('Map cleared.')
        elif k == ord('p'):
            self.enable_planning = not self.enable_planning
            self.path = []
            self.heading = 0.0
            self.plan_status = 'idle'
            self.get_logger().info(
                f'Planning {"ON" if self.enable_planning else "OFF"}.')
        elif k in (ord('q'), 27):
            raise KeyboardInterrupt

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
