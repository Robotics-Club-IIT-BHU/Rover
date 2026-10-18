#!/usr/bin/env python3
"""
generate_terrain.py

Procedural terrain generator for the IRC 2027 "Sproscape-Sim" Mars-analog arena.
Produces:
  1. An Ogre-compatible 8-bit heightmap PNG (257x257 pixels, 120m x 120m arena).
  2. A Gazebo Fortress (gz-sim 6) SDF world file (sproscape.sdf) referencing the heightmap
     with boundary walls, competition delivery pads, beacons, service panel, danger zones, and tools.
  3. An arena layout metadata YAML (sproscape_zones.yaml) used by briefing_tool and mission_runner.

All algorithms are implemented in pure Python (standard library struct + zlib)
for deterministic, reproducible builds without external image dependencies.
"""

import math
import os
import struct
import zlib
from typing import Dict, List, Tuple
import yaml

ARENA_SIZE_M = 120.0
HEIGHTMAP_DIM = 257  # Standard 2^n + 1 dimension for Ogre terrain
MAX_HEIGHT_M = 6.0   # Vertical elevation range from min to max


def write_grayscale_png(filename: str, width: int, height: int, data: bytes):
    """Write an 8-bit grayscale PNG using standard library zlib and struct."""
    signature = b'\x89PNG\r\n\x1a\n'

    # IHDR chunk: width, height, bit depth 8, color type 0 (grayscale), compression 0, filter 0, interlace 0
    ihdr_data = struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0)
    ihdr_crc = struct.pack('>I', zlib.crc32(b'IHDR' + ihdr_data))
    ihdr = struct.pack('>I', len(ihdr_data)) + b'IHDR' + ihdr_data + ihdr_crc

    # IDAT chunk: filter byte 0 (None) before every scanline
    raw_scanlines = bytearray()
    for row in range(height):
        raw_scanlines.append(0)  # Filter byte None
        start = row * width
        raw_scanlines.extend(data[start : start + width])

    compressed = zlib.compress(bytes(raw_scanlines), level=9)
    idat_crc = struct.pack('>I', zlib.crc32(b'IDAT' + compressed))
    idat = struct.pack('>I', len(compressed)) + b'IDAT' + compressed + idat_crc

    # IEND chunk
    iend_crc = struct.pack('>I', zlib.crc32(b'IEND'))
    iend = struct.pack('>I', 0) + b'IEND' + iend_crc

    os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)
    with open(filename, 'wb') as f:
        f.write(signature + ihdr + idat + iend)


def write_rgb_png(filename: str, width: int, height: int, data: bytes):
    """Write an 8-bit RGB color PNG using standard library zlib and struct."""
    signature = b'\x89PNG\r\n\x1a\n'

    # IHDR chunk: width, height, bit depth 8, color type 2 (Truecolor RGB), compression 0, filter 0, interlace 0
    ihdr_data = struct.pack('>IIBBBBB', width, height, 8, 2, 0, 0, 0)
    ihdr_crc = struct.pack('>I', zlib.crc32(b'IHDR' + ihdr_data))
    ihdr = struct.pack('>I', len(ihdr_data)) + b'IHDR' + ihdr_data + ihdr_crc

    raw_scanlines = bytearray()
    row_bytes = width * 3
    for row in range(height):
        raw_scanlines.append(0)  # Filter byte None
        start = row * row_bytes
        raw_scanlines.extend(data[start : start + row_bytes])

    compressed = zlib.compress(bytes(raw_scanlines), level=9)
    idat_crc = struct.pack('>I', zlib.crc32(b'IDAT' + compressed))
    idat = struct.pack('>I', len(compressed)) + b'IDAT' + compressed + idat_crc

    iend_crc = struct.pack('>I', zlib.crc32(b'IEND'))
    iend = struct.pack('>I', 0) + b'IEND' + iend_crc

    os.makedirs(os.path.dirname(os.path.abspath(filename)), exist_ok=True)
    with open(filename, 'wb') as f:
        f.write(signature + ihdr + idat + iend)


def smoothstep(edge0: float, edge1: float, x: float) -> float:
    """Quintic Hermite smootherstep (C2 continuous: S'(0)=S'(1)=S''(0)=S''(1)=0)."""
    if x <= edge0:
        return 0.0
    if x >= edge1:
        return 1.0
    t = (x - edge0) / (edge1 - edge0)
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0)


def get_surface_elevation(x: float, y: float) -> float:
    """
    Parametric surface elevation function for the IRC 2027 Sproscape terrain in meters.
    Guarantees:
      - Designated base station spawn zone ([0, 25m] x [0, 20m] and y <= 20m) is exactly 0.000m.
      - Purely C2 continuous transitions into undulating dunes and rocky terrain.
      - No sudden slopes, ridges, or abrupt step cliffs anywhere in the arena.
      - Max terrain slope is strictly gentle (< 7 degrees) across the entire arena for realistic rover mobility.
      - Smooth 2D separable boundary tapering down to 0.000m at perimeter walls.
      - Minimum elevation across entire arena is guaranteed >= 0.000m.
    """
    # Clamp bounds for evaluation
    x = max(0.0, min(120.0, x))
    y = max(0.0, min(120.0, y))

    # 1. Base station spawn zone: [0, 25m] x [0, 20m]
    dx = max(0.0, x - 25.0)
    dy = max(0.0, y - 20.0)
    dist_spawn = math.hypot(dx, dy)
    spawn_blend = smoothstep(0.0, 18.0, dist_spawn)
    if spawn_blend <= 0.0:
        return 0.0

    # 2. Smooth boundary taper (2D separable product down to 0 at walls)
    taper_x = smoothstep(0.0, 15.0, x) * (1.0 - smoothstep(105.0, 120.0, x))
    taper_y = smoothstep(0.0, 15.0, y) * (1.0 - smoothstep(105.0, 120.0, y))
    boundary_blend = taper_x * taper_y

    # 3. Base rolling terrain (gentle grade, amplitude ~0.5m)
    base_wave = (
        0.30 * math.sin(x * 0.055 + 0.5) * math.cos(y * 0.045 - 0.3)
        + 0.18 * math.sin(x * 0.085 - y * 0.065)
        + 0.12 * math.cos(x * 0.035 + y * 0.075)
    )

    # 4. Sand dune field in central-east region
    dune_env = (
        smoothstep(35.0, 48.0, x) * (1.0 - smoothstep(92.0, 105.0, x)) *
        smoothstep(28.0, 40.0, y) * (1.0 - smoothstep(88.0, 100.0, y))
    )
    dunes = dune_env * (0.28 * math.sin(x * 0.14 + y * 0.09) + 0.12 * math.cos(y * 0.22))

    # 5. Smooth impact crater at (x=45, y=70, R=16m)
    dist_crater = math.hypot(x - 45.0, y - 70.0)
    crater_r = 16.0
    crater = 0.0
    if dist_crater < crater_r:
        u = dist_crater / crater_r
        env = (1.0 - u * u) ** 2
        crater = (-0.35 * math.cos(u * math.pi * 0.5) + 0.20 * math.sin(u * math.pi)) * env

    # 6. Smooth rocky knoll at (x=85, y=45, R=18m)
    dist_knoll = math.hypot(x - 85.0, y - 45.0)
    knoll_r = 18.0
    knoll = 0.0
    if dist_knoll < knoll_r:
        u = dist_knoll / knoll_r
        knoll = 0.35 * ((1.0 - u * u) ** 2)

    raw_z = 0.55 + base_wave + dunes + crater + knoll
    z = raw_z * spawn_blend * boundary_blend
    return max(0.0, z)


def get_surface_normal(x: float, y: float, eps: float = 0.02) -> Tuple[float, float, float, float, float]:
    """
    Compute analytical / central-difference surface normal unit vector (nx, ny, nz)
    and surface gradients (gx, gy) = (dz/dx, dz/dy).
    """
    gx = (get_surface_elevation(min(120.0, x + eps), y) - get_surface_elevation(max(0.0, x - eps), y)) / (2.0 * eps)
    gy = (get_surface_elevation(x, min(120.0, y + eps)) - get_surface_elevation(x, max(0.0, y - eps))) / (2.0 * eps)
    norm = math.sqrt(gx * gx + gy * gy + 1.0)
    return -gx / norm, -gy / norm, 1.0 / norm, gx, gy


def compute_object_pose(x: float, y: float, half_x: float, half_y: float, height: float, intrinsic_yaw: float = 0.0) -> Tuple[float, float, float, float]:
    """
    Compute slope-aligned pose (z_center, roll, pitch, yaw) for an object at (x, y).
    Aligns the object's vertical axis with the terrain surface normal so it lies flush
    along the slope without clipping or intersecting the ground.
    """
    z0 = get_surface_elevation(x, y)
    nx, ny, nz, gx, gy = get_surface_normal(x, y)

    pitch = math.atan(-gx)
    roll = math.atan(gy)

    # Lift center so bottom corners do not clip into terrain
    delta_z = half_x * abs(math.sin(pitch)) + half_y * abs(math.sin(roll)) + 0.005  # 5mm margin
    z_center = z0 + (height / 2.0) + delta_z
    return z_center, roll, pitch, intrinsic_yaw


def generate_heightmap(dim: int = 257, size_m: float = 120.0) -> Tuple[bytes, Dict]:
    """
    Generate parametric height values representing the IRC 2027 Sproscape terrain:
      - Scanline row 0 corresponds to North (Y = 120.0m)
      - Scanline row dim-1 corresponds to South (Y = 0.0m)
      - Scanline col 0 corresponds to West (X = 0.0m)
      - Scanline col dim-1 corresponds to East (X = 120.0m)
      - Spawn area (X in [0, 25m], Y in [0, 20m]) is 100% flat at Z = 0.00m.
    """
    pixels = bytearray(dim * dim)
    grid_res = size_m / (dim - 1)

    elevations = [[0.0 for _ in range(dim)] for _ in range(dim)]

    for row in range(dim):
        # Row 0 is North (Y = size_m), Row dim - 1 is South (Y = 0.0)
        y = (dim - 1 - row) * grid_res
        for col in range(dim):
            x = col * grid_res
            elevations[row][col] = get_surface_elevation(x, y)

    # Normalize elevations to 0..255 for 8-bit heightmap
    min_z = min(min(r) for r in elevations)  # Exactly 0.0
    max_z = max(max(r) for r in elevations)
    z_range = max(max_z - min_z, 1e-3)

    for row in range(dim):
        for col in range(dim):
            norm = (elevations[row][col] - min_z) / z_range
            val = int(min(255, max(0, round(norm * 255))))
            pixels[row * dim + col] = val

    metadata = {
        'min_elevation_m': round(min_z, 3),
        'max_elevation_m': round(max_z, 3),
        'elevation_range_m': round(z_range, 3),
        'dimension': dim,
        'size_m': size_m,
    }

    return bytes(pixels), metadata


def _generate_red_zone_pylons_xml(model_name: str, cx: float, cy: float, radius: float, num_pylons: int = 8) -> str:
    """Generate SDF XML for perimeter warning pylons / red cones around a hazardous red zone."""
    xml_lines = [
        f'    <model name="{model_name}">',
        '      <static>true</static>',
        '      <link name="link">',
    ]
    for i in range(num_pylons):
        angle = (2.0 * math.pi * i) / num_pylons
        px = cx + radius * math.cos(angle)
        py = cy + radius * math.sin(angle)
        pz, roll, pitch, _ = compute_object_pose(px, py, 0.18, 0.18, 0.04)
        xml_lines.append(f'''        <!-- Pylon {i} -->
        <collision name="pylon_{i}_col">
          <pose>{px:.3f} {py:.3f} {pz + 0.35:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>0.7</length></cylinder></geometry>
        </collision>
        <visual name="pylon_{i}_base">
          <pose>{px:.3f} {py:.3f} {pz + 0.02:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><cylinder><radius>0.18</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_cone">
          <pose>{px:.3f} {py:.3f} {pz + 0.35:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><cylinder><radius>0.1</radius><length>0.7</length></cylinder></geometry>
          <material><ambient>0.8 0.1 0.05 1</ambient><diffuse>0.95 0.15 0.05 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_stripe">
          <pose>{px:.3f} {py:.3f} {pz + 0.45:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><cylinder><radius>0.105</radius><length>0.15</length></cylinder></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_top">
          <pose>{px:.3f} {py:.3f} {pz + 0.72:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material><ambient>0.9 0.1 0.0 1</ambient><diffuse>1.0 0.1 0.0 1</diffuse><emissive>1.0 0.1 0.0 1</emissive></material>
        </visual>''')
    xml_lines.append('      </link>')
    xml_lines.append('    </model>')
    return '\n'.join(xml_lines)

def generate_world_sdf(heightmap_rel_path: str, z_range_m: float, min_elevation_m: float = 0.0) -> str:
    """Generate the complete Gazebo Fortress SDF world for Sproscape with full arena assets."""
    # Checkpoints and delivery pads
    p_alpha = compute_object_pose(30.0, 70.0, 1.25, 1.25, 0.04)
    p_bravo = compute_object_pose(80.0, 80.0, 1.25, 1.25, 0.04)
    p_cache = compute_object_pose(60.0, 30.0, 1.20, 1.20, 0.05)
    p_idmo = compute_object_pose(15.0, 50.0, 0.25, 0.60, 0.05)

    # Tool surface poses
    p_wrench = compute_object_pose(25.0, 35.0, 0.21, 0.04, 0.05)
    p_hammer = compute_object_pose(28.0, 38.0, 0.19, 0.07, 0.07, intrinsic_yaw=0.60)
    p_screwdriver = compute_object_pose(40.0, 60.0, 0.14, 0.09, 0.05)
    p_toolbox = compute_object_pose(70.0, 45.0, 0.23, 0.15, 0.22)
    p_rock = compute_object_pose(55.0, 85.0, 0.13, 0.11, 0.18, intrinsic_yaw=0.40)

    # Obstacle avoidance test objects
    p_boulder_a = compute_object_pose(22.0, 28.0, 1.20, 1.00, 1.80)
    p_cargo = compute_object_pose(45.0, 42.0, 1.80, 0.90, 1.70, intrinsic_yaw=0.45)
    p_crag = compute_object_pose(50.0, 62.0, 1.10, 1.10, 2.50)
    p_pillar_1 = compute_object_pose(72.0, 65.0, 0.75, 0.75, 2.20)
    p_pillar_2 = compute_object_pose(76.0, 72.0, 0.90, 0.90, 2.00, intrinsic_yaw=0.50)
    p_debris = compute_object_pose(28.0, 52.0, 2.10, 0.90, 0.50, intrinsic_yaw=0.80)

    red_zone_1_xml = _generate_red_zone_pylons_xml('red_zone_1_pylons', 50.0, 75.0, 6.0, num_pylons=8)
    red_zone_2_xml = _generate_red_zone_pylons_xml('red_zone_2_pylons', 20.0, 90.0, 8.0, num_pylons=8)

    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="sproscape">
    <!-- Physics configuration -->
    <physics name="1ms" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Core Gazebo system plugins -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-navsat-system" name="ignition::gazebo::systems::NavSat"/>

    <!-- Default GUI user camera view framing spawn area and arena -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>60.0 -40.0 80.0 0 0.6 1.5708</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

    <!-- WGS-84 Competition Datum (DSU, Bengaluru) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>12.8614</latitude_deg>
      <longitude_deg>77.4380</longitude_deg>
      <elevation>860.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <!-- Sun directional light -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.9 0.85 0.8 1</diffuse>
      <specular>0.3 0.3 0.3 1</specular>
      <direction>-0.5 0.2 -0.9</direction>
    </light>

    <!-- Procedural Sproscape-Sim Mars analog heightmap -->
    <model name="sproscape_terrain">
      <static>true</static>
      <pose>60.0 60.0 0 0 0 0</pose>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{heightmap_rel_path}</uri>
              <size>{ARENA_SIZE_M} {ARENA_SIZE_M} {z_range_m:.3f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.8</mu>
                <mu2>0.6</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <use_terrain_paging>false</use_terrain_paging>
              <texture>
                <diffuse>materials/textures/martian_soil.png</diffuse>
                <normal>materials/textures/flat_normal.png</normal>
                <size>10</size>
              </texture>
              <uri>{heightmap_rel_path}</uri>
              <size>{ARENA_SIZE_M} {ARENA_SIZE_M} {z_range_m:.3f}</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- 1. Arena Boundary Containment Barriers (120m x 120m, height 2.5m, thickness 0.8m) -->
    <model name="arena_boundary_walls">
      <static>true</static>
      <link name="walls_link">
        <!-- South Wall (y = 0.0) -->
        <collision name="wall_south_col">
          <pose>60.0 0.0 1.25 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_south_vis">
          <pose>60.0 0.0 1.25 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- North Wall (y = 120.0) -->
        <collision name="wall_north_col">
          <pose>60.0 120.0 1.25 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_north_vis">
          <pose>60.0 120.0 1.25 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- West Wall (x = 0.0) -->
        <collision name="wall_west_col">
          <pose>0.0 60.0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_west_vis">
          <pose>0.0 60.0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- East Wall (x = 120.0) -->
        <collision name="wall_east_col">
          <pose>120.0 60.0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_east_vis">
          <pose>120.0 60.0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 2. Delivery Marker Alpha: Green target pad with concentric rings and 1.5m beacon -->
    <model name="marker_drop_alpha">
      <static>true</static>
      <pose>30.0 70.0 {p_alpha[0]:.3f} {p_alpha[1]:.4f} {p_alpha[2]:.4f} {p_alpha[3]:.4f}</pose>
      <link name="link">
        <!-- Landing Pad (outer ring 2.5m diameter) -->
        <collision name="pad_col">
          <pose>0 0 0.02 0 0 0</pose>
          <geometry><cylinder><radius>1.25</radius><length>0.04</length></cylinder></geometry>
        </collision>
        <visual name="outer_ring">
          <pose>0 0 0.02 0 0 0</pose>
          <geometry><cylinder><radius>1.25</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.05 0.5 0.1 1</ambient><diffuse>0.1 0.8 0.2 1</diffuse></material>
        </visual>
        <visual name="mid_ring">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><cylinder><radius>0.85</radius><length>0.042</length></cylinder></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="inner_bullseye">
          <pose>0 0 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.45</radius><length>0.045</length></cylinder></geometry>
          <material><ambient>0.05 0.6 0.1 1</ambient><diffuse>0.05 0.9 0.15 1</diffuse></material>
        </visual>
        <!-- 1.5m Beacon Pole and Emerald Beacon Light -->
        <collision name="beacon_pole_col">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><cylinder><radius>0.035</radius><length>1.5</length></cylinder></geometry>
        </collision>
        <visual name="beacon_pole">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><cylinder><radius>0.035</radius><length>1.5</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
        <visual name="beacon_light">
          <pose>0 0 1.5 0 0 0</pose>
          <geometry><sphere><radius>0.12</radius></sphere></geometry>
          <material>
            <ambient>0.1 0.9 0.2 1</ambient>
            <diffuse>0.2 1.0 0.3 1</diffuse>
            <emissive>0.1 0.9 0.2 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- 3. Delivery Marker Bravo: Blue target pad with concentric rings and 1.5m beacon -->
    <model name="marker_drop_bravo">
      <static>true</static>
      <pose>80.0 80.0 {p_bravo[0]:.3f} {p_bravo[1]:.4f} {p_bravo[2]:.4f} {p_bravo[3]:.4f}</pose>
      <link name="link">
        <!-- Landing Pad (outer ring 2.5m diameter) -->
        <collision name="pad_col">
          <pose>0 0 0.02 0 0 0</pose>
          <geometry><cylinder><radius>1.25</radius><length>0.04</length></cylinder></geometry>
        </collision>
        <visual name="outer_ring">
          <pose>0 0 0.02 0 0 0</pose>
          <geometry><cylinder><radius>1.25</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.05 0.2 0.6 1</ambient><diffuse>0.1 0.4 0.9 1</diffuse></material>
        </visual>
        <visual name="mid_ring">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><cylinder><radius>0.85</radius><length>0.042</length></cylinder></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="inner_bullseye">
          <pose>0 0 0.03 0 0 0</pose>
          <geometry><cylinder><radius>0.45</radius><length>0.045</length></cylinder></geometry>
          <material><ambient>0.05 0.25 0.7 1</ambient><diffuse>0.15 0.5 1.0 1</diffuse></material>
        </visual>
        <!-- 1.5m Beacon Pole and Cyan/Blue Beacon Light -->
        <collision name="beacon_pole_col">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><cylinder><radius>0.035</radius><length>1.5</length></cylinder></geometry>
        </collision>
        <visual name="beacon_pole">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><cylinder><radius>0.035</radius><length>1.5</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
        <visual name="beacon_light">
          <pose>0 0 1.5 0 0 0</pose>
          <geometry><sphere><radius>0.12</radius></sphere></geometry>
          <material>
            <ambient>0.1 0.4 1.0 1</ambient>
            <diffuse>0.2 0.5 1.0 1</diffuse>
            <emissive>0.1 0.4 1.0 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- 4. ABEx Science Sample Cache Platform with orange/white markings -->
    <model name="abex_cache_platform">
      <static>true</static>
      <pose>60.0 30.0 {p_cache[0]:.3f} {p_cache[1]:.4f} {p_cache[2]:.4f} {p_cache[3]:.4f}</pose>
      <link name="link">
        <!-- Platform Base (2.4m x 2.4m) -->
        <collision name="platform_col">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>2.4 2.4 0.05</size></box></geometry>
        </collision>
        <visual name="platform_outer">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>2.4 2.4 0.05</size></box></geometry>
          <material><ambient>0.6 0.25 0.05 1</ambient><diffuse>0.95 0.45 0.05 1</diffuse></material>
        </visual>
        <visual name="platform_inner_white">
          <pose>0 0 0.03 0 0 0</pose>
          <geometry><box><size>1.8 1.8 0.055</size></box></geometry>
          <material><ambient>0.7 0.7 0.7 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="platform_center_orange">
          <pose>0 0 0.035 0 0 0</pose>
          <geometry><box><size>0.9 0.9 0.06</size></box></geometry>
          <material><ambient>0.6 0.25 0.05 1</ambient><diffuse>0.9 0.4 0.0 1</diffuse></material>
        </visual>
        <!-- Antenna Mast & Amber Beacon -->
        <collision name="mast_col">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><cylinder><radius>0.03</radius><length>1.5</length></cylinder></geometry>
        </collision>
        <visual name="mast_visual">
          <pose>0 0 0.75 0 0 0</pose>
          <geometry><cylinder><radius>0.03</radius><length>1.5</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>
        <visual name="mast_beacon">
          <pose>0 0 1.5 0 0 0</pose>
          <geometry><sphere><radius>0.12</radius></sphere></geometry>
          <material>
            <ambient>0.9 0.5 0.0 1</ambient>
            <diffuse>1.0 0.55 0.0 1</diffuse>
            <emissive>1.0 0.5 0.0 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- 5. IDMO Maintenance Service Panel Stand (1.8m vertical mock service panel) -->
    <model name="idmo_maintenance_panel">
      <static>true</static>
      <pose>15.0 50.0 {p_idmo[0]:.3f} {p_idmo[1]:.4f} {p_idmo[2]:.4f} {p_idmo[3]:.4f}</pose>
      <link name="link">
        <!-- Base Plate -->
        <collision name="base_col">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>0.5 1.2 0.05</size></box></geometry>
        </collision>
        <visual name="base_vis">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>0.5 1.2 0.05</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.25 0.25 0.28 1</diffuse></material>
        </visual>
        <!-- Vertical Upright Posts (1.8m tall) -->
        <collision name="post_left_col">
          <pose>0 -0.45 0.9 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.8</length></cylinder></geometry>
        </collision>
        <visual name="post_left_vis">
          <pose>0 -0.45 0.9 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.8</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
        </visual>
        <collision name="post_right_col">
          <pose>0 0.45 0.9 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.8</length></cylinder></geometry>
        </collision>
        <visual name="post_right_vis">
          <pose>0 0.45 0.9 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.8</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
        </visual>
        <!-- Top Crossbar -->
        <visual name="top_bar_vis">
          <pose>0 0 1.78 0 0 0</pose>
          <geometry><box><size>0.08 1.0 0.06</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
        </visual>
        <!-- Service Panel Console Face (width 0.8m, height 0.6m) -->
        <collision name="panel_col">
          <pose>0.05 0 1.25 0 0 0</pose>
          <geometry><box><size>0.15 0.8 0.6</size></box></geometry>
        </collision>
        <visual name="panel_face">
          <pose>0.05 0 1.25 0 0 0</pose>
          <geometry><box><size>0.15 0.8 0.6</size></box></geometry>
          <material><ambient>0.25 0.25 0.28 1</ambient><diffuse>0.35 0.35 0.4 1</diffuse></material>
        </visual>
        <!-- Yellow Grab Handles -->
        <visual name="handle_left">
          <pose>0.14 -0.25 1.25 0 0 0</pose>
          <geometry><cylinder><radius>0.015</radius><length>0.25</length></cylinder></geometry>
          <material><ambient>0.6 0.5 0.05 1</ambient><diffuse>0.95 0.8 0.05 1</diffuse></material>
        </visual>
        <visual name="handle_right">
          <pose>0.14 0.25 1.25 0 0 0</pose>
          <geometry><cylinder><radius>0.015</radius><length>0.25</length></cylinder></geometry>
          <material><ambient>0.6 0.5 0.05 1</ambient><diffuse>0.95 0.8 0.05 1</diffuse></material>
        </visual>
        <!-- Status LEDs -->
        <visual name="led_green">
          <pose>0.13 -0.15 1.45 0 0 0</pose>
          <geometry><sphere><radius>0.025</radius></sphere></geometry>
          <material><ambient>0.1 0.8 0.1 1</ambient><diffuse>0.2 1.0 0.2 1</diffuse><emissive>0.1 0.9 0.1 1</emissive></material>
        </visual>
        <visual name="led_amber">
          <pose>0.13 0.0 1.45 0 0 0</pose>
          <geometry><sphere><radius>0.025</radius></sphere></geometry>
          <material><ambient>0.8 0.5 0.0 1</ambient><diffuse>1.0 0.6 0.0 1</diffuse><emissive>0.9 0.55 0.0 1</emissive></material>
        </visual>
        <visual name="led_red">
          <pose>0.13 0.15 1.45 0 0 0</pose>
          <geometry><sphere><radius>0.025</radius></sphere></geometry>
          <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>1.0 0.1 0.1 1</diffuse><emissive>0.9 0.1 0.1 1</emissive></material>
        </visual>
        <!-- Emergency Stop Push Button -->
        <visual name="estop_button">
          <pose>0.135 0.0 1.1 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.035</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.8 0.05 0.05 1</ambient><diffuse>0.95 0.05 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 6. Red Danger Contamination Zones (Pylons & Warning Barriers) -->
{red_zone_1_xml}

{red_zone_2_xml}

    <!-- 7. Competition Tools and Geological Specimens -->
    <model name="tool_heavy_wrench">
      <static>true</static>
      <pose>25.0 35.0 {p_wrench[0]:.3f} {p_wrench[1]:.4f} {p_wrench[2]:.4f} {p_wrench[3]:.4f}</pose>
      <link name="link">
        <inertial><mass>1.8</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        <collision name="wrench_col">
          <pose>0 0 0.025 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.025</radius><length>0.42</length></cylinder></geometry>
        </collision>
        <visual name="handle_grip">
          <pose>-0.05 0 0.025 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.025</radius><length>0.30</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.15 0.15 0.18 1</diffuse></material>
        </visual>
        <visual name="head_jaw">
          <pose>0.15 0 0.025 0 0 0</pose>
          <geometry><box><size>0.12 0.08 0.035</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.8 0.8 0.85 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="tool_hammer_mallet">
      <static>true</static>
      <pose>28.0 38.0 {p_hammer[0]:.3f} {p_hammer[1]:.4f} {p_hammer[2]:.4f} {p_hammer[3]:.4f}</pose>
      <link name="link">
        <inertial><mass>1.5</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        <collision name="hammer_col">
          <pose>0 0 0.025 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.022</radius><length>0.38</length></cylinder></geometry>
        </collision>
        <visual name="handle_wood">
          <pose>-0.05 0 0.025 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.02</radius><length>0.32</length></cylinder></geometry>
          <material><ambient>0.5 0.3 0.15 1</ambient><diffuse>0.65 0.4 0.2 1</diffuse></material>
        </visual>
        <visual name="hammer_head">
          <pose>0.15 0 0.035 0 0 0</pose>
          <geometry><box><size>0.14 0.07 0.07</size></box></geometry>
          <material><ambient>0.3 0.3 0.35 1</ambient><diffuse>0.45 0.45 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="tool_screwdriver_set">
      <static>true</static>
      <pose>40.0 60.0 {p_screwdriver[0]:.3f} {p_screwdriver[1]:.4f} {p_screwdriver[2]:.4f} {p_screwdriver[3]:.4f}</pose>
      <link name="link">
        <inertial><mass>0.9</mass><inertia><ixx>0.005</ixx><iyy>0.005</iyy><izz>0.005</izz></inertia></inertial>
        <collision name="col">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>0.28 0.18 0.05</size></box></geometry>
        </collision>
        <visual name="case">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>0.28 0.18 0.05</size></box></geometry>
          <material><ambient>0.7 0.55 0.05 1</ambient><diffuse>0.95 0.8 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="tool_instrument_toolbox">
      <static>true</static>
      <pose>70.0 45.0 {p_toolbox[0]:.3f} {p_toolbox[1]:.4f} {p_toolbox[2]:.4f} {p_toolbox[3]:.4f}</pose>
      <link name="link">
        <inertial><mass>4.5</mass><inertia><ixx>0.05</ixx><iyy>0.05</iyy><izz>0.05</izz></inertia></inertial>
        <collision name="col">
          <pose>0 0 0.11 0 0 0</pose>
          <geometry><box><size>0.45 0.30 0.22</size></box></geometry>
        </collision>
        <visual name="casing">
          <pose>0 0 0.11 0 0 0</pose>
          <geometry><box><size>0.45 0.30 0.22</size></box></geometry>
          <material><ambient>0.8 0.55 0.05 1</ambient><diffuse>0.95 0.7 0.05 1</diffuse></material>
        </visual>
        <visual name="handle_latches">
          <pose>0 0.16 0.11 0 0 0</pose>
          <geometry><box><size>0.20 0.04 0.08</size></box></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.15 0.15 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="rock_basalt_specimen">
      <static>true</static>
      <pose>55.0 85.0 {p_rock[0]:.3f} {p_rock[1]:.4f} {p_rock[2]:.4f} {p_rock[3]:.4f}</pose>
      <link name="link">
        <inertial><mass>3.2</mass><inertia><ixx>0.02</ixx><iyy>0.02</iyy><izz>0.02</izz></inertia></inertial>
        <collision name="col">
          <pose>0 0 0.09 0 0 0</pose>
          <geometry><box><size>0.26 0.22 0.18</size></box></geometry>
        </collision>
        <visual name="rock_vis">
          <pose>0 0 0.09 0 0 0</pose>
          <geometry><box><size>0.26 0.22 0.18</size></box></geometry>
          <material><ambient>0.12 0.12 0.14 1</ambient><diffuse>0.18 0.18 0.2 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- =====================================================================
         LARGE OBSTACLE AVOIDANCE TEST ASSETS
         ===================================================================== -->

    <!-- Obstacle 1: Monolithic Boulder Formation (transit corridor between Spawn and IDMO) -->
    <model name="obstacle_monolith_boulder_a">
      <static>true</static>
      <pose>22.0 28.0 {p_boulder_a[0]:.3f} {p_boulder_a[1]:.4f} {p_boulder_a[2]:.4f} {p_boulder_a[3]:.4f}</pose>
      <link name="link">
        <collision name="main_boulder_col">
          <pose>0 0 0.9 0 0 0.3</pose>
          <geometry><box><size>2.4 2.0 1.8</size></box></geometry>
        </collision>
        <visual name="main_boulder_vis">
          <pose>0 0 0.9 0 0 0.3</pose>
          <geometry><box><size>2.4 2.0 1.8</size></box></geometry>
          <material><ambient>0.15 0.12 0.10 1</ambient><diffuse>0.28 0.22 0.18 1</diffuse></material>
        </visual>
        <collision name="flank_rock_col">
          <pose>0.8 0.5 0.6 0.2 0.3 0.6</pose>
          <geometry><box><size>1.6 1.4 1.2</size></box></geometry>
        </collision>
        <visual name="flank_rock_vis">
          <pose>0.8 0.5 0.6 0.2 0.3 0.6</pose>
          <geometry><box><size>1.6 1.4 1.2</size></box></geometry>
          <material><ambient>0.18 0.14 0.12 1</ambient><diffuse>0.32 0.26 0.20 1</diffuse></material>
        </visual>
        <visual name="cap_stone_vis">
          <pose>-0.3 0.2 1.7 0 0 0</pose>
          <geometry><sphere><radius>0.65</radius></sphere></geometry>
          <material><ambient>0.12 0.10 0.08 1</ambient><diffuse>0.24 0.18 0.15 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Obstacle 2: Fallen Habitat Cargo Module (between Cache site and Crater) -->
    <model name="obstacle_habitat_cargo_module">
      <static>true</static>
      <pose>45.0 42.0 {p_cargo[0]:.3f} {p_cargo[1]:.4f} {p_cargo[2]:.4f} {p_cargo[3]:.4f}</pose>
      <link name="link">
        <collision name="module_body_col">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><box><size>3.6 1.8 1.7</size></box></geometry>
        </collision>
        <visual name="module_body_vis">
          <pose>0 0 0.85 0 0 0</pose>
          <geometry><box><size>3.6 1.8 1.7</size></box></geometry>
          <material><ambient>0.6 0.6 0.65 1</ambient><diffuse>0.85 0.85 0.90 1</diffuse></material>
        </visual>
        <!-- Hazard stripes accent border -->
        <visual name="hazard_stripe_top">
          <pose>0 0 1.71 0 0 0</pose>
          <geometry><box><size>3.6 0.25 0.02</size></box></geometry>
          <material><ambient>0.8 0.6 0.05 1</ambient><diffuse>0.95 0.8 0.05 1</diffuse></material>
        </visual>
        <!-- Bulkhead Endcap -->
        <collision name="bulkhead_col">
          <pose>-1.85 0 0.85 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.85</radius><length>0.3</length></cylinder></geometry>
        </collision>
        <visual name="bulkhead_vis">
          <pose>-1.85 0 0.85 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.85</radius><length>0.3</length></cylinder></geometry>
          <material><ambient>0.2 0.2 0.25 1</ambient><diffuse>0.35 0.35 0.40 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Obstacle 3: Massive Crag Formation (corridor heading to Marker Alpha) -->
    <model name="obstacle_crag_formation_b">
      <static>true</static>
      <pose>50.0 62.0 {p_crag[0]:.3f} {p_crag[1]:.4f} {p_crag[2]:.4f} {p_crag[3]:.4f}</pose>
      <link name="link">
        <collision name="spire_col">
          <pose>0 0 1.25 0.08 0.12 0.3</pose>
          <geometry><cylinder><radius>1.1</radius><length>2.5</length></cylinder></geometry>
        </collision>
        <visual name="spire_vis">
          <pose>0 0 1.25 0.08 0.12 0.3</pose>
          <geometry><cylinder><radius>1.1</radius><length>2.5</length></cylinder></geometry>
          <material><ambient>0.14 0.11 0.09 1</ambient><diffuse>0.26 0.20 0.16 1</diffuse></material>
        </visual>
        <collision name="buttress_col">
          <pose>1.1 -0.4 0.7 0.2 0.1 0.7</pose>
          <geometry><box><size>2.2 1.6 1.4</size></box></geometry>
        </collision>
        <visual name="buttress_vis">
          <pose>1.1 -0.4 0.7 0.2 0.1 0.7</pose>
          <geometry><box><size>2.2 1.6 1.4</size></box></geometry>
          <material><ambient>0.16 0.13 0.10 1</ambient><diffuse>0.30 0.24 0.18 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Obstacle 4: Slalom Pillars (corridor heading to Marker Bravo) -->
    <model name="obstacle_slalom_pillar_1">
      <static>true</static>
      <pose>72.0 65.0 {p_pillar_1[0]:.3f} {p_pillar_1[1]:.4f} {p_pillar_1[2]:.4f} {p_pillar_1[3]:.4f}</pose>
      <link name="link">
        <collision name="col">
          <pose>0 0 1.1 0 0 0</pose>
          <geometry><cylinder><radius>0.75</radius><length>2.2</length></cylinder></geometry>
        </collision>
        <visual name="vis">
          <pose>0 0 1.1 0 0 0</pose>
          <geometry><cylinder><radius>0.75</radius><length>2.2</length></cylinder></geometry>
          <material><ambient>0.20 0.15 0.12 1</ambient><diffuse>0.35 0.27 0.22 1</diffuse></material>
        </visual>
      </link>
    </model>

    <model name="obstacle_slalom_pillar_2">
      <static>true</static>
      <pose>76.0 72.0 {p_pillar_2[0]:.3f} {p_pillar_2[1]:.4f} {p_pillar_2[2]:.4f} {p_pillar_2[3]:.4f}</pose>
      <link name="link">
        <collision name="col">
          <pose>0 0 1.0 0 0 0</pose>
          <geometry><box><size>1.8 1.8 2.0</size></box></geometry>
        </collision>
        <visual name="vis">
          <pose>0 0 1.0 0 0 0</pose>
          <geometry><box><size>1.8 1.8 2.0</size></box></geometry>
          <material><ambient>0.18 0.14 0.11 1</ambient><diffuse>0.32 0.25 0.20 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- Obstacle 5: Decommissioned Weather/Comm Mast Debris (near IDMO panel) -->
    <model name="obstacle_comm_mast_debris">
      <static>true</static>
      <pose>28.0 52.0 {p_debris[0]:.3f} {p_debris[1]:.4f} {p_debris[2]:.4f} {p_debris[3]:.4f}</pose>
      <link name="link">
        <!-- Base concrete / composite footing -->
        <collision name="footing_col">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><cylinder><radius>0.9</radius><length>0.5</length></cylinder></geometry>
        </collision>
        <visual name="footing_vis">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><cylinder><radius>0.9</radius><length>0.5</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>
        <!-- Fallen lattice steel beam -->
        <collision name="beam_col">
          <pose>1.8 0 0.45 0.1 0.18 0</pose>
          <geometry><box><size>4.2 0.55 0.55</size></box></geometry>
        </collision>
        <visual name="beam_vis">
          <pose>1.8 0 0.45 0.1 0.18 0</pose>
          <geometry><box><size>4.2 0.55 0.55</size></box></geometry>
          <material><ambient>0.7 0.3 0.05 1</ambient><diffuse>0.85 0.35 0.05 1</diffuse></material>
        </visual>
        <!-- Antenna dish wreckage -->
        <collision name="dish_col">
          <pose>-0.3 1.2 0.65 1.1 0.2 0</pose>
          <geometry><cylinder><radius>0.85</radius><length>0.15</length></cylinder></geometry>
        </collision>
        <visual name="dish_vis">
          <pose>-0.3 1.2 0.65 1.1 0.2 0</pose>
          <geometry><cylinder><radius>0.85</radius><length>0.15</length></cylinder></geometry>
          <material><ambient>0.5 0.5 0.55 1</ambient><diffuse>0.75 0.75 0.8 1</diffuse></material>
        </visual>
      </link>
    </model>

  </world>
</sdf>
"""


def generate_flat_world_sdf() -> str:
    """Generate minimal flat ground SDF world for fast unit testing and CI."""
    return """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="test_flat">
    <physics name="default_physics" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Core Gazebo system plugins -->
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <plugin filename="ignition-gazebo-sensors-system" name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system" name="ignition::gazebo::systems::Imu"/>
    <plugin filename="ignition-gazebo-navsat-system" name="ignition::gazebo::systems::NavSat"/>

    <!-- WGS-84 Competition Datum (DSU, Bengaluru) -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>12.8614</latitude_deg>
      <longitude_deg>77.4380</longitude_deg>
      <elevation>860.0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 50 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>0.5</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.5 0.5 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
"""


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Generate Sproscape simulation terrain and worlds.")
    parser.add_argument('--output-dir', default='worlds', help='Directory to write worlds and heightmap')
    args = parser.parse_args()

    os.makedirs(args.output_dir, exist_ok=True)
    png_path = os.path.join(args.output_dir, 'sproscape_heightmap.png')
    world_path = os.path.join(args.output_dir, 'sproscape.sdf')
    flat_world_path = os.path.join(args.output_dir, 'test_flat.sdf')
    zones_path = os.path.join(args.output_dir, 'sproscape_zones.yaml')

    # 1. Heightmap PNG
    png_bytes, meta = generate_heightmap(dim=HEIGHTMAP_DIM, size_m=ARENA_SIZE_M)
    write_grayscale_png(png_path, HEIGHTMAP_DIM, HEIGHTMAP_DIM, png_bytes)
    print(f"Generated {png_path} ({HEIGHTMAP_DIM}x{HEIGHTMAP_DIM}, z-range: {meta['elevation_range_m']}m)")

    # 2. Textures for Ogre2 rendering
    tex_dir = os.path.join(args.output_dir, 'materials', 'textures')
    os.makedirs(tex_dir, exist_ok=True)
    soil_tex_path = os.path.join(tex_dir, 'martian_soil.png')
    normal_tex_path = os.path.join(tex_dir, 'flat_normal.png')

    # 64x64 procedural Martian reddish soil texture
    soil_bytes = bytearray()
    for y in range(64):
        for x in range(64):
            grain = int(math.sin(x * 0.7) * math.cos(y * 0.7) * 15)
            r = min(255, max(0, 185 + grain))
            g = min(255, max(0, 95 + int(grain * 0.6)))
            b = min(255, max(0, 60 + int(grain * 0.4)))
            soil_bytes.extend([r, g, b])
    write_rgb_png(soil_tex_path, 64, 64, bytes(soil_bytes))

    # Flat normal map (128, 128, 255)
    normal_bytes = bytearray([128, 128, 255] * (32 * 32))
    write_rgb_png(normal_tex_path, 32, 32, bytes(normal_bytes))
    print(f"Generated terrain textures in {tex_dir}")

    # 3. Main Sproscape World SDF
    world_sdf = generate_world_sdf('sproscape_heightmap.png', meta['elevation_range_m'], min_elevation_m=meta['min_elevation_m'])
    with open(world_path, 'w') as f:
        f.write(world_sdf)
    print(f"Generated {world_path}")

    # 4. Flat test world SDF
    with open(flat_world_path, 'w') as f:
        f.write(generate_flat_world_sdf())
    print(f"Generated {flat_world_path}")

    # 5. Arena zones layout YAML
    zones_data = {
        'datum': {'lat': 12.8614, 'lon': 77.4380, 'alt': 860.0},
        'arena_size_m': ARENA_SIZE_M,
        'start_line': {'x': 0.0, 'y': 0.0, 'name': 'Base Station Start Area'},
        'delivery_markers': [
            {'id': 'DROP_ALPHA', 'name': 'Marker Alpha (Green Pad)', 'x': 30.0, 'y': 70.0},
            {'id': 'DROP_BRAVO', 'name': 'Marker Bravo (Blue Pad)',  'x': 80.0, 'y': 80.0},
            {'id': 'DROP_CACHE', 'name': 'ABEx Cache Drop / Pickup', 'x': 60.0, 'y': 30.0},
        ],
        'rado_objects': [
            {'id': 'TOOL_1', 'name': 'Heavy Wrench (≤7cm dia)', 'x': 25.0, 'y': 35.0, 'mass_kg': 1.8},
            {'id': 'TOOL_HAMMER', 'name': 'Hammer / Mallet', 'x': 28.0, 'y': 38.0, 'mass_kg': 1.5},
            {'id': 'TOOL_2', 'name': 'Screwdriver Set', 'x': 40.0, 'y': 60.0, 'mass_kg': 0.9},
            {'id': 'BOX_1',  'name': 'Instrument Toolbox', 'x': 70.0, 'y': 45.0, 'mass_kg': 4.5},
            {'id': 'ROCK_1', 'name': 'Basalt Specimen A', 'x': 55.0, 'y': 85.0, 'mass_kg': 3.2},
        ],
        'abex_sites': [
            {'id': 'ABEX_SITE_A', 'name': 'Stratigraphy Outcrop', 'x': 45.0, 'y': 95.0},
            {'id': 'ABEX_SITE_B', 'name': 'Fine-Soil Basin',       'x': 90.0, 'y': 100.0},
        ],
        'idmo_panel': {'x': 15.0, 'y': 50.0, 'name': 'Mock Maintenance Panel (1.8m)'},
        'red_zones': [
            {'id': 'RED_ZONE_1', 'name': 'Contaminated Zone North', 'center_x': 50.0, 'center_y': 75.0, 'radius_m': 6.0},
            {'id': 'RED_ZONE_2', 'name': 'Contaminated Crater Rift', 'center_x': 20.0, 'center_y': 90.0, 'radius_m': 8.0},
        ],
        'obstacles': [
            {'id': 'OBSTACLE_BOULDER_A', 'name': 'Monolithic Boulder Formation', 'x': 22.0, 'y': 28.0, 'size': [2.4, 2.0, 1.8]},
            {'id': 'OBSTACLE_HABITAT_MODULE', 'name': 'Habitat Cargo Module', 'x': 45.0, 'y': 42.0, 'size': [3.6, 1.8, 1.7]},
            {'id': 'OBSTACLE_CRAG_B', 'name': 'Massive Crag Spire & Buttress', 'x': 50.0, 'y': 62.0, 'size': [2.5, 2.2, 2.5]},
            {'id': 'OBSTACLE_SLALOM_1', 'name': 'Slalom Pillar 1', 'x': 72.0, 'y': 65.0, 'size': [1.5, 1.5, 2.2]},
            {'id': 'OBSTACLE_SLALOM_2', 'name': 'Slalom Pillar 2', 'x': 76.0, 'y': 72.0, 'size': [1.8, 1.8, 2.0]},
            {'id': 'OBSTACLE_COMM_MAST', 'name': 'Decommissioned Comm Mast Debris', 'x': 28.0, 'y': 52.0, 'size': [4.2, 1.8, 0.9]},
        ],
    }
    with open(zones_path, 'w') as f:
        yaml.dump(zones_data, f, sort_keys=False)
    print(f"Generated {zones_path}")


if __name__ == '__main__':
    main()
