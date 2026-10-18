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


def get_surface_elevation(x: float, y: float) -> float:
    """
    Parametric surface elevation function for the IRC 2027 Sproscape terrain in meters.
    Guarantees:
      - Designated base station spawn zone ([0, 25m] x [0, 25m] and y <= 20m) is exactly 0.000m.
      - Smooth C1/C2 continuous transition into undulating dunes and rocky terrain (no cliff or ridge).
      - Realistic Martian analog features (dunes, crater, rift trench, rocky outcrops).
      - Smooth boundary tapering so perimeter walls cleanly enclose the arena without gap or spill.
      - Minimum elevation across entire arena is guaranteed >= 0.000m.
    """
    # 1. Base station spawn zone: [0, 25m] x [0, 25m] or y <= 20m
    dx = max(0.0, x - 25.0)
    dy = max(0.0, y - 20.0)
    dist_flat = math.hypot(dx, dy)

    # Smoothstep transition over 8.0 meters
    blend_width = 8.0
    if dist_flat <= 0.0:
        return 0.0
    elif dist_flat >= blend_width:
        blend = 1.0
    else:
        t = dist_flat / blend_width
        blend = t * t * (3.0 - 2.0 * t)

    # Boundary taper: keep arena edges gentle near perimeter walls (margin 4m)
    margin = 4.0
    edge_dist = min(x, y, 120.0 - x, 120.0 - y)
    edge_factor = edge_dist / margin if edge_dist < margin else 1.0

    # Gentle base elevation ramping up to ~0.75m away from spawn
    base_elev = 0.75 * (1.0 - math.exp(-dist_flat / 15.0))

    # Sand dune field (y in [25, 85], x >= 28)
    dune = 0.0
    if 25.0 <= y <= 85.0 and x >= 28.0:
        dune_wave = math.sin((x * 0.15) + (y * 0.10)) * 1.2
        dune_ripple = math.cos(y * 0.35) * 0.25
        dune = max(0.0, dune_wave + dune_ripple)

    # Rocky garden roughness (x <= 50, y >= 65)
    rock = 0.0
    if x <= 50.0 and y >= 65.0:
        rock = max(0.0, math.sin(x * 0.8) * math.cos(y * 0.8) * 0.35)

    # Impact Crater (center at x=35, y=55, radius 10m)
    crater = 0.0
    dist_crater = math.hypot(x - 35.0, y - 55.0)
    if dist_crater < 10.0:
        if dist_crater < 6.0:
            # Depression inside crater bowl
            crater = -(1.0 - (dist_crater / 6.0)) * 0.7
        else:
            # Raised ejecta rim
            rim_t = (dist_crater - 6.0) / 4.0
            crater = math.sin(rim_t * math.pi) * 0.6

    # Rift / trench (along x ≈ 25 from y=80 to y=110, width 5m)
    trench = 0.0
    if 80.0 <= y <= 110.0 and abs(x - 25.0) <= 2.5:
        trench = -(1.0 - (abs(x - 25.0) / 2.5)) * 0.55

    # Negative drop ledge (around y=70, x in [15, 25])
    ledge = 0.0
    if 68.0 <= y <= 72.0 and 15.0 <= x <= 25.0:
        ledge = -0.35

    total_z = (base_elev + dune + rock + crater + trench + ledge) * blend * edge_factor
    return max(0.0, total_z)


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
        pz = get_surface_elevation(px, py)
        xml_lines.append(f'''        <!-- Pylon {i} -->
        <collision name="pylon_{i}_col">
          <pose>{px:.3f} {py:.3f} {pz + 0.35:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>0.7</length></cylinder></geometry>
        </collision>
        <visual name="pylon_{i}_base">
          <pose>{px:.3f} {py:.3f} {pz + 0.02:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.18</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_cone">
          <pose>{px:.3f} {py:.3f} {pz + 0.35:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.1</radius><length>0.7</length></cylinder></geometry>
          <material><ambient>0.8 0.1 0.05 1</ambient><diffuse>0.95 0.15 0.05 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_stripe">
          <pose>{px:.3f} {py:.3f} {pz + 0.45:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.105</radius><length>0.15</length></cylinder></geometry>
          <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_top">
          <pose>{px:.3f} {py:.3f} {pz + 0.72:.3f} 0 0 0</pose>
          <geometry><sphere><radius>0.05</radius></sphere></geometry>
          <material><ambient>0.9 0.1 0.0 1</ambient><diffuse>1.0 0.1 0.0 1</diffuse><emissive>1.0 0.1 0.0 1</emissive></material>
        </visual>''')
    xml_lines.append('      </link>')
    xml_lines.append('    </model>')
    return '\n'.join(xml_lines)


def generate_world_sdf(heightmap_rel_path: str, z_range_m: float, min_elevation_m: float = 0.0) -> str:
    """Generate the complete Gazebo Fortress SDF world for Sproscape with full arena assets."""
    z_alpha = get_surface_elevation(30.0, 70.0)
    z_bravo = get_surface_elevation(80.0, 80.0)
    z_cache = get_surface_elevation(60.0, 30.0)
    z_idmo = get_surface_elevation(15.0, 50.0)

    # Tool surface elevations
    z_wrench = get_surface_elevation(25.0, 35.0)
    z_hammer = get_surface_elevation(28.0, 38.0)
    z_screwdriver = get_surface_elevation(40.0, 60.0)
    z_toolbox = get_surface_elevation(70.0, 45.0)
    z_rock = get_surface_elevation(55.0, 85.0)

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
        <pose>2.5 3.5 1.2 0 0.126 0.685</pose>
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
      <pose>{ARENA_SIZE_M / 2.0} {ARENA_SIZE_M / 2.0} 0 0 0 0</pose>
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{heightmap_rel_path}</uri>
              <size>{ARENA_SIZE_M} {ARENA_SIZE_M} {z_range_m:.2f}</size>
              <pos>0.0 0.0 {min_elevation_m:.3f}</pos>
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
              <size>{ARENA_SIZE_M} {ARENA_SIZE_M} {z_range_m:.2f}</size>
              <pos>0.0 0.0 {min_elevation_m:.3f}</pos>
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
          <pose>60.0 0.0 1.0 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_south_vis">
          <pose>60.0 0.0 1.0 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- North Wall (y = 120.0) -->
        <collision name="wall_north_col">
          <pose>60.0 120.0 1.0 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_north_vis">
          <pose>60.0 120.0 1.0 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- West Wall (x = 0.0) -->
        <collision name="wall_west_col">
          <pose>0.0 60.0 1.0 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_west_vis">
          <pose>0.0 60.0 1.0 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- East Wall (x = 120.0) -->
        <collision name="wall_east_col">
          <pose>120.0 60.0 1.0 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_east_vis">
          <pose>120.0 60.0 1.0 0 0 0</pose>
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
      <pose>30.0 70.0 {z_alpha:.3f} 0 0 0</pose>
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
      <pose>80.0 80.0 {z_bravo:.3f} 0 0 0</pose>
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
      <pose>60.0 30.0 {z_cache:.3f} 0 0 0</pose>
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
      <pose>15.0 50.0 {z_idmo:.3f} 0 0 0</pose>
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
      <pose>25.0 35.0 {z_wrench:.3f} 0 0 0</pose>
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
      <pose>28.0 38.0 {z_hammer:.3f} 0 0 0.6</pose>
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
      <pose>40.0 60.0 {z_screwdriver:.3f} 0 0 0</pose>
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
      <pose>70.0 45.0 {z_toolbox:.3f} 0 0 0</pose>
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
      <pose>55.0 85.0 {z_rock:.3f} 0.1 0.2 0.4</pose>
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
    }
    with open(zones_path, 'w') as f:
        yaml.dump(zones_data, f, sort_keys=False)
    print(f"Generated {zones_path}")


if __name__ == '__main__':
    main()
