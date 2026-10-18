#!/usr/bin/env python3
"""
generate_terrain.py

Procedural terrain generator for the IRC 2027 "Sproscape-Sim" Mars-analog arena.
Produces:
  1. An Ogre-compatible 8-bit heightmap PNG (257x257 pixels, 120m x 120m arena).
  2. A Gazebo Fortress (gz-sim 6) SDF world file (sproscape.sdf) referencing the heightmap.
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


def generate_heightmap(dim: int = 257, size_m: float = 120.0) -> Tuple[bytes, Dict]:
    """
    Generate parametric height values representing the IRC 2027 Sproscape terrain:
      - Start line & base station (y in [0, 15]): flat ground (z ≈ 0.0)
      - Sand dune field (y in [30, 80], x in [40, 110]): 15-20° sinusoidal dunes
      - Rocky garden (x in [0, 50], y in [70, 120]): bumpy terrain with noise
      - Impact crater (center at x=35, y=55): depression with raised ejecta rim
      - Rift / trench (along x ≈ 25 from y=80 to y=110): -0.8m depression
      - Negative drop ledge (at x=20, y=70): -0.5m step drop
    """
    pixels = bytearray(dim * dim)
    grid_res = size_m / (dim - 1)

    # Elevation array in meters
    elevations = [[0.0 for _ in range(dim)] for _ in range(dim)]

    for j in range(dim):
        y = j * grid_res
        for i in range(dim):
            x = i * grid_res
            z = 0.0

            # 1. Base station zone (flat clear start area)
            if y < 15.0:
                z = 0.0

            # 2. Sand dunes (sinusoidal waves with 15-20° slopes)
            elif 25.0 <= y <= 85.0 and x >= 30.0:
                dune_wave = math.sin((x * 0.15) + (y * 0.10)) * 1.4
                dune_ripple = math.cos(y * 0.4) * 0.25
                z += max(0.0, dune_wave + dune_ripple)

            # 3. Rocky garden roughness
            if x <= 50.0 and y >= 65.0:
                rock_noise = math.sin(x * 0.8) * math.cos(y * 0.8) * 0.4
                z += max(0.0, rock_noise)

            # 4. Crater (center at x=35, y=55, radius=7m, depth=1.4m, rim height=0.6m)
            dist_crater = math.hypot(x - 35.0, y - 55.0)
            if dist_crater < 10.0:
                if dist_crater < 6.0:
                    # Depression inside crater
                    z -= (1.0 - (dist_crater / 6.0)) * 1.4
                else:
                    # Raised rim
                    rim_t = (dist_crater - 6.0) / 4.0
                    z += math.sin(rim_t * math.pi) * 0.6

            # 5. Rift / trench (along x ≈ 25 from y=80 to 110, width 3m, depth 0.8m)
            if 80.0 <= y <= 110.0 and abs(x - 25.0) <= 2.5:
                trench_depth = (1.0 - (abs(x - 25.0) / 2.5)) * 0.8
                z -= trench_depth

            # 6. Negative obstacle / ledge drop (step down around y=70, x in [15, 25])
            if 68.0 <= y <= 72.0 and 15.0 <= x <= 25.0:
                z -= 0.5

            elevations[j][i] = z

    # Normalize elevations to 0..255 for 8-bit heightmap
    min_z = min(min(row) for row in elevations)
    max_z = max(max(row) for row in elevations)
    z_range = max(max_z - min_z, 1e-3)

    for j in range(dim):
        for i in range(dim):
            norm = (elevations[j][i] - min_z) / z_range
            val = int(min(255, max(0, round(norm * 255))))
            pixels[j * dim + i] = val

    metadata = {
        'min_elevation_m': round(min_z, 3),
        'max_elevation_m': round(max_z, 3),
        'elevation_range_m': round(z_range, 3),
        'dimension': dim,
        'size_m': size_m,
    }

    return bytes(pixels), metadata


def generate_world_sdf(heightmap_rel_path: str, z_range_m: float) -> str:
    """Generate the complete Gazebo Fortress SDF world for Sproscape."""
    return f"""<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="sproscape">
    <!-- Physics configuration -->
    <physics name="1ms" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

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
      <link name="terrain_link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>{heightmap_rel_path}</uri>
              <size>{ARENA_SIZE_M} {ARENA_SIZE_M} {z_range_m:.2f}</size>
              <pos>{ARENA_SIZE_M / 2.0} {ARENA_SIZE_M / 2.0} 0</pos>
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
              <uri>{heightmap_rel_path}</uri>
              <size>{ARENA_SIZE_M} {ARENA_SIZE_M} {z_range_m:.2f}</size>
              <pos>{ARENA_SIZE_M / 2.0} {ARENA_SIZE_M / 2.0} 0</pos>
              <texture>
                <diffuse>materials/textures/martian_soil.jpg</diffuse>
                <normal>materials/textures/martian_soil_normal.jpg</normal>
                <size>10</size>
              </texture>
            </heightmap>
          </geometry>
        </visual>
      </link>
    </model>

    <!-- Base Station Tent & Start Gate marker -->
    <model name="start_gate">
      <static>true</static>
      <pose>0 0 0.5 0 0 0</pose>
      <link name="gate_link">
        <visual name="post_left">
          <pose>-1.5 0 0.5 0 0 0</pose>
          <geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
          <material><ambient>0.1 0.8 0.2 1</ambient><diffuse>0.1 0.8 0.2 1</diffuse></material>
        </visual>
        <visual name="post_right">
          <pose>1.5 0 0.5 0 0 0</pose>
          <geometry><cylinder><radius>0.05</radius><length>1.0</length></cylinder></geometry>
          <material><ambient>0.1 0.8 0.2 1</ambient><diffuse>0.1 0.8 0.2 1</diffuse></material>
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

    # 2. Main Sproscape World SDF
    world_sdf = generate_world_sdf('sproscape_heightmap.png', meta['elevation_range_m'])
    with open(world_path, 'w') as f:
        f.write(world_sdf)
    print(f"Generated {world_path}")

    # 3. Flat test world SDF
    with open(flat_world_path, 'w') as f:
        f.write(generate_flat_world_sdf())
    print(f"Generated {flat_world_path}")

    # 4. Arena zones layout YAML
    zones_data = {
        'datum': {'lat': 12.8614, 'lon': 77.4380, 'alt': 860.0},
        'arena_size_m': ARENA_SIZE_M,
        'start_line': {'x': 0.0, 'y': 0.0, 'name': 'Base Station Start Gate'},
        'delivery_markers': [
            {'id': 'DROP_ALPHA', 'name': 'Marker Alpha (Green Pad)', 'x': 30.0, 'y': 70.0},
            {'id': 'DROP_BRAVO', 'name': 'Marker Bravo (Blue Pad)',  'x': 80.0, 'y': 80.0},
            {'id': 'DROP_CACHE', 'name': 'ABEx Cache Drop / Pickup', 'x': 60.0, 'y': 30.0},
        ],
        'rado_objects': [
            {'id': 'TOOL_1', 'name': 'Heavy Wrench (≤7cm dia)', 'x': 25.0, 'y': 35.0, 'mass_kg': 1.8},
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
