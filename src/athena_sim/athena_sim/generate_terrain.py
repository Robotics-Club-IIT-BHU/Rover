#!/usr/bin/env python3
"""
generate_terrain.py

Procedural terrain generator for the IRC 2027 "Sproscape-Sim" Mars-analog arena.
Produces:
  1. An Ogre-compatible 8-bit heightmap PNG (257x257 pixels, 120m x 120m arena).
  2. A Gazebo Fortress (gz-sim 6) SDF world file (sproscape.sdf) referencing the heightmap
     with boundary walls, competition delivery pads, beacons, service panel, danger zones,
     base station perimeter survey markers, scientific crater stake, rock field, and tools.
  3. An arena layout metadata YAML (sproscape_zones.yaml) used by briefing_tool and mission_runner.

All algorithms are implemented in pure Python (standard library struct + zlib)
for deterministic, reproducible builds without external image dependencies.

Coordinate System Standard:
  Arena bounds: [-60.0m, +60.0m] x [-60.0m, +60.0m]
  Origin (0, 0, 0): Center of arena, matching Gazebo Fortress Ogre2 Terra and DART physics.
  Base Station Spawn: SW quadrant centered at (-48.0m, -48.0m), flat at Z = 0.000m.
  Perimeter walls: South at y=-60, North at y=+60, West at x=-60, East at x=+60.
"""

import math
import os
import struct
import zlib
from typing import Dict, List, Tuple
import yaml

ARENA_SIZE_M = 120.0
HEIGHTMAP_DIM = 257  # Standard 2^n + 1 dimension for Ogre terrain


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
    Coordinate domain: x, y in [-60.0m, +60.0m].
    Distinct Geological Environments:
      1. Base station staging pad in SW (around -48m, -48m) is exactly 0.000m flat.
      2. Perimeter wall boundary taper down to 0.000m flush at all 4 walls (+-60m).
      3. Deep complex impact crater in NW (R = 16.5m, true rim-to-bowl relief ~1.44m, central mound).
      4. Asymmetric Barchan sand dunes in East/SE (dune crests ~2.2m, slip faces ~14-17 deg).
      5. Terraced Promontory Ridge in NE (peak ~5.5m with layered plateau overlooks).
      6. Sinuous outflow gully / meandering dried regolith wash cutting across central plain.
      7. Red Danger / Contamination Zones with natural depression rift and fault trench.
      8. Strictly C2 continuous, zero spikes, fully drivable by Athena rover rocker-bogie.
    """
    x = max(-60.0, min(60.0, x))
    y = max(-60.0, min(60.0, y))

    # 1. Base station spawn zone: SW quadrant centered at (-48, -48)
    dist_spawn = math.hypot(max(0.0, abs(x - (-48.0)) - 7.0), max(0.0, abs(y - (-48.0)) - 7.0))
    spawn_blend = smoothstep(0.0, 14.0, dist_spawn)
    if spawn_blend <= 0.0:
        return 0.0

    # 2. Boundary taper down to 0 at perimeter walls (x = +-60, y = +-60)
    taper_w = 14.0
    taper_x = smoothstep(-60.0, -60.0 + taper_w, x) * (1.0 - smoothstep(60.0 - taper_w, 60.0, x))
    taper_y = smoothstep(-60.0, -60.0 + taper_w, y) * (1.0 - smoothstep(60.0 - taper_w, 60.0, y))
    boundary_blend = taper_x * taper_y

    # 3. Ambient Martian undulating regolith
    wave = (
        0.45 * math.sin(x * 0.052 + 0.4) * math.cos(y * 0.046 - 0.2)
        + 0.25 * math.sin(x * 0.10 - y * 0.08 + 1.2)
        + 0.16 * math.cos(x * 0.07 + y * 0.12)
    )

    # 4. Terraced Promontory Ridge in North-East (peaks up to ~5.5m)
    dx_hill = x - 28.0
    dy_hill = y - 26.0
    hill_env = math.exp(-(dx_hill**2 / 520.0 + dy_hill**2 / 420.0))
    hill_ridge = hill_env * (3.0 + 0.7 * math.sin(x * 0.14 + 0.3) * math.cos(y * 0.11))

    # Secondary NE knoll
    dx_knoll = x - 14.0
    dy_knoll = y - 44.0
    knoll = 1.4 * math.exp(-(dx_knoll**2 / 180.0 + dy_knoll**2 / 220.0))

    # 5. Asymmetric Barchan Sand Dune Field (East / Central-East)
    dune_env = (
        smoothstep(-6.0, 8.0, x) * (1.0 - smoothstep(44.0, 54.0, x)) *
        smoothstep(-44.0, -28.0, y) * (1.0 - smoothstep(2.0, 14.0, y))
    )
    rad_wind = 0.62  # Wind azimuth ~35 degrees
    u = x * math.cos(rad_wind) + y * math.sin(rad_wind)
    v = -x * math.sin(rad_wind) + y * math.cos(rad_wind)
    phase = u * 0.20
    saw = math.sin(phase) - 0.22 * math.sin(2.0 * phase)
    crest = math.cos(v * 0.13)
    dunes = dune_env * 1.30 * (1.0 * saw + 0.35 * crest * saw)

    # 6. Deep Impact Crater in North-West (x = -25.0, y = 20.0, R = 16.5m)
    dist_crater = math.hypot(x - (-25.0), y - 20.0)
    crater_r = 16.5
    crater = 0.0
    if dist_crater < crater_r:
        s = dist_crater / crater_r
        env = (1.0 - s * s) ** 2
        crater_shape = (
            -1.25 * math.cos(s * math.pi * 0.65)
            + 0.85 * math.sin(s * math.pi) ** 2 * smoothstep(0.35, 0.75, s)
            + 0.35 * math.exp(-(s / 0.18) ** 2)
        )
        crater = crater_shape * env

    # 7. Sinuous Meandering Outflow Gully / Dry Wash
    gully_xc = -5.0 + 10.0 * math.sin(y * 0.055 + 0.6)
    dist_gully = abs(x - gully_xc)
    gully_env = smoothstep(-32.0, -18.0, y) * (1.0 - smoothstep(28.0, 42.0, y))
    gully = 0.0
    if dist_gully < 5.5:
        u_g = dist_gully / 5.5
        gully = -0.50 * (1.0 - u_g * u_g) ** 2 * gully_env

    # 8. Red Zone 1 trench at (10.0, 22.0, R = 6.5m)
    dist_rz1 = math.hypot(x - 10.0, y - 22.0)
    rz1 = 0.0
    if dist_rz1 < 6.5:
        s1 = dist_rz1 / 6.5
        rz1 = -0.45 * ((1.0 - s1 * s1) ** 2)

    raw_z = 1.30 + wave + hill_ridge + knoll + dunes + crater + gully + rz1
    z = raw_z * spawn_blend * boundary_blend
    return max(0.0, z)


def get_surface_normal(x: float, y: float, eps: float = 0.02) -> Tuple[float, float, float, float, float]:
    """
    Compute analytical / central-difference surface normal unit vector (nx, ny, nz)
    and surface gradients (gx, gy) = (dz/dx, dz/dy).
    """
    gx = (get_surface_elevation(min(60.0, x + eps), y) - get_surface_elevation(max(-60.0, x - eps), y)) / (2.0 * eps)
    gy = (get_surface_elevation(x, min(60.0, y + eps)) - get_surface_elevation(x, max(-60.0, y - eps))) / (2.0 * eps)
    norm = math.sqrt(gx * gx + gy * gy + 1.0)
    return -gx / norm, -gy / norm, 1.0 / norm, gx, gy


def compute_object_pose(x: float, y: float, half_x: float, half_y: float, height: float, embed_depth: float = 0.0, intrinsic_yaw: float = 0.0) -> Tuple[float, float, float, float]:
    """
    Compute slope-aligned pose (z_base, roll, pitch, yaw) for an object at (x, y).
    z_base is the ground surface contact elevation (minus embed_depth for natural seating).
    Child links/geometries in the model have their base at z=0, guaranteeing ZERO floating.
    """
    z0 = get_surface_elevation(x, y)
    nx, ny, nz, gx, gy = get_surface_normal(x, y)

    pitch = math.atan(-gx)
    roll = math.atan(gy)

    z_base = z0 - embed_depth
    return z_base, roll, pitch, intrinsic_yaw


def generate_heightmap(dim: int = 257, size_m: float = 120.0) -> Tuple[bytes, Dict]:
    """
    Generate parametric height values representing the IRC 2027 Sproscape terrain:
      - Scanline row 0 corresponds to North (Y = +60.0m)
      - Scanline row dim-1 corresponds to South (Y = -60.0m)
      - Scanline col 0 corresponds to West (X = -60.0m)
      - Scanline col dim-1 corresponds to East (X = +60.0m)
      - Base station spawn area around (-48m, -48m) is 100% flat at Z = 0.00m.
    """
    pixels = bytearray(dim * dim)
    grid_res = size_m / (dim - 1)

    elevations = [[0.0 for _ in range(dim)] for _ in range(dim)]

    for row in range(dim):
        y = (size_m / 2.0) - row * grid_res
        for col in range(dim):
            x = -(size_m / 2.0) + col * grid_res
            elevations[row][col] = get_surface_elevation(x, y)

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


def _generate_red_zone_pylons_xml(model_name: str, cx: float, cy: float, radius: float, num_pylons: int = 12) -> str:
    """Generate SDF XML for perimeter warning pylons, continuous barrier beams, and central danger warning tower."""
    cz0 = get_surface_elevation(cx, cy)
    xml_lines = [
        f'    <model name="{model_name}">',
        '      <static>true</static>',
        '      <link name="link">',
        '        <!-- Central Danger Warning Tower -->',
        f'        <visual name="center_mast">',
        f'          <pose>{cx:.3f} {cy:.3f} {cz0 + 1.25:.3f} 0 0 0</pose>',
        f'          <geometry><cylinder><radius>0.05</radius><length>2.5</length></cylinder></geometry>',
        f'          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>',
        f'        </visual>',
        f'        <visual name="center_beacon">',
        f'          <pose>{cx:.3f} {cy:.3f} {cz0 + 2.58:.3f} 0 0 0</pose>',
        f'          <geometry><sphere><radius>0.22</radius></sphere></geometry>',
        f'          <material><ambient>1.0 0.1 0.1 1</ambient><diffuse>1.0 0.1 0.1 1</diffuse><emissive>1.0 0.1 0.1 1</emissive></material>',
        f'        </visual>',
        f'        <visual name="hazard_sign">',
        f'          <pose>{cx:.3f} {cy:.3f} {cz0 + 1.8:.3f} 0 0 0</pose>',
        f'          <geometry><box><size>0.9 0.05 0.55</size></box></geometry>',
        f'          <material><ambient>0.85 0.1 0.05 1</ambient><diffuse>0.95 0.15 0.05 1</diffuse><emissive>0.4 0.05 0.02 1</emissive></material>',
        f'        </visual>',
    ]
    for i in range(num_pylons):
        angle = (2.0 * math.pi * i) / num_pylons
        px = cx + radius * math.cos(angle)
        py = cy + radius * math.sin(angle)
        pz, roll, pitch, _ = compute_object_pose(px, py, 0.18, 0.18, 0.04, embed_depth=0.01)
        xml_lines.append(f'''        <!-- Pylon {i} -->
        <collision name="pylon_{i}_col">
          <pose>{px:.3f} {py:.3f} {pz + 0.35:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><cylinder><radius>0.12</radius><length>0.7</length></cylinder></geometry>
        </collision>
        <visual name="pylon_{i}_base">
          <pose>{px:.3f} {py:.3f} {pz + 0.02:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><cylinder><radius>0.20</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_cone">
          <pose>{px:.3f} {py:.3f} {pz + 0.35:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><cylinder><radius>0.11</radius><length>0.7</length></cylinder></geometry>
          <material><ambient>0.85 0.1 0.05 1</ambient><diffuse>0.95 0.15 0.05 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_stripe">
          <pose>{px:.3f} {py:.3f} {pz + 0.45:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><cylinder><radius>0.115</radius><length>0.16</length></cylinder></geometry>
          <material><ambient>0.85 0.85 0.85 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>
        </visual>
        <visual name="pylon_{i}_top">
          <pose>{px:.3f} {py:.3f} {pz + 0.72:.3f} {roll:.4f} {pitch:.4f} 0</pose>
          <geometry><sphere><radius>0.06</radius></sphere></geometry>
          <material><ambient>1.0 0.1 0.0 1</ambient><diffuse>1.0 0.1 0.0 1</diffuse><emissive>1.0 0.1 0.0 1</emissive></material>
        </visual>''')

        # Perimeter warning barrier segment linking pylon i to pylon i+1
        next_angle = (2.0 * math.pi * (i + 1)) / num_pylons
        next_px = cx + radius * math.cos(next_angle)
        next_py = cy + radius * math.sin(next_angle)
        mid_x = (px + next_px) / 2.0
        mid_y = (py + next_py) / 2.0
        mid_z = get_surface_elevation(mid_x, mid_y)
        seg_len = math.hypot(next_px - px, next_py - py)
        seg_yaw = math.atan2(next_py - py, next_px - px)
        xml_lines.append(f'''        <!-- Hazard Barrier Segment {i} -->
        <visual name="hazard_barrier_{i}">
          <pose>{mid_x:.3f} {mid_y:.3f} {mid_z + 0.35:.3f} 0 0 {seg_yaw:.4f}</pose>
          <geometry><box><size>{seg_len:.3f} 0.04 0.08</size></box></geometry>
          <material><ambient>0.9 0.15 0.05 1</ambient><diffuse>0.95 0.2 0.05 1</diffuse><emissive>0.3 0.05 0.02 1</emissive></material>
        </visual>''')

    xml_lines.append('      </link>')
    xml_lines.append('    </model>')
    return '\n'.join(xml_lines)


def _generate_base_station_markers_xml(model_name: str = 'base_station_markers') -> str:
    """Generate high-contrast perimeter boundary survey markers and start gate for the Base Station."""
    xml_lines = [
        f'    <model name="{model_name}">',
        '      <static>true</static>',
        '      <link name="link">',
    ]
    # 4 Corner survey beacons defining the [-56, -38] x [-56, -38] staging area
    corners = [
        ('sw', -56.0, -56.0),
        ('se', -38.0, -56.0),
        ('nw', -56.0, -38.0),
        ('ne', -38.0, -38.0),
    ]
    for tag, cx, cy in corners:
        cz = get_surface_elevation(cx, cy)
        xml_lines.append(f'''        <!-- Corner Survey Stanchion {tag.upper()} -->
        <visual name="survey_post_{tag}">
          <pose>{cx:.3f} {cy:.3f} {cz + 0.6:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.035</radius><length>1.2</length></cylinder></geometry>
          <material><ambient>0.8 0.4 0.05 1</ambient><diffuse>0.95 0.5 0.05 1</diffuse></material>
        </visual>
        <visual name="survey_beacon_{tag}">
          <pose>{cx:.3f} {cy:.3f} {cz + 1.25:.3f} 0 0 0</pose>
          <geometry><sphere><radius>0.09</radius></sphere></geometry>
          <material><ambient>0.1 0.9 0.3 1</ambient><diffuse>0.2 1.0 0.4 1</diffuse><emissive>0.2 1.0 0.4 1</emissive></material>
        </visual>''')

    # Start Gate Stanchions at (-48, -48)
    gate_posts = [
        ('gate_left', -48.0 - 1.2, -48.0 + 1.2),
        ('gate_right', -48.0 + 1.2, -48.0 - 1.2),
    ]
    for gtag, gx, gy in gate_posts:
        gz = get_surface_elevation(gx, gy)
        xml_lines.append(f'''        <!-- Start Gate Post {gtag} -->
        <visual name="{gtag}_post">
          <pose>{gx:.3f} {gy:.3f} {gz + 0.75:.3f} 0 0 0</pose>
          <geometry><cylinder><radius>0.04</radius><length>1.5</length></cylinder></geometry>
          <material><ambient>0.1 0.7 0.2 1</ambient><diffuse>0.2 0.9 0.3 1</diffuse></material>
        </visual>
        <visual name="{gtag}_light">
          <pose>{gx:.3f} {gy:.3f} {gz + 1.55:.3f} 0 0 0</pose>
          <geometry><sphere><radius>0.11</radius></sphere></geometry>
          <material><ambient>0.2 1.0 0.2 1</ambient><diffuse>0.2 1.0 0.2 1</diffuse><emissive>0.2 1.0 0.2 1</emissive></material>
        </visual>''')

    xml_lines.append('      </link>')
    xml_lines.append('    </model>')
    return '\n'.join(xml_lines)


def _generate_crater_science_stake_xml() -> str:
    """Generate ABEx Scientific Exploration survey stake on the crater floor."""
    cx, cy = -25.0, 20.0
    cz = get_surface_elevation(cx, cy)
    return f'''    <!-- ABEx Crater Science Outcrop Survey Stake -->
    <model name="crater_science_marker">
      <static>true</static>
      <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
      <link name="link">
        <visual name="stake_pole">
          <pose>0 0 0.9 0 0 0</pose>
          <geometry><cylinder><radius>0.025</radius><length>1.8</length></cylinder></geometry>
          <material><ambient>0.8 0.6 0.1 1</ambient><diffuse>0.95 0.7 0.1 1</diffuse></material>
        </visual>
        <visual name="science_flag">
          <pose>0.25 0 1.6 0 0 0</pose>
          <geometry><box><size>0.5 0.02 0.3</size></box></geometry>
          <material><ambient>0.9 0.8 0.1 1</ambient><diffuse>1.0 0.9 0.2 1</diffuse><emissive>0.4 0.3 0.05 1</emissive></material>
        </visual>
        <visual name="beacon_purple">
          <pose>0 0 1.85 0 0 0</pose>
          <geometry><sphere><radius>0.09</radius></sphere></geometry>
          <material><ambient>0.7 0.1 0.9 1</ambient><diffuse>0.8 0.2 1.0 1</diffuse><emissive>0.8 0.2 1.0 1</emissive></material>
        </visual>
      </link>
    </model>'''


def generate_world_sdf(heightmap_rel_path: str, z_range_m: float, min_elevation_m: float = 0.0) -> str:
    """Generate the complete Gazebo Fortress SDF world for Sproscape with full arena assets."""
    # Checkpoints and delivery pads (embed 5mm to eliminate daylight/gap)
    p_alpha = compute_object_pose(-10.0, 35.0, 1.25, 1.25, 0.04, embed_depth=0.005)
    p_bravo = compute_object_pose(35.0, 35.0, 1.25, 1.25, 0.04, embed_depth=0.005)
    p_cache = compute_object_pose(15.0, -25.0, 1.20, 1.20, 0.05, embed_depth=0.005)
    p_idmo = compute_object_pose(-30.0, -10.0, 0.25, 0.60, 0.05, embed_depth=0.008)

    # Tool surface poses (embed 5mm)
    p_wrench = compute_object_pose(-28.0, -25.0, 0.21, 0.04, 0.05, embed_depth=0.005)
    p_hammer = compute_object_pose(-22.0, -20.0, 0.19, 0.07, 0.07, embed_depth=0.005, intrinsic_yaw=0.60)
    p_screwdriver = compute_object_pose(-5.0, 5.0, 0.14, 0.09, 0.05, embed_depth=0.005)
    p_toolbox = compute_object_pose(18.0, -5.0, 0.23, 0.15, 0.22, embed_depth=0.005)
    p_rock = compute_object_pose(5.0, 30.0, 0.13, 0.11, 0.18, embed_depth=0.02, intrinsic_yaw=0.40)

    # Obstacle avoidance test objects (properly seated / embedded 8-20cm into terrain)
    p_boulder_a = compute_object_pose(-25.0, -32.0, 1.20, 1.00, 1.80, embed_depth=0.15)
    p_cargo = compute_object_pose(-5.0, -15.0, 1.80, 0.90, 1.70, embed_depth=0.08, intrinsic_yaw=0.45)
    p_crag = compute_object_pose(8.0, 10.0, 1.10, 1.10, 2.50, embed_depth=0.20)
    p_pillar_1 = compute_object_pose(25.0, 12.0, 0.75, 0.75, 2.20, embed_depth=0.15)
    p_pillar_2 = compute_object_pose(30.0, 18.0, 0.90, 0.90, 2.00, embed_depth=0.15, intrinsic_yaw=0.50)
    p_debris = compute_object_pose(-15.0, -2.0, 2.10, 0.90, 0.50, embed_depth=0.10, intrinsic_yaw=0.80)

    # Natural scattered rock field boulders
    p_rock1 = compute_object_pose(-2.0, 6.0, 0.60, 0.50, 0.80, embed_depth=0.10)
    p_rock2 = compute_object_pose(18.0, 16.0, 0.70, 0.55, 0.90, embed_depth=0.12)
    p_rock3 = compute_object_pose(-14.0, 26.0, 0.75, 0.60, 1.00, embed_depth=0.15)
    p_rock4 = compute_object_pose(26.0, -8.0, 0.55, 0.45, 0.70, embed_depth=0.10)
    p_rock5 = compute_object_pose(-34.0, -4.0, 0.40, 0.35, 0.50, embed_depth=0.08)

    red_zone_1_xml = _generate_red_zone_pylons_xml('red_zone_1_pylons', 10.0, 22.0, 6.5, num_pylons=12)
    red_zone_2_xml = _generate_red_zone_pylons_xml('red_zone_2_pylons', -36.0, 34.0, 7.5, num_pylons=14)
    base_station_xml = _generate_base_station_markers_xml('base_station_markers')
    crater_stake_xml = _generate_crater_science_stake_xml()

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

    <!-- Default GUI user camera view framing spawn area and entire arena -->
    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>0.0 -78.0 62.0 0 0.58 1.5708</pose>
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
      <pose>0 0 0 0 0 0</pose>
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
        <!-- South Wall (y = -60.0) -->
        <collision name="wall_south_col">
          <pose>0.0 -60.0 1.25 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_south_vis">
          <pose>0.0 -60.0 1.25 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- North Wall (y = +60.0) -->
        <collision name="wall_north_col">
          <pose>0.0 60.0 1.25 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_north_vis">
          <pose>0.0 60.0 1.25 0 0 0</pose>
          <geometry><box><size>120.8 0.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- West Wall (x = -60.0) -->
        <collision name="wall_west_col">
          <pose>-60.0 0.0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_west_vis">
          <pose>-60.0 0.0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
        <!-- East Wall (x = +60.0) -->
        <collision name="wall_east_col">
          <pose>60.0 0.0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
        </collision>
        <visual name="wall_east_vis">
          <pose>60.0 0.0 1.25 0 0 0</pose>
          <geometry><box><size>0.8 120.8 2.5</size></box></geometry>
          <material>
            <ambient>0.15 0.12 0.10 1</ambient>
            <diffuse>0.28 0.22 0.18 1</diffuse>
            <specular>0.05 0.05 0.05 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- 2. Base Station Perimeter Survey Markers & Start Gate -->
{base_station_xml}

    <!-- 3. Delivery Marker Alpha: Green target pad with concentric rings and 1.5m beacon -->
    <model name="marker_drop_alpha">
      <static>true</static>
      <pose>-10.0 35.0 {p_alpha[0]:.3f} {p_alpha[1]:.4f} {p_alpha[2]:.4f} {p_alpha[3]:.4f}</pose>
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

    <!-- 4. Delivery Marker Bravo: Blue target pad with concentric rings and 1.5m beacon -->
    <model name="marker_drop_bravo">
      <static>true</static>
      <pose>35.0 35.0 {p_bravo[0]:.3f} {p_bravo[1]:.4f} {p_bravo[2]:.4f} {p_bravo[3]:.4f}</pose>
      <link name="link">
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

    <!-- 5. ABEx Science Sample Cache Platform with orange/white markings -->
    <model name="abex_cache_platform">
      <static>true</static>
      <pose>15.0 -25.0 {p_cache[0]:.3f} {p_cache[1]:.4f} {p_cache[2]:.4f} {p_cache[3]:.4f}</pose>
      <link name="link">
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
            <emissive>1.0 0.55 0.0 1</emissive>
          </material>
        </visual>
      </link>
    </model>

    <!-- 6. IDMO Maintenance Service Panel Stand (1.8m vertical mock service panel) -->
    <model name="idmo_maintenance_panel">
      <static>true</static>
      <pose>-30.0 -10.0 {p_idmo[0]:.3f} {p_idmo[1]:.4f} {p_idmo[2]:.4f} {p_idmo[3]:.4f}</pose>
      <link name="link">
        <collision name="base_col">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>0.5 1.2 0.05</size></box></geometry>
        </collision>
        <visual name="base_vis">
          <pose>0 0 0.025 0 0 0</pose>
          <geometry><box><size>0.5 1.2 0.05</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.25 0.25 0.28 1</diffuse></material>
        </visual>
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
        <visual name="top_bar_vis">
          <pose>0 0 1.78 0 0 0</pose>
          <geometry><box><size>0.08 1.0 0.06</size></box></geometry>
          <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.3 0.3 0.35 1</diffuse></material>
        </visual>
        <collision name="panel_col">
          <pose>0.05 0 1.25 0 0 0</pose>
          <geometry><box><size>0.15 0.8 0.6</size></box></geometry>
        </collision>
        <visual name="panel_face">
          <pose>0.05 0 1.25 0 0 0</pose>
          <geometry><box><size>0.15 0.8 0.6</size></box></geometry>
          <material><ambient>0.25 0.25 0.28 1</ambient><diffuse>0.35 0.35 0.4 1</diffuse></material>
        </visual>
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
        <visual name="estop_button">
          <pose>0.135 0.0 1.1 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.035</radius><length>0.04</length></cylinder></geometry>
          <material><ambient>0.8 0.05 0.05 1</ambient><diffuse>0.95 0.05 0.05 1</diffuse></material>
        </visual>
      </link>
    </model>

    <!-- 7. ABEx Crater Science Exploration Survey Stake -->
{crater_stake_xml}

    <!-- 8. Red Danger Contamination Zones (Pylons & Warning Barriers) -->
{red_zone_1_xml}

{red_zone_2_xml}

    <!-- 9. Competition Tools and Geological Specimens -->
    <model name="tool_heavy_wrench">
      <static>true</static>
      <pose>-28.0 -25.0 {p_wrench[0]:.3f} {p_wrench[1]:.4f} {p_wrench[2]:.4f} {p_wrench[3]:.4f}</pose>
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
      <pose>-22.0 -20.0 {p_hammer[0]:.3f} {p_hammer[1]:.4f} {p_hammer[2]:.4f} {p_hammer[3]:.4f}</pose>
      <link name="link">
        <inertial><mass>1.5</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz></inertia></inertial>
        <collision name="hammer_col">
          <pose>0 0 0.022 0 1.5708 0</pose>
          <geometry><cylinder><radius>0.022</radius><length>0.38</length></cylinder></geometry>
        </collision>
        <visual name="handle_wood">
          <pose>-0.05 0 0.022 0 1.5708 0</pose>
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
      <pose>-5.0 5.0 {p_screwdriver[0]:.3f} {p_screwdriver[1]:.4f} {p_screwdriver[2]:.4f} {p_screwdriver[3]:.4f}</pose>
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
      <pose>18.0 -5.0 {p_toolbox[0]:.3f} {p_toolbox[1]:.4f} {p_toolbox[2]:.4f} {p_toolbox[3]:.4f}</pose>
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
      <pose>5.0 30.0 {p_rock[0]:.3f} {p_rock[1]:.4f} {p_rock[2]:.4f} {p_rock[3]:.4f}</pose>
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
         LARGE OBSTACLE AVOIDANCE TEST ASSETS (FIRM GROUND SEATING)
         ===================================================================== -->

    <!-- Obstacle 1: Monolithic Boulder Formation (transit corridor between Spawn and IDMO) -->
    <model name="obstacle_monolith_boulder_a">
      <static>true</static>
      <pose>-25.0 -32.0 {p_boulder_a[0]:.3f} {p_boulder_a[1]:.4f} {p_boulder_a[2]:.4f} {p_boulder_a[3]:.4f}</pose>
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
      <pose>-5.0 -15.0 {p_cargo[0]:.3f} {p_cargo[1]:.4f} {p_cargo[2]:.4f} {p_cargo[3]:.4f}</pose>
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
        <visual name="hazard_stripe_top">
          <pose>0 0 1.71 0 0 0</pose>
          <geometry><box><size>3.6 0.25 0.02</size></box></geometry>
          <material><ambient>0.8 0.6 0.05 1</ambient><diffuse>0.95 0.8 0.05 1</diffuse></material>
        </visual>
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
      <pose>8.0 10.0 {p_crag[0]:.3f} {p_crag[1]:.4f} {p_crag[2]:.4f} {p_crag[3]:.4f}</pose>
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
      <pose>25.0 12.0 {p_pillar_1[0]:.3f} {p_pillar_1[1]:.4f} {p_pillar_1[2]:.4f} {p_pillar_1[3]:.4f}</pose>
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
      <pose>30.0 18.0 {p_pillar_2[0]:.3f} {p_pillar_2[1]:.4f} {p_pillar_2[2]:.4f} {p_pillar_2[3]:.4f}</pose>
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
      <pose>-15.0 -2.0 {p_debris[0]:.3f} {p_debris[1]:.4f} {p_debris[2]:.4f} {p_debris[3]:.4f}</pose>
      <link name="link">
        <collision name="footing_col">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><cylinder><radius>0.9</radius><length>0.5</length></cylinder></geometry>
        </collision>
        <visual name="footing_vis">
          <pose>0 0 0.25 0 0 0</pose>
          <geometry><cylinder><radius>0.9</radius><length>0.5</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.25 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material>
        </visual>
        <collision name="beam_col">
          <pose>1.8 0 0.45 0.1 0.18 0</pose>
          <geometry><box><size>4.2 0.55 0.55</size></box></geometry>
        </collision>
        <visual name="beam_vis">
          <pose>1.8 0 0.45 0.1 0.18 0</pose>
          <geometry><box><size>4.2 0.55 0.55</size></box></geometry>
          <material><ambient>0.7 0.3 0.05 1</ambient><diffuse>0.85 0.35 0.05 1</diffuse></material>
        </visual>
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

    <!-- 10. Natural Planetary Rock Field (Scattered Basalt Boulders) -->
    <model name="rock_field_boulder_1">
      <static>true</static>
      <pose>-2.0 6.0 {p_rock1[0]:.3f} {p_rock1[1]:.4f} {p_rock1[2]:.4f} {p_rock1[3]:.4f}</pose>
      <link name="link">
        <collision name="col"><pose>0 0 0.4 0 0 0</pose><geometry><box><size>1.2 1.0 0.8</size></box></geometry></collision>
        <visual name="vis"><pose>0 0 0.4 0 0 0</pose><geometry><box><size>1.2 1.0 0.8</size></box></geometry><material><ambient>0.14 0.12 0.10 1</ambient><diffuse>0.26 0.20 0.16 1</diffuse></material></visual>
      </link>
    </model>

    <model name="rock_field_boulder_2">
      <static>true</static>
      <pose>18.0 16.0 {p_rock2[0]:.3f} {p_rock2[1]:.4f} {p_rock2[2]:.4f} {p_rock2[3]:.4f}</pose>
      <link name="link">
        <collision name="col"><pose>0 0 0.45 0 0 0.4</pose><geometry><box><size>1.4 1.1 0.9</size></box></geometry></collision>
        <visual name="vis"><pose>0 0 0.45 0 0 0.4</pose><geometry><box><size>1.4 1.1 0.9</size></box></geometry><material><ambient>0.16 0.13 0.11 1</ambient><diffuse>0.28 0.22 0.18 1</diffuse></material></visual>
      </link>
    </model>

    <model name="rock_field_ejecta">
      <static>true</static>
      <pose>-14.0 26.0 {p_rock3[0]:.3f} {p_rock3[1]:.4f} {p_rock3[2]:.4f} {p_rock3[3]:.4f}</pose>
      <link name="link">
        <collision name="col"><pose>0 0 0.5 0.1 0.1 0.6</pose><geometry><box><size>1.5 1.2 1.0</size></box></geometry></collision>
        <visual name="vis"><pose>0 0 0.5 0.1 0.1 0.6</pose><geometry><box><size>1.5 1.2 1.0</size></box></geometry><material><ambient>0.12 0.10 0.08 1</ambient><diffuse>0.22 0.18 0.14 1</diffuse></material></visual>
      </link>
    </model>

    <model name="rock_field_dune_spur">
      <static>true</static>
      <pose>26.0 -8.0 {p_rock4[0]:.3f} {p_rock4[1]:.4f} {p_rock4[2]:.4f} {p_rock4[3]:.4f}</pose>
      <link name="link">
        <collision name="col"><pose>0 0 0.35 0 0 0</pose><geometry><box><size>1.1 0.9 0.7</size></box></geometry></collision>
        <visual name="vis"><pose>0 0 0.35 0 0 0</pose><geometry><box><size>1.1 0.9 0.7</size></box></geometry><material><ambient>0.18 0.14 0.12 1</ambient><diffuse>0.30 0.24 0.20 1</diffuse></material></visual>
      </link>
    </model>

    <model name="rock_field_plain_stone">
      <static>true</static>
      <pose>-34.0 -4.0 {p_rock5[0]:.3f} {p_rock5[1]:.4f} {p_rock5[2]:.4f} {p_rock5[3]:.4f}</pose>
      <link name="link">
        <collision name="col"><pose>0 0 0.25 0 0 0</pose><geometry><box><size>0.8 0.7 0.5</size></box></geometry></collision>
        <visual name="vis"><pose>0 0 0.25 0 0 0</pose><geometry><box><size>0.8 0.7 0.5</size></box></geometry><material><ambient>0.15 0.12 0.10 1</ambient><diffuse>0.25 0.20 0.16 1</diffuse></material></visual>
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
    zones_path = os.path.join(args.output_dir, 'sproscape_zones.yaml')

    # 1. Heightmap PNG
    png_bytes, meta = generate_heightmap(dim=HEIGHTMAP_DIM, size_m=ARENA_SIZE_M)
    write_grayscale_png(png_path, HEIGHTMAP_DIM, HEIGHTMAP_DIM, png_bytes)
    print(f"Generated {png_path} ({HEIGHTMAP_DIM}x{HEIGHTMAP_DIM}, z-range: {meta['elevation_range_m']}m)")

    # 2. Textures for Ogre2 rendering (256x256 Martian soil texture for crisp visuals)
    tex_dir = os.path.join(args.output_dir, 'materials', 'textures')
    os.makedirs(tex_dir, exist_ok=True)
    soil_tex_path = os.path.join(tex_dir, 'martian_soil.png')
    normal_tex_path = os.path.join(tex_dir, 'flat_normal.png')

    soil_bytes = bytearray()
    for y in range(256):
        for x in range(256):
            g1 = math.sin(x * 0.15) * math.cos(y * 0.15) * 14.0
            g2 = math.sin(x * 0.45 + 1.2) * math.cos(y * 0.38 - 0.7) * 9.0
            g3 = math.sin(x * 1.1 - y * 0.85) * 6.0
            grain = g1 + g2 + g3
            r = min(255, max(0, int(185 + grain)))
            g = min(255, max(0, int(95 + grain * 0.55)))
            b = min(255, max(0, int(60 + grain * 0.35)))
            soil_bytes.extend([r, g, b])
    write_rgb_png(soil_tex_path, 256, 256, bytes(soil_bytes))

    # Flat normal map (128, 128, 255)
    normal_bytes = bytearray([128, 128, 255] * (64 * 64))
    write_rgb_png(normal_tex_path, 64, 64, bytes(normal_bytes))
    print(f"Generated terrain textures in {tex_dir}")

    # 3. Main Sproscape World SDF (Strictly exactly one world file for clean repo cloning)
    world_sdf = generate_world_sdf('sproscape_heightmap.png', meta['elevation_range_m'], min_elevation_m=meta['min_elevation_m'])
    with open(world_path, 'w') as f:
        f.write(world_sdf)
    print(f"Generated {world_path}")

    # 4. Arena zones layout YAML
    zones_data = {
        'datum': {'lat': 12.8614, 'lon': 77.4380, 'alt': 860.0},
        'arena_size_m': ARENA_SIZE_M,
        'base_station': {
            'x': -48.0, 'y': -48.0,
            'bounds': {'x_min': -56.0, 'x_max': -38.0, 'y_min': -56.0, 'y_max': -38.0},
            'name': 'Base Station Staging & Survey Area'
        },
        'geological_zones': [
            {'id': 'ZONE_PROMONTORY', 'name': 'Promontory Terraced Ridge', 'peak_x': 28.0, 'peak_y': 26.0, 'max_elevation_m': 5.5},
            {'id': 'ZONE_CRATER', 'name': 'Amphi-Crater Complex', 'center_x': -25.0, 'center_y': 20.0, 'radius_m': 16.5, 'rim_relief_m': 1.44},
            {'id': 'ZONE_DUNES', 'name': 'Barchan Sand Dune Field', 'center_x': 20.0, 'center_y': -18.0, 'wave_height_m': 2.2},
            {'id': 'ZONE_GULLY', 'name': 'Sinuous Outflow Wash Channel', 'center_x': -5.0, 'center_y': 5.0, 'width_m': 5.5},
        ],
        'delivery_markers': [
            {'id': 'DROP_ALPHA', 'name': 'Marker Alpha (Valley Green Pad)', 'x': -10.0, 'y': 35.0},
            {'id': 'DROP_BRAVO', 'name': 'Marker Bravo (Promontory Blue Pad)', 'x': 35.0, 'y': 35.0},
            {'id': 'DROP_CACHE', 'name': 'ABEx Cache Drop / Pickup (Dune Pass)', 'x': 15.0, 'y': -25.0},
        ],
        'rado_objects': [
            {'id': 'TOOL_1', 'name': 'Heavy Wrench (≤7cm dia)', 'x': -28.0, 'y': -25.0, 'mass_kg': 1.8},
            {'id': 'TOOL_HAMMER', 'name': 'Hammer / Mallet', 'x': -22.0, 'y': -20.0, 'mass_kg': 1.5},
            {'id': 'TOOL_2', 'name': 'Screwdriver Set', 'x': -5.0, 'y': 5.0, 'mass_kg': 0.9},
            {'id': 'BOX_1',  'name': 'Instrument Toolbox', 'x': 18.0, 'y': -5.0, 'mass_kg': 4.5},
            {'id': 'ROCK_1', 'name': 'Basalt Specimen A', 'x': 5.0, 'y': 30.0, 'mass_kg': 3.2},
        ],
        'abex_sites': [
            {'id': 'ABEX_SITE_A', 'name': 'Crater Stratigraphy Outcrop', 'x': -25.0, 'y': 20.0},
            {'id': 'ABEX_SITE_B', 'name': 'Fine-Soil Dune Basin', 'x': 40.0, 'y': -20.0},
        ],
        'idmo_panel': {'x': -30.0, 'y': -10.0, 'name': 'Mock Maintenance Panel (1.8m)'},
        'red_zones': [
            {'id': 'RED_ZONE_1', 'name': 'Contaminated Zone North Trench', 'center_x': 10.0, 'center_y': 22.0, 'radius_m': 6.5},
            {'id': 'RED_ZONE_2', 'name': 'Contaminated Crater Rift', 'center_x': -36.0, 'center_y': 34.0, 'radius_m': 7.5},
        ],
        'obstacles': [
            {'id': 'OBSTACLE_BOULDER_A', 'name': 'Monolithic Boulder Formation', 'x': -25.0, 'y': -32.0, 'size': [2.4, 2.0, 1.8]},
            {'id': 'OBSTACLE_HABITAT_MODULE', 'name': 'Habitat Cargo Module', 'x': -5.0, 'y': -15.0, 'size': [3.6, 1.8, 1.7]},
            {'id': 'OBSTACLE_CRAG_B', 'name': 'Massive Crag Spire & Buttress', 'x': 8.0, 'y': 10.0, 'size': [2.5, 2.2, 2.5]},
            {'id': 'OBSTACLE_SLALOM_1', 'name': 'Slalom Pillar 1', 'x': 25.0, 'y': 12.0, 'size': [1.5, 1.5, 2.2]},
            {'id': 'OBSTACLE_SLALOM_2', 'name': 'Slalom Pillar 2', 'x': 30.0, 'y': 18.0, 'size': [1.8, 1.8, 2.0]},
            {'id': 'OBSTACLE_COMM_MAST', 'name': 'Decommissioned Comm Mast Debris', 'x': -15.0, 'y': -2.0, 'size': [4.2, 1.8, 0.9]},
        ],
        'natural_rock_field': [
            {'id': 'ROCK_GULLY_1', 'name': 'Gully Wash Rock', 'x': -2.0, 'y': 6.0, 'size': [1.2, 1.0, 0.8]},
            {'id': 'ROCK_HILL_2', 'name': 'Promontory Slope Rock', 'x': 18.0, 'y': 16.0, 'size': [1.4, 1.1, 0.9]},
            {'id': 'ROCK_CRATER_EJECTA', 'name': 'Crater Ejecta Boulder', 'x': -14.0, 'y': 26.0, 'size': [1.5, 1.2, 1.0]},
            {'id': 'ROCK_DUNE_SPUR', 'name': 'Dune Pass Guide Rock', 'x': 26.0, 'y': -8.0, 'size': [1.1, 0.9, 0.7]},
            {'id': 'ROCK_PLAIN_STONE', 'name': 'West Plain Rock', 'x': -34.0, 'y': -4.0, 'size': [0.8, 0.7, 0.5]},
        ],
    }
    with open(zones_path, 'w') as f:
        yaml.dump(zones_data, f, sort_keys=False)
    print(f"Generated {zones_path}")


if __name__ == '__main__':
    main()
