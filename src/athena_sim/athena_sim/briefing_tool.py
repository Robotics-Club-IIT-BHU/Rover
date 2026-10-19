#!/usr/bin/env python3
"""
briefing_tool.py

IRC 2027 Operator Mission Briefing & Waypoint Generator.

Transforms Sproscape-Sim arena cartesian coordinates (meters relative to the start line)
into official WGS-84 decimal degree coordinates using the competition datum:
  - Venue Datum: Dayananda Sagar University, Bengaluru (12.8614°N, 77.4380°E, 860 m ASL)
  - Exports an operator mission briefing document with targets, zones, and rules.
  - Generates a waypoint YAML file ready for athena_gps_nav's gps_waypoint_follower.
"""

import math
import os
import sys
from typing import Dict, List, Tuple
import yaml

R_EARTH = 6378137.0


def meter_offsets_to_latlon(
    datum_lat: float, datum_lon: float, dx_east: float, dy_north: float
) -> Tuple[float, float]:
    """Convert local ENU offsets (meters) to WGS-84 latitude and longitude (decimal degrees)."""
    lat_rad = math.radians(datum_lat)
    d_lat_deg = math.degrees(dy_north / R_EARTH)
    d_lon_deg = math.degrees(dx_east / (R_EARTH * max(math.cos(lat_rad), 1e-6)))
    return datum_lat + d_lat_deg, datum_lon + d_lon_deg


def latlon_to_meter_offsets(
    datum_lat: float, datum_lon: float, target_lat: float, target_lon: float
) -> Tuple[float, float]:
    """Convert WGS-84 latitude and longitude back to local ENU offsets (meters)."""
    lat_rad = math.radians(datum_lat)
    dy_north = math.radians(target_lat - datum_lat) * R_EARTH
    dx_east = math.radians(target_lon - datum_lon) * R_EARTH * math.cos(lat_rad)
    return dx_east, dy_north


def generate_briefing(
    zones_yaml_path: str,
    datum_lat: float = 12.8614,
    datum_lon: float = 77.4380,
    output_md_path: Optional[str] = None,
    output_waypoints_yaml_path: Optional[str] = None,
) -> Dict:
    """
    Load arena zones YAML, calculate WGS-84 coordinates for all targets,
    and generate briefing artifacts.
    """
    if os.path.exists(zones_yaml_path):
        with open(zones_yaml_path, 'r') as f:
            data = yaml.safe_load(f)
    else:
        # Default Sproscape arena layout (120m x 120m centered at 0, 0)
        data = {
            'datum': {'lat': datum_lat, 'lon': datum_lon, 'alt': 860.0},
            'start_line': {'x': -48.0, 'y': -48.0, 'name': 'Base Station Start Gate'},
            'rado_objects': [
                {'id': 'TOOL_1', 'name': 'Heavy Wrench (≤7cm dia)', 'x': -28.0, 'y': -25.0, 'mass_kg': 1.8},
                {'id': 'TOOL_HAMMER', 'name': 'Hammer / Mallet', 'x': -22.0, 'y': -20.0, 'mass_kg': 1.5},
                {'id': 'TOOL_2', 'name': 'Screwdriver Set', 'x': -5.0, 'y': 5.0, 'mass_kg': 0.9},
                {'id': 'BOX_1',  'name': 'Instrument Toolbox', 'x': 18.0, 'y': -5.0, 'mass_kg': 4.5},
                {'id': 'ROCK_1', 'name': 'Basalt Specimen A', 'x': 5.0, 'y': 30.0, 'mass_kg': 3.2},
            ],
            'delivery_markers': [
                {'id': 'DROP_ALPHA', 'name': 'Marker Alpha (Green Pad)', 'x': -10.0, 'y': 35.0},
                {'id': 'DROP_BRAVO', 'name': 'Marker Bravo (Blue Pad)',  'x': 35.0, 'y': 35.0},
                {'id': 'DROP_CACHE', 'name': 'ABEx Cache Drop / Pickup', 'x': 15.0, 'y': -25.0},
            ],
            'abex_sites': [
                {'id': 'ABEX_SITE_A', 'name': 'Stratigraphy Outcrop', 'x': 0.0, 'y': 42.0},
                {'id': 'ABEX_SITE_B', 'name': 'Fine-Soil Basin',       'x': 40.0, 'y': -20.0},
            ],
            'idmo_panel': {'x': -30.0, 'y': -10.0, 'name': 'Mock Maintenance Panel (1.8m)'},
            'red_zones': [
                {'id': 'RED_ZONE_1', 'name': 'Contaminated Zone North Trench', 'center_x': 10.0, 'center_y': 20.0, 'radius_m': 6.0},
                {'id': 'RED_ZONE_2', 'name': 'Contaminated Crater Rift', 'center_x': -35.0, 'center_y': 32.0, 'radius_m': 7.5},
            ],
        }

    datum_lat = data.get('datum', {}).get('lat', datum_lat)
    datum_lon = data.get('datum', {}).get('lon', datum_lon)

    # Compute WGS-84 coordinates for each feature
    briefing = {'datum': {'lat': datum_lat, 'lon': datum_lon}, 'features': {}}

    waypoints_list = []

    # Delivery targets
    briefing['delivery_markers'] = []
    for item in data.get('delivery_markers', []):
        lat, lon = meter_offsets_to_latlon(datum_lat, datum_lon, item['x'], item['y'])
        entry = {**item, 'lat': round(lat, 7), 'lon': round(lon, 7)}
        briefing['delivery_markers'].append(entry)
        waypoints_list.append({
            'name': item['id'],
            'latitude': round(lat, 7),
            'longitude': round(lon, 7),
            'yaw': 0.0,
        })

    # RADO Recon Objects
    briefing['rado_objects'] = []
    for item in data.get('rado_objects', []):
        lat, lon = meter_offsets_to_latlon(datum_lat, datum_lon, item['x'], item['y'])
        briefing['rado_objects'].append({**item, 'lat': round(lat, 7), 'lon': round(lon, 7)})

    # Science Sites
    briefing['abex_sites'] = []
    for item in data.get('abex_sites', []):
        lat, lon = meter_offsets_to_latlon(datum_lat, datum_lon, item['x'], item['y'])
        briefing['abex_sites'].append({**item, 'lat': round(lat, 7), 'lon': round(lon, 7)})

    # Red No-Go Zones
    briefing['red_zones'] = []
    for rz in data.get('red_zones', []):
        lat, lon = meter_offsets_to_latlon(datum_lat, datum_lon, rz['center_x'], rz['center_y'])
        briefing['red_zones'].append({**rz, 'center_lat': round(lat, 7), 'center_lon': round(lon, 7)})

    # IDMO Panel
    if 'idmo_panel' in data:
        p = data['idmo_panel']
        lat, lon = meter_offsets_to_latlon(datum_lat, datum_lon, p['x'], p['y'])
        briefing['idmo_panel'] = {**p, 'lat': round(lat, 7), 'lon': round(lon, 7)}

    # Generate Markdown Briefing Text
    md_lines = [
        "# IRC 2027 Field Mission Briefing — Sproscape Arena",
        "",
        f"**Venue GPS Datum**: {datum_lat:.6f}°N, {datum_lon:.6f}°E  ",
        "**Base Station Link**: Foxglove Bridge over Telemetry (Zero Direct Line-of-Sight)  ",
        "**Mission Duration**: 25 Minutes Maximum | Checkpoint: 7 Min (≥1 pt required) | 15 Min (>30% unlock)  ",
        "",
        "---",
        "",
        "## 1. Designated Autonomous Delivery Locations (RADO)",
        "",
        "| Marker ID | Name | Local X (m) | Local Y (m) | Latitude (°N) | Longitude (°E) |",
        "|---|---|---|---|---|---|",
    ]
    for m in briefing['delivery_markers']:
        md_lines.append(f"| **{m['id']}** | {m['name']} | {m['x']:.1f} | {m['y']:.1f} | `{m['lat']:.7f}` | `{m['lon']:.7f}` |")

    md_lines.extend([
        "",
        "## 2. Reconnaissance Target Objects (Search & Document)",
        "",
        "| Object ID | Description | Expected Approx X, Y | Approx Lat, Lon | Mass |",
        "|---|---|---|---|---|",
    ])
    for o in briefing['rado_objects']:
        md_lines.append(f"| **{o['id']}** | {o['name']} | ({o['x']:.1f}, {o['y']:.1f}) | `{o['lat']:.7f}, {o['lon']:.7f}` | {o.get('mass_kg', 1.0)} kg |")

    md_lines.extend([
        "",
        "## 3. Astrobiology Expedition Sites (ABEx)",
        "",
        "| Site ID | Site Type | Local X, Y | Target Coordinates |",
        "|---|---|---|---|",
    ])
    for s in briefing['abex_sites']:
        md_lines.append(f"| **{s['id']}** | {s['name']} | ({s['x']:.1f}, {s['y']:.1f}) | `{s['lat']:.7f}°N, {s['lon']:.7f}°E` |")

    md_lines.extend([
        "",
        "## 4. Contaminated No-Go Zones (10% Penalty per Infraction)",
        "",
        "| Zone ID | Description | Center Lat, Lon | Radius | Penalty |",
        "|---|---|---|---|---|",
    ])
    for rz in briefing['red_zones']:
        md_lines.append(f"| **{rz['id']}** | {rz['name']} | `{rz['center_lat']:.7f}, {rz['center_lon']:.7f}` | {rz['radius_m']:.1f} m | -10% of total mission score |")

    md_lines.extend([
        "",
        "## 5. Instrument Maintenance Panel (IDMO)",
        "",
        f"- Location: `{briefing.get('idmo_panel', {}).get('lat', 0.0):.7f}°N, {briefing.get('idmo_panel', {}).get('lon', 0.0):.7f}°E`  ",
        "- Task: Approach within 0.8m, orient optical axis normal to panel face, capture code board.",
        "",
    ])

    md_content = "\n".join(md_lines)

    if output_md_path:
        os.makedirs(os.path.dirname(os.path.abspath(output_md_path)), exist_ok=True)
        with open(output_md_path, 'w') as f:
            f.write(md_content)

    if output_waypoints_yaml_path:
        os.makedirs(os.path.dirname(os.path.abspath(output_waypoints_yaml_path)), exist_ok=True)
        wp_yaml_content = yaml.dump({'waypoints': waypoints_list}, sort_keys=False)
        with open(output_waypoints_yaml_path, 'w') as f:
            f.write(wp_yaml_content)

    return briefing


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Generate IRC 2027 mission briefing and waypoints.")
    parser.add_argument('--zones', default='worlds/sproscape_zones.yaml', help='Path to arena zones YAML')
    parser.add_argument('--out-md', default='docs/MISSION_BRIEFING.md', help='Output Markdown briefing path')
    parser.add_argument('--out-waypoints', default='config/delivery_waypoints.yaml', help='Output waypoints YAML')
    parser.add_argument('--lat', type=float, default=12.8614, help='Venue datum latitude')
    parser.add_argument('--lon', type=float, default=77.4380, help='Venue datum longitude')
    args = parser.parse_args()

    briefing = generate_briefing(
        zones_yaml_path=args.zones,
        datum_lat=args.lat,
        datum_lon=args.lon,
        output_md_path=args.out_md,
        output_waypoints_yaml_path=args.out_waypoints,
    )
    print(f"Generated briefing for {len(briefing.get('delivery_markers', []))} delivery markers and {len(briefing.get('red_zones', []))} red zones.")


if __name__ == '__main__':
    main()
