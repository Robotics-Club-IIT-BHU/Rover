#!/usr/bin/env python3
import sys
import os
import math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.colors import LightSource, LinearSegmentedColormap

sys.path.insert(0, '/home/jashan/Rover/src/athena_sim')
from athena_sim.generate_terrain import get_surface_elevation

# 1. Compute high-resolution elevation grid (120m x 120m centered at 0, 0)
res = 0.25  # 0.25m per grid cell -> 481 x 481 grid
xs = np.arange(-60.0, 60.0 + res * 0.5, res)
ys = np.arange(-60.0, 60.0 + res * 0.5, res)
X, Y = np.meshgrid(xs, ys)

print(f"Generating elevation grid: {X.shape[0]}x{X.shape[1]} points...")
Z = np.zeros_like(X)
for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        Z[i, j] = get_surface_elevation(float(X[i, j]), float(Y[i, j]))

print(f"Grid generated: min Z = {Z.min():.3f}m, max Z = {Z.max():.3f}m")

# 2. Setup figure
fig = plt.figure(figsize=(18, 18), dpi=200)
ax = fig.add_subplot(111)

# Custom Martian Colormap: Ochre, Terracotta, Sandy Red, Dusty Highlight
martian_colors = [
    (0.00, "#2b1a13"),  # Deep basal bedrock / crater basin
    (0.15, "#4a2c1d"),  # Dark martian soil
    (0.35, "#804323"),  # Terracotta regolith
    (0.60, "#b8632e"),  # Red sand dunes
    (0.85, "#d98845"),  # Sandy ridges
    (1.00, "#f4b97a"),  # High sunlit promontory crests
]
cmap = LinearSegmentedColormap.from_list("martian_sand", [c[1] for c in martian_colors])

# 3. Hillshading for realistic 3D relief
ls = LightSource(azdeg=315, altdeg=45)
rgb = ls.shade(Z, cmap=cmap, blend_mode='overlay', vert_exag=3.5, dx=res, dy=res)

im = ax.imshow(
    rgb,
    origin='lower',
    extent=[-60, 60, -60, 60],
    interpolation='bicubic'
)

# Elevation contours
contours = ax.contour(
    X, Y, Z,
    levels=np.arange(0.3, 5.5, 0.45),
    colors='white',
    alpha=0.25,
    linewidths=0.7
)
ax.clabel(contours, inline=True, fontsize=7, fmt='%.1fm', colors='white')

# 4. Arena Boundary Wall Border
wall_rect = patches.Rectangle(
    (-60.0, -60.0), 120.0, 120.0,
    linewidth=4.5,
    edgecolor='#e0e0e0',
    facecolor='none',
    linestyle='-',
    zorder=10,
    label='Perimeter Boundary Wall (120m x 120m)'
)
ax.add_patch(wall_rect)

# 5. Base Station Staging Zone & Survey Area [-56, -38] x [-56, -38]
spawn_rect = patches.Rectangle(
    (-56.0, -56.0), 18.0, 18.0,
    linewidth=2.5,
    edgecolor='#00e676',
    facecolor='#00e676',
    alpha=0.15,
    linestyle='--',
    zorder=5
)
ax.add_patch(spawn_rect)
# 4 Corner survey beacons
for cx, cy in [(-56, -56), (-38, -56), (-56, -38), (-38, -38)]:
    ax.plot(cx, cy, marker='s', markersize=9, color='#00e676', markeredgecolor='#ffffff', markeredgewidth=1.5, zorder=12)

ax.text(
    -47.0, -53.5,
    "BASE STATION STAGING ZONE\n([-56..-38m] x [-56..-38m] | Z = 0.000m)\n[Corner Survey Beacons Marked]",
    ha='center', va='center',
    fontsize=9, fontweight='bold',
    color='#00e676',
    bbox=dict(boxstyle='round,pad=0.35', facecolor='#111111', edgecolor='#00e676', alpha=0.88),
    zorder=12
)

# 6. Athena Rover Spawn Marker (-48.0, -48.0)
rover_marker = ax.plot(
    -48.0, -48.0,
    marker='o', markersize=14,
    color='#ffff00', markeredgecolor='#000000', markeredgewidth=2.0,
    zorder=15
)
ax.annotate(
    "ATHENA ROVER START GATE\n(-48.0, -48.0) | Z = 0.000m\nHeading: 45° NE [Gate Beacons]",
    xy=(-48.0, -48.0), xytext=(-42.0, -42.0),
    arrowprops=dict(facecolor='#ffff00', edgecolor='#000000', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#212121', edgecolor='#ffff00', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#ffff00',
    zorder=20
)

# 7. Red Danger Contamination Zones (Continuous Warning Barrier Beams & Central Towers)
def draw_red_zone(cx, cy, r, name, num_pylons=12):
    circle = patches.Circle(
        (cx, cy), r,
        facecolor='#d50000', edgecolor='#ff1744',
        linewidth=3.0, linestyle='-',
        alpha=0.30, zorder=6
    )
    ax.add_patch(circle)
    # Pylons
    for i in range(num_pylons):
        ang = 2.0 * math.pi * i / num_pylons
        px = cx + r * math.cos(ang)
        py = cy + r * math.sin(ang)
        ax.plot(px, py, marker='^', markersize=8, color='#ff1744', markeredgecolor='#ffffff', markeredgewidth=1.0, zorder=8)
    
    # Central Danger Tower
    ax.plot(cx, cy, marker='X', markersize=12, color='#ff1744', markeredgecolor='#ffffff', markeredgewidth=2.0, zorder=9)
    ax.annotate(
        f"{name}\nCenter: ({cx:.1f}, {cy:.1f}) | R = {r:.1f}m\n[Continuous Warning Barrier & Tower]",
        xy=(cx, cy), xytext=(cx + 3.0, cy + r + 3.0),
        arrowprops=dict(facecolor='#ff1744', edgecolor='#ffffff', width=1.5, headwidth=7.0, shrink=0.05),
        bbox=dict(boxstyle='round,pad=0.4', facecolor='#260000', edgecolor='#ff1744', linewidth=1.5, alpha=0.92),
        fontsize=9, fontweight='bold', color='#ff5252',
        zorder=20
    )

draw_red_zone(10.0, 22.0, 6.5, "RED HAZARD ZONE 1 (NORTH FAULT TRENCH)", num_pylons=12)
draw_red_zone(-36.0, 34.0, 7.5, "RED HAZARD ZONE 2 (CRATER NORTH RIFT)", num_pylons=14)

# 8. Delivery Checkpoints / Landing Pads
# Marker Alpha (-10.0, 35.0)
ax.plot(-10.0, 35.0, marker='o', markersize=18, color='#00e676', markeredgecolor='#ffffff', markeredgewidth=2.5, zorder=15)
ax.plot(-10.0, 35.0, marker='+', markersize=14, color='#ffffff', markeredgewidth=2.0, zorder=16)
ax.annotate(
    "DELIVERY MARKER ALPHA\n(DROP_ALPHA)\nCoord: (-10.0, 35.0)\nBeacon: 1.5m Emerald Green",
    xy=(-10.0, 35.0), xytext=(-26.0, 42.0),
    arrowprops=dict(facecolor='#00e676', edgecolor='#ffffff', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#002811', edgecolor='#00e676', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#69f0ae',
    zorder=20
)

# Marker Bravo (35.0, 35.0)
ax.plot(35.0, 35.0, marker='o', markersize=18, color='#2979ff', markeredgecolor='#ffffff', markeredgewidth=2.5, zorder=15)
ax.plot(35.0, 35.0, marker='+', markersize=14, color='#ffffff', markeredgewidth=2.0, zorder=16)
ax.annotate(
    "DELIVERY MARKER BRAVO\n(DROP_BRAVO)\nCoord: (35.0, 35.0) | Z = 4.4m\nBeacon: 1.5m Sapphire Blue (Summit Overlook)",
    xy=(35.0, 35.0), xytext=(22.0, 46.0),
    arrowprops=dict(facecolor='#2979ff', edgecolor='#ffffff', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#001538', edgecolor='#2979ff', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#82b1ff',
    zorder=20
)

# ABEx Cache Site (15.0, -25.0)
ax.plot(15.0, -25.0, marker='s', markersize=16, color='#ff9100', markeredgecolor='#ffffff', markeredgewidth=2.5, zorder=15)
ax.annotate(
    "ABEx CACHE PLATFORM\n(DROP_CACHE)\nCoord: (15.0, -25.0) | Z = 0.8m\nSample Drop / Pickup (Dune Pass)",
    xy=(15.0, -25.0), xytext=(22.0, -34.0),
    arrowprops=dict(facecolor='#ff9100', edgecolor='#ffffff', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#331600', edgecolor='#ff9100', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#ffd180',
    zorder=20
)

# IDMO Panel (-30.0, -10.0)
ax.plot(-30.0, -10.0, marker='P', markersize=16, color='#00e5ff', markeredgecolor='#ffffff', markeredgewidth=2.0, zorder=15)
ax.annotate(
    "IDMO SERVICE PANEL\nCoord: (-30.0, -10.0)\n1.8m Maintenance Console",
    xy=(-30.0, -10.0), xytext=(-45.0, -5.0),
    arrowprops=dict(facecolor='#00e5ff', edgecolor='#ffffff', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#002329', edgecolor='#00e5ff', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#84ffff',
    zorder=20
)

# 9. Competition Tools (RADO Items)
tools = [
    ("Heavy Wrench", -28.0, -25.0, "#ffeb3b", (-42.0, -28.0), "Heavy Wrench (TOOL_1)\n(-28.0, -25.0) | Mass: 1.8kg"),
    ("Hammer", -22.0, -20.0, "#ffc107", (-18.0, -24.0), "Hammer / Mallet (TOOL_HAMMER)\n(-22.0, -20.0) | Mass: 1.5kg"),
    ("Screwdriver Set", -5.0, 5.0, "#ff9800", (-18.0, 8.0), "Screwdriver Set (TOOL_2)\n(-5.0, 5.0) | Mass: 0.9kg"),
    ("Instrument Toolbox", 18.0, -5.0, "#ffa726", (24.0, -8.0), "Toolbox (BOX_1)\n(18.0, -5.0) | Mass: 4.5kg"),
    ("Basalt Specimen", 5.0, 30.0, "#b0bec5", (8.0, 36.0), "Basalt Specimen (ROCK_1)\n(5.0, 30.0) | Mass: 3.2kg"),
]

for name, tx, ty, col, text_pos, label_str in tools:
    ax.plot(tx, ty, marker='*', markersize=16, color=col, markeredgecolor='#000000', markeredgewidth=1.5, zorder=15)
    ax.annotate(
        label_str,
        xy=(tx, ty), xytext=text_pos,
        arrowprops=dict(facecolor=col, edgecolor='#000000', width=1.5, headwidth=6.0, shrink=0.08),
        bbox=dict(boxstyle='round,pad=0.35', facecolor='#1f1c11', edgecolor=col, linewidth=1.2, alpha=0.92),
        fontsize=8.5, fontweight='bold', color=col,
        zorder=20
    )

# 10. Large Obstacles & Natural Rock Field
obstacles = [
    ("Monolith Boulders", -25.0, -32.0, (-38.0, -37.0), "Obstacle: Monolith Boulders\n(-25.0, -32.0) | 2.4x2.0x1.8m"),
    ("Habitat Cargo Module", -5.0, -15.0, (-10.0, -10.0), "Obstacle: Habitat Cargo\n(-5.0, -15.0) | 3.6x1.8x1.7m"),
    ("Martian Rock Crag", 8.0, 10.0, (14.0, 6.0), "Obstacle: Crag Spire & Buttress\n(8.0, 10.0) | 2.5x2.2x2.5m"),
    ("Slalom Pillars", 27.5, 15.0, (32.0, 10.0), "Obstacle: Slalom Pillars (x2)\n(25.0, 12.0) & (30.0, 18.0)"),
    ("Comm Mast Debris", -15.0, -2.0, (-28.0, -2.0), "Obstacle: Comm Mast Debris\n(-15.0, -2.0) | 4.2x1.8x0.9m"),
]

for name, ox, oy, tpos, ostr in obstacles:
    ax.plot(ox, oy, marker='D', markersize=13, color='#e53935', markeredgecolor='#ffffff', markeredgewidth=1.8, zorder=15)
    ax.annotate(
        ostr,
        xy=(ox, oy), xytext=tpos,
        arrowprops=dict(facecolor='#e53935', edgecolor='#ffffff', width=1.5, headwidth=6.0, shrink=0.08),
        bbox=dict(boxstyle='round,pad=0.35', facecolor='#2b0f0f', edgecolor='#e53935', linewidth=1.2, alpha=0.90),
        fontsize=8.5, fontweight='bold', color='#ff8a80',
        zorder=20
    )

# Natural scattered boulders
natural_rocks = [(-2.0, 6.0), (18.0, 16.0), (-14.0, 26.0), (26.0, -8.0), (-34.0, -4.0)]
for rx, ry in natural_rocks:
    ax.plot(rx, ry, marker='h', markersize=8, color='#8d6e63', markeredgecolor='#ffffff', markeredgewidth=0.8, zorder=14)

# 11. Region Annotations (Impact Crater & Dunes & Promontory Ridge & Sinuous Gully)
# Impact Crater Center (-25.0, 20.0)
crater_circ = patches.Circle(
    (-25.0, 20.0), 16.5,
    facecolor='none', edgecolor='#b39ddb',
    linewidth=2.2, linestyle=':', alpha=0.85, zorder=4
)
ax.add_patch(crater_circ)
ax.plot(-25.0, 20.0, marker='p', markersize=12, color='#ba68c8', markeredgecolor='#ffffff', markeredgewidth=1.5, zorder=15)
ax.text(
    -25.0, 21.5, "AMPHI-CRATER COMPLEX (NW)\n(R = 16.5m | Relief ~1.44m | Center Mound)\n[ABEx Scientific Survey Stake Flag]",
    ha='center', va='center', fontsize=8.5, fontweight='bold',
    color='#e1bee7', alpha=0.90, zorder=6
)

# Sinuous Outflow Gully Wash
gully_ys = np.linspace(-25, 35, 100)
gully_xs = -5.0 + 10.0 * np.sin(gully_ys * 0.055 + 0.6)
ax.plot(gully_xs, gully_ys, color='#80deea', linestyle='-.', linewidth=2.0, alpha=0.65, zorder=7)
ax.text(
    -1.0, 2.0, "SINUOUS OUTFLOW GULLY WASH\n(Natural Sinuous Regolith Channel)",
    ha='center', va='center', fontsize=8.0, fontweight='bold',
    color='#80deea', alpha=0.85, rotation=65, zorder=8
)

# Sand Dune Field annotation
ax.text(
    22.0, -18.0, "ASYMMETRIC BARCHAN SAND DUNES (EAST)\n(Windward ~6°, Slip Face ~16°, Wave Height ~2.2m)\n[Testing Wheel Slip & Traction Control]",
    ha='center', va='center', fontsize=9, fontweight='bold',
    color='#ffe082', alpha=0.88,
    bbox=dict(boxstyle='round,pad=0.35', facecolor='#2b1a09', edgecolor='#ffe082', alpha=0.75),
    zorder=6
)

# Promontory Ridge NE
ax.text(
    32.0, 26.0, "PROMONTORY TERRACED RIDGE (NE)\n(Peak Elevation ~5.5m | Summit Overlook)\n[Grade Climbing & Rocker Articulation]",
    ha='center', va='center', fontsize=8.5, fontweight='bold',
    color='#cfd8dc', alpha=0.85,
    bbox=dict(boxstyle='round,pad=0.35', facecolor='#1b2124', edgecolor='#cfd8dc', alpha=0.75),
    zorder=6
)

# 12. Grid lines & axes styling
ax.set_xlim(-62.0, 62.0)
ax.set_ylim(-62.0, 62.0)
ax.set_xticks(np.arange(-60, 61, 10))
ax.set_yticks(np.arange(-60, 61, 10))
ax.set_xticks(np.arange(-60, 61, 5), minor=True)
ax.set_yticks(np.arange(-60, 61, 5), minor=True)
ax.grid(True, which='major', color='#ffffff', linestyle='-', alpha=0.25, linewidth=0.8)
ax.grid(True, which='minor', color='#ffffff', linestyle=':', alpha=0.12, linewidth=0.5)

ax.set_xlabel("East-West Coordinate (X) [meters]", fontsize=13, fontweight='bold', labelpad=10)
ax.set_ylabel("North-South Coordinate (Y) [meters]", fontsize=13, fontweight='bold', labelpad=10)
ax.tick_params(axis='both', which='major', labelsize=11)

# Compass / North Arrow
ax.annotate(
    "N", xy=(54.0, 54.0), xytext=(54.0, 46.0),
    arrowprops=dict(facecolor='#ffffff', edgecolor='#000000', width=3.0, headwidth=12.0),
    ha='center', va='center', fontsize=16, fontweight='bold', color='#ffffff',
    bbox=dict(boxstyle='circle,pad=0.3', facecolor='#111111', edgecolor='#ffffff', linewidth=1.5),
    zorder=25
)

# Title and header
title_str = (
    "IRC 2027 'SPROSCAPE-SIM' MARS ANALOG ARENA — TOP-DOWN ORTHOGRAPHIC MAP\n"
    "Procedural Terrain (120m x 120m centered at 0,0) | WGS-84 Datum (DSU, Bengaluru: 12.8614°N, 77.4380°E)\n"
    "C² Continuous Slopes (~6-16° grade, Peak 5.5m) | Zero-Gap Ground Contact | Perfectly Centered Physics"
)
ax.set_title(title_str, fontsize=13, fontweight='bold', pad=15, linespacing=1.4)

# Colorbar
norm = matplotlib.colors.Normalize(vmin=0.0, vmax=float(Z.max()))
sm = plt.cm.ScalarMappable(cmap=cmap, norm=norm)
sm.set_array([])
cbar = fig.colorbar(sm, ax=ax, fraction=0.035, pad=0.02, aspect=30)
cbar.set_label("Terrain Surface Elevation Z [meters]", fontsize=11, fontweight='bold', labelpad=10)
cbar.ax.tick_params(labelsize=10)

plt.tight_layout()

# Save image
out_file1 = "/home/jashan/Pictures/Screenshots/annotated_map_top_view.png"
out_file2 = "/home/jashan/Rover/src/athena_sim/worlds/annotated_map_top_view.png"
os.makedirs(os.path.dirname(out_file1), exist_ok=True)
os.makedirs(os.path.dirname(out_file2), exist_ok=True)

fig.savefig(out_file1, dpi=200, bbox_inches='tight')
fig.savefig(out_file2, dpi=200, bbox_inches='tight')
plt.close(fig)

print(f"SUCCESS: Generated annotated map images:")
print(f"  1. {out_file1}")
print(f"  2. {out_file2}")
