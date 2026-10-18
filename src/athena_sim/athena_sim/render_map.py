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

# 1. Compute high-resolution elevation grid (120m x 120m)
res = 0.25  # 0.25m per grid cell -> 481 x 481 grid
xs = np.arange(0.0, 120.0 + res * 0.5, res)
ys = np.arange(0.0, 120.0 + res * 0.5, res)
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
    (0.00, "#2b1a13"),  # Deep basal bedrock
    (0.15, "#4a2c1d"),  # Dark martian soil
    (0.35, "#804323"),  # Terracotta regolith
    (0.60, "#b8632e"),  # Red sand dunes
    (0.85, "#d98845"),  # Sandy ridges
    (1.00, "#f4b97a"),  # High sunlit crests
]
cmap = LinearSegmentedColormap.from_list("martian_sand", [c[1] for c in martian_colors])

# 3. Hillshading for realistic 3D relief
ls = LightSource(azdeg=315, altdeg=45)
rgb = ls.shade(Z, cmap=cmap, blend_mode='overlay', vert_exag=4.0, dx=res, dy=res)

im = ax.imshow(
    rgb,
    origin='lower',
    extent=[0, 120, 0, 120],
    interpolation='bicubic'
)

# Subtle elevation contours
contours = ax.contour(
    X, Y, Z,
    levels=np.arange(0.1, 1.4, 0.15),
    colors='white',
    alpha=0.22,
    linewidths=0.7
)
ax.clabel(contours, inline=True, fontsize=7, fmt='%.1fm', colors='white')

# 4. Arena Boundary Wall Border
wall_rect = patches.Rectangle(
    (0.0, 0.0), 120.0, 120.0,
    linewidth=4.5,
    edgecolor='#e0e0e0',
    facecolor='none',
    linestyle='-',
    zorder=10,
    label='Perimeter Boundary Wall (120m x 120m)'
)
ax.add_patch(wall_rect)

# 5. Base Station / Safe Spawn Flat Basin [0, 25] x [0, 20]
spawn_rect = patches.Rectangle(
    (0.0, 0.0), 25.0, 20.0,
    linewidth=2.0,
    edgecolor='#00e676',
    facecolor='#00e676',
    alpha=0.18,
    linestyle='--',
    zorder=5
)
ax.add_patch(spawn_rect)
ax.text(
    12.5, 3.5,
    "BASE STATION FLAT ZONE\n(0..25m x 0..20m | Z = 0.000m)",
    ha='center', va='center',
    fontsize=9, fontweight='bold',
    color='#00e676',
    bbox=dict(boxstyle='round,pad=0.3', facecolor='#111111', edgecolor='#00e676', alpha=0.85),
    zorder=12
)

# 6. Athena Rover Spawn Marker (8.0, 8.0)
rover_marker = ax.plot(
    8.0, 8.0,
    marker='o', markersize=14,
    color='#ffff00', markeredgecolor='#000000', markeredgewidth=2.0,
    zorder=15
)
ax.annotate(
    "ATHENA ROVER SPAWN\n(8.0, 8.0) | Z = 0.000m\nHeading: North (+Y)",
    xy=(8.0, 8.0), xytext=(12.0, 14.0),
    arrowprops=dict(facecolor='#ffff00', edgecolor='#000000', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#212121', edgecolor='#ffff00', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#ffff00',
    zorder=20
)

# 7. Red Danger Contamination Zones (Pylons & Circles)
def draw_red_zone(cx, cy, r, name, num_pylons=8):
    # Circle
    circle = patches.Circle(
        (cx, cy), r,
        facecolor='#d50000', edgecolor='#ff1744',
        linewidth=2.5, linestyle='--',
        alpha=0.25, zorder=6
    )
    ax.add_patch(circle)
    # Pylons
    for i in range(num_pylons):
        ang = 2.0 * math.pi * i / num_pylons
        px = cx + r * math.cos(ang)
        py = cy + r * math.sin(ang)
        ax.plot(px, py, marker='^', markersize=8, color='#ff1744', markeredgecolor='#ffffff', markeredgewidth=1.0, zorder=8)
    
    ax.annotate(
        f"{name}\nCenter: ({cx:.1f}, {cy:.1f}) | R = {r:.1f}m\n[Warning Pylons Enclosed]",
        xy=(cx, cy), xytext=(cx + 3.0, cy + r + 3.5),
        arrowprops=dict(facecolor='#ff1744', edgecolor='#ffffff', width=1.5, headwidth=7.0, shrink=0.05),
        bbox=dict(boxstyle='round,pad=0.4', facecolor='#260000', edgecolor='#ff1744', linewidth=1.5, alpha=0.92),
        fontsize=9, fontweight='bold', color='#ff5252',
        zorder=20
    )

draw_red_zone(50.0, 75.0, 6.0, "RED HAZARD ZONE 1")
draw_red_zone(20.0, 90.0, 8.0, "RED HAZARD ZONE 2")

# 8. Delivery Checkpoints / Landing Pads
# Marker Alpha (30.0, 70.0)
ax.plot(30.0, 70.0, marker='o', markersize=18, color='#00e676', markeredgecolor='#ffffff', markeredgewidth=2.5, zorder=15)
ax.plot(30.0, 70.0, marker='+', markersize=14, color='#ffffff', markeredgewidth=2.0, zorder=16)
ax.annotate(
    "DELIVERY MARKER ALPHA\n(DROP_ALPHA)\nCoord: (30.0, 70.0)\nBeacon: 1.5m Emerald Green",
    xy=(30.0, 70.0), xytext=(12.0, 76.0),
    arrowprops=dict(facecolor='#00e676', edgecolor='#ffffff', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#002811', edgecolor='#00e676', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#69f0ae',
    zorder=20
)

# Marker Bravo (80.0, 80.0)
ax.plot(80.0, 80.0, marker='o', markersize=18, color='#2979ff', markeredgecolor='#ffffff', markeredgewidth=2.5, zorder=15)
ax.plot(80.0, 80.0, marker='+', markersize=14, color='#ffffff', markeredgewidth=2.0, zorder=16)
ax.annotate(
    "DELIVERY MARKER BRAVO\n(DROP_BRAVO)\nCoord: (80.0, 80.0)\nBeacon: 1.5m Cyan Blue",
    xy=(80.0, 80.0), xytext=(88.0, 86.0),
    arrowprops=dict(facecolor='#2979ff', edgecolor='#ffffff', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#001538', edgecolor='#2979ff', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#82b1ff',
    zorder=20
)

# ABEx Cache Site (60.0, 30.0)
ax.plot(60.0, 30.0, marker='s', markersize=16, color='#ff9100', markeredgecolor='#ffffff', markeredgewidth=2.5, zorder=15)
ax.annotate(
    "ABEx CACHE PLATFORM\n(DROP_CACHE)\nCoord: (60.0, 30.0)\nSample Drop / Pickup",
    xy=(60.0, 30.0), xytext=(68.0, 22.0),
    arrowprops=dict(facecolor='#ff9100', edgecolor='#ffffff', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#331600', edgecolor='#ff9100', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#ffd180',
    zorder=20
)

# IDMO Panel (15.0, 50.0)
ax.plot(15.0, 50.0, marker='P', markersize=16, color='#00e5ff', markeredgecolor='#ffffff', markeredgewidth=2.0, zorder=15)
ax.annotate(
    "IDMO SERVICE PANEL\nCoord: (15.0, 50.0)\n1.8m Maintenance Console",
    xy=(15.0, 50.0), xytext=(3.0, 57.0),
    arrowprops=dict(facecolor='#00e5ff', edgecolor='#ffffff', width=2.0, headwidth=8.0, shrink=0.08),
    bbox=dict(boxstyle='round,pad=0.4', facecolor='#002329', edgecolor='#00e5ff', linewidth=1.5, alpha=0.92),
    fontsize=9.5, fontweight='bold', color='#84ffff',
    zorder=20
)

# 9. Competition Tools (RADO Items)
tools = [
    ("Heavy Wrench", 25.0, 35.0, "#ffeb3b", (16.0, 28.0), "Heavy Wrench (TOOL_1)\n(25.0, 35.0) | Mass: 1.8kg"),
    ("Hammer", 28.0, 38.0, "#ffc107", (33.0, 36.0), "Hammer / Mallet (TOOL_HAMMER)\n(28.0, 38.0) | Mass: 1.5kg"),
    ("Screwdriver Set", 40.0, 60.0, "#ff9800", (25.0, 63.0), "Screwdriver Set (TOOL_2)\n(40.0, 60.0) | Mass: 0.9kg"),
    ("Instrument Toolbox", 70.0, 45.0, "#ffa726", (76.0, 38.0), "Toolbox (BOX_1)\n(70.0, 45.0) | Mass: 4.5kg"),
    ("Basalt Specimen", 55.0, 85.0, "#b0bec5", (42.0, 92.0), "Basalt Specimen (ROCK_1)\n(55.0, 85.0) | Mass: 3.2kg"),
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

# 10. Large Obstacles
obstacles = [
    ("Monolith Boulders", 22.0, 28.0, (12.0, 22.0), "Obstacle: Monolith Boulders\n(22.0, 28.0) | 2.4x2.0x1.8m"),
    ("Habitat Cargo Module", 45.0, 42.0, (46.0, 48.0), "Obstacle: Habitat Cargo\n(45.0, 42.0) | 3.6x1.8x1.7m"),
    ("Martian Rock Crag", 50.0, 62.0, (58.0, 65.0), "Obstacle: Crag Spire & Buttress\n(50.0, 62.0) | 2.5x2.2x2.5m"),
    ("Slalom Pillars", 74.0, 68.5, (82.0, 64.0), "Obstacle: Slalom Pillars (x2)\n(72.0, 65.0) & (76.0, 72.0)"),
    ("Comm Mast Debris", 28.0, 52.0, (18.0, 46.0), "Obstacle: Comm Mast Debris\n(28.0, 52.0) | 4.2x1.8x0.9m"),
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

# 11. Region Annotations (Impact Crater & Dunes & Knoll)
# Impact Crater Center (45.0, 70.0)
crater_circ = patches.Circle(
    (45.0, 70.0), 16.0,
    facecolor='none', edgecolor='#b39ddb',
    linewidth=1.8, linestyle=':', alpha=0.75, zorder=4
)
ax.add_patch(crater_circ)
ax.text(
    45.0, 71.5, "IMPACT CRATER REGION\n(R = 16m Smooth Basin)",
    ha='center', va='center', fontsize=8.5, fontweight='bold',
    color='#d1c4e9', alpha=0.85, zorder=6
)

# Sand Dune Field annotation
ax.text(
    75.0, 95.0, "UNDULATING SAND DUNE FIELD\n(Sinusoidal Crests, Slope < 6.5°)",
    ha='center', va='center', fontsize=9, fontweight='bold',
    color='#ffe082', alpha=0.85,
    bbox=dict(boxstyle='round,pad=0.3', facecolor='#2b1a09', edgecolor='#ffe082', alpha=0.7),
    zorder=6
)

# Rocky Knoll
ax.text(
    88.0, 45.0, "ROCKY KNOLL HILL\n(Elevation ~1.3m)",
    ha='center', va='center', fontsize=8.5, fontweight='bold',
    color='#cfd8dc', alpha=0.80, zorder=6
)

# 12. Grid lines & axes styling
ax.set_xlim(-2.0, 122.0)
ax.set_ylim(-2.0, 122.0)
ax.set_xticks(np.arange(0, 121, 10))
ax.set_yticks(np.arange(0, 121, 10))
ax.set_xticks(np.arange(0, 121, 5), minor=True)
ax.set_yticks(np.arange(0, 121, 5), minor=True)
ax.grid(True, which='major', color='#ffffff', linestyle='-', alpha=0.25, linewidth=0.8)
ax.grid(True, which='minor', color='#ffffff', linestyle=':', alpha=0.12, linewidth=0.5)

ax.set_xlabel("East-West Coordinate (X) [meters]", fontsize=13, fontweight='bold', labelpad=10)
ax.set_ylabel("North-South Coordinate (Y) [meters]", fontsize=13, fontweight='bold', labelpad=10)
ax.tick_params(axis='both', which='major', labelsize=11)

# Compass / North Arrow
ax.annotate(
    "N", xy=(114.0, 114.0), xytext=(114.0, 106.0),
    arrowprops=dict(facecolor='#ffffff', edgecolor='#000000', width=3.0, headwidth=12.0),
    ha='center', va='center', fontsize=16, fontweight='bold', color='#ffffff',
    bbox=dict(boxstyle='circle,pad=0.3', facecolor='#111111', edgecolor='#ffffff', linewidth=1.5),
    zorder=25
)

# Title and header
title_str = (
    "IRC 2027 'SPROSCAPE-SIM' MARS ANALOG ARENA — TOP-DOWN ORTHOGRAPHIC MAP\n"
    "Procedural Terrain (120m x 120m) | WGS-84 Datum (DSU, Bengaluru: 12.8614°N, 77.4380°E)\n"
    "C² Continuous Slopes (<6.5° grade) | Slope-Aligned Objects | Centered Collision Physics"
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
