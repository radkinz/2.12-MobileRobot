import math
from collections import defaultdict
from scipy.spatial import KDTree
import json

# Define map size (in meters)
MAP_WIDTH = 2.9
MAP_HEIGHT = 2.3

# Define resolutions (in meters)
FINE_RESOLUTION = 0.06
COARSE_RESOLUTION = 0.1

# Max distance to connect neighbors (adjust depending on your movement model)
NEIGHBOR_RADIUS = .15  

# List of fine resolution zones (you manually define these)
# Each zone is (xmin, ymin, xmax, ymax)
FINE_ZONES = [
    (0.1, 1.5, 1.3, 2.2), #Bin start location
    (0.0, 0.0, 0.9, 1.0),  #Bin target location
    (1.3716-0.3, 0.8-0.1, 2.0828-0.254, 1.4224-0.3), #Coop bin start
    (1.37, 0.0, 2.89, 0.61), #Ramp component
    (2.29,0.61,2.89,1.55),
]

# Simple function to check if a point is inside any fine-resolution zone
def is_in_fine_zone(x, y):
    for (xmin, ymin, xmax, ymax) in FINE_ZONES:
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return True
    return False

OBSTACLE_ZONES = [
    (0, 0, 2.9,0.02),
    (0, 2.28, 2.9,2.33),
    (0, 0, 0.02, 2.33),
    (2.88, 0, 2.91, 2.33),
    (0.21,1.91,0.23,2.3),
    (0.21+0.20955,1.91,0.23+0.20955,2.3),
    (0.21+0.20955*2,1.91,0.23+0.20955*2,2.3),
    (0.21+0.20955*3,1.91,0.23+0.20955*3,2.3),
    (0.21+0.20955*3+0.1016,1.91,0.23+0.20955*3+0.1016,2.3),
    (0.21+0.20955*3+0.1016+0.2286,1.91,0.23+0.20955*3+0.1016+0.2286,2.3),
    (1.34,0.6,2.286,0.635),
    (2.286,0.6,2.286+0.035,1.5748),
    (0,0.32512,0.43688,0.33528),
    (0,0.32512+0.3302,0.43688,0.33528+0.3302),
    (0,0.32512+0.3302*2,0.43688,0.33528+0.3302*2),
    (1.3716,0.8128-0.0127,1.778,0.8128+0.0127),
    (1.3716,0.8128-0.0127+0.2286,1.778,0.8128+0.0127+0.2286),
    (1.778-0.0127,0.8128-0.0127,1.778+0.0127,0.8128+0.0127+0.2286)
    # do the ramp boundary, bin parking boundaries
]
def is_obstacle(x, y):
    for (xmin, ymin, xmax, ymax) in OBSTACLE_ZONES:
        if xmin <= x <= xmax and ymin <= y <= ymax:
            return True
    return False

def line_crosses_obstacle(x1, y1, x2, y2, step=0.01):
    # Early reject: check bounding boxes first
    min_x, max_x = min(x1, x2), max(x1, x2)
    min_y, max_y = min(y1, y2), max(y1, y2)

    # Only check obstacles that overlap this line segment's bounding box
    for (oxmin, oymin, oxmax, oymax) in OBSTACLE_ZONES:
        # If there's no overlap, skip the obstacle
        if oxmax < min_x or oxmin > max_x or oymax < min_y or oymin > max_y:
            continue  # No overlap with this obstacle

        # Now do line sampling only if bounding box overlaps
        dist = math.hypot(x2 - x1, y2 - y1)
        steps = max(9, int(dist / step))  # More points to better track the line
        for i in range(1, steps):  # Skip the endpoints
            t = i / steps
            xi = x1 + t * (x2 - x1)
            yi = y1 + t * (y2 - y1)

            # Ensure sample points are within the obstacle's bounding box
            if oxmin <= xi <= oxmax and oymin <= yi <= oymax:
                return True  # We found a collision point
    return False  # No intersection with obstacles



# Function to determine step size based on location
def dynamic_step_size(x, y):
    if is_in_fine_zone(x, y):
        return FINE_RESOLUTION
    else:
        return COARSE_RESOLUTION

import numpy as np

nodes = []
node_positions = {}
index = 0

# Step 1: Coarse grid generation
x_coarse = np.arange(0, MAP_WIDTH + COARSE_RESOLUTION, COARSE_RESOLUTION)
y_coarse = np.arange(0, MAP_HEIGHT + COARSE_RESOLUTION, COARSE_RESOLUTION)

coarse_set = set()

for x in x_coarse:
    for y in y_coarse:
        if not is_obstacle(x, y) and not is_in_fine_zone(x, y):
            nodes.append((x, y))
            node_positions[index] = (x, y)
            coarse_set.add((round(x, 4), round(y, 4)))
            index += 1

# Step 2: Fine grid in fine zones
x_fine = np.arange(0, MAP_WIDTH + FINE_RESOLUTION, FINE_RESOLUTION)
y_fine = np.arange(0, MAP_HEIGHT + FINE_RESOLUTION, FINE_RESOLUTION)

for x in x_fine:
    for y in y_fine:
        if not is_obstacle(x, y) and is_in_fine_zone(x, y):
            key = (round(x, 4), round(y, 4))
            if key not in coarse_set:
                nodes.append((x, y))
                node_positions[index] = (x, y)
                index += 1


# Build a KD-Tree for fast neighbor search
kdtree = KDTree(nodes)

# Create adjacency list
adjacency_list = defaultdict(list)

for idx, (x, y) in enumerate(nodes):
    neighbor_indices = kdtree.query_ball_point((x, y), NEIGHBOR_RADIUS)
    for neighbor_idx in neighbor_indices:
        if neighbor_idx == idx:
            continue
        x2, y2 = nodes[neighbor_idx]
        if not line_crosses_obstacle(x, y, x2, y2):
            adjacency_list[idx].append(neighbor_idx)

# print(f"Built adjacency list with {len(adjacency_list)} entries.")
# with open('adjacency_list.json', 'w') as jsonfile:
#     json.dump(adjacency_list, jsonfile, indent=4)

print("Adjacency list saved to 'adjacency_list.csv' and 'adjacency_list.json'.")
# Example access:
# node_positions[0] gives (x, y) of node 0
# adjacency_list[0] gives list of neighbor indices for node 0

# OPTIONAL: Save to file, visualize, etc.


import matplotlib.pyplot as plt
import matplotlib.patches as patches

fig, ax = plt.subplots(figsize=(10, 10))

# Plot all nodes
for (x, y) in nodes:
    ax.plot(x, y, 'k.', markersize=2)

# Plot edges
for idx, neighbors in adjacency_list.items():
    x1, y1 = node_positions[idx]
    for neighbor in neighbors:
        x2, y2 = node_positions[neighbor]
        ax.plot([x1, x2], [y1, y2], 'gray', linewidth=0.3)

# Plot fine zones
for (xmin, ymin, xmax, ymax) in FINE_ZONES:
    rect = patches.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, 
                             linewidth=1, edgecolor='blue', facecolor='blue', alpha=0.2)
    ax.add_patch(rect)

# Plot obstacles
for (xmin, ymin, xmax, ymax) in OBSTACLE_ZONES:
    rect = patches.Rectangle((xmin, ymin), xmax - xmin, ymax - ymin, 
                             linewidth=1, edgecolor='red', facecolor='red', alpha=0.4)
    ax.add_patch(rect)

# Set map limits and aspect ratio
ax.set_xlim(0, MAP_WIDTH)
ax.set_ylim(0, MAP_HEIGHT)
ax.set_aspect('equal')
ax.set_title('Roadmap Visualization')
ax.set_axis_off()
plt.grid(True)
plt.tight_layout()
plt.show()
