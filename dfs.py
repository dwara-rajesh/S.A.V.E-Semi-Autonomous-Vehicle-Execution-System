import cv2
import numpy as np
from skimage.morphology import skeletonize
import matplotlib.pyplot as plt
from collections import defaultdict
import csv
import os

def dfs_path(start, graph, visited_edges):
    path = [start]
    stack = [start]

    while stack:
        current = stack[-1]
        neighbors = graph[current]

        for neighbor in neighbors:
            edge = tuple(sorted((current, neighbor)))
            if edge not in visited_edges:
                visited_edges.add(edge)
                stack.append(neighbor)
                path.append(neighbor)
                break
        else:
            stack.pop()

    return path

# Load and preprocess image
print("DFS Path Generation")
path = input("Enter full path of image\n")
image = cv2.imread(path)
image = cv2.resize(image,(1920, 1080))
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

_, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
binary_bool = binary > 0
skeleton = skeletonize(binary_bool)
skeleton_img = (skeleton * 255).astype(np.uint8)

plt.imshow(skeleton_img, cmap='gray')
plt.title("Skeleton Image")
plt.axis('off')
plt.show()

# Find contours on skeleton image
contours, _ = cv2.findContours(skeleton_img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
print(f"Number of contours found: {len(contours)}")

graph = defaultdict(list)
all_edges = set()

for cnt in contours:
    points = [tuple(pt[0]) for pt in cnt]

    for i in range(len(points) - 1):
        pt1 = points[i]
        pt2 = points[i + 1]

        if pt1 != pt2:
            # Add both directions (undirected)
            graph[pt1].append(pt2)
            graph[pt2].append(pt1)

            # Store unique edge (sorted to remove directionality)
            edge = tuple(sorted((pt1, pt2)))
            all_edges.add(edge)

print(f"Total unique edges: {len(all_edges)}")

visited_edges = set()
all_waypoints = []

for node in graph.keys():
    # Check if this node has unvisited edges
    edges_from_node = [tuple(sorted((node, nbr))) for nbr in graph[node]]
    unvisited = any(e not in visited_edges for e in edges_from_node)
    if unvisited:
        segment_path = dfs_path(node, graph, visited_edges)
        if len(segment_path) > 1:
            all_waypoints.extend(segment_path)


print(f"Generated waypoint path with {len(all_waypoints)} points")

waypoints_np = np.array(all_waypoints, dtype=np.int32).reshape((-1, 1, 2))

# Simplify path using RDP
epsilon = 2.0  # Adjust: higher = fewer points
simplified = cv2.approxPolyDP(waypoints_np, epsilon, False)

# Convert back to list of (x, y)
simplified_waypoints = [tuple(pt[0]) for pt in simplified]

print(f"Simplified waypoints: {len(simplified_waypoints)}")

path_img = np.zeros_like(image)

for i in range(len(simplified_waypoints) - 1):
    cv2.line(path_img, simplified_waypoints[i], simplified_waypoints[i+1], (0, 255, 0), 2)

plt.imshow(path_img)
plt.title("Robot Path")
plt.axis('off')
imgoppath = os.path.join("robotpaths",f"DFS_output_path_{os.path.splitext(os.path.basename(path))[0]}.png")
plt.savefig(imgoppath, dpi=300, bbox_inches='tight')
plt.show()

destination_folder = r"\\wsl.localhost\Ubuntu-24.04\home\dwarakesh\ROS2Projects\ros2_ws\waypointcsvs"
destination_path = os.path.join(destination_folder,f"DFS_output_waypoints_{os.path.splitext(os.path.basename(path))[0]}.csv")
with open(destination_path, 'w', newline='') as file:
    writer = csv.writer(file)

    # Write each tuple as a row
    for row in simplified_waypoints:
        writer.writerow(row)
    writer.writerow(simplified_waypoints[0])
