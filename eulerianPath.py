import cv2
import numpy as np
from skimage.morphology import skeletonize
import matplotlib.pyplot as plt
from collections import defaultdict
import csv
import networkx as nx
from networkx.algorithms.euler import eulerize
import os

# Load and preprocess image
print("Eulerian Path Generation")
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

# Build an undirected graph
G = nx.Graph()

# Add edges to the graph
for pt1, pt2 in all_edges:
    G.add_edge(pt1, pt2)

# Check if Eulerian path exists
if nx.is_eulerian(G):
    print("Graph has an Eulerian circuit.")
    euler_path = list(nx.eulerian_circuit(G))
elif nx.has_eulerian_path(G):
    print("Graph has an Eulerian path.")
    euler_path = list(nx.eulerian_path(G))
else:
    print("Graph does not have an Eulerian path or circuit.")
    euler_path = []

# Convert Eulerian edge path to node path
if euler_path == []:
    waypoints = []
    components = list(nx.connected_components(G))
    print(f"Graph has {len(components)} connected components.")
    for i, component_nodes in enumerate(components):
        subgraph = G.subgraph(component_nodes).copy()

        if not nx.is_connected(subgraph):
            continue  # Skip if somehow still disconnected

        # Ensure Eulerian
        if not nx.is_eulerian(subgraph):
            try:
                subgraph = eulerize(subgraph)
            except:
                print(f"Could not eulerize component {i}")
                continue

        # Get Eulerian circuit
        circuit = list(nx.eulerian_circuit(subgraph))
        if not circuit:
            continue

        # Convert edges to path
        subpath = [circuit[0][0]] + [v for _, v in circuit]
        waypoints.extend(subpath)
else:
    waypoints = [euler_path[0][0]] + [v for u, v in euler_path] if euler_path else []
print(f"Eulerian path length: {len(waypoints)}")

# Simplify path
waypoints_np = np.array(waypoints, dtype=np.int32).reshape((-1, 1, 2))
epsilon = 2.0
simplified = cv2.approxPolyDP(waypoints_np, epsilon, False)
simplified_waypoints = [tuple(pt[0]) for pt in simplified]
print(f"Simplified waypoints: {len(simplified_waypoints)}")

# Draw path
path_img = np.zeros_like(image)
for i in range(len(simplified_waypoints) - 1):
    cv2.line(path_img, simplified_waypoints[i], simplified_waypoints[i+1], (0, 255, 0), 2)

plt.imshow(cv2.cvtColor(path_img, cv2.COLOR_BGR2RGB))
plt.title("Eulerian Robot Path")
plt.axis('off')
imgoppath = os.path.join("robotpaths",f"Eulerian_output_path_{os.path.splitext(os.path.basename(path))[0]}.png")
plt.savefig(imgoppath, dpi=300, bbox_inches='tight')
plt.show()

destination_folder = r"\\wsl.localhost\Ubuntu-24.04\home\dwarakesh\ROS2Projects\ros2_ws\waypointcsvs"
destination_path = os.path.join(destination_folder,f"Eulerian_output_waypoints_{os.path.splitext(os.path.basename(path))[0]}.csv")
with open(destination_path, 'w', newline='') as file:
    writer = csv.writer(file)

    # Write each tuple as a row
    for row in simplified_waypoints:
        writer.writerow(row)
    writer.writerow(simplified_waypoints[0])