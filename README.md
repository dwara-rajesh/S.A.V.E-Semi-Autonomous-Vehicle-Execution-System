# 🚗 S.A.V.E - Semi-Autonomous Vehicle Execution System

S.A.V.E (Semi-Autonomous Vehicle Execution System) is a full-stack robotics platform designed to autonomously trace complex configuration spaces in physical environments using a DFS or Eulerian-based offline path planner. Built with ROS 2-jazzy, S.A.V.E bridges the digital and physical domains to support modular and scalable motion planning applications—particularly for sports field layout and configuration.

<p style="font-size:small;">
<strong>Demo:</strong> <a href="https://drive.google.com/file/d/1U7dqD4Cdola3IYuyUPecHm_zfnImnF6q/view?usp=drive_link">Watch here</a>
</p>
---

## 📦 Features

- 🧠 **Offline DFS/Eulerian-based Path Planner**: Offline path planning that ensures complete coverage of configuration space.
- 🧱 **ROS2 + Python + CoppeliaSim Edu**: This workflow allows for efficicent communication between ROS2 on WSL2 and CoppeliaSim Edu on Windows for flawless simulation.
- 🔁 **Replayable Paths & Visual Debugging**: Full matplotlib path visualizations and repeatable planning.

---

## 🛠️ System Architecture
```mermaid
graph LR
A[Image Processing + Skeletonization] --> B[Python script - dfs.py or eulerianPath.py]
B --> C[DFS/Eulerian Path Planner]
C --> D[Coordinate Conversion]
D --> E[ROS2 Node]
E --> F[Motor Commands]
F --> G[CoppeliaSim Edu]
```
---

## Tech Stack

| Layer        | Tech                                                        |
|--------------|-------------------------------------------------------------|
| Simulation   | CoppeliaSim                                                 |
| Planner      | Python3, scikit-image, OpenCV, matplotlib, networkx, NumPy  |
| Control      | ROS2 (Jazzy)                                                |

---

## 🧪 How It Works

1. 🖼 **Upload** a white-background, black-foreground image (PNG/JPG) into Python script.
2. 🧠 **Skeletonization**: The image is processed into single-pixel-width paths using `scikit-image`.
3. 📍 **Path Planning**: A **DFS/Eulerian Path - based algorithm** computes a complete path that covers all black segments.
4. 🌐 **Coordinate Mapping**: The path is converted to real-world coordinates and published to `/cmd_vel`.
5. 🤖 **Execution**: The robot (via ROS2) follows the trajectory.


---

## 🚀 Getting Started - On Windows

### 1. Clone the Repository

```bash
git clone https://github.com/dwara-rajesh/S.A.V.E-Semi-Autonomous-Vehicle-Execution-System.git
cd S.A.V.E-Semi-Autonomous-Vehicle-Execution-System
python -m venv venv
source venv\Scripts\activate
pip install -r requirements.txt
```

### 2. Get CoppeliaSim Edu

https://www.coppeliarobotics.com/  
Platform - Windows(installer package)[x86_64]

### 3. Build ROS2 Package in WSL2
```bash
colcon build --packages-select waypoint_follower_pkg
source install/setup.bash
```

### 4. Launch ROS2 bridge in WSL2
```bash
ros2 run rosbridge_server rosbridge_websocket
```

### 5. Run DFS or Eulerian Path Planner - on Windows
```bash
python dfs.py
#or
python eulerianPath.py
```

### 6. Run waypoint_follower on ROS2 in WSL2
```bash
ros2 run waypoint_follower_pkg waypoint_follower
```
### 7. Open CoppeliaSim scene (.ttt file)
### 8. Run ROS2->CoppeliaSim communicator script on Windows
```bash
python coppeliasimros2.py
```
