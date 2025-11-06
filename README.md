# 🧭 Mobile Robot Simulation with CoppeliaSim and ROS 2

Doc with SCOPERTA plan: [https://univr-my.sharepoint.com/:w:/g/personal/daniele_meli_univr_it/EQSUs2pW0eZAlAX_kmPSOt8BmGFLFFfHhOqrR6jWNNCb_w?e=K5cKhV]https://univr-my.sharepoint.com/:w:/g/personal/daniele_meli_univr_it/EQSUs2pW0eZAlAX_kmPSOt8BmGFLFFfHhOqrR6jWNNCb_w?e=K5cKhV

This repository provides a **CoppeliaSim–ROS 2 integration workspace** for mobile robot simulation and control.  
It includes both **low-level perception and control nodes** (e.g. LIDAR-based navigation) and **high-level logic** (e.g. finite state machine coordination).

---

## 🧩 Requirements

### 🐧 Ubuntu 22.04 LTS  
Download the official ISO image:  
👉 [https://releases.ubuntu.com/22.04/](https://releases.ubuntu.com/22.04/)

### 🤖 CoppeliaSim 4.1.0 (Edu or Pro)  
Download from the official Coppelia Robotics website:  
👉 [https://www.coppeliarobotics.com/downloads](https://www.coppeliarobotics.com/downloads)

After installation:
```bash
chmod +x CoppeliaSim.sh
./CoppeliaSim.sh
```

### 🧠 ROS 2 Humble Hawksbill  
Official installation guide (Debian packages):  
👉 [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

After installation, source your ROS 2 environment:
```bash
source /opt/ros/humble/setup.bash
```

---

## 📚 Introductory Material

Before running this project, it is strongly recommended to go through:

- **ROS 2 Basics:**  
  [https://docs.ros.org/en/humble/Tutorials.html](https://docs.ros.org/en/humble/Tutorials.html)

- **CoppeliaSim + ROS 2 Integration Tutorial:**  
  [https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm](https://www.coppeliarobotics.com/helpFiles/en/ros2Tutorial.htm)

These tutorials explain how to connect ROS 2 nodes with CoppeliaSim via topics, services, and transforms.

---

## 🗂️ Repository Structure

This project is organized as a standard **ROS 2 colcon workspace**:

```
src/
├── follow_pkg/          # LIDAR-based reactive navigation
└── high_level_control/  # Finite-state machine (FSM) for mission control
README.md
scenes/
├── limo.ttt             # The Coppelia scene with the robot (File -> Open Scene...)
```

---

## ⚙️ Build and Source Instructions

1. Build the workspace (ignore the syntax errors, the provided codes are only sketches to solve the respective tasks, hence incomplete):
   ```bash
   colcon build
   ```

2. Source the setup file before running any node:
   ```bash
   source install/setup.bash
   ```

---

## 🚀 Running the Simulation

1. Start **CoppeliaSim**, open the provided `.ttt` scene, and run the simulation.  
2. In a terminal (after sourcing the workspace), launch your ROS 2 nodes:
   ```bash
   ros2 run follow_pkg follow_node
   ros2 run high_level_control fsm_node
   ```

- The **`follow_pkg`** package handles reactive navigation and obstacle avoidance using LIDAR data.  
- The **`high_level_control`** package manages mission-level logic through a finite state machine.

---

## 🧾 License

This project is distributed for **educational and research purposes** only.  
Please refer to the [CoppeliaSim licensing terms](https://www.coppeliarobotics.com/licensing) for simulator usage.

---
