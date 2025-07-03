# TurtleBot3 Navigation Project

This project demonstrates global path planning and obstacle avoidance using the TurtleBot3 Burger robot in a simulated Gazebo environment. It includes both obstacle-free and obstacle-rich navigation scenarios with visualizations and performance evaluation.

---

## 📦 Package Structure

- `scripts/` – Python nodes for path planning, navigation, controllers, and avoidance.
- `launch/` – Launch files for different simulation scenarios.
- `maps/` – PGM/YAML maps for testing with and without obstacles.
- `rviz/` – RViz configuration files for visualization.

---

## 🛠️ Setup

```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

---

## 🚀 Launch Instructions

### 🔁 General

```bash
roscore
```

---

### 🧱 With Obstacles

```bash
rosrun map_server map_server ~/catkin_ws/src/turtlebot3_nav/maps/map.yaml
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_nav global_planner.launch
rviz
roslaunch turtlebot3_nav navigator.launch
```

---

### 🧭 Without Obstacles

```bash
rosrun map_server map_server ~/catkin_ws/src/turtlebot3_nav/maps/no_obstacle.yaml
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
roslaunch turtlebot3_nav global_planner_no_obstacles.launch
rviz
roslaunch turtlebot3_nav navigator.launch
```

---

### ⚙️ Extra Components

```bash
roslaunch turtlebot3_nav kinematic_controller.launch
roslaunch turtlebot3_nav potential_fields.launch
```

---

## 📈 Performance Metrics

| Metric                   | Without Obstacles | With Obstacles     |
|--------------------------|-------------------|---------------------|
| Total Time to Goal       | ~35 seconds       | ~48 seconds         |
| Trajectory Deviation     | Minimal           | Moderate to High    |
| Number of Deviations     | 1 (minor start fix)| 6–8 rerouting events|
| Tracking Error (avg)     | ~0.05 m           | ~0.20 m             |
| Path Smoothness          | Smooth            | Wavy due to avoidance |
| Path Length (estimated)  | ~3.5 meters       | ~5.2 meters         |
| Success Reaching Goal    | ✅ Yes             | ✅ Yes              |

---

## 🧠 Features

- A* Path Planning
- Real-time Obstacle Avoidance using LIDAR
- Kinematic Controller (Unicycle Model)
- RViz and Terminal Feedback
- Full support for obstacle-free and obstacle-rich navigation
- Performance Evaluation Metrics and Graphs

---

## 📊 Results

- Robot was able to reach the goal in both cases.
- With obstacles: Robot dynamically avoids and reroutes.
- Without obstacles: Smooth trajectory close to planned path.

---

## 📝 Author

Bhavinkumar Gopalbhai Prajapati  
Technische Hochschule Deggendorf  
Matriculation Number: 12404779  
bhavinkumar.prajapati@stud.th-deg.de

---

## 📃 License

This project is for academic and educational purposes.

---

## 📌 Note

Please ensure your environment has `ROS Noetic`, `TurtleBot3`, `Gazebo`, and `RViz` set up properly. Adjust topic names and robot model as per your local setup if needed.
