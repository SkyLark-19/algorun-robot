<div align="center">

# 🤖 AlgoRun Robot

*Autonomous SLAM-based robot in Webots, learning and optimizing its navigation.*

[![PeraBots 2025](https://img.shields.io/badge/Competition-PeraBots%202025-blue)](https://eees-uop.edu.lk/perabots/)

</div>

---

**AlgoRun Robot** is a **fully autonomous**, two-wheeled robot simulation developed in [Webots](https://cyberbotics.com/).  It learns, optimizes, and adapts its navigation in a virtual environment without human intervention, using **SLAM** and **classical robotics principles**.

---

## 🌟 Features

- **Learns and maps environments (SLAM)**
- **Red line detection via camera**
- **Obstacle navigation and avoidance**
- **Adaptive path planning and speed control**
- **All logic runs onboard**
- **No supervised learning or external computation**

---

## 🌐 Simulation World

- **Arena:** Flat, white floor (2m x 2m) with black textured walls.
- **Obstacles:** Static black cubes/boxes.
- **Start/Finish:** Red lines are part of the arena texture.
- **Robot:** Always within 20×20×20 cm size constraint.

---

## ⚙️ Robot Sensors & Hardware (Webots)

- **Camera:** (640x480) Mounted on the robot for vision-based navigation.
- **Distance Sensors:** 5 IR sensors for obstacle avoidance/wall following.
- **Encoders:** Wheel position sensors for odometry tracking.
- **IMU:** Used for orientation and improving SLAM/localization.
- **Motors:** Two-wheel differential drive.

---

## 🧠 Learning Method

- **Exploration Mode:** Initial runs perform SLAM to map the environment and record the robot’s trajectory (all sensor-fused).
- **Exploitation Mode:** Subsequent runs use the learned optimal path and occupancy grid to maximize speed and minimize completion time.

---

## 🕹️ Quick Start

1. **Install [Webots](https://cyberbotics.com/).**
2. **Clone this repo:**
   ```bash
   git clone https://github.com/SkyLark-19/algorun-robot.git
   ```
3. **Open either `worlds/With_obstacles.wbt` or `worlds/Without_obstacles.wbt` in Webots.**
4. **Assign the appropriate controller:**
   - Use `with_obstacles.py` for the obstacle world.
   - Use `without_obstacles.py` for the no-obstacle world.
5. **Run the simulation.**
   - The robot will detect the red start line, begin mapping, and learn to improve its time over repeated runs—fully autonomously.
   - Learning data is saved automatically in `competition_memory.json`.

---

## 📦 Repo Structure

```
algorun-robot/
├── controllers/
│    └── 0.Final controller/
│         ├── with_obstacles.py
│         └── without_obstacles.py
├── worlds/
│    ├── With_obstacles.wbt
│    └── Without_obstacles.wbt
├── models/
├── competition_memory.json
└── README.md
```

---
<div align="center">
   
**Enjoy exploring autonomous learning and path optimization in Webots!**

</div>
