[![PeraBots 2025](https://img.shields.io/badge/Competition-PeraBots%202025-blue)](https://eees-uop.edu.lk/perabots/)

# 🤖 AlgoRun Robot — Autonomous Path Learning in Webots

Welcome to **AlgoRun Robot!**  
This project features a fully autonomous, two-wheeled robot built in [Webots](https://cyberbotics.com/), capable of learning, optimizing, and adapting its navigation in a simulated environment—without human intervention between runs. The code and approach are based on SLAM and classical robotics, in line with competition-grade standards.

---

## 🌟 What Can the Robot Do?

- **Learns and Maps the Environment:** Uses SLAM (Simultaneous Localization and Mapping), fusing data from distance sensors, wheel encoders, and an IMU to build an occupancy grid.
- **Red Line Detection:** Vision-based detection of start and finish lines using an onboard camera. (Lines are part of the arena texture.)
- **Obstacle Navigation:** Avoids black cube/box obstacles using real-time sensor fusion.
- **Adaptive Path Planning:** Remembers the best path from previous runs and exploits this knowledge for improved times.
- **Dynamic Speed Control:** Automatically adjusts speed based on obstacle proximity and past performance.
- **All Logic Onboard:** Designed for microcontroller constraints—no external computation.
- **No Supervised Learning:** Pure SLAM, clustering, and heuristic methods.

---

## 🏞️ Simulation World

- **Arena:** Flat, white floor (2m x 2m) with black textured walls.
- **Obstacles:** Static black cubes/boxes.
- **Start/Finish:** Red lines are part of the arena texture.
- **Robot:** Always within 20×20×20 cm size constraint.

---

## 🤖 Robot Sensors & Hardware (Webots)

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
   - Use `controllers/0.Final controller/with_obstacles.py` for the obstacle world.
   - Use `controllers/0.Final controller/without_obstacles.py` for the no-obstacle world.
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

## 💬 Notes

- The robot’s logic is fully self-contained and requires no manual intervention between runs.
- The project is competition-ready and can be a starting point for further autonomous robotics research or education.
- For questions, open an issue or explore the code for comments and usage details.

---

**Enjoy exploring autonomous learning and path optimization in Webots!**
