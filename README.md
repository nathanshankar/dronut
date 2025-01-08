# Dronut ROS 2 Simulation

This repository contains a ROS 2 simulation for the Dronut, the world's first bi-rotor ducted drone. Designed for safe indoor operation, confined spaces, and GPS-denied environments, the Dronut excels in high maneuverability, real-time high-definition video streaming, LiDAR data collection, and providing operators with critical insights to enhance efficiency, reduce costs, and improve safety in hazardous environments. For more details about the Dronut, visit the official [product page](https://cleorobotics.com/product/).
![image](https://github.com/user-attachments/assets/ae429d89-a84f-48c3-b145-3ca5d94d0510)

[Image Source](https://cleorobotics.com/product/)


[Simulation](https://github.com/user-attachments/assets/95d4e163-17b7-4247-a3b8-0e919f6e7af4)

## Features
- **Bi-rotor ducted drone**: Ensures safe operation in tight spaces.
- **Indoor and GPS-denied navigation**: Equipped with advanced algorithms for localization and control.
- **High-definition video streaming**: Provides real-time video feedback.
- **LiDAR data collection**: Maps confined spaces and improves situational awareness.
- **High maneuverability**: Agile performance in complex environments.

## Installation
### Prerequisites
Ensure you have the following installed:
- ROS 2 (Works on Humble and Jazzy)
- Gazebo or Ignition for simulation
- `rosdep` for managing dependencies

### Step 1: Clone the Repository
```bash
git clone https://github.com/yourusername/dronut_ros2_sim.git
```

### Step 2: Download Realsense cam 
```bash
cd ~/dronut/src
git clone https://github.com/nathanshankar/realsense_cam -b d455
```

### Step 3: Install Dependencies
Navigate to your workspace and install the required dependencies using `rosdep`:
```bash
cd ~/dronut
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4: Build the Workspace
Build the ROS 2 workspace with `colcon`:
```bash
colcon build
```

### Step 5: Source the Setup File
Source the workspace to overlay the environment:
```bash
source ~/ros2_ws/install/setup.bash
```

## Running the Simulation
1. Launch the Dronut simulation in Gazebo:
   ```bash
   ros2 launch dronut_controller drone.launch.py
   ```

## Repository Structure
```
.
├── src
  ├── dronut_controller    # Flying the drone around in gazebo
  ├── dronut_description   # URDF files for drone physical model and configuration
  ├── dronut_w_cam         # Updated Drone model equipped with a camera
  ├── joy_test             # Scripts or nodes for joystick-based teleoperation
  ├── realsense_cam        # Integration with Intel RealSense camera for depth and vision data
└── README.md            # Project documentation
```

## Contributing
Contributions are welcome! Please follow these steps:
1. Fork the repository.
2. Create a feature branch (`git checkout -b feature-name`).
3. Commit your changes (`git commit -m 'Add feature'`).
4. Push to your branch (`git push origin feature-name`).
5. Create a pull request.

## Contact
For inquiries or support, please contact [Nathan Shankar](mailto:nathanshankar465@gmail.com).

---

Happy flying with Dronut!

