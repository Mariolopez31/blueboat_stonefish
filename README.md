# CIRTESU Blueboat Stonefish Package

This repository contains a specialized **Stonefish** simulation suite developed at **CIRTESU (Castellón)**. It includes all the necessary packages and configurations required to work with the **Blueboat** surface vehicle across various highly detailed scenarios that replicate the CIRTESU facilities.

## Description

The main objective of this package is to provide a high-fidelity simulation environment for maritime robotics. 
* **Detailed Scenarios:** Includes complete and precise environments modeled after the CIRTESU facility in Castellón.
* **Blueboat Integration:** Pre-configured physics, sensors, and actuators tailored for the Blueboat surface robot.
* **Ready for Research:** Designed for testing navigation, control, and autonomy algorithms in a realistic maritime setting.

---

## Prerequisites

To use this package, you must have the following dependencies installed in your ROS 2 workspace:

1.  **Stonefish**: The core C++ physics engine for maritime simulation.
2.  **stonefish_ros2**: The ROS 2 interface and library for the Stonefish simulator.

---

## Installation & Setup

Follow these steps to clone and build the package in your local environment:

### 1. Clone the repository
Navigate to your ROS 2 workspace `src` folder and clone the repository:
```bash
cd ~/your_ws/src
git clone 