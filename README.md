# blueboat_stonefish — CIRTESU BlueBoat Stonefish Simulation (ROS 2)

This repository contains a specialized **Stonefish** simulation suite developed at **CIRTESU (Castellón, Spain)**.  
It provides the required packages, assets, and configurations to simulate the **BlueBoat** USV in highly detailed scenarios that replicate the CIRTESU facilities.

## Overview

The main goal of this package is to provide a high-fidelity maritime robotics simulation environment:

- **Detailed scenarios:** Realistic environments modeled after the CIRTESU facility.
- **BlueBoat integration:** Pre-configured robot model, sensors, and actuation suited for BlueBoat.
- **Research-ready:** Useful for testing navigation, control, localization, and autonomy in a realistic setting.

## Notes on Stonefish modifications

This project is developed against a **modified Stonefish** version that includes support for a **terrestrial (2D) LiDAR** sensor.  
Upstream/public release of that LiDAR work is planned, but it may not be available in the official repositories yet.

## Prerequisites

You must have the following dependencies in your ROS 2 environment:

- **Stonefish (simulator core)**  
  Repository: https://github.com/patrykcieslak/stonefish.git

- **stonefish_ros2 (ROS 2 integration)**  
  Repository: https://github.com/patrykcieslak/stonefish_ros2.git

> Tip: Ensure you use compatible branches/versions for your ROS 2 distribution.

## Installation

### 1. Clone into your ROS 2 workspace

```bash
mkdir -p ~/your_ws/src
cd ~/your_ws/src

# Dependencies
git clone https://github.com/patrykcieslak/stonefish.git
git clone https://github.com/patrykcieslak/stonefish_ros2.git

# This package
git clone <THIS_REPO_URL>
