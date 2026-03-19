# blueboat_stonefish

Main workspace repository for the CIRTESU BlueBoat simulation stack.

This repository acts as the **top-level project container** and groups together the different ROS 2 packages and external repositories used in the project. 

Some of them are regular packages inside the workspace, and others are included as **git submodules**.

## What you will find in this repository

The workspace is organized around several main components inside `src/`, such as:

- `blueboat_stonefish_core`  
  Main package containing the BlueBoat robot description, simulation assets, scenarios, and core project resources.

- `blueboat_stonefish_mav2ros2`  
  Bridge and MAVROS-related integration for simulation and control.

- `FASTLIO_cirtesu`  
  CIRTESU adaptation of FAST-LIO for LiDAR-Inertial odometry, mapping, and relocalization workflows.

- `livox_ros_driver2`  
  Livox ROS 2 driver.

- `livox2_to_pc2`, `pc2_to_livox2`, `tandem_girona`, etc.  
  Additional support packages or external repositories used by the project.

Some of these repositories are tracked as **submodules**, which means this parent repository stores the exact version of each one used by the project.

## Clone the full workspace

To clone the repository together with all submodules:

```bash
git clone --recursive https://github.com/Mariolopez31/blueboat_stonefish.git
````

If the repository was already cloned without submodules:

```bash
git submodule update --init --recursive
```

## After pulling changes

If you pull changes from the parent repository, it is recommended to update submodules as well:

```bash
git pull
git submodule update --init --recursive
```

## Working with submodules

If you modify something inside a submodule, such as `FASTLIO_cirtesu`, remember that there are **two repositories involved**:

1. the submodule repository itself
2. the parent repository, which stores the submodule commit pointer

So the usual workflow is:

### 1. Commit and push inside the submodule

```bash
cd src/FASTLIO_cirtesu
git add .
git commit -m "Your change"
git push
```

### 2. Commit and push the updated submodule pointer in the parent repository

```bash
cd ../..
git add src/FASTLIO_cirtesu
git commit -m "Update FASTLIO_cirtesu submodule"
git push
```

The same idea applies to any other submodule in the workspace.

## Build

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
git submodule update --init --recursive
colcon build
source install/setup.bash
```

Some packages may have their own dependency or build notes, so check the README of each package when needed.

## Notes

* This repository is the **parent workspace**, not a single package.
* Package-specific documentation should be kept in the README of each package or submodule.
* If something looks updated in `src/` but not in the parent repo, it is often just a submodule pointer that still needs to be committed.
