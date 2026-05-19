# UIUC SE494 Team 14 — ROS Workspace

This repository contains a ROS workspace for the Spray Robotics Project, Team 14 in SE494 Senior Design, Fall 2025, UIUC, sponsored by Marmon Holdings. It includes robot-related packages, third-party robot drivers, and URDF/description assets.

## Project information

Written by Junyang Guan (jg73@illinois.edu)

The GitHub repository contains all the code for the Spray Robotics Project, Team 14 in SE494 Senior Design, Fall 2025, UIUC, sponsored by Marmon Holdings.

Team Members: Christian F. Belga, William Deng, Sonali Manjunath, Amit J. Mathai, Junyang Guan.

Special thanks to James Nam (sn29@illinois.edu), a PhD student at UIUC, for providing many constructive suggestions in robot programming.

The team extends its gratitude to the following groups and individuals. Project advisor Mr. Michael Brunetto provided extensive support with the economic analysis of the project and facilitated the connection with the General Motors automation team. Appreciation is also given to Prof. James Allison and Dr. Daniel Metz for their detailed feedback and constructive suggestions on reports and presentations, which significantly strengthened the overall quality of the project.

Special thanks are extended to the UTLX team in Muscatine, Iowa, for generously pausing plant operations to ensure a safe environment for the design team to enter the confined space of a railcar. Mr. Troy McKim and Mr. Derek Adams offered exceptional assistance throughout the semester, going above and beyond by providing thoughtful, detailed responses to technical questions and arranging an additional meeting to discuss economic considerations. Mr. Wyatt Blake contributed valuable technical expertise regarding the spray coating process used for the railcar interior. Appreciation is also given to Marmon Holdings in Chicago and Mr. Vincenzo DiFatta for their guidance and efforts in the coordination of the shipment of the industrial robot to the UIUC campus.

Lastly, gratitude is extended to the UIUC Industrial and Systems Engineering Department for providing this senior capstone experience and coordinating a hands-on engineering project with industry partners that generated meaningful real-world impact. Special thanks go to Dr. Tom Titone and Tracey Rich for facilitating the senior design course, and to Lucas Osborne for coordinating the purchase of the materials and tools needed to complete the project.

## What is included

- `src/Universal_Robots_ROS2_Driver/` — Universal Robots ROS2 driver package set (external/third-party package)
- `src/robotiq_hande_driver/` — Robotiq Hand-E gripper ROS2 controller package (external/third-party package)
- `src/robotiq_hande_description/` — Robotiq Hand-E description and URDF files (external/third-party package)
- `src/ur_description/` — Universal Robots description and URDF resources
- `robot_calibration.yaml` — calibration-related configuration file

## Why some folders cannot be clicked on GitHub

Some directories inside `src/` are not shown as normal browsable folders on GitHub because they contain their own Git metadata (`.git` directories). That means they are nested repositories rather than regular tracked subfolders.

The following packages are included from other authors or external open-source repositories:

- `src/Universal_Robots_ROS2_Driver/`
- `src/robotiq_hande_driver/`
- `src/robotiq_hande_description/`

Those are not original files created by Team 14. They are packaged here as nested repositories for convenience.

On GitHub, nested repositories are not displayed the same way as ordinary directories. If you want them to behave normally in the parent repository, you should either:

1. Remove the nested `.git` directories and commit the package contents as part of this repository, or
2. Convert them to proper Git submodules and add a root `.gitmodules` file.

For now, GitHub may show those entries as opaque repository links rather than clickable file trees.

## Getting started

### 1. Clone the repository

```bash
git clone https://github.com/lloyd9179/UIUC-SE494-Fall-2025---Team-14.git
cd UIUC-SE494-Fall-2025---Team-14
```

### 2. Create and build a ROS workspace

This repo is structured as a ROS workspace. From the top level:

```bash
mkdir -p install
colcon build --symlink-install
source install/setup.bash
```

If you prefer, you can also build within `src/` by creating a separate workspace and copying or linking this directory into `src/`.

### 3. Launch a driver or package

The repository contains multiple ROS2 packages. To use the Universal Robots driver, source the workspace and run one of the package launch files. For example:

```bash
source install/setup.bash
ros2 launch ur_robot_driver ur5e.launch.py
```

To use the Robotiq Hand-E gripper package:

```bash
source install/setup.bash
ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=true
```

If you are not sure which launch file to use, search inside `src/` for available launch scripts:

```bash
find src -name '*.launch.py'
```

> Note: The package name on ROS2 is `ur_robot_driver`, not the top-level workspace directory name.

## Recommended workflow

1. Install ROS2 and any required dependencies for the packages in this workspace.
2. Build the workspace:
   ```bash
   colcon build --symlink-install
   ```
3. Source the generated setup file:
   ```bash
   source install/setup.bash
   ```
4. Launch the desired package using `ros2 launch`.

## How to use the packages

### Universal Robots ROS2 Driver

The `src/Universal_Robots_ROS2_Driver/` package contains a complete ROS2 driver for Universal Robots manipulators. It includes:

- `ur_robot_driver`
- `ur_controllers`
- `ur_calibration`
- `ur_bringup`
- `ur_moveit_config`

Follow the package README inside `src/Universal_Robots_ROS2_Driver/README.md` for detailed installation and usage instructions.

### Robotiq Hand-E driver

The `src/robotiq_hande_driver/` package is a ROS2 controller for the Robotiq Hand-E gripper. Its README has quick-start commands and usage examples for both fake-hardware and real hardware modes.

### Robotiq Hand-E description

The `src/robotiq_hande_description/` package provides URDF and mesh resources for the gripper. It is intended to be used together with `robotiq_hande_driver`.

## License

This project is licensed under the MIT License. See `LICENSE` for details.

## Notes

- If you want GitHub to display package contents normally, remove nested `.git` folders or use proper submodules.
- If the repository is intended to be a standalone workspace, keeping the package content tracked directly is usually the best approach.
- If the package directories are shared across multiple repositories, then submodules are the more appropriate solution.
