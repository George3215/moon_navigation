
## Mars Navigation Framework

This repository contains the open-source implementation for the paper "VLM-empowered Multi-Mode System for Efficient and Safe Planetary Navigation" (IROS 2025). Further details and project materials are available at: https://chengsn1234.github.io/multi-mode-planetary-navigation/.

### Dependencies

ROS packages should be installed through your ROS Noetic workspace as usual.

For elevation_mapping_cupy, follow the instructions at: https://leggedrobotics.github.io/elevation_mapping_cupy/getting_started/installation.html.

```bash
conda create -n mars_nav python=3.8 -y
conda activate mars_nav
pip install -r requirements.txt
```

After the environment is created, point [tools/python_env.sh](tools/python_env.sh) to your interpreter path, for example:

```bash
PYTHON_BIN="$HOME/miniconda3/envs/mars_nav/bin/python"
```

You may also export `PYTHON_BIN` before launching the stack.

### Build the Workspace

Clone the repository to your workspace, and run:
```bash
catkin build globalmap_server
catkin build elevation_mapping_cupy -DCMAKE_BUILD_TYPE=Release
catkin build -DCMAKE_BUILD_TYPE=Release
```
Source the workspace or add it to `.bashrc`:
```bash
source devel/setup.bash
```

### Quickstart via GUI
To start the navigation stack interactively, use the provided RQt plugin. We provide a shortcut for quickstart. For step-by-step instructions see [launch_from_gui.md](launch_from_gui.md).
Start Rqt and open the Navigation GUI widget from the Plugins menu:

```bash
rqt
```

Before launching any GUI action that starts a Python node, make sure the Python environment above is installed and [tools/python_env.sh](tools/python_env.sh) points to the correct interpreter.

<table>
	<tr>
		<td><img src="docs/picture/gui_manual/gui_plugin.png" alt="GUI" width="48%"/></td>
		<td><img src="docs/picture/gui_manual/gui_launch.png" alt="Launch" width="48%"/></td>
	</tr>
	<tr>
		<td align="center">Navigation GUI</td>
		<td align="center">Launch controls (Simulation / Initialization / Navigation)</td>
	</tr>
</table>

#### Step 1: Simulation

The Simulation action launch the simulation environment (Gazebo) and RViz visualization.

#### Step 2: Initialization
Initialization prepares the map server and required clients, and generates the initial global map and path.

Start the Initialization action in the GUI and wait until the global map appears in RViz. Use the "2D Nav Goal" tool in RViz to set a global goal; the planned global path (blue) should appear.

#### Step 3: Navigation

The Navigation action launches the mapping, planning, and control modules for the three operating modes. Note: mode switching is not performed automatically by this action.

#### Step 4: Mode Switching
You can switch modes manually for testing, or enable the VLM-based mode activator.

![ManualModeActivator](docs/picture/gui_manual/08_manual_mode_activate.png)

When the manual activator runs, the node prints the following and awaits input:

```
Mode Handler Initialized
Enter mode: 1 for efficient, 2 for safe, 3 for conservative:
```

Enter the desired mode number and press Enter; the node will publish the chosen mode on the `/navigation_mode` topic.

The VLM mode activator subscribes to RGB and depth topics, calls the VLM API, and publishes the predicted navigation mode on `/navigation_mode`. Enable the VLM activator by using `VLM Update`. You need an API Key to set up the script.

### NOTE
This system was ported from MarsSim, another simulation platform. Some parameters still require tuning (e.g. elevation and planner cost parameters), which may lead to unexpected behaviour in some scenarios. We apologize for the inconvenience.

### Acknowledgements
This project builds on ideas and code from the following works:

- Miki T., Wellhausen L., Grandia R., et al. "Elevation mapping for locomotion and navigation using GPU." IROS 2022.
- Azkarate M., Gerdes L., Joudrier L., et al. "A GNC architecture for planetary rovers with autonomous navigation." ICRA 2020.
- Zhou R., Feng W., Ding L., et al. "MarsSim: A high-fidelity physical and visual simulation for Mars rovers." IEEE Trans. on Aerospace and Electronic Systems, 2022.

We also used AI-assisted tools (e.g., ChatGPT, Claude, Grok, Qwen) during development for assistance.






