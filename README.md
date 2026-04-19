# Arduino 3D LiDAR

**TL;DR**: This LiDAR sensor has a **range of roughly 3 meters**, a **scan rate of 2 Hz**, and an opening **angle of 30° (vertical field of view)**. 
The indivdual **vl53l0x** components have a **high spread* of 35° for the emitter and 25° for the receiver, which results in a **very rough point cloud**. 
This is sufficient for applications such as mapping rooms for Roomba-style robots or assisting in autonomous navigation within your home, but **do not expect high-quality point cloud data**. Please check the sample data in the repository to verify the quality of the resulting point cloud data before embarking on this project.

If you are interested in longer-range sensors and not necessarily in 3D point cloud data, consider this alternative project: https://github.com/iliasam/OpenTOFLidar    

## Introduction
This repository houses all the necessary resources to build a budget-friendly 3D rotating LiDAR sensor.
With this sensor you can capture a rough 360° view with a 30° opening angle at a scan rate of 2 Hz. 

Included in this repository are .stl files, a list of required electronic components, wiring instructions, and firmware. 


Rotating LiDAR Sensor     |    LiDAR Data Visualization (5x Speed)
:------------------------:|:--------------------------:
![Rotating LiDAR Sensor](Images/spinning_lidar_small.gif) |![Live Visualization of the LiDAR Sensor](Images/data_fast.gif)

## How It Works
8 vl53l0x laser range sensors are mounted on a rotating axis, each oriented toward a common origin at a different vertical angle. The rotation angle is measured by a rotation encoder disc at the base of the LiDAR. The three values - range, elevation angle, and rotation angle - allow us to reconstruct the relative position in a Cartesian coordinate system from the common origin.

|  ![Schematic Drawing](Images/schematic.png)  |
| :-----------------------------------------------------------------: |
|  *Schematic Drawing of the Mounting Angles*                         |

One example applications for this 3D LiDAR is e.g. mapping rooms for Roomba-style robots to assisting in autonomous navigation within your home.

|  ![Occupancy Grid Map using the LiDAR Sensor Data](Images/ogm.gif)  |
| :-----------------------------------------------------------------: |
|  *Occupancy Grid Map Using the LiDAR Sensor Data (1x Speed)*        |

## Data Sample
You can verify the quality of the resulting point cloud data before embarking on this project by examining the log sequence displayed above. This can be found [here](https://github.com/FrederikHasecke/arduino-3d-lidar/tree/main/data/sequences/01/lidar_points). Use [this script](https://github.com/FrederikHasecke/arduino-3d-lidar/tree/main/python-receiver-code/offline_visualization.py) to visualize the points in Python.

## Getting Started
1. **3D Printing Part List** - Download the STL files for 3D printing [here](https://github.com/FrederikHasecke/arduino-3d-lidar/tree/main/stl-files) and find the recommended printing orientation [here](https://github.com/FrederikHasecke/arduino-3d-lidar/tree/main/stl-files/assembly-instructions.md). 

2. **Sensor Code** - The C code for the microcontroller is available for [Arduino UNO](https://github.com/FrederikHasecke/arduino-3d-lidar/tree/main/arduino-code) and for [ESP32](https://github.com/FrederikHasecke/arduino-3d-lidar/tree/main/esp32-code). Though untested on other microcontrollers, slight modifications should make it compatible.

3. **Python Interface** - If you prefer Python for its simplicity, feel free to use [my prepared Python scripts](https://github.com/FrederikHasecke/arduino-3d-lidar/tree/main/python-receiver-code) to record and/or visualize the sensor output.  

    - Requirements
        - Python 3.10 - 3.12
        - numpy
        - [Open3D](https://github.com/isl-org/Open3D) for visualization

    - Install with `uv` (recommended)

```bash
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
uv venv --python 3.10
.venv\Scripts\activate
uv sync
```

    - Install with `pip`

```bash
python -m venv .venv
.venv\Scripts\activate
python -m pip install --upgrade pip
python -m pip install .
```

    - Offline visualization

```bash
python python-receiver-code/offline_visualization.py
```

When no `--path` is provided, the script automatically plays the bundled sample sequence from `data/sequences/01/lidar_points` and prints a disclaimer that this is the repository's basic input pointcloud sample. The sample data is useful for checking that the viewer works, but it is intentionally rough and should not be treated as a high-quality point cloud.

To visualize a different recording, pass either a `.npy` frame or a directory of `.npy` frames:

```bash
python python-receiver-code/offline_visualization.py --path path\to\lidar_points
```

4. **ROS Interface** - Currently under development. If you wish to contribute to communication between the Arduino/ESP32 and a Master, please feel free to submit a PR.

## Required Components
Below is a complete list of parts needed to build this 8-channel LiDAR sensor:

Part                                  | Quantity | Cost per Item | Amazon Link
--------------------------------------|--------|---------------|------------
Vl53l0x                               |   8    |               |
Basic 3-12V DC Motor                  |   1    |               |
Motor Driver                          |   1    |               |
Microcontroller                       |   1    |               |
12 Wire Slip Ring                     |   1    |               |
Motor Comparator Speed Sensor Module  |   1    |               |
Jumper Wires                          |  37    |               |
Set of M3 Screws                      |   1    |               |
Rubber Band                           |   1    |               |
