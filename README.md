## MBot Simulator

A few things about this project:
1. Based on Pygame
2. Run on your laptop not the mbot
3. You need to install LCM

## How to install (on ubuntu)
1. Git clone this project
2. Install LCM onto your laptop
    ```bash
    ./ubuntu_install.sh
    ```
3. Create virtual env and install Python dependencies
    ```
    $ python3 -m venv pysim_env
    $ source pysim_env/bin/activate
    $ python3 -m pip install -U pygame
    $ python3 -m pip install numpy
    ```
4. Install mbot_lcm_msgs, go to `mbot_msgs` and read the README.md there.


## How to read the code
---

To familiarize yourself with this code:

1. Start with `mbot.py`:
This file defines the core functionality of the robot, such as initialization, motor commands, and high-level robot control. Understanding this will give you insight into how the robot is structured, what commands it accepts, and how it interacts with other parts of the system.
2. Move to `sim.py`:
`sim.py` simulates the robot's movement, collision detection, and updates the state based on inputs (like motor commands).
3. Explore `teleop.py`:
If your project involves teleoperation (remote control), this file is crucial. It translates user inputs into motor commands that control the robot.
4. Study `geometry.py`:
This file defines essential geometric utilities like poses (position and orientation) and transformations.
5. Look into `map.py` and `map_generator.py`:
These files handle the environment where the robot operates. `map_generator.py` creates the world, while `map.py` manages it. 
6. Check `lidar.py`:
If your robot uses sensors to perceive its surroundings, this file simulates or processes data from the LiDAR sensor.
7. Timing Consideration in `timing.py`:
This file helps ensure that the robot’s actions are properly synchronized in the simulation.