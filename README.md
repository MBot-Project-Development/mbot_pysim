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
    $ python3 -m pip install pynput
    ```
4. Install mbot_lcm_msgs, go to `mbot_msgs` and read the README.md there.
