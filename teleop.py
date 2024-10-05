import time
import numpy as np
import lcm
import sys
from mbot_lcm_msgs.twist2D_t import twist2D_t
from pynput import keyboard

LIN_VEL_CMD = 0.2  # m/s
ANG_VEL_CMD = 0.75  # rad/s

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
time.sleep(0.5)

fwd_vel = 0.0
turn_vel = 0.0
running = True
key_states = {"up": False, "down": False, "left": False, "right": False}

# Function to handle key press events
def on_press(key):
    global fwd_vel, turn_vel

    try:
        if key == keyboard.Key.up:
            key_states["up"] = True
            fwd_vel = LIN_VEL_CMD
        elif key == keyboard.Key.down:
            key_states["down"] = True
            fwd_vel = -LIN_VEL_CMD
        elif key == keyboard.Key.left:
            key_states["left"] = True
            turn_vel = ANG_VEL_CMD
        elif key == keyboard.Key.right:
            key_states["right"] = True
            turn_vel = -ANG_VEL_CMD

    except AttributeError:
        pass

# Function to handle key release events
def on_release(key):
    global fwd_vel, turn_vel, running

    if key == keyboard.Key.esc:
        running = False
        return False  # Stop the listener when ESC is pressed

    try:
        if key == keyboard.Key.up:
            key_states["up"] = False
            fwd_vel = 0 if not key_states["down"] else -LIN_VEL_CMD
        elif key == keyboard.Key.down:
            key_states["down"] = False
            fwd_vel = 0 if not key_states["up"] else LIN_VEL_CMD
        elif key == keyboard.Key.left:
            key_states["left"] = False
            turn_vel = 0 if not key_states["right"] else -ANG_VEL_CMD
        elif key == keyboard.Key.right:
            key_states["right"] = False
            turn_vel = 0 if not key_states["left"] else ANG_VEL_CMD

    except AttributeError:
        pass

# Start a background listener
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

print("Press arrow keys to move! (Press ESC to quit) ")
while running:
    # Create and publish the LCM command
    command = twist2D_t()
    command.vx = fwd_vel
    command.wz = turn_vel
    lc.publish("MBOT_VEL_CMD", command.encode())
    
    # Control the loop speed
    time.sleep(0.05)

# Wait for listener to stop (if ESC is pressed)
listener.join()
