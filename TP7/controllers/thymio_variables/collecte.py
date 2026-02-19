# Copyright 1996-2019 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This controller gives to its node the following behavior:
Listen the keyboard. According to the pressed key, send a
message through an emitter or handle the position of Robot1.
"""


## Reference
## https://www.cyberbotics.com/doc/guide/thymio2?version=develop

import math
import numpy as np
import cv2
import h5py


from controller import Robot

MAX_SPEED = 9.53

robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = robot.getKeyboard()
keyboard.enable(timestep)

print(chr(27) + "[2J")  # ANSI code for clearing command line
print("Initialization of thymio_variables controller")


camera = robot.getDevice("camera")
camera.enable(timestep)
cam_w = camera.getWidth()
cam_h = camera.getHeight()


lidar = robot.getDevice("lidar")
lidar.enable(timestep)
lidar.enablePointCloud()

distanceSensors = []
for i in list(range(0, 7)):
    distanceSensors.append(robot.getDevice("prox.horizontal." + str(i)))
    distanceSensors[i].enable(timestep)

motor_left = robot.getDevice("motor.left")
motor_right = robot.getDevice("motor.right")
motor_left.setPosition(float("inf"))
motor_right.setPosition(float("inf"))

robot_speed = 0.0
distanceVal = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
state = 0
time = 0
once = True

# Touch sensors
# button.[forward-backward-left-right-center]
# the first measurement will be available only after the first sampling period has elapsed.
button_fwd = robot.getDevice("button.forward")
button_fwd.enable(timestep)

# LEDs
led_top = robot.getDevice("leds.top")
led_bottomr = robot.getDevice("leds.bottom.right")
led_bottoml = robot.getDevice("leds.bottom.left")
led_buttons0 = robot.getDevice("leds.buttons.led2")


def step(s):
    if s >= 0:
        return 1
    else:
        return 0


def clip(x):
    return np.clip(x, -1, 1)


def mlp(weights, x):
    """
    weigths is a 3D-array contaning layers,
    which are matrices containing perceptron weight
    """
    out = np.asarray(x)
    for layer in weights:
        out = clip(layer @ np.concatenate(([1], out)))
    return out


w_evitement_obstacle = [2, 1, 1.2, 1.3]
w_back, w_fwd, w_pos, w_neg = w_evitement_obstacle

weights = [
    np.array([[w_fwd, -w_neg, -w_back, w_pos], [w_fwd, w_pos, -w_back, -w_neg]]),
]

print("Sampling period : ", timestep, "ms")

storage = {"prox": [], "scans": [], "cam": [], "cmds": []}


def collecte():
    while robot.step(timestep) != -1:
        # Read the proximity sensors, like:
        global distanceVal
        for i in list(range(0, 7)):
            distanceVal[i] = distanceSensors[i].getValue()
        prox = [distanceVal[0] / 4500, distanceVal[2] / 4500, distanceVal[4] / 4500]
        distance_norm = np.sqrt(np.clip(distanceVal, 0, 4500) / 4500)
        s = np.sum(distance_norm)
        distance_norm /= s + 1e-6
        storage["prox"].append(distance_norm)

        lidar_pt = np.zeros((90, 2))
        for i, d in enumerate(lidar.getRangeImage()):
            lidar_pt[i][0] = d * np.sin(i * np.pi / 45)
            lidar_pt[i][1] = d * np.cos(i * np.pi / 45)
        storage["scans"].append(lidar_pt)

        image = camera.getImage()
        img = np.frombuffer(image, np.uint8).reshape(
            (camera.getHeight(), camera.getWidth(), 4)
        )
        frame = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        y_start = cam_h // 2 - 10
        y_end = cam_h // 2 + 10
        sub_frame = frame[:, y_start:y_end, :]
        v_cam = np.zeros(9)
        for i in range(3):
            h, s, v = np.mean(
                sub_frame[i * (cam_w // 3) : (i + 1) * (cam_w // 3)], axis=(0, 1)
            )
            v_cam[i * 3] = h / 179
            v_cam[i * 3 + 1] = s / 255
            v_cam[i * 3 + 2] = v / 255
        storage["cam"].append(v_cam)

        [speed_r, speed_l] = mlp(weights, prox)

        ## Controle clavier ##
        command = keyboard.getKey()
        if command == keyboard.LEFT:
            speed_l -= 0.2
            speed_r += 0.2
        elif command == keyboard.RIGHT:
            speed_l += 0.2
            speed_r -= 0.2
        else:
            speed_l += 0.1
            speed_r += 0.1

        storage["cmds"].append([speed_l, speed_r])

        if command == ord("X"):
            motor_left.setVelocity(0)
            motor_right.setVelocity(0)
            robot.step(timestep)
            break
        
        motor_left.setVelocity(5 * speed_l)
        motor_right.setVelocity(5 * speed_r)

    with h5py.File("dataset_webots.hdf5", "w") as f:
        array1 = np.array(storage["cmds"])
        array2 = np.array(storage["scans"])
        array3 = np.array(storage["prox"])
        array4 = np.array(storage["cam"])

        f.create_dataset("thymio_commands", data=array1)
        f.create_dataset("thymio_scans", data=array2)
        f.create_dataset("thymio_prox", data=array3)
        f.create_dataset("thymio_cam", data=array4)

    print("Fichier dataset_webots.hdf5 sauvegardé avec succès.")
