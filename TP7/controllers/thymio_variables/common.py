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

def process_prox():
    global distanceVal
    for i in list(range(0, 7)):
        distanceVal[i] = distanceSensors[i].getValue()
    distance_norm = np.sqrt(np.clip(distanceVal, 0, 4500) / 4500)
    s = np.sum(distance_norm)
    distance_norm /= s + 1e-6
    return distance_norm


def process_lidar():
    lidar_pt = np.zeros((90, 2))
    for i, d in enumerate(lidar.getRangeImage()):
        lidar_pt[i][0] = d * np.sin(i * np.pi / 45)
        lidar_pt[i][1] = d * np.cos(i * np.pi / 45)
    return lidar_pt

def process_cam():
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
    return v_cam
