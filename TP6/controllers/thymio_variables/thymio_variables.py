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


def perceptron(w, x, func_act=clip):
    return func_act(w.T @ x)


def mlp(weights, x):
    """
    weigths is a 3D-array contaning layers, which are arrays containing perceptron weight
    """
    out = np.asarray(x)
    for layer in weights:
        out = clip(layer @ np.concatenate(([1], out)))
    return out


def mlp_rec(weights, memory, x):
    """
    weigths is a 3D-array contaning layers, which are arrays containing perceptron weight
    """
    out = np.array(x)
    for j, (layer, layer_mem) in enumerate(weights):
        out = clip(
            layer @ np.concatenate(([1], out)) + np.multiply(layer_mem, memory[j])
        )
        memory[j] = out
    return out


weights_q4 = [
    np.array([[1, 0], [0, 1]]),
    np.array([[0.2, -1], [-1, 0.2]]),
]

w_evitement_obstacle = [2, 1, 1.2, 1.3]
w_back, w_fwd, w_pos, w_neg = w_evitement_obstacle

weights_q6 = [
    (
        np.array([[w_fwd, -w_neg, -w_back, w_pos], [w_fwd, w_pos, -w_back, -w_neg]]),
        [0, 0],
    )
]

w_back, w_fwd, w_pos, w_neg = [2, 1, 3, 2.8]
w_mem = 1
weights_q6_mem = [
    (
        np.array([[w_fwd, -w_neg, -w_back, w_pos], [w_fwd, w_pos, -w_back, -w_neg]]),
        [w_mem, w_mem],
    )
]

memory_q6 = np.zeros((1, 2))

weights_q7 = [
    np.array(
        [
            [0, 4, -4, 0, 0, 0],
            [0, -2, 4, -2, 0, 0],
            [0, 0, -2, 4, -2, 0],
            [0, 0, 0, -2, 4, -2],
            [0, 0, 0, 0, -4, 4],
        ]
    ),
]

weights_q8 = [
    np.array(
        [
            [0, 4, -4, 0, 0, 0],
            [0, -2, 4, -2, 0, 0],
            [0, 0, -2, 4, -2, 0],
            [0, 0, 0, -2, 4, -2],
            [0, 0, 0, 0, -4, 4],
        ]
    ),
    np.array(
        [
            [0, 0.4, 0.6, 0, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0.6, 0.4],
        ]
    ),
    np.array(
        [
            [0, -2.5, -0.2, 0],
            [0, 0, -0.2, -2.5],
        ]
    ),
]


print("Sampling period : ", timestep, "ms")

while robot.step(timestep) != -1:
    # Read the proximity sensors, like:
    for i in list(range(0, 7)):
        distanceVal[i] = distanceSensors[i].getValue() / 4500.0

    prox = distanceVal[:5]

    [speed_l, speed_r] = mlp(weights_q8, prox)

    motor_left.setVelocity(5 * speed_l)
    motor_right.setVelocity(5 * speed_r)
