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

w_and = np.array([-1.5, 1, 1])

w_back = 2
w_fwd = 1
w_pos = 1.2
w_neg = 1.3


def sum(w, x):
    return w.T @ np.concatenate(([1], x))


def step(s):
    if s >= 0:
        return 1
    else:
        return 0


def perceptron(w, x, func_act=step):
    return func_act(sum(w, x))


w_analog = np.array([0, -1, -0.1])

print("Sampling period : ", timestep, "ms")

while robot.step(timestep) != -1:
    # Read the proximity sensors, like:
    for i in list(range(0, 7)):
        distanceVal[i] = distanceSensors[i].getValue() / 4500.0

    # Set motors speed :
    dist = np.array([distanceVal[0], distanceVal[2], distanceVal[4]])
    speed_r = 5 * perceptron(
        np.array([w_fwd, -w_neg, -w_back, w_pos]),
        dist,
        func_act=math.tanh,
    )
    speed_l = 5 * perceptron(
        np.array([w_fwd, w_pos, -w_back, -w_neg]),
        dist,
        func_act=math.tanh,
    )

    motor_left.setVelocity(speed_l)
    motor_right.setVelocity(speed_r)


# Enter here exit cleanup code
