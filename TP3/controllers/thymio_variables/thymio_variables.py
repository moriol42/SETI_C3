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
import matplotlib.pyplot as plt

from controller import Supervisor
import numpy as np

robot = Supervisor()
node = robot.getFromDef("thymio")
timestep = int(robot.getBasicTimeStep())
keyboard = robot.getKeyboard()
keyboard.enable(timestep)
robot_speed = 0.0
MAX_SPEED = 8
e = 54  # mm
wheel_radius = 21  # mm

lidar = robot.getDevice("lidar")
lidar.enable(timestep)
lidar.enablePointCloud()

HORZ_RES = lidar.getHorizontalResolution()

print(chr(27) + "[2J")  # ANSI code for clearing command line
print("Initialization of thymio_variables controller")

motor_left = robot.getDevice("motor.left")
motor_right = robot.getDevice("motor.right")
motor_left.setPosition(float("inf"))
motor_right.setPosition(float("inf"))


def get_point_cloud():
    point_cloud = lidar.getRangeImage()
    angle = 0
    point_cloud_xy = np.zeros((len(point_cloud), 2))
    for i, p in enumerate(point_cloud):
        point_cloud_xy[i] = np.array([p * math.sin(angle), p * math.cos(angle)])
        angle += 2 * math.pi / HORZ_RES
    return point_cloud_xy

def draw_point_cloud(pc):
    """Affiche le nuage de points du lidar"""
    plt.ion()
    x = pc[:, 0]
    y = pc[:, 1]
    plt.scatter(x, y)
    plt.tight_layout()
    plt.axis('scaled')
    plt.draw()
    plt.pause(0.01)
    plt.clf()

c = 0

while robot.step(timestep) != -1:
    ## Lidar ##
    point_cloud = get_point_cloud()
    if c == 100:
        draw_point_cloud(point_cloud)
        c = 0
    c += 1

    ## Controle clavier ##
    command = keyboard.getKey()
    if command == keyboard.LEFT:
        # print('Left')
        motor_left.setVelocity(robot_speed - 2)
        motor_right.setVelocity(robot_speed + 2)
        if abs(robot_speed) > 3:
            robot_speed *= 0.99
    elif command == keyboard.RIGHT:
        # print('right')
        motor_left.setVelocity(robot_speed + 2)
        motor_right.setVelocity(robot_speed - 2)
        if abs(robot_speed) > 3:
            robot_speed *= 0.99
    else:
        if command == keyboard.UP:
            if robot_speed < MAX_SPEED:
                robot_speed += 0.1
        elif command == keyboard.DOWN:
            if robot_speed > -2:
                robot_speed -= 0.1
        elif command == 83:  # capture S key
            robot_speed = 0
        motor_left.setVelocity(robot_speed)
        motor_right.setVelocity(robot_speed)

    if command == ord("P"):
        print(node.getPosition())
