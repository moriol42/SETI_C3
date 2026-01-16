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
import matplotlib.patches as patches
from matplotlib.transforms import Affine2D

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


def multmatr(X, Y, T):
    res = np.array(
        [
            X[0] * Y[0] + X[3] * Y[1] + X[6] * Y[2] - T[0],
            X[1] * Y[0] + X[4] * Y[1] + X[7] * Y[2] + T[1],
            X[2] * Y[0] + X[5] * Y[1] + X[8] * Y[2] + T[2],
        ]
    )
    return res


def get_point_cloud():
    point_cloud = lidar.getRangeImage()
    rotation = node.getOrientation()
    xyz = node.getPosition()
    angle = 0
    point_cloud_xy = np.zeros((len(point_cloud), 3))
    for i, p in enumerate(point_cloud):
        xy = [p * math.sin(angle), 0, p * math.cos(angle)]
        point_cloud_xy[i] = multmatr(rotation, xy, xyz)
        angle += 2 * math.pi / HORZ_RES
    return point_cloud_xy


# wall from webots config
walls = [
    {"translation": (-0.16, 0.23, 0), "size": (0.01, 1, 0.1)},
    {"translation": (0.05, 0.73, 0), "size": (0.43, 0.01, 0.1)},
    {"translation": (0.25, 0, 0), "size": (0.4, 0.01, 0.1)},
    {
        "translation": (0.0732246, -0.0387871, 0),
        "rotation": 0.785398,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (0.631834, -0.215563, 0),
        "rotation": 0.785398,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (0.0732248, 0.0319252, 0),
        "rotation": 2.35619,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (0.631834, 0.2087, 0),
        "rotation": 2.35619,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (0.242929, 0.696599, 0),
        "rotation": 2.35619,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (-0.124765, 0.696599, 0),
        "rotation": 0.785398,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (0.214645, -0.717604, 0),
        "rotation": -2.3561953071795863,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (0.00251402, -0.47719, 0),
        "rotation": -2.3561953071795863,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (-0.612664, -0.314557, 0),
        "rotation": -2.3561953071795863,
        "size": (0.1, 0.01, 0.1),
    },
    {
        "translation": (-0.612664, -0.710533, 0),
        "rotation": 2.35619,
        "size": (0.1, 0.01, 0.1),
    },
    {"translation": (-0.4, -0.27, 0), "size": (0.49, 0.01, 0.1)},
    {"translation": (-0.16, -0.51, 0), "size": (0.4, 0.01, 0.1)},
    {"translation": (0.46, -0.25, 0), "size": (0.43, 0.01, 0.1)},
    {"translation": (-0.2, -0.75, 0), "size": (0.9, 0.01, 0.1)},
    {"translation": (0.47, 0.25, 0), "size": (0.4, 0.01, 0.1)},
    {"translation": (0.04, -0.01, 0), "size": (0.01, 1, 0.1)},
    {"translation": (0.25, -0.5, 0), "size": (0.01, 0.5, 0.1)},
    {"translation": (-0.65, -0.5, 0), "size": (0.01, 0.5, 0.1)},
    {"translation": (0.27, 0.49, 0), "size": (0.01, 0.48, 0.1)},
    {"translation": (0.67, 0, 0), "size": (0.01, 0.5, 0.1)},
]


def draw_point_cloud(pc):
    """Affiche le nuage de points du lidar"""
    plt.ion()
    # create persistent figure/axes on first call
    if not hasattr(draw_point_cloud, "fig_ax") or draw_point_cloud.fig_ax is None:
        fig, ax = plt.subplots(1, 2)
        draw_point_cloud.fig_ax = (fig, ax)
        plt.show(block=False)
    else:
        fig, ax = draw_point_cloud.fig_ax

    x = pc[:, 0]
    y = pc[:, 2]

    # Draw point cloud
    ax[0].cla()
    ax[0].scatter(x, y, s=2)
    ax[0].set_title("Lidar point cloud")
    ax[0].set_aspect("equal")

    # Draw walls
    ax[1].cla()
    for w in walls:
        tx, ty, _ = w["translation"]
        sx, sy, _ = w["size"]
        angle = w.get("rotation", 0)
        rect = patches.Rectangle(
            (-sx / 2, -sy / 2),
            sx,
            sy,
            linewidth=1,
            edgecolor="black",
            facecolor="gray",
            alpha=0.7,
        )
        transform = Affine2D().rotate(angle).translate(tx, ty) + ax[1].transData
        rect.set_transform(transform)
        ax[1].add_patch(rect)

    ax[1].set_aspect("equal")
    ax[1].set_xlabel("x (m)")
    ax[1].set_ylabel("y (m)")
    ax[1].grid(True)
    ax[1].relim()
    ax[1].autoscale_view()

    plt.tight_layout()
    fig.canvas.draw_idle()
    plt.pause(0.01)


c = 0

while robot.step(timestep) != -1:
    ## Lidar ##
    point_cloud = get_point_cloud()
    if c == 100:
        # draw_point_cloud(point_cloud)
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
