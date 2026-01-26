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

import time

robot = Supervisor()
node = robot.getFromDef("thymio")
timestep = int(robot.getBasicTimeStep())
keyboard = robot.getKeyboard()
keyboard.enable(timestep)
robot_speed = 0.0
MAX_SPEED = 8
e = 0.054  # m
wheel_radius = 0.021  # m

lidar = robot.getDevice("lidar")
lidar.enable(timestep)
lidar.enablePointCloud()

HORZ_RES = lidar.getHorizontalResolution()
THRESHOLD_GAP = 0.1  # 10cm
BUBBLE_RADIUS = 40

print(chr(27) + "[2J")  # ANSI code for clearing command line
print("Initialization of thymio_variables controller")

motor_left = robot.getDevice("motor.left")
motor_right = robot.getDevice("motor.right")
motor_left.setPosition(float("inf"))
motor_right.setPosition(float("inf"))


def pos_robot():
    """Return the exact position of the robot using the simulator values"""
    x, y, _ = node.getPosition()
    rotation = node.getOrientation()
    theta = -math.atan2(rotation[3], rotation[0]) + math.pi / 2
    return x, y, theta


def base_robot2main(p):
    """Changement de base du robot vers la base principal de la scene"""
    # Apply the correction
    x, y, theta = pos_robot()
    proj = np.array(
        [
            math.cos(theta) * p[0] + math.sin(theta) * p[1] + x,
            -math.sin(theta) * p[0] + math.cos(theta) * p[1] + y,
        ]
    )
    return proj


def proj_point_cloud(point_cloud):
    angle = -math.pi / 2
    point_cloud_xy = np.zeros((len(point_cloud), 2))
    for i, p in enumerate(point_cloud):
        xy = [p * math.sin(angle), p * math.cos(angle)]
        point_cloud_xy[i] = base_robot2main(xy)
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


g_best_i = 0
g_best_len = 0


def draw_point_cloud(point_cloud, best):
    """Affiche le nuage de points du lidar"""
    pc_simu = proj_point_cloud(point_cloud)
    plt.ion()
    if not hasattr(draw_point_cloud, "fig_ax") or draw_point_cloud.fig_ax is None:
        fig, ax = plt.subplots()
        draw_point_cloud.fig_ax = (fig, ax)
        plt.show(block=False)
    else:
        fig, ax = draw_point_cloud.fig_ax

    # Draw walls
    ax.cla()
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
        transform = Affine2D().rotate(angle).translate(tx, ty) + ax.transData
        rect.set_transform(transform)
        ax.add_patch(rect)

    is_gap = point_cloud > THRESHOLD_GAP
    plt.scatter(
        pc_simu[is_gap, 0],
        pc_simu[is_gap, 1],
    )
    plt.scatter(
        pc_simu[~is_gap, 0],
        pc_simu[~is_gap, 1],
    )
    if best is not None:
        plt.scatter(
            pc_simu[best, 0],
            pc_simu[best, 1],
        )

    # Draw robot position and orientation
    x, y, theta = pos_robot()
    arrow_length = 0.1  # meters
    dx = arrow_length * math.sin(theta)
    dy = arrow_length * math.cos(theta)
    ax.arrow(
        x,
        y,
        dx,
        dy,
        head_width=0.05,
        head_length=0.03,
        fc="green",
        ec="green",
        linewidth=2,
    )

    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    ax.relim()
    ax.autoscale_view()

    plt.tight_layout()
    fig.canvas.draw_idle()
    plt.pause(0.01)


def max_gap(cloud):
    """Returns the start and length of the biggest gap"""
    best_i = 0
    best_len = 0
    curr_i = 0
    curr_len = 0
    # is_gap = cloud > THRESHOLD_GAP
    for i, c in enumerate(cloud):
        if c > THRESHOLD_GAP:
            if curr_len == 0:
                curr_i = i
                curr_len = 1
            else:
                curr_len += 1
        else:
            if curr_len > best_len:
                best_i = curr_i
                best_len = curr_len
            curr_len = 0
    if cloud[-1] > THRESHOLD_GAP and curr_len > best_len:
        best_i = curr_i
        best_len = curr_len
    return best_i, best_len


def add_bubble(cloud, i):
    for j in range(max(0, i - BUBBLE_RADIUS), min(len(cloud), i + BUBBLE_RADIUS + 1)):
        cloud[j] = cloud[i]


def follow_the_gap(cloud):
    # Add bubbles
    i = np.argmin(cloud)
    add_bubble(cloud, i)

    # Find the max-gap
    gap_start, gap_len = max_gap(cloud)

    # Find the best point
    if gap_len == 0:
        return math.pi, None
    best = gap_start + np.argmax(cloud[gap_start : gap_start + gap_len])

    return math.pi / 2 - 2 * best * math.pi / HORZ_RES, best


def rotate_ts(ts, speed=1):
    """if speed is positive goes right, and left if otherwise"""
    for _ in range(ts):
        motor_left.setVelocity(speed)
        motor_right.setVelocity(-speed)
        robot.step(timestep)
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)


def rotate(angle):
    speed = 1 if angle < 0 else -1
    rotate_ts(int(abs(angle) * 4.9), speed=speed)


c = 0


while robot.step(timestep) != -1:
    ## Lidar ##
    if c == 0:
        point_cloud = np.asarray(lidar.getRangeImage()[90:271])
        theta_rad, best = follow_the_gap(point_cloud)
        theta = theta_rad * 180 / math.pi
        print(theta)
        draw_point_cloud(point_cloud, best)
        rotate(theta)
        if theta == 180:
            continue
        c = 100
    c -= 1
    motor_left.setVelocity(2)
    motor_right.setVelocity(2)
