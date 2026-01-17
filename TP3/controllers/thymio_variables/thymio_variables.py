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
from scipy.spatial import KDTree

from controller import Supervisor
import numpy as np

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

print(chr(27) + "[2J")  # ANSI code for clearing command line
print("Initialization of thymio_variables controller")

motor_left = robot.getDevice("motor.left")
motor_right = robot.getDevice("motor.right")
motor_left.setPosition(float("inf"))
motor_right.setPosition(float("inf"))

pos0 = node.getPosition()
rot0 = node.getOrientation()
Xk = [pos0[0], pos0[1], math.atan2(rot0[3], rot0[0])]
print(Xk)


def pos_robot():
    """Return the exact position of the robot using the simulator values"""
    x, y, _ = node.getPosition()
    rotation = node.getOrientation()
    theta = -math.atan2(rotation[3], rotation[0]) + math.pi / 2
    return x, y, theta


def base_robot2main(p, repere_proj="odo"):
    """Changement de base du robot vers la base principal de la scene"""
    if repere_proj == "simu":
        x, y, theta = pos_robot()
    elif repere_proj == "odo":
        x, y, theta = Xk
        theta = -theta + math.pi / 2
    else:
        raise ValueError
    return np.array(
        [
            math.cos(theta) * p[0] + math.sin(theta) * p[1] + x,
            -math.sin(theta) * p[0] + math.cos(theta) * p[1] + y,
        ]
    )


def get_point_cloud(repere_proj="odo"):
    point_cloud = lidar.getRangeImage()
    angle = math.pi
    point_cloud_xy = np.zeros((len(point_cloud), 2))
    for i, p in enumerate(point_cloud):
        xy = [p * math.sin(angle), p * math.cos(angle)]
        if repere_proj is not None:
            point_cloud_xy[i] = base_robot2main(xy, repere_proj=repere_proj)
        else:
            point_cloud_xy[i] = np.array(xy)
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


def walls2points(walls):
    points = []
    for w in walls:
        tx, ty, _ = w["translation"]
        sx, sy, _ = w["size"]
        angle = w.get("rotation", 0)
        if sx >= sy:
            for i in range(math.floor(sx * 100)):
                points.append(
                    np.array(
                        [
                            math.cos(angle) * (-sx / 2 + i / 100) + tx,
                            math.sin(angle) * (-sx / 2 + i / 100) + ty,
                        ]
                    )
                )
        else:
            for i in range(math.floor(sy * 100)):
                points.append(
                    np.array(
                        [
                            -math.sin(angle) * (-sy / 2 + i / 100) + tx,
                            math.cos(angle) * (-sy / 2 + i / 100) + ty,
                        ]
                    )
                )
    return np.array(points)


def draw_point_cloud(pc_odo, pc_simu):
    """Affiche le nuage de points du lidar"""
    plt.ion()
    # create persistent figure/axes on first call
    if not hasattr(draw_point_cloud, "fig_ax") or draw_point_cloud.fig_ax is None:
        fig, ax = plt.subplots(1, 2)
        draw_point_cloud.fig_ax = (fig, ax)
        plt.show(block=False)
    else:
        fig, ax = draw_point_cloud.fig_ax

    x = pc_odo[:, 0]
    y = pc_odo[:, 1]

    # Draw point cloud
    ax[0].cla()
    ax[0].scatter(x, y)
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

    ax[1].scatter(
        pc_odo[:, 0],
        pc_odo[:, 1],
        label="Projection en utilisant les données de l'odométrie",
    )
    ax[1].scatter(
        pc_simu[:, 0],
        pc_simu[:, 1],
        label="Projection en utilisant les données du simulateur",
    )
    ax[1].legend()

    ax[1].set_aspect("equal")
    ax[1].set_xlabel("x (m)")
    ax[1].set_ylabel("y (m)")
    ax[1].grid(True)
    ax[1].relim()
    ax[1].autoscale_view()

    plt.tight_layout()
    fig.canvas.draw_idle()
    plt.pause(0.01)


def angle_principal(theta):
    while theta > math.pi:
        theta -= 2 * math.pi
        return theta
    while theta <= -math.pi:
        theta += 2 * math.pi
        return theta
    return theta


def update_pos():
    """This function update the compiuted position of the robot it must be called once every step"""
    dleft = motor_left.getVelocity() * timestep * wheel_radius * 1e-3
    dright = motor_right.getVelocity() * timestep * wheel_radius * 1e-3

    ds = (dleft + dright) / 2
    dtheta = (dright - dleft) / (2 * e)
    Xk[0] += ds * math.cos(Xk[2] + dtheta / 2)
    Xk[1] += ds * math.sin(Xk[2] + dtheta / 2)
    Xk[2] = angle_principal(Xk[2] + dtheta)
    # list_pos_x.append(Xk[0])
    # list_pos_y.append(Xk[1])
    # list_theta.append(Xk[2])
    # print(Xk)

    ## Simulateur
    # xyz = node.getPosition()
    # rotation = node.getOrientation()
    # list_pos_x_sim.append(xyz[0] * 1e3)
    # list_pos_y_sim.append(xyz[1] * 1e3)
    # list_theta_sim.append(math.atan2(rotation[3], rotation[0]))


def indxtMean(index, arrays):
    indxSum = np.array([0.0, 0.0, 0.0])
    for i in range(np.size(index, 0)):
        indxSum = np.add(
            indxSum, np.array(arrays[index[i]]), out=indxSum, casting="unsafe"
        )
    return indxSum / np.size(index, 0)


def indxtfixed(index, arrays):
    T = []
    for i in index:
        T.append(arrays[i])
    return np.asanyarray(T)


def ICPSVD(fixedX, fixedY, movingX, movingY):
    reqR = np.identity(3)
    reqT = [0.0, 0.0, 0.0]

    fixedt = []
    movingt = []
    for i in range(len(fixedX)):
        fixedt.append([fixedX[i], fixedY[i], 0])
    for i in range(len(movingX)):
        movingt.append([movingX[i], movingY[i], 0])
    moving = np.asarray(movingt)
    fixed = np.asarray(fixedt)
    n = np.size(moving, 0)
    tree = KDTree(fixed)
    for i in range(10):
        distance, index = tree.query(moving)
        err = np.mean(distance**2)

        com = np.mean(moving, 0)
        cof = indxtMean(index, fixed)
        W = np.dot(np.transpose(moving), indxtfixed(index, fixed)) - n * np.outer(
            com, cof
        )
        U, _, V = np.linalg.svd(W, full_matrices=False)

        tempR = np.dot(V.T, U.T)
        tempT = cof - np.dot(tempR, com)

        moving = (tempR.dot(moving.T)).T
        moving = np.add(moving, tempT)
        reqR = np.dot(tempR, reqR)
        reqT = np.add(np.dot(tempR, reqT), tempT)


c = 0

while robot.step(timestep) != -1:
    ## Lidar ##
    if c == 100:
        pc_odo = get_point_cloud()
        pc_simu = get_point_cloud(repere_proj="simu")
        # draw_point_cloud(point_cloud)
        draw_point_cloud(pc_odo, pc_simu)
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

    update_pos()
    if command == ord("P"):
        print(node.getPosition())
