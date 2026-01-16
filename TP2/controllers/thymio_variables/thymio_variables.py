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
import time

robot = Supervisor()
node = robot.getFromDef("thymio")
timestep = int(robot.getBasicTimeStep())
keyboard = robot.getKeyboard()
keyboard.enable(timestep)
robot_speed = 0.0
MAX_SPEED = 8
e = 54  # mm
wheel_radius = 21  # mm

print(chr(27) + "[2J")  # ANSI code for clearing command line
print("Initialization of thymio_variables controller")

motor_left = robot.getDevice("motor.left")
motor_right = robot.getDevice("motor.right")
motor_left.setPosition(float("inf"))
motor_right.setPosition(float("inf"))


x_right = 0
x_left = 0
Xk = [149.1, -157.4, -2.63059]
dest = [157.4, -607.8]
path = [
    [157.4, -607.8],
    [-467.47, -627.27],
    [-468.02, -380],
    [-50, -400],
    [-060.50, 046.22],
    [-070.10, 585.88],
    [106.17, 599.60],
    [160.95, 170.76],
    [543.82, 129.62],
    [557.74, -067.46],
    [178.06, -136.92],
]
DRAW = False

list_pos_x = []
list_pos_y = []

list_pos_x_sim = []
list_pos_y_sim = []

list_theta = []
list_theta_sim = []


def draw_pos():
    """Affiche les positions calculee via l'odometrie et reele du robot"""
    plt.ion()
    plt.plot(list_pos_x, list_pos_y)
    plt.plot(list_pos_x_sim, list_pos_y_sim)
    plt.tight_layout()
    plt.draw()
    plt.pause(0.0001)
    plt.clf()


def draw_comp():
    # plt.figure()
    fig, axs = plt.subplots(3, 1, figsize=(6, 8))

    t = range(len(list_pos_x))

    # X
    axs[0].plot(t, list_pos_x, label="estimation")
    axs[0].plot(
        t, list_pos_x_sim, label="reference simulation", color="red", linestyle="--"
    )
    axs[0].set_ylabel("x (mm)")
    axs[0].legend()
    axs[0].grid(True)

    # Y
    axs[1].plot(t, list_pos_y, label="estimation")
    axs[1].plot(
        t, list_pos_y_sim, label="reference simulation", color="red", linestyle="--"
    )
    axs[1].set_ylabel("y (mm)")
    axs[1].legend()
    axs[1].grid(True)

    # Theta
    axs[2].plot(t, list_theta, label="estimation")
    axs[2].plot(
        t, list_theta_sim, label="reference simulation", color="red", linestyle="--"
    )
    axs[2].set_ylabel("Theta")
    axs[2].set_xlabel("Timestep samples")
    axs[2].legend()
    axs[2].grid(True)

    fig.tight_layout()
    # fig.canvas.draw()
    print()
    fig.savefig("composantes.png")


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
    global x_left, x_right
    dleft = motor_left.getVelocity() * timestep * wheel_radius * 1e-3
    x_left += dleft
    dright = motor_right.getVelocity() * timestep * wheel_radius * 1e-3
    x_right += dright

    # print(f"Deplacement depuis dernier timestep roue droite: {dright} roue gauche: {dleft}")
    # print(f"Deplacement total roue droite: {x_right} mm roue gauche: {x_left} mm")

    ds = (dleft + dright) / 2
    dtheta = (dright - dleft) / (2 * e)
    Xk[0] += ds * math.cos(Xk[2] + dtheta / 2)
    Xk[1] += ds * math.sin(Xk[2] + dtheta / 2)
    Xk[2] = angle_principal(Xk[2] + dtheta)
    list_pos_x.append(Xk[0])
    list_pos_y.append(Xk[1])
    list_theta.append(Xk[2])
    # print(Xk)

    ## Simulateur
    xyz = node.getPosition()
    rotation = node.getOrientation()
    list_pos_x_sim.append(xyz[0] * 1e3)
    list_pos_y_sim.append(xyz[1] * 1e3)
    list_theta_sim.append(math.atan2(rotation[3], rotation[0]))


def chgmt_base(p):
    theta = Xk[2]
    return [
        math.cos(theta) * (p[0] - Xk[0]) + math.sin(theta) * (p[1] - Xk[1]),
        -math.sin(theta) * (p[0] - Xk[0]) + math.cos(theta) * (p[1] - Xk[1]),
    ]


def compute_move(p):
    """p: point dans le repere du robot"""
    trans = math.sqrt(p[0] ** 2 + p[1] ** 2)
    rot = math.atan(p[1] / p[0])
    if p[0] < 0:
        rot = angle_principal(math.pi + rot)
    return (rot, trans)


def goto_rot_trans(rot, trans):
    theta_obj = Xk[2] + rot
    speed = 1 if rot > 0 else -1
    error = abs(angle_principal(Xk[2] - theta_obj))
    old_error = error + 1
    while error <= old_error:
        motor_left.setVelocity(-speed)
        motor_right.setVelocity(speed)
        update_pos()
        robot.step(timestep)
        old_error = error
        error = abs(angle_principal(Xk[2] - theta_obj))

    d = 0
    while abs(trans - d) > 1:
        motor_left.setVelocity(2)
        motor_right.setVelocity(2)
        dx = (
            (motor_right.getVelocity() + motor_left.getVelocity())
            * timestep
            * wheel_radius
            * 0.5e-3
        )
        d += dx
        update_pos()
        robot.step(timestep)
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)


c = 0


def goto(p):
    dest = chgmt_base(p)
    print("Point dans la base du robot :", dest)
    rot, trans = compute_move(dest)
    print(f"Mouvement a effectuer rotation : {rot}, translation : {trans}")
    goto_rot_trans(rot, trans)


while True:
    for p in path:
        print("Point cible :", p)
        goto(p)

while robot.step(timestep) != -1:
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

    if command == ord("G"):
        draw_comp()
    elif command == ord("P"):
        print(node.getPosition())
    update_pos()
    if c == 100:
        if DRAW:
            draw_pos()
        c = 0
    c += 1
