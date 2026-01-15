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

list_pos_x = []
list_pos_y = []

list_pos_x_sim = []
list_pos_y_sim = []

list_theta = []
list_theta_sim = []


def draw_pos():
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


c = 0

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
            print("up")
            if robot_speed < MAX_SPEED:
                robot_speed += 0.1
        elif command == keyboard.DOWN:
            print("down")
            if robot_speed > -2:
                robot_speed -= 0.1
        elif command == 83:  # capture S key
            print("stop")
            robot_speed = 0
        motor_left.setVelocity(robot_speed)
        motor_right.setVelocity(robot_speed)

    if command == 84:
        draw_comp()
    update_pos()
    if c == 100:
        draw_pos()
        c = 0
    c += 1
