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

from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())
keyboard = robot.getKeyboard()
keyboard.enable(timestep)
robot_speed = 0.0

speed_left = 0.0
speed_right = 0.0
direction = "up"

print(chr(27) + "[2J")  # ANSI code for clearing command line
print("Initialization of thymio_variables controller")

motor_left = robot.getDevice("motor.left")
motor_right = robot.getDevice("motor.right")
motor_left.setPosition(float("inf"))
motor_right.setPosition(float("inf"))

ir_sens = []

for i in range(0, 7):
    ir_sens.append(robot.getDevice(f"prox.horizontal.{i}"))
    ir_sens[i].enable(timestep)

print("Sampling period : ", timestep, "ms")


def avancer_ts(ts=100, speed=0.5):
    for _ in range(ts):
        motor_left.setVelocity(speed)
        motor_right.setVelocity(speed)
        robot.step(timestep)
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)


def avancer(speed=2):
    speed_left = speed
    speed_right = speed


def avancer1carreau():
    avancer_ts(ts=1000, speed=1)


# avancer1carreau()


def rotate_ts(ts, speed=1):
    """if speed is positive goes right, and left if otherwise"""
    for _ in range(ts):
        motor_left.setVelocity(speed)
        motor_right.setVelocity(-speed)
        robot.step(timestep)
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)


def rotate(angle):
    speed = 1 if angle > 0 else -1
    rotate_ts(int(abs(angle) * 4.9), speed=speed)


def wait(ts):
    for _ in range(ts):
        robot.step(timestep)


def getProx():
    return [s.getValue() for s in ir_sens]



def is_obstacle():
    prox = getProx()
    dist = (prox[1] + prox[2] + prox[3]) / 3
    return (dist > 2500)


def stop():
    motor_left.setVelocity(0)
    motor_right.setVelocity(0)


while robot.step(timestep) != -1:
    ## Controle auto
    # prox = getProx()
    # for p in prox:
    #     print(p)
    # print("\n")

    # speed_right = 2
    # speed_left = 2

    # # Si obstacle devant
    # if is_obstacle():
    #     rotate(90)
    #     for i in range(0, 6):
    #         if prox[i] > 0:
    #             rotate(180)
    #             break

    # # Si il est trop proche d'un mur (a droite ou a gauche)
    # if prox[0] > 0:
    #     rotate(10)
    # if prox[4] > 0:
    #     rotate(-10)

    # motor_left.setVelocity(speed_left)
    # motor_right.setVelocity(speed_right)

    ## Controle clavier ##
    command = keyboard.getKey()
    if command == keyboard.LEFT:
        # print('Left')
        motor_left.setVelocity(-robot_speed)
        motor_right.setVelocity(robot_speed)
    elif command == keyboard.RIGHT:
        # print('right')
        motor_left.setVelocity(robot_speed)
        motor_right.setVelocity(-robot_speed)
    else:
        if command == keyboard.UP:
            print("up")
            if robot_speed < 2:
                robot_speed += 0.2
        elif command == keyboard.DOWN:
            print("down")
            if robot_speed > -2:
                robot_speed -= 0.2
        elif command == 83:  # capture S key
            print("stop")
            robot_speed = 0
        motor_left.setVelocity(robot_speed)
        motor_right.setVelocity(robot_speed)
