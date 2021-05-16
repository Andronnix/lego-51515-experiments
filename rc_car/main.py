""" Docs at: https://hubmodule.readthedocs.io/en/latest/ """
import time
import math

from rshell import pyboard
from pynput.keyboard import Key, KeyCode, Listener

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

KeyW = KeyCode.from_char('w')
KeyA = KeyCode.from_char('a')
KeyS = KeyCode.from_char('s')
KeyD = KeyCode.from_char('d')
KeyP = KeyCode.from_char('p')
KeyO = KeyCode.from_char('o')


def drive():
    if not keys[Key.up] and not keys[Key.down]:
        left = 0
        right = 0
    elif keys[Key.down]:
        left = -1
        right = -1
    else:  # Up
        left = 1
        right = 1

    if keys[Key.left]:
        left = left * 0.5

    if keys[Key.right]:
        right = right * 0.5

    left = int(left * 100)
    right = int(right * 100)

    return "hub.port.E.motor.pwm({});hub.port.F.motor.pwm({})".format(-left, right)


def gimble_ctrl():
    H = 60
    V = 60
    horizontal = 0
    vertical = 0

    if keys[KeyA]:
        horizontal = -H
    elif keys[KeyD]:
        horizontal = H

    if horizontal == 0:
        if keys[KeyW]:
            vertical = V
        elif keys[KeyS]:
            vertical = -V
    else:
        vertical = int(-horizontal / 3)

    return "hub.port.A.motor.pwm({});hub.port.B.motor.pwm({});".format(vertical, horizontal)


def define_gimble_read(board):
    board.exec_raw(
        "def gimble_read(motorA, motorB, dist_sensor):"
        "return '{};{};{}'.format(motorA.motor.get()[0],motorB.motor.get()[0],dist_sensor.device.get()[0])"
    )


def gimble_read(board):
    d, e = board.exec_raw("print(gimble_read(motorA,motorB,dist_sensor),end='')")
    e = e.decode("utf8")
    if e != "":
        print(e)
        return None

    a, b, c = d.decode("utf8").split(";")
    a = float(a) / 180.0 * math.pi / 3.0
    b = float(b) / 180.0 * math.pi / 3.0
    c = 200 if c == "None" else int(c)

    l = c * math.cos(a)
    x = l * math.cos(b)
    y = l * math.sin(b)
    z = c * math.sin(a)

    return x, y, z


def define_port(board, port, type, name):
    d = None
    e = 'Not none'
    while d is None or e != "":
        d, e = board.exec_raw("hub.port.{}.{}.get()".format(port, type))
        d = d.decode('ascii')
        e = e.decode('ascii')
        if e != '':
            print(e)
        time.sleep(0.5)

    board.exec_raw("{} = hub.port.{}".format(name, port))
    print("Defined {}=hub.port.{}".format(name, port))


class Plot:
    def __init__(self):
        self._z = np.array([])
        self._x = np.array([])
        self._y = np.array([])
        self._prev = None

    def add(self, x, y, z):
        if self._prev == (x, y, z):
            return

        self._prev = (x, y, z)

        self._x = np.append(self._x, x)
        self._y = np.append(self._y, y)
        self._z = np.append(self._z, z)

    def show(self):
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.set_xlim3d(-200, 200)
        ax.set_ylim3d(-200, 200)
        ax.set_zlim3d(0, 210)

        ax.scatter3D(self._x, self._y, self._z, cmap='Greens')

        plt.show()

    def print(self):
        print(self._x)
        print(self._y)
        print(self._z)


if __name__ == "__main__":
    print("Connecting")
    board = pyboard.Pyboard("COM3")
    board.enter_raw_repl()

    board.exec_raw("import hub")
    board.exec_raw("import time")

    define_port(board, "A", "motor", "motorA")
    define_port(board, "B", "motor", "motorB")
    define_port(board, "C", "device", "dist_sensor")

    print(board.exec_raw("motorA.motor.mode(3);motorB.motor.mode(3)"))

    define_gimble_read(board)

    plot = Plot()
    # board.exec_raw("from mindstorms import DistanceSensor")

    print("Connected")

    keys = {Key.left: False,
            Key.right: False,
            Key.up: False,
            Key.down: False,
            Key.shift: False,
            Key.esc: False,
            KeyW: False,
            KeyA: False,
            KeyS: False,
            KeyD: False,
            KeyP: False,
            KeyO: False
    }

    def on_press(key):
        if key in keys:
            keys[key] = True

    def on_release(key):
        if key in keys:
            keys[key] = False

    prev_command = time.monotonic()
    prev_draw = time.monotonic()
    with Listener(on_press=on_press, on_release=on_release) as listener:
        print("Started listening")
        while not keys[Key.esc]:
            drive_command = drive()
            gimble_command = gimble_ctrl()

            delta = time.monotonic() - prev_command
            if delta > 0.05:
                board.exec_raw(str.encode(drive_command + ";" + gimble_command))

                gimble_pos = gimble_read(board)
                if gimble_pos is not None:

                    plot.add(*gimble_read(board))

                prev_command = time.monotonic()

            draw_delta = time.monotonic() - prev_draw
            if draw_delta > 0.5:
                plot.show()
                prev_draw = time.monotonic()

        listener.stop()
        print("Stopped listening")

    print("Exiting")
    board.exit_raw_repl()
    print("Done")
