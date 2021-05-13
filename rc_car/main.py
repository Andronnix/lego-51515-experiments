""" Docs at: https://hubmodule.readthedocs.io/en/latest/ """
import time

from rshell import pyboard
from pynput.keyboard import Key, KeyCode, Listener


KeyW = KeyCode.from_char('w')
KeyA = KeyCode.from_char('a')
KeyS = KeyCode.from_char('s')
KeyD = KeyCode.from_char('d')


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


if __name__ == "__main__":
    print("Connecting")
    board = pyboard.Pyboard("COM3")
    board.enter_raw_repl()

    board.exec_raw("import hub")

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
    }

    def on_press(key):
        if key in keys:
            keys[key] = True

    def on_release(key):
        if key in keys:
            keys[key] = False

    prev_command = time.monotonic_ns()
    with Listener(on_press=on_press, on_release=on_release) as listener:
        print("Started listening")
        while not keys[Key.esc]:
            drive_command = drive()
            gimble_command = gimble_ctrl()

            delta = time.monotonic_ns() - prev_command
            if delta > 500:
                board.exec_raw(str.encode(drive_command + ";" + gimble_command))

                prev_command = time.monotonic()
            else:
                print(delta)

        listener.stop()
        print("Stopped listening")

    print("Exiting")
    board.exit_raw_repl()
    print("Done")
