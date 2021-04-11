import time

from rshell import pyboard
from pynput.keyboard import Key, Listener


if __name__ == "__main__":
    board = pyboard.Pyboard("COM5")
    board.enter_raw_repl()

    board.exec_raw("import hub")

    print("Connected")

    keys = {Key.left: False, Key.right: False, Key.up: False, Key.down: False, Key.shift: False}

    def on_press(key):
        if key in keys:
            keys[key] = True

    def on_release(key):
        if key in keys:
            keys[key] = False

    prev_left = 0
    prev_right = 0
    prev_command = 0

    # Collect events until released
    with Listener(on_press=on_press, on_release=on_release) as listener:
        print("Starting..")
        while not keys[Key.shift]:
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
            delta = time.clock() - prev_command

            if delta > 0.05:
                command = "hub.port.E.motor.pwm({});hub.port.F.motor.pwm({})"\
                    .format(-left, right)
                board.exec_raw(str.encode(command))

                prev_left = left
                prev_right = right

                prev_command = time.clock()


        listener.join()

    print("exiting")
    board.exit_raw_repl()
