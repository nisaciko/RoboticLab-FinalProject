#!/usr/bin/python3
# NB: must run on the SYSTEM python (/usr/bin/python3) — the gz-transport python
# bindings live in /usr/lib/python3/dist-packages and are NOT visible from a
# conda env. Run as ./simulation/teleop_key.py (this shebang forces the right
# interpreter even inside conda).
"""Hold-to-drive keyboard teleop for the Gazebo robot (publishes /cmd_vel).

Unlike the in-GUI Key Publisher (which latches a velocity until you press Space),
this gives press-and-hold to move / release to stop:

    - run this in a TERMINAL (keep the terminal focused while driving)
    - arrow keys or W/A/S/D : drive while held
    - release the key       : the robot stops after a short idle timeout
    - space                 : stop now
    - q                     : quit

How it works: terminals report key PRESS (with OS auto-repeat) but not release,
so we use a watchdog — while a key is held the OS repeats it (~30 ms), keeping
the command fresh; when you let go the repeats stop and after IDLE_STOP seconds
we publish zero. Works on Wayland (no global key listener needed).

Requires the gz-transport python bindings (ship with Gazebo Harmonic). Run the
simulation first (./simulation/run.sh), then this script.
"""
import os
import sys
import select
import termios
import time
import tty

from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist

LIN = 0.5        # m/s forward/back
ANG = 0.6        # rad/s turn
IDLE_STOP = 0.25 # seconds without a key -> stop
RATE = 50.0      # publish Hz

KEYS = {  # key -> (linear x, angular z)
    "w": (LIN, 0.0), "s": (-LIN, 0.0), "a": (0.0, ANG), "d": (0.0, -ANG),
    "\x1b[A": (LIN, 0.0), "\x1b[B": (-LIN, 0.0),   # up / down arrows
    "\x1b[D": (0.0, ANG), "\x1b[C": (0.0, -ANG),   # left / right arrows
    " ": (0.0, 0.0),
}


def read_keys(timeout):
    """Return the bytes available on stdin within `timeout` seconds ('' if none).

    Uses os.read (NOT sys.stdin.read, which blocks until its full count) so a
    single keystroke returns immediately and the watchdog can stop the robot.
    """
    r, _, _ = select.select([sys.stdin], [], [], timeout)
    if not r:
        return ""
    return os.read(sys.stdin.fileno(), 64).decode(errors="ignore")  # whatever's buffered


def main():
    node = Node()
    pub = node.advertise("/cmd_vel", Twist)
    print(__doc__)
    print(">>> driving... (q to quit)")

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    vx = wz = 0.0
    last_key = 0.0
    try:
        tty.setcbreak(fd)
        period = 1.0 / RATE
        while True:
            buf = read_keys(period)
            if buf:
                if "q" in buf:
                    break
                # take the last recognised key in the buffer
                cmd = None
                if buf[-3:] in KEYS:        # arrow escape sequence
                    cmd = KEYS[buf[-3:]]
                elif buf[-1] in KEYS:       # single char
                    cmd = KEYS[buf[-1]]
                if cmd is not None:
                    vx, wz = cmd
                    last_key = time.time()

            if time.time() - last_key > IDLE_STOP:
                vx = wz = 0.0

            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = wz
            pub.publish(msg)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        stop = Twist()
        pub.publish(stop)
        print("\nstopped.")


if __name__ == "__main__":
    main()
