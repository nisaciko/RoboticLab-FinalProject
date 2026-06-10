"""Arrow-key / WASD keyboard teleop.

Hold a key → robot moves continuously.
Release the key → robot stops within 150 ms.
Works by exploiting OS key-repeat: while a key is held the terminal receives
repeated characters; when released, characters stop arriving and the watchdog
timer zeroes the velocity command.
"""
import sys
import select
import termios
import tty
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

_LIN = 0.5   # m/s forward / backward
_ANG = 0.8   # rad/s turn

# terminal keycode → (linear_x, angular_z)
_KEY_MAP = {
    '\x1b[A': ( _LIN,  0.0),   # Up    arrow
    '\x1b[B': (-_LIN,  0.0),   # Down  arrow
    '\x1b[D': ( 0.0,   _ANG),  # Left  arrow
    '\x1b[C': ( 0.0,  -_ANG),  # Right arrow
    'w':      ( _LIN,  0.0),
    's':      (-_LIN,  0.0),
    'a':      ( 0.0,   _ANG),
    'd':      ( 0.0,  -_ANG),
    ' ':      ( 0.0,   0.0),   # Space = stop
}

_STOP_TIMEOUT = 0.15   # seconds without a key repeat → send stop


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self._pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._lin = 0.0
        self._ang = 0.0
        self._last_key_t = 0.0

        # publish at 20 Hz; also acts as the watchdog that zeros velocity
        self.create_timer(0.05, self._publish_cb)

        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()

        print('\n--- Robot Keyboard Teleop ---')
        print('  ↑ / W  : forward')
        print('  ↓ / S  : backward')
        print('  ← / A  : turn left')
        print('  → / D  : turn right')
        print('  Space  : stop')
        print('  Ctrl-C : quit')
        print('Hold any key to keep moving. Release to stop.\n')

    # ── key reader (runs in background thread) ────────────────────────────────

    def _read_loop(self):
        fd = sys.stdin.fileno()
        saved = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while rclpy.ok():
                ready = select.select([sys.stdin], [], [], 0.05)[0]
                if not ready:
                    continue
                ch = sys.stdin.read(1)
                if ch == '\x1b':
                    # escape sequence — read up to 2 more bytes (arrow keys)
                    more = select.select([sys.stdin], [], [], 0.02)[0]
                    if more:
                        ch += sys.stdin.read(1)
                        more2 = select.select([sys.stdin], [], [], 0.02)[0]
                        if more2:
                            ch += sys.stdin.read(1)
                if ch == '\x03':    # Ctrl-C
                    break
                if ch in _KEY_MAP:
                    self._lin, self._ang = _KEY_MAP[ch]
                    self._last_key_t = self.get_clock().now().nanoseconds * 1e-9
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, saved)

    # ── publish timer ─────────────────────────────────────────────────────────

    def _publish_cb(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_key_t > 0 and (now - self._last_key_t) > _STOP_TIMEOUT:
            self._lin = 0.0
            self._ang = 0.0
        twist = Twist()
        twist.linear.x  = self._lin
        twist.angular.z = self._ang
        self._pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # send a final stop before quitting
        stop = Twist()
        node._pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
