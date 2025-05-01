#!/usr/bin/env python3
"""
Publish raw WASD key-strokes on /key_cmd as std_msgs/String.

Press Q or Ctrl-C to quit.
"""
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.pub = self.create_publisher(String, 'key_cmd', 1)
        # Timer at 50 Hz merely to give rclpy spinning time
        self.create_timer(0.02, lambda: None)
        # Save and switch the terminal into raw (non-canonical) mode
        self._old_tty_attr = termios.tcgetattr(sys.stdin.fileno())
        tty.setraw(sys.stdin.fileno())
        self.get_logger().info('Listening for WASD (q to quit)…')

    def destroy_node(self):
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._old_tty_attr)
        super().destroy_node()

    def spin(self):
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)      # non-blocking thanks to raw mode
                if not ch:
                    continue
                if ch.lower() in ['w', 'a', 's', 'd', 'x', 'r', 'f', 'u', 
                                  'i', 'o', 'j', 'k', 'l', 'n', ',']:
                    msg = String()
                    msg.data = ch.lower()
                    self.pub.publish(msg)
                elif ch.lower() == 'q':
                    break
        finally:
            # Restore TTY even on exceptions
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPublisher()
    node.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
