#!/usr/bin/env python3
#
#  control_node.py  –  serial bridge for LiDAR + encoder data
#  Publishes:
#     /scan   (sensor_msgs/LaserScan)
#     /odom   (nav_msgs/Odometry)  +  TF odom -> base_link
#
#  Adjust the CONST_* section to match the chassis

import math, re, serial, threading, time
import rclpy
from rclpy.node            import Node
from sensor_msgs.msg       import LaserScan
from nav_msgs.msg          import Odometry
from geometry_msgs.msg     import Quaternion, TransformStamped
from tf2_ros               import TransformBroadcaster
from std_msgs.msg          import String

# ------------------ USER-TUNABLE CONSTANTS ---------------------
CONST_TICKS_PER_REV   = 450   # encoder counts per wheel rev  (example)
CONST_WHEEL_DIAMETER  = 0.080 # 80 mm Mecanum wheels (m)
CONST_BASELINE        = 0.205 # distance between left/right wheel centres (m)
SCAN_SIZE             = 256   # 0-255 angles → 256 points/rev
PORT                  = '/dev/ttyUSB0'
BAUD                  = 115200
# ---------------------------------------------------------------

re_lidar = re.compile(r"LIDAR\s+angle=([\d.]+).*\s+dist=([\d.]+)\s+mm")
re_enc   = re.compile(r"ENC\s+LF=(-?\d+)\s+LB=(-?\d+)\s+RF=(-?\d+)\s+RB=(-?\d+)")

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # --- ROS 2 pubs/bcasters --------------------------------
        self.pub_scan  = self.create_publisher(LaserScan, 'scan', 10)
        self.pub_odom  = self.create_publisher(Odometry, 'odom', 10)
        self.tf_bcast  = TransformBroadcaster(self)

        self.tf_timer = self.create_timer(0.05, self.publish_tf)  # 20 Hz

        self.create_subscription(String, 'key_cmd', self.cb_key, 1)

        # --- Serial port ----------------------------------------
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.01)
            self.get_logger().info(f'Opened {PORT} @ {BAUD} baud')
        except Exception as e:
            self.get_logger().fatal(f'Cannot open {PORT}: {e}')
            raise SystemExit
        
        # --- Keyboard ----------------------------------------
        # Optional watchdog: stop the device if no key received for a while
        # self.last_rx = self.get_clock().now()
        # self.x_count = 0
        # self.declare_parameter('timeout_ms', 25)        # configurable
        # self.create_timer(0.075, self.watchdog_cb)

        # --- LiDAR scan buffer ----------------------------------
        self.ranges  = [float('inf')] * SCAN_SIZE
        self.intens  = [0.0] * SCAN_SIZE
        self.prev_angle = 0.0

        # --- Odometry state -------------------------------------
        self.x = self.y = self.yaw = 0.0
        self.last_ticks = None
        self.last_time  = time.time()
        self.stag_counter = 0

        # --- Background reader thread ---------------------------
        self.running = True
        self.th = threading.Thread(target=self.serial_loop, daemon=True)
        self.th.start()

    # -----------------------------------------------------------
    #  Main serial-reader/writer loop
    # -----------------------------------------------------------
    def serial_loop(self):
        while self.running:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:            # timeout; let Python breathe
                time.sleep(0.001)
                continue

            if line.startswith('LIDAR'):
                self.handle_lidar(line)
            elif line.startswith('ENC'):
                self.handle_enc(line)
            else:
                self.get_logger().info(line)

    def cb_key(self, msg: String):
        """Forward one ASCII byte to the serial port."""
        try:
            self.ser.write(msg.data.encode('ascii', 'ignore'))
            self.get_logger().info('Sent on Serial: ' + str(msg.data))
            #self.x_count = 0
            # self.ser.flush()            # optional – forces immediate TX
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
        self.last_rx = self.get_clock().now()

    # def watchdog_cb(self):
    #     """Send a stop byte if nothing received recently (optional)."""
    #     timeout_ms = self.get_parameter('timeout_ms').value
    #     if (self.get_clock().now() - self.last_rx).nanoseconds / 1e6 > timeout_ms:
    #         # Example: send ASCII 'x' to stop the robot
    #         try:
    #             if self.x_count < 20:
    #                 self.ser.write(b'x')
    #                 self.get_logger().info('Sent on Serial: x')
    #                 self.x_count += 1
    #         except serial.SerialException:
    #             pass
    #         self.last_rx = self.get_clock().now()

    # -----------------------------------------------------------
    #  LiDAR: collect points until angle wraps, then publish scan
    # -----------------------------------------------------------
    def handle_lidar(self, line):
        m = re_lidar.search(line)
        if not m:
            return
        angle_deg = float(m.group(1))
        dist_mm   = float(m.group(2))
        
        idx = int(angle_deg / 360.0 * SCAN_SIZE) % SCAN_SIZE
        self.ranges[idx] = dist_mm / 1000.0      # → metres
        self.intens[idx] = 1.0                   # quality placeholder

        # angle went from high to low ⇒ full revolution finished
        if angle_deg < self.prev_angle:
            self.publish_scan()
            # clear for next revolution
            self.ranges  = [float('inf')] * SCAN_SIZE
            self.intens  = [0.0] * SCAN_SIZE
        self.prev_angle = angle_deg

    def publish_scan(self):
        now = self.get_clock().now().to_msg()
        msg = LaserScan()
        msg.header.stamp = now
        msg.header.frame_id = 'laser'
        msg.angle_min    = 0.0
        msg.angle_max    = 2*math.pi
        msg.angle_increment = 2*math.pi / SCAN_SIZE
        msg.time_increment  = 0.0
        msg.scan_time       = 0.0
        msg.range_min   = 0.05
        msg.range_max   = 8.0
        msg.ranges      = self.ranges
        msg.intensities = self.intens
        self.pub_scan.publish(msg)

    # -----------------------------------------------------------
    #  Encoders: integrate differential odom, publish /odom & TF
    # -----------------------------------------------------------
    def handle_enc(self, line):
        m = re_enc.search(line)
        if not m:
            return
        ticks = [int(m.group(i)) for i in range(1,5)]  # LF, LB, RF, RB

        if ticks == self.last_ticks:
            self.stag_counter += 1
        else:
            self.stag_counter = 0
        
        if self.stag_counter == 3:
            return

        time_now = time.time()
        dt = time_now - self.last_time
        self.last_time = time_now

        if self.last_ticks is None:
            self.last_ticks = ticks
            return

        # Average left vs right wheels
        dL = sum(ticks[0:2]) - sum(self.last_ticks[0:2])
        dR = sum(ticks[2:4]) - sum(self.last_ticks[2:4])
        self.last_ticks = ticks

        distL = (dL / 2) * math.pi*CONST_WHEEL_DIAMETER / CONST_TICKS_PER_REV
        distR = (dR / 2) * math.pi*CONST_WHEEL_DIAMETER / CONST_TICKS_PER_REV
        d_center = (distL + distR) * 0.5
        d_yaw    = (distR - distL) / CONST_BASELINE

        # Update pose in odom frame
        self.yaw += d_yaw
        self.x   += d_center * math.cos(self.yaw)
        self.y   += d_center * math.sin(self.yaw)

        # Publish nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp    = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        q = quaternion_from_yaw(self.yaw)
        odom.pose.pose.orientation = q
        # (optional) compute linear/angular velocities:
        odom.twist.twist.linear.x  = d_center / dt if dt else 0.0
        odom.twist.twist.angular.z = d_yaw / dt    if dt else 0.0
        self.pub_odom.publish(odom)

        # Publish TF
    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation      = quaternion_from_yaw(self.yaw)
        self.tf_bcast.sendTransform(t)

    # -----------------------------------------------------------
    #  Shutdown cleanly
    # -----------------------------------------------------------
    def destroy_node(self):
        self.running = False
        self.th.join()
        self.ser.close()
        super().destroy_node()

# ---------- helpers -------------------------------------------
def quaternion_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(yaw/2.0)
    q.w = math.cos(yaw/2.0)
    return q

# ---------- main ----------------------------------------------
def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
