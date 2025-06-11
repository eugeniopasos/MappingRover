# *****************************************************************
#  Bridge from ROS2 processing to ESP Station
#
#  control_node.py  –  serial bridge for LiDAR + encoder data
#  Publishes:
#     /scan   (sensor_msgs/LaserScan)
#     /odom   (nav_msgs/Odometry)  +  TF odom -> base_link

#  Subcribed:
#     key_cmd              (std_msgs/String)
#     /lookahead_point     (geometry_msg/PointStamped)
# *****************************************************************

import math, re, serial, threading, time, struct

import rclpy
from rclpy.node            import Node
from sensor_msgs.msg       import LaserScan
from nav_msgs.msg          import Odometry
from geometry_msgs.msg     import Quaternion, TransformStamped, PointStamped, Twist
from tf2_ros               import TransformBroadcaster
from std_msgs.msg          import String

# ------------------ USER-TUNABLE CONSTANTS ---------------------
CONST_TICKS_PER_REV   = 450   # encoder counts per wheel rev  (example)
CONST_WHEEL_DIAMETER  = 0.080 # 80 mm Mecanum wheels (m)
CONST_BASELINE        = 0.205 # distance between left/right wheel centres (m)
SCAN_SIZE             = 512   # points/rev

STOP_RADIUS           = 0.50          # m
CLEAR_RADIUS          = 0.55          # m  (must be > STOP_RADIUS)

PORT                  = '/dev/ttyUSB0' # USB Device name
BAUD                  = 921600
# ENCODER COUNTS
QUARTER_SPAN          = 2**14  # 16384
HALF_SPAN             = 2**15     # 32768
# ---------------------------------------------------------------

class SerialBridge(Node): 
    def __init__(self):
        super().__init__('serial_bridge')

        # --- ROS 2 pubs/bcasters --------------------------------

        self.pub_scan  = self.create_publisher(LaserScan, 'scan', 10)
        self.pub_odom  = self.create_publisher(Odometry, 'odom', 10)
        self.tf_bcast  = TransformBroadcaster(self)

        self.create_timer(0.5, self.direction_sender)

        self.create_subscription(String, 'key_cmd', self.cb_key, 1)
        self.create_subscription(PointStamped, '/lookahead_point', self.direction_finder ,10)

        # --- Serial port ----------------------------------------
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.01)
            self.get_logger().info(f'Opened {PORT} @ {BAUD} baud')
        except Exception as e:
            self.get_logger().fatal(f'Cannot open {PORT}: {e}')
            raise SystemExit

        # --- LiDAR scan buffer ----------------------------------
        self.ranges  = [float('inf')] * SCAN_SIZE
        self.intens  = [0.0] * SCAN_SIZE
        self.prev_angle = 0.0

        self.stopped = False
        self.reversing = False

        # --- Odometry state / Pose -------------------------------------
        self.x = self.y = self.yaw = self.yaw_prev = 0.0
        self.last_ticks = None
        self.last_time  = None

        self.r = 0.0
        self.theta = 0.0
        self.deg = 0.0

        self.autonomous = False

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

            if line.startswith('L,'):
                self.handle_lidar(line)
                #self.get_logger().info(line)
            elif line.startswith('E,'):
                self.handle_enc(line)
                #self.get_logger().info(line)
            else:
                self.get_logger().info(line)

    def cb_key(self, msg: String):

        """Forward one ASCII byte to the serial port."""
        if str(msg.data) == 't':
            self.autonomous = not(self.autonomous)
            self.get_logger().info("Mode: Autonomous: " + str(self.autonomous))

        try:
            self.ser.write(bytes([0x12]))
            self.ser.write(msg.data.encode('ascii', 'ignore'))
            #self.get_logger().info('Sent on Serial: ' + str(msg.data))
            #self.x_count = 0
            #self.ser.flush()            # optional – forces immediate TX
        except serial.SerialException as e:
            self.get_logger().error(f'Serial command write error: {e}')
            
    def direction_finder(self, msg: PointStamped):

        if self.autonomous == True:
            x = msg.point.x
            y = msg.point.y

            self.r = math.sqrt(x**2 + y**2)

            if self.r < 0.3:
                self.stop_avoid()
                return

            self.theta = math.atan2(y,x) # returns radians
            self.deg = self.theta * 180 / math.pi
            # self.get_logger().info(
            # f"r: {self.r:.2f}   theta: {self.deg:.2f}")

    def direction_sender(self):

        if self.autonomous == True and self.r >= 0.3 and self.stopped == False: 

            data = struct.pack('<f', self.deg)

            try:
                self.ser.write(bytes([0x13]))
                self.ser.write(data)
                # self.get_logger().info(
                # f"r: {self.r:.2f}   theta: {self.deg:.2f}")
                # self.get_logger().info('Sent on Serial: ' + str(msg.data))
                #self.x_count = 0
                # self.ser.flush()            # optional – forces immediate TX
            except serial.SerialException as e:
                self.get_logger().error(f'Serial direction write error: {e}')

    def stop_avoid(self):

        try:
            stop = 'x'
            self.ser.write(bytes([0x12]))
            self.ser.write(stop.encode('ascii', 'ignore'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial stop send error: {e}')

    def reverse(self):

        try:
            msg = 's'
            self.ser.write(bytes([0x12]))
            self.ser.write(msg.encode('ascii', 'ignore'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial stop send error: {e}')

    # -----------------------------------------------------------
    #  LiDAR: collect points until angle wraps, then publish scan
    # -----------------------------------------------------------
    def handle_lidar(self, line: str):
        if not line.startswith('L,'):
            return
        try:
            _, a, d, q = line.split(',')
            angle_deg  = float(a)
            dist_mm    = float(d) - 310.0
            quality    = float(q) # quality q for intensity
        except ValueError:                # bad / short CSV line
            return
        
        idx = int((360.0 - angle_deg) / 360.0 * SCAN_SIZE) % SCAN_SIZE
        self.ranges[idx] = dist_mm / 1000.0      # → metres
        self.intens[idx] = quality                  # quality

        if self.autonomous:
            if self.ranges[0]< STOP_RADIUS and not(self.stopped) and not(self.reversing):
                self.stop_avoid()
                self.get_logger().info("Stopping!")
                self.stopped = True
                
            elif self.ranges[0]< CLEAR_RADIUS and self.stopped and not(self.reversing):
                self.reverse()
                self.get_logger().info("Reversing!")
                self.reversing = True

            elif self.ranges[0]> CLEAR_RADIUS and self.stopped and self.reversing:
                self.stopped = False
                self.reversing = False


        # angle went from high to low ⇒ full revolution finished
        if angle_deg < self.prev_angle:
            self.publish_scan()
            # clear for next revolution
            self.intens  = [0.0] * SCAN_SIZE
            self.ranges  = [float('inf')] * SCAN_SIZE

        self.prev_angle = angle_deg

    def publish_scan(self):
        scan = LaserScan()
        scan.header.stamp = self.last_time.to_msg()
        scan.header.frame_id = 'laser'
        scan.angle_min    = 0.0
        scan.angle_max    = 2*math.pi
        scan.angle_increment = 2*math.pi / SCAN_SIZE
        scan.scan_time       = 0.1
        scan.time_increment  = scan.scan_time / SCAN_SIZE
        scan.range_min   = 0.01
        scan.range_max   = 12.0
        scan.ranges      = self.ranges
        scan.intensities = self.intens
        self.pub_scan.publish(scan)

    # -----------------------------------------------------------
    #  Encoders: integrate differential odom, publish /odom & TF
    # -----------------------------------------------------------
    def handle_enc(self, line: str):
        if not line.startswith('E,'):
            return
        try:
            _, lf, lb, rf, rb , yaw = line.split(',')
            ticks = [int(lf), int(lb), int(rf), int(rb)]
            if yaw == 'nan': 
                self.yaw = 0.0
            else:
                self.yaw = -float(yaw) * math.pi / 180

            # self.get_logger().info('yaw: ' + str(yaw))
            # self.get_logger().info('lf: ' + str(lf) + ' lb: ' + str(lb) + 
            #                        ' rf: ' + str(rf) + ' rb: '+ str(rb))
        except ValueError:
            return
        
        # first message: initialise and quit
        if self.last_ticks is None:
            self.last_ticks = ticks
            self.last_time  = self.get_clock().now()
            return
        
        # --- Δ ticks and Δ time (ROS 2 clock) ----------------------
        raw_deltas = [c - p for c, p in zip(ticks, self.last_ticks)]
        corrected = []
        for d in raw_deltas:
            if   d >  QUARTER_SPAN:
                d -= HALF_SPAN
            elif d < -QUARTER_SPAN:
                d += HALF_SPAN
            corrected.append(d)
        deltas = corrected
            
        self.last_ticks = ticks

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if not any(deltas):     
            d_center = 0.0            # robot is stationary
            d_yaw    = 0.0
        else:

            dL = (deltas[0] + deltas[1]) / 2
            dR = (deltas[2] + deltas[3]) / 2

            distL = dL * math.pi * CONST_WHEEL_DIAMETER / CONST_TICKS_PER_REV
            distR = dR * math.pi * CONST_WHEEL_DIAMETER / CONST_TICKS_PER_REV
            d_center = 0.5 * (distL + distR)

        # --- pose ---------------------------------------
        d_yaw = self.yaw - self.yaw_prev
        self.x   += d_center * math.cos(self.yaw)
        self.y   += d_center * math.sin(self.yaw)
        self.yaw_prev = self.yaw

        # --- publish odom + TF -------------
        q = quaternion_from_yaw(self.yaw)

        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x  = d_center / dt
        odom.twist.twist.angular.z = d_yaw / dt
    
        self.pub_odom.publish(odom)
        #self.get_logger().info(str(odom))

        t = TransformStamped()
        t.header = odom.header        # re‑use same stamp
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation      = q
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
