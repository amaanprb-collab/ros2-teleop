import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import math
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk
    q = [0.0]*4
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')

        # Robot Physical Parameters
        self.declare_parameter('wheel_radius', 0.04) # 4cm
        self.declare_parameter('track_width', 0.26) # 26cm
        self.declare_parameter('cpr', 2320.0) # 2320 ticks per rev
        self.declare_parameter('max_rpm', 333.0) 
        self.declare_parameter('max_linear_speed', 1.4)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('left_pwm_multiplier', 1.0)
        self.declare_parameter('right_pwm_multiplier', 1.0)
        self.declare_parameter('serial_port', '/dev/ttyCH341USB0')
        self.declare_parameter('baud_rate', 115200)

        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.track_width = self.get_parameter('track_width').value
        self.cpr = self.get_parameter('cpr').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.left_pwm_multiplier = self.get_parameter('left_pwm_multiplier').value
        self.right_pwm_multiplier = self.get_parameter('right_pwm_multiplier').value
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value

        # Connect to Arduino
        try:
            self.serial = serial.Serial(serial_port, baud_rate, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.serial = None

        # Odometry State
        self.last_ticks_l = 0
        self.last_ticks_r = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # Smooth Motion State
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.current_w = 0.0
        
        # Acceleration Limits (prevents jerking)
        self.max_accel_v = 0.5  # max change of 0.5 m/s per second
        self.max_accel_w = 2.0  # max change of 2.0 rad/s per second
        self.dt = 0.05 # 20 Hz control loop

        # ROS 2 Interfaces
        self.sub_cmd = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Main control loop
        self.create_timer(self.dt, self.control_loop)
        
        self.get_logger().info("Base driver node ready. Awaiting /cmd_vel messages.")

    def cmd_callback(self, msg):
        # 1. Input Constraints: Enforce safe speed limits
        max_v = (self.max_rpm / 60.0) * 2 * math.pi * self.wheel_radius
        max_w = (max_v * 2) / self.track_width
        
        # Limit to configured safe speeds
        safe_v = min(max_v, self.max_linear_speed) 
        safe_w = min(max_w, self.max_angular_speed)
        
        self.target_v = max(min(msg.linear.x, safe_v), -safe_v)
        self.target_w = max(min(msg.angular.z, safe_w), -safe_w)

    def control_loop(self):
        # 2. Smooth Acceleration Ramping
        dv = self.target_v - self.current_v
        if dv > self.max_accel_v * self.dt:
            self.current_v += self.max_accel_v * self.dt
        elif dv < -self.max_accel_v * self.dt:
            self.current_v -= self.max_accel_v * self.dt
        else:
            self.current_v = self.target_v

        dw = self.target_w - self.current_w
        if dw > self.max_accel_w * self.dt:
            self.current_w += self.max_accel_w * self.dt
        elif dw < -self.max_accel_w * self.dt:
            self.current_w -= self.max_accel_w * self.dt
        else:
            self.current_w = self.target_w

        # 3. Differential Drive Kinematics (v, w -> v_l, v_r)
        v_l = self.current_v - (self.current_w * self.track_width / 2.0)
        v_r = self.current_v + (self.current_w * self.track_width / 2.0)

        # 4. Wheel Velocities to PWM
        # Convert m/s to RPM
        rpm_l = (v_l / (2 * math.pi * self.wheel_radius)) * 60.0
        rpm_r = (v_r / (2 * math.pi * self.wheel_radius)) * 60.0

        # Map RPM to PWM (0-255) and apply calibration multipliers
        pwm_l_raw = (rpm_l / self.max_rpm) * 255.0 * self.left_pwm_multiplier
        pwm_r_raw = (rpm_r / self.max_rpm) * 255.0 * self.right_pwm_multiplier

        pwm_l = int(max(min(pwm_l_raw, 255.0), -255.0))
        pwm_r = int(max(min(pwm_r_raw, 255.0), -255.0))

        # 5. Send Command to Arduino
        if self.serial:
            cmd_str = f"<{pwm_l},{pwm_r}>\n"
            try:
                self.serial.write(cmd_str.encode())
            except Exception as e:
                self.get_logger().error(f"Serial write error: {e}")

            # 6. Read Odometry Ticks from Arduino
            while self.serial.in_waiting > 0:
                try:
                    line = self.serial.readline().decode().strip()
                    if line.startswith("<") and line.endswith(">"):
                        line = line[1:-1]
                        parts = line.split(',')
                        if len(parts) == 2:
                            ticks_l = int(parts[0])
                            ticks_r = int(parts[1])
                            self.update_odometry(ticks_l, ticks_r)
                except Exception as e:
                    pass

    def update_odometry(self, current_ticks_l, current_ticks_r):
        current_time = self.get_clock().now()
        dt_time = (current_time - self.last_time).nanoseconds / 1e9
        if dt_time <= 0:
            return

        # Delta Ticks
        delta_l = current_ticks_l - self.last_ticks_l
        delta_r = current_ticks_r - self.last_ticks_r

        self.last_ticks_l = current_ticks_l
        self.last_ticks_r = current_ticks_r

        # Distance per tick (meters)
        dist_per_tick = (2.0 * math.pi * self.wheel_radius) / self.cpr

        d_l = delta_l * dist_per_tick
        d_r = delta_r * dist_per_tick

        # Center distance and change in heading
        d_c = (d_l + d_r) / 2.0
        d_th = (d_r - d_l) / self.track_width

        # Integrate Odometry
        self.th += d_th
        self.x += d_c * math.cos(self.th)
        self.y += d_c * math.sin(self.th)

        # Calculate Velocities
        vx = d_c / dt_time
        vth = d_th / dt_time

        self.last_time = current_time

        # Publish Odometry Message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, self.th)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = vth

        self.pub_odom.publish(odom)

        # Publish TF Transform
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial:
            node.serial.write(b"<0,0>\n") # Stop motors on exit
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
