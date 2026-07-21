"""
this calculates attitute readings from the imu by fusing gyroscope + accelerometer + magnetomer readings using a complementary filter.

subscribes:
- imu data

publishes:
- topic called /attitude 
    - x = roll
    - y = pitch
    - z = yaw

it will work trust
"""

import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3Stamped
# ADJUST ME: import your actual combined 9DOF message type here, e.g.:
#   from my_imu_pkg.msg import Imu9DOF
# Using sensor_msgs/Imu as a structural placeholder just so the file runs;
# replace with the real type once you know it.
from sensor_msgs.msg import Imu as CombinedImuMsg


def wrap_angle(a):
    #effectively just keeps the angle in the range -pi to pi so that u don't get 387 degrees
    return math.atan2(math.sin(a), math.cos(a))

class ComplementaryFilter:
    def __init__(self, alpha = 0.98, yaw_alpha = 0.98):
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = alpha
        self.yaw_alpha = yaw_alpha
        self.initialized = False

    def get_accel_roll_pitch(self, accel):
        ax, ay, az = accel
        roll = math.atan2(ay, az)
        pitch = math.atan2(-ax, math.sqrt(ay*ay + az*az))
        return roll, pitch
    
    def get_yaw_from_mag(self, mag, roll, pitch):
        mx, my, mz = mag
        #doing some tilt compensation down here
        mx2 = mx * math.cos(pitch) + mz * math.sin(pitch)
        my2 = (mx * math.sin(roll) * math.sin(pitch)
               + my * math.cos(roll)
               - mz * math.sin(roll) * math.cos(pitch))
        return math.atan2(-my2, mx2)

    def update_gyro(self, gyro, dt):
        wx, wy, wz = gyro
        #yeah this is hte formula alright
        self.roll += wx * dt
        self.pitch += wy * dt
        self.yaw += wz * dt
        #making sure the angles aren't fried
        self.roll = wrap_angle(self.roll)
        self.pitch = wrap_angle(self.pitch)
        self.yaw = wrap_angle(self.yaw)
    
    def filter_accel_gyro(self, accel):
        roll_meas, pitch_meas = self.get_accel_roll_pitch(accel)
        if not self.initialized:
            self.roll, self.pitch = roll_meas, pitch_meas
            self.initialized = True
            return
        #basically self.roll is the gyro meas, and ur doing the filter
        self.roll = wrap_angle(
            self.alpha * self.roll + (1 - self.alpha) * roll_meas
        )
        self.pitch = wrap_angle(
            self.alpha * self.pitch + (1 - self.alpha) * pitch_meas
        )
    
    def filter_accel_mag(self, mag):
        #the same complementary filter, but wrapped b/c 179 and -179 degrees are basically the same
        #sub to make sure
        yaw_meas = self.get_yaw_from_mag(mag, self.roll, self.pitch)
        diff = wrap_angle(yaw_meas - self.yaw)
        self.yaw = wrap_angle(self.yaw + (1 - self.yaw_alpha) * diff)

class AttitudeNode(Node):
    def __init__(self):
        super().__init__('attitude_node')
        #check below
        self.declare_parameter('sources', ['/imu/realsense', '/imu/lsm9ds1'])
        self.declare_parameter('output_topic', '/attitude')
        self.declare_parameter('publish_rate_hz', 100.0)
        self.declare_parameter('alpha', 0.98) 
        self.declare_parameter('yaw_alpha', 0.98)
        #below parameters is the window in which we can safely assume that the reading is mostly gravity, and you can use it for tilt correction
        self.declare_parameter('accel_gate_low', 0.85)
        self.declare_parameter('accel_gate_high', 1.15)

        self._sources = list(self.get_parameter('sources').value)
        output_topic = self.get_parameter('output_topic').value
        rate = float(self.get_parameter('publish_rate_hz').value)
        alpha = self.get_parameter('alpha').value
        yaw_alpha = self.get_parameter('yaw_alpha').value
        self.accel_gate_low = self.get_parameter('accel_gate_low').value
        self.accel_gate_high = self.get_parameter('accel_gate_high').value
        #learn qos

        #self._latest = {}  # topic -> (Imu, monotonic stamp) find out what this is forw
        self.filter = ComplementaryFilter(alpha=alpha, yaw_alpha=yaw_alpha)
        self.last_time = None

        #replace 20 w/ qos when i figure out what that is. also 10
        for topic in self._sources:
            self.create_subscription(Imu, topic, self._make_cb(topic), 20)
        
        self.pub = self.create_publisher(Vector3Stamped, output_topic, 10)
        self._last_active = None
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            f'Complementary filter.{self._sources} -> {output_topic} @ {rate}Hz'
        )

    def imu_callback(self, msg):   
        now = time.monotonic()
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        self.last_time = now
        if dt <= 0.0:
            return

        # ADJUST ME: these three lines assume field names matching
        # sensor_msgs/Imu + a magnetic_field field. Replace with your
        # actual message's field names once you've checked with
        # `ros2 interface show`.
        gyro = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        mag = np.array([
            msg.magnetic_field.x,   # ADJUST ME if your combined msg names this differently
            msg.magnetic_field.y,
            msg.magnetic_field.z,
        ])

        # --- gyro: always integrate forward ---
        self.filter.update_gyro(gyro, dt)
 
        # --- accel: correct roll/pitch, only when the reading looks like pure gravity ---
        g = 9.80665
        accel_mag_g = np.linalg.norm(accel) / g
        if self.accel_gate_low < accel_mag_g < self.accel_gate_high:
            self.filter.correct_accel(accel)

        
        # --- mag: correct yaw, using this same message's reading (no caching needed
        #     now, since everything arrives together on one topic) ---
        if np.linalg.norm(mag) > 1e-9:
            self.filter.correct_mag(mag)
 
        self.publish_attitude(msg.header.frame_id or 'imu_link')
    
    def publish_attitude(self, frame_id):
        out = Vector3Stamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = frame_id
        out.vector.x = self.filter.roll
        out.vector.y = self.filter.pitch
        out.vector.z = self.filter.yaw
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = AttitudeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()
