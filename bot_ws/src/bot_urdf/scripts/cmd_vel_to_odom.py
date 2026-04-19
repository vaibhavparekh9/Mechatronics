#!/usr/bin/env python3
"""Integrate /cmd_vel: holonomic odom/tf + optional wheel joint states (mecanum-style visual).

Not compiled: installed via CMake install(PROGRAMS) to lib/bot_urdf/.

Wheel spin uses only vx and angular.z (yaw rate). Lateral vy drives holonomic translation
but does not change wheel angles — a common RViz-only simplification (rollers do strafe).
"""

import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

WHEEL_JOINTS = (
    'base_to_whl_1',
    'base_to_whl_2',
    'base_to_whl_3',
    'base_to_whl_4',
)


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class CmdVelToOdom(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('max_dt', 0.2)
        self.declare_parameter('publish_wheel_joint_states', True)
        self.declare_parameter('wheel_radius', 0.048)
        self.declare_parameter('track_width', 0.34)
        # URDF: whl_1/2 axis +Y, whl_3/4 axis -Y — flip left pair if spin looks reversed
        self.declare_parameter('left_wheel_spin_sign', -1.0)

        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._publish_odom = self.get_parameter('publish_odom').get_parameter_value().bool_value
        self._max_dt = self.get_parameter('max_dt').get_parameter_value().double_value
        self._publish_joints = self.get_parameter(
            'publish_wheel_joint_states').get_parameter_value().bool_value
        self._wheel_r = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self._track = self.get_parameter('track_width').get_parameter_value().double_value
        self._left_sign = self.get_parameter('left_wheel_spin_sign').get_parameter_value().double_value

        self._half_track = 0.5 * self._track
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._vyaw = 0.0
        self._wheel_q = [0.0, 0.0, 0.0, 0.0]
        self._last_time = self.get_clock().now()

        self._tf_broadcaster = TransformBroadcaster(self)
        self._odom_pub = self.create_publisher(Odometry, 'odom', 10) if self._publish_odom else None
        self._joint_pub = (
            self.create_publisher(JointState, 'joint_states', 10) if self._publish_joints else None
        )

        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)
        self._timer = self.create_timer(1.0 / 50.0, self._integrate_and_publish)

        self.get_logger().info(
            f'Publishing tf {self._odom_frame} -> {self._base_frame} from /cmd_vel'
        )
        if self._publish_joints:
            self.get_logger().info(
                'Publishing wheel joint_states (vx + yaw only; vy strafe does not spin wheels)'
            )

    def _cmd_vel_cb(self, msg: Twist):
        self._vx = msg.linear.x
        self._vy = msg.linear.y
        self._vyaw = msg.angular.z

    def _wheel_qdots(self):
        """Right wheels 0,1; left 2,3. Differential approximation: no vy component."""
        if self._wheel_r <= 1e-6:
            return [0.0, 0.0, 0.0, 0.0]
        inv_r = 1.0 / self._wheel_r
        v_r = (self._vx + self._vyaw * self._half_track) * inv_r
        v_l = (self._vx - self._vyaw * self._half_track) * inv_r
        v_l *= self._left_sign
        return [v_r, v_r, v_l, v_l]

    def _integrate_and_publish(self):
        now = self.get_clock().now()
        dt = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now
        if dt <= 0.0:
            return
        dt = min(dt, self._max_dt)

        self._yaw += self._vyaw * dt
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._x += (c * self._vx - s * self._vy) * dt
        self._y += (s * self._vx + c * self._vy) * dt

        qdots = self._wheel_qdots()
        for i in range(4):
            self._wheel_q[i] += qdots[i] * dt

        qx, qy, qz, qw = yaw_to_quat(self._yaw)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(t)

        if self._odom_pub is not None:
            o = Odometry()
            o.header = t.header
            o.child_frame_id = self._base_frame
            o.pose.pose.position.x = self._x
            o.pose.pose.position.y = self._y
            o.pose.pose.position.z = 0.0
            o.pose.pose.orientation.x = qx
            o.pose.pose.orientation.y = qy
            o.pose.pose.orientation.z = qz
            o.pose.pose.orientation.w = qw
            o.twist.twist.linear.x = self._vx
            o.twist.twist.linear.y = self._vy
            o.twist.twist.angular.z = self._vyaw
            self._odom_pub.publish(o)

        if self._joint_pub is not None:
            js = JointState()
            js.header.stamp = now.to_msg()
            js.name = list(WHEEL_JOINTS)
            js.position = list(self._wheel_q)
            js.velocity = list(qdots)
            self._joint_pub.publish(js)


def main():
    rclpy.init()
    node = CmdVelToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
