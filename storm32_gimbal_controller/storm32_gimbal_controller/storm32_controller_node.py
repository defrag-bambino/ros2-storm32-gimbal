#!/usr/bin/env python3

import traceback
import tf_transformations
import rclpy
from rclpy.node import Node
from .storm32_usb_driver import Storm32
import numpy as np
import serial
import diagnostic_updater
import diagnostic_msgs.msg
from storm32_gimbal_interfaces.msg import GimbalOrientation, TargetGimbalOrientation
from std_srvs.srv import Trigger
from geometry_msgs.msg import QuaternionStamped, Quaternion


class Storm32GimbalController(Node):
    def __init__(self):
        super(Storm32GimbalController, self).__init__("storm32_gimbal_controller")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("frame_id", "gimbal_ref")
        self.port = str(self.get_parameter("serial_port").value)
        self.frame_id = str(self.get_parameter("frame_id").value)

        self.gimbal = Storm32(port=self.port)
        version = self.gimbal.get_version()

        # print some diagnostics
        if version:
            self.get_logger().info("Gimbal found at {0}".format(self.port))
            for k, v in version.items():
                self.get_logger().info("{0}: {1}".format(k, v))

            # set up the diagnostics updater
            self.updater = diagnostic_updater.Updater(self)
            self.updater.setHardwareID(self.frame_id)
            self.updater.add("Gimbal Diagnostics", self.get_diagnostics_status)

            # set up the publisher and subscriber
            self.target_orientation_sub = self.create_subscription(
                TargetGimbalOrientation,
                "~/target_orientation",
                self.set_target_orientation_callback,
                1,
            )
            self.camera_orientation_pub = self.create_publisher(
                GimbalOrientation, "~/camera_orientation", 1
            )
            self.controller_orientation_pub = self.create_publisher(
                GimbalOrientation, "~/controller_orientation", 1
            )
            self.restart_srv = self.create_service(
                Trigger, "~/restart", self.restart_controller
            )

            # Setup periodic callback for orientation publisher
            self.pub_timer = self.create_timer(0.01, self.pub_timer_callback)
        else:
            self.get_logger().fatal("Gimbal not found at {0}".format(self.port))

    def pub_timer_callback(self):
        """Periodic callback function to publish the current orientation of the
        gimbal. This function  will terminate the ROS node if it ecounters a
        SerialException.
        """
        # Create new message and header
        msg = GimbalOrientation()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        try:
            imu1 = (self.gimbal.get_imu1_angles(), self.camera_orientation_pub, "imu1")
            imu2 = (
                self.gimbal.get_imu2_angles(),
                self.controller_orientation_pub,
                "imu2",
            )
            for euler, pub, name in (imu1, imu2):
                if euler:
                    self.get_logger().debug(
                        "get_{0}_angles: pitch={1:.2f}, roll={2:.2f}, yaw={3:.2f}".format(
                            name, *euler
                        )
                    )
                    msg.pitch = euler[0]
                    msg.roll = euler[1]
                    msg.yaw = euler[2]
                    euler = list(map(np.radians, euler))
                    q = tf_transformations.quaternion_from_euler(
                        euler[0], euler[1], euler[2]
                    )
                    msg.quaternion.x = q[0]
                    msg.quaternion.y = q[1]
                    msg.quaternion.z = q[2]
                    msg.quaternion.w = q[3]
                    pub.publish(msg)

            # diagnostic update is called here because this run at a high rate
            self.updater.force_update()
        except serial.serialutil.SerialException as e:
            self.get_logger().fatal(traceback.format_exc())
            rclpy.shutdown()

    def restart_controller(self, request, response):
        """Callback function to restart the gimbal controller. This function will
        terminate the ROS node if it encounters a SerialException.

        Arg:
            req: Request object for the ROS Service.

        Returns: Service Response, "True" for success, "False" for error.
        """
        try:
            success = self.gimbal.restart_controller()
            response.success = success
            if success:
                response.message = "Gimbal restarted successfully!"

                # Shutdown the node on success restart
                self.destroy_subscription(self.target_orientation_sub)
                self.pub_timer.destroy()
                self.destroy_publisher(self.camera_orientation_pub)
                self.destroy_publisher(self.controller_orientation_pub)
                self.create_timer(0.1, self._restart_shutdown_callback)
            else:
                response.message = "Gimbal restart failed!"
            return response

        except serial.serialutil.SerialException as e:
            self.get_logger().fatal(traceback.format_exc())
            rclpy.shutdown()

    def _restart_shutdown_callback(self):
        """Restart callback used to make sure service is able to respond before
        shutting down.

        """
        self.get_logger().info("Gimbal restarted, node shutting down!")
        rclpy.shutdown()

    def get_diagnostics_status(self, stat):
        """Callback function for periodically update diagnostics. This function
        will terminate the ROS node if it encounters a SerialException.

        Arg:
            stat: Status object required for diaganostic_updater.

        Returns: Updated status object.
        """
        # Get the status
        status = self.gimbal.get_status()

        # Put all status into stat object
        if status:
            for k, v in status.items():
                stat.add(k, str(v))
            self.get_logger().info("Gimbal States: {0}".format(status["State"]))
            if status["Battery Connected"]:
                self.get_logger().info(
                    "VBAT={0}, Voltage {1}".format(
                        status["VBAT"], "low" if status["Bat Voltage Low"] else "OK"
                    )
                )
            else:
                self.get_logger().warn("Battery disconnected!")
            # Put the state of the gimbal into diagnostic summary
            state = status["State"]
            if state == "Normal":
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.OK, state)
            else:
                stat.summary(diagnostic_msgs.msg.DiagnosticStatus.WARN, state)
        return stat

    def set_target_orientation_callback(self, msg):
        """Callback function for setting target_orientation. This function will
        terminate the ROS node if it encounters a SerialException.

        Arg:
            msg: target_orientation message.
        """
        if msg.use_quaternion:
            quaternion = (
                msg.orientation.quaternion.x,
                msg.orientation.quaternion.y,
                msg.orientation.quaternion.z,
                msg.orientation.quaternion.w,
            )
            euler = tf_transformations.euler_from_quaternion(quaternion)
            euler = list(map(self.encode_angle, euler))
        else:
            euler = (msg.orientation.pitch, msg.orientation.roll, msg.orientation.yaw)
        self.get_logger().debug(
            "set_angles: pitch={0}, roll={1}, yaw={2}, unlimited={3}".format(
                euler[0], euler[1], euler[2], msg.unlimited
            )
        )
        try:
            response = self.gimbal.set_angles(*euler, unlimited=msg.unlimited)
            if response != [1, 150, 0]:
                self.get_logger().error(
                    "STorM32: set_pitch_roll_yaw:"
                    + " response error : {0}:s".format(response)
                )
        except serial.serialutil.SerialException as e:
            self.get_logger().fatal(traceback.format_exc())
            rclpy.shutdown()

    def encode_angle(self, x):
        """Convert radian angle into degree within +-180 degree.

        Args:
            x: Radian angle to be converted.

        Returns: Degree anagle within +-180 degree.
        """
        while x > np.pi:
            x -= 2 * np.pi
        while x < -np.pi:
            x += 2 * np.pi
        return x * 180 / np.pi


def main(args=None):
    rclpy.init(args=args)

    node = Storm32GimbalController()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
