# This Python file uses the following encoding: utf-8

# import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import HistoryPolicy, LivelinessPolicy


class RosNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                                 durability=DurabilityPolicy.VOLATILE,
                                 history=HistoryPolicy.KEEP_LAST,
                                 liveliness=LivelinessPolicy.AUTOMATIC,
                                 depth=1)

        self.publisher_ESC = self.create_publisher(Int16, '/Robot1/ESC', qos_profile)  # nopep8
        self.publisher_SER = self.create_publisher(Int16, '/Robot1/Servo', qos_profile)  # nopep8
        self.subscription_phidget_speed = self.create_subscription(Float32, "/Robot1/Speed", self.imu_callback, qos_profile)  # nopep8
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.esc_pub_value = 1500
        self.servo_pub_value = 1400
        self.imu_speed_value = 0

    def timer_callback(self):
        esc_msg = Int16()
        esc_msg.data = self.esc_pub_value
        self.publisher_ESC.publish(esc_msg)
        ser_msg = Int16()

        ser_msg.data = self.servo_pub_value
        self.publisher_SER.publish(ser_msg)
        self._esc_last_value = self.esc_pub_value

    def imu_callback(self, msg):
        self.imu_speed_value = int(msg.data)
