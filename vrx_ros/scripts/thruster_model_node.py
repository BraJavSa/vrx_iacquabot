#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import Float64
import math

class ThrusterModelNode(Node):
    def __init__(self):
        super().__init__('thruster_model_node')
        
        # Richards generalized logistic curve parameters (asymmetric)
        # Positive command parameters
        self.A_pos = 0.000001
        self.K_pos = 40.0209
        self.B_pos = 2.6249
        self.v_pos = 0.1615
        self.C_pos = 0.9432
        self.M_pos = 0.00001
        
        # Negative command parameters
        self.A_neg = -31.4990
        self.K_neg = -0.00001
        self.B_neg = 3.6986
        self.v_neg = 0.3264
        self.C_neg = 0.9713
        self.M_neg = -1.0000
        
        # Timeout tracking for thruster commands (0.5s watchdog)
        self.last_lf_time = None
        self.last_lr_time = None
        self.last_rf_time = None
        self.last_rr_time = None
        self.timeout_sec = 0.5
        
        # Subscriptions: Normalized command topics in range [-1.0, 1.0] (relative to namespace)
        self.sub_lf_cmd = self.create_subscription(Float64, 'thrusters/left_front/cmd', self.lf_cmd_callback, 10)
        self.sub_lr_cmd = self.create_subscription(Float64, 'thrusters/left_rear/cmd', self.lr_cmd_callback, 10)
        self.sub_rf_cmd = self.create_subscription(Float64, 'thrusters/right_front/cmd', self.rf_cmd_callback, 10)
        self.sub_rr_cmd = self.create_subscription(Float64, 'thrusters/right_rear/cmd', self.rr_cmd_callback, 10)
        
        # Publishers: Raw thrust force in Newtons sent to the Gazebo simulation bridge (relative to namespace)
        self.pub_lf_thrust = self.create_publisher(Float64, 'thrusters/left_front/thrust', 10)
        self.pub_lr_thrust = self.create_publisher(Float64, 'thrusters/left_rear/thrust', 10)
        self.pub_rf_thrust = self.create_publisher(Float64, 'thrusters/right_front/thrust', 10)
        self.pub_rr_thrust = self.create_publisher(Float64, 'thrusters/right_rear/thrust', 10)
        
        # Watchdog timer running at 20Hz (every 0.05s) to enforce command timeout
        self.watchdog_timer = self.create_timer(0.05, self.watchdog_callback)
        
        self.get_logger().info("Asymmetric Richard's Thruster Model Node initialized with 0.5s timeout.")

    def compute_thrust(self, cmd):
        if abs(cmd) <= 0.01:
            return 0.0
        elif cmd > 0.01:
            # Richards curve (Positive command)
            exponent = -self.B_pos * (cmd - self.M_pos)
            # Clamp exponent to prevent overflow in math.exp
            exponent = max(-50.0, min(50.0, exponent))
            denominator = (self.C_pos + math.exp(exponent)) ** (1.0 / self.v_pos)
            thrust = self.A_pos + (self.K_pos - self.A_pos) / denominator
            return thrust
        else:
            # Richards curve (Negative command)
            exponent = -self.B_neg * (cmd - self.M_neg)
            # Clamp exponent to prevent overflow in math.exp
            exponent = max(-50.0, min(50.0, exponent))
            denominator = (self.C_neg + math.exp(exponent)) ** (1.0 / self.v_neg)
            thrust = self.A_neg + (self.K_neg - self.A_neg) / denominator
            return thrust

    def lf_cmd_callback(self, msg):
        self.last_lf_time = self.get_clock().now().nanoseconds * 1e-9
        cmd = msg.data
        thrust_val = self.compute_thrust(cmd)
        out_msg = Float64()
        out_msg.data = thrust_val
        self.pub_lf_thrust.publish(out_msg)

    def lr_cmd_callback(self, msg):
        self.last_lr_time = self.get_clock().now().nanoseconds * 1e-9
        cmd = msg.data
        thrust_val = self.compute_thrust(cmd)
        out_msg = Float64()
        out_msg.data = thrust_val
        self.pub_lr_thrust.publish(out_msg)

    def rf_cmd_callback(self, msg):
        self.last_rf_time = self.get_clock().now().nanoseconds * 1e-9
        cmd = msg.data
        thrust_val = self.compute_thrust(cmd)
        out_msg = Float64()
        out_msg.data = thrust_val
        self.pub_rf_thrust.publish(out_msg)

    def rr_cmd_callback(self, msg):
        self.last_rr_time = self.get_clock().now().nanoseconds * 1e-9
        cmd = msg.data
        thrust_val = self.compute_thrust(cmd)
        out_msg = Float64()
        out_msg.data = thrust_val
        self.pub_rr_thrust.publish(out_msg)

    def watchdog_callback(self):
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        zero_msg = Float64()
        zero_msg.data = 0.0

        if self.last_lf_time is not None and (now_sec - self.last_lf_time) > self.timeout_sec:
            self.pub_lf_thrust.publish(zero_msg)

        if self.last_lr_time is not None and (now_sec - self.last_lr_time) > self.timeout_sec:
            self.pub_lr_thrust.publish(zero_msg)

        if self.last_rf_time is not None and (now_sec - self.last_rf_time) > self.timeout_sec:
            self.pub_rf_thrust.publish(zero_msg)

        if self.last_rr_time is not None and (now_sec - self.last_rr_time) > self.timeout_sec:
            self.pub_rr_thrust.publish(zero_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ThrusterModelNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
