#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import time
import re

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'battery_status', 10)
        self.serial_port = '/dev/ttyACM0'  # Adjust as necessary
        self.baud_rate = 115200
        self.timeout = 1
        self.declare_parameter('serial_port', self.serial_port)
        self.declare_parameter('baud_rate', self.baud_rate)
        self.declare_parameter('timeout', self.timeout)
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.timer_period = 0.5  # 5 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.serial_connection = None
        self.connect_to_serial()

    def connect_to_serial(self):
        try:
            self.serial_connection = serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout)
            self.get_logger().info(f"Connected to {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.serial_connection = None

    def timer_callback(self):
        if self.serial_connection and self.serial_connection.in_waiting > 0:
            line = self.serial_connection.readline().decode('utf-8').strip()
            voltage = self.parse_voltage(line)
            if voltage is not None:
                battery_percentage = self.estimate_battery_percentage(voltage)
                msg = Float32MultiArray()
                msg.data = [voltage, battery_percentage]
                self.publisher_.publish(msg)
                #self.get_logger().info(f"Published: Voltage={voltage:.2f}V, Battery Percentage={battery_percentage:.2f}%")
        else:
            self.get_logger().warn("No data received from serial connection.")

    def parse_voltage(self, line):
        try:
            # Extract the first floating-point number from the line
            match = re.search(r"[-+]?\d*\.\d+|\d+", line)
            if match:
                return float(match.group())
            else:
                self.get_logger().warn(f"Unable to parse voltage from line: {line}")
                return None
        except ValueError:
            self.get_logger().warn(f"Invalid voltage reading: {line}")
            return None

    def estimate_battery_percentage(self, voltage):
        # LiPo battery voltage characteristics
        full_charge_voltage = 4.2
        empty_voltage = 3.2
        nominal_voltage = 3.7

        # Number of cells in series (adjust based on your battery configuration)
        num_cells = 2  # Example for a 2S battery

        # Calculate per-cell voltage
        per_cell_voltage = voltage / num_cells

        # Estimate battery percentage based on per-cell voltage
        if per_cell_voltage >= full_charge_voltage:
            return 100.0
        elif per_cell_voltage <= empty_voltage:
            return 0.0
        else:
            # Linear approximation between full and empty
            return ((per_cell_voltage - empty_voltage) / (full_charge_voltage - empty_voltage)) * 100

def main(args=None):
    rclpy.init(args=args)
    battery_monitor = BatteryMonitorNode()
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
