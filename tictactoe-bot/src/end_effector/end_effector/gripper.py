"""
GRIPPER NODE    -> Sends a string command to the arduino through a serial port

IS A SERVICE TO -> brain node

ASSUMPTIONS     -> Possible commands from brain node are "open" and "close"

Recieves a command from the brain node and sends it to the arduino. Commands are
specifically "open\n" and "close\n". The arduino code can be something like:

// Not 100% sure this is correct
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "close") {
      closeGripper();
      Serial.println("Gripper closed");
    } else if (cmd == "open") {
      openGripper();
      Serial.println("Gripper opened");
    } else {
      Serial.println("Unknown command");
    }
  }
}
"""

import rclpy
from rclpy.node import Node
from serial.serialutil import SerialException
from interfaces.srv import CloseGripper
import serial

class Gripper(Node):
    def __init__(self):
        super().__init__('gripper_node')

        try:
            self.serial_port = serial.Serial('/dev/ttyS4', 9600)
            self.serial_connected = True
            self.get_logger().info("Serial port opened successfully")
        except SerialException:
            self.serial_port = None
            self.serial_connected = False
            self.get_logger().warn("Arduino not connected")

        self.srv = self.create_service(CloseGripper, 'gripper_service', self.handle_end_effector_command)

    def handle_end_effector_command(self, request, response):
        command = request.command.strip().lower()
        self.get_logger().info(f"Received gripper command: {command}")

        if command in ["open", "close"]:
            self.serial_port.write((command + "\n").encode('utf-8'))
            response.success = True
            response.message = f"Sent {command} to Arduino."
        else:
            response.success = False
            response.message = f"Unknown command: {command}"

        return response

def main(args=None):
    rclpy.init(args=args)
    node = Gripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
