import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from my_gripper.srv import EndEffectorCmd

class EndEffectorNode(Node):
    def __init__(self):
        super().__init__('end_effector_node')

        # Publisher to send motor commands to the arduino_serial_node
        self.arduino_command_publisher = self.create_publisher(String, 'arduinoCommand', 10)

        # Service to handle commands from your main robot "brain"
        self.srv = self.create_service(EndEffectorCmd, 'end_effector_srv', self.handle_end_effector_command)
        
        self.get_logger().info("End Effector service is ready. Waiting for commands...")

    def handle_end_effector_command(self, request, response):
        command = request.command
        self.get_logger().info(f"Received service command: {command}")

        # Handle commands from your UR5e node
        if command == "GRASP":
            self.send_command_to_arduino("GRASP") # Send "GRASP" to arduino_serial_node
            response.success = True
            response.message = "Grasp command sent."

        elif command == "RELEASE":
            self.send_command_to_arduino("RELEASE") # Send "RELEASE" to arduino_serial_node
            response.success = True
            response.message = "Release command sent."
            
        else:
            self.get_logger().warn(f"Unknown command received: {command}")
            response.success = False
            response.message = f"Unknown command: {command}"

        return response

    def send_command_to_arduino(self, command):
        msg = String()
        msg.data = command
        self.get_logger().info(f"Publishing to /arduinoCommand: {command}")
        self.arduino_command_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EndEffectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()