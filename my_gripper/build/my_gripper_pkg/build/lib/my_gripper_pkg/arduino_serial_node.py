import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32 # Import Int32
import serial

class ArduinoNode(Node):
    def __init__(self):
        super().__init__('arduino_serial_node')
        
        # Subscribes to the /arduinoCommand topic
        self.subscription = self.create_subscription(String, 'arduinoCommand', self.command_callback, 10)

        # Publishes data from Arduino to /arduinoResponse
        # --- FIX: Changed from String to Int32 ---
        self.response_publisher = self.create_publisher(Int32, 'arduinoResponse', 10)

        try:
            # --- !! YOU MUST CHANGE THIS !! ---
            usb_port = '/dev/ttyACM0' # Find your port with 'ls /dev/tty*'
            # -----------------------------------
            baud_rate = 9600 # Must match Serial.begin(9600) in Arduino
            
            self.serial_port = serial.Serial(usb_port, baud_rate, timeout=1.0)  
            self.get_logger().info(f"Serial connection established on {usb_port} at {baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            rclpy.shutdown() # Exit if we can't connect
            return

        # Timer to read from serial port every 0.1 seconds
        self.create_timer(0.1, self.read_arduino_response)

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Sending to Arduino: '{command}'")
        self.serial_port.write(command.encode('utf-8'))
        self.serial_port.write(b'\n') # Send a newline to mark end of command

    def read_arduino_response(self):
        if self.serial_port.in_waiting > 0:
            try:
                response = self.serial_port.readline().decode('utf-8').strip()
                if response:
                    self.get_logger().info(f"Arduino response: '{response}'")
                    
                    # --- FIX: Try to publish as Int32 for torque, etc. ---
                    try:
                        pwm_value = int(response)
                        response_msg = Int32()
                        response_msg.data = pwm_value
                        self.response_publisher.publish(response_msg)
                    except ValueError:
                        # If Arduino sends "GRASP" or "OK", just log it
                        self.get_logger().debug(f"Arduino sent non-integer string: {response}")

            except Exception as e:
                self.get_logger().warn(f"Error reading from serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()