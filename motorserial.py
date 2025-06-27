import rclpy    # Python library for ROS
from rclpy.node import Node     # Base class for all ROS2 nodes
from geometry_msgs.msg import Twist     # Message type for velocity commands
import serial       # For communication with Arduino

class MotorController(Node):
    def __init__(self):     
        super().__init__('motor_controller')    # Initialize the ROS2 node with the name "motor_controller"
        
        # Initialize serial communication with Arduino

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)  # Adjust port and baud rate
            self.get_logger().info("the motor_controller node has started")     # Bascially printing
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")        # In case of failure
            return
        
        # Subscribe to /cmd_vel topic

        self.subscription = self.create_subscription(
            Twist,                  # Message type: geometry_msgs/Twist
            'cmd_vel',              # Topic name
            self.cmd_vel_callback,  # Callback function to handle messages
            10                          # Queue size
        )
        
    def cmd_vel_callback(self, msg):

        """
        Callback function that runs every time a Twist message is received.
        It extracts linear_x and angular_z velocities, calculates left and right motor speeds,
        converts them to PWM, and sends them over serial to the Arduino.
        """

        # Extract linear and angular velocities

        linear_x = msg.linear.x  # Forward speed
        angular_z = msg.angular.z  # Rotational speed
        
        # Calculate left and right motor speeds
        # Assuming a wheelbase factor of 0.27 change according to your wheels

        left_speed = max(0, linear_x - (0.27*angular_z))  # Prevent negative speeds
        right_speed = max(0, linear_x + (0.27*angular_z))   # Prevent negative speeds
        
        # Map speeds to PWM range (0 - 255)

        left_pwm = int(self.speed_to_pwm(left_speed))
        right_pwm = int(self.speed_to_pwm(right_speed))
        
        # Send PWM values to Arduino as LXXXRXXX (e.g., L127R130)

        command = f"L{left_pwm:03d}R{right_pwm:03d}\n"      # Formatting the command to be sent
        self.serial_port.write(command.encode('utf-8'))
        self.get_logger().info(f"Sent command to Arduino: {command.strip()}")
    
    def speed_to_pwm(self, speed):
        # Maps a normalized speed value (0.0 to 1.0) to the PWM range (0 to 255).
        return speed * 255
    
def main(args=None):
    rclpy.init(args=args)   # Initialize ROS2 Python system
    motor_controller = MotorController()    # Create the MotorController node
    try:
        # Keep the node running and responsive to incoming messages
        rclpy.spin(motor_controller)    

    finally:
        # Clean up before exiting
        motor_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()