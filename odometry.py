import rclpy    # Python library for ROS
from rclpy.node import Node     # Base class for all ROS2 nodes
from geometry_msgs.msg import Twist     # Message type for velocity commands
import serial       # For communication with Arduino

class Odometry(Node):
    def __init__(self):
        super().__init__('odometry')    # Initialize the ROS2 node with the name "odometry"

        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)   # Adjust port and baud rate
            self.get_logger().info("The odometry node has started.")     # Bascially printing
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {str(e)}")        # In case of failure
            return
        
        # Create a publisher that publishes Twist messages on the "/odometrydata" topic
        self.odom_vel_pub = self.create_publisher(
            Twist,      # Message type: geometry_msgs/Twist
            "/odometrydata",    # Topic name
            10                  # Queue size
        )

        # Create a timer that calls send_odometry_data() every 0.1 seconds
        self.timer = self.create_timer(0.1, self.send_odometry_data)

    def send_odometry_data(self):
        """
        Reads line from the serial port,
        converts it to linear and angular velocities,
        and publishes it as a Twist message.
        """
        try:
            # Read a line from the Arduino, decode to string
            data = self.serial_port.readline().decode('utf-8').strip()
            if not data:
                return  # Stop if no data receieved

            # Expecting data format like "LEFT|RIGHT" 
            parts = data.split('|')
            if len(parts) != 2:
                self.get_logger().warn(f"erroneous data format received: {data}")
                return
            
            # Wheelchair-specific constants in cm
            R,L=10.16,40

            # Convert readings (in RPM) to wheel linear velocity of wheels
            left_vel = (2 * 3.14159265 *int(parts[0])*R)/60
            right_vel = (2 * 3.14159265 *int(parts[1])*R)/60

            # Compute wheelchair's linear and angular velocity
            linear_velocity = (left_vel + right_vel) / 2
            angular_velocity = (right_vel - left_vel) /L

            # Create Twist message and assign velocities
            msg = Twist()
            msg.linear.x = linear_velocity
            msg.angular.z = angular_velocity

            # Publish the message on /odometrydata
            self.odom_vel_pub.publish(msg)
            self.get_logger().info(f"Published: linear_x={msg.linear.x:.2f}, angular_z={msg.angular.z:.2f}")

        except (ValueError, serial.SerialException) as e:
            self.get_logger().error(f"Error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)   # Initialize ROS2 Python system   
    odom_node = Odometry()  # Create the Odometry node

    if not hasattr(odom_node, "serial_port"):  # If serial failed, don't run
        return

    try:
        # Keep the node running and responsive to incoming messages
        rclpy.spin(odom_node)
    except KeyboardInterrupt:
        # Incase of KeyboardInterrupt
        odom_node.get_logger().info("Shutting down odometry node.")
    finally:
        # Clean up before exiting
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()