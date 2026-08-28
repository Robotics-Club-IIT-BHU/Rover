import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist 

class Move(Node):

    def __init__(self):
        super().__init__('move')
        
        # 1. Open serial port
        try:
            # Added dsrdtr=None to prevent some ESP32s from auto-resetting
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1, dsrdtr=None)
            time.sleep(2) 
            self.get_logger().info("Serial port /dev/ttyACM1 opened successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.ser = None

        # 2. Subscription
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if self.ser is None or not self.ser.is_open:
            return

        # Default to Stop
        command = "S"
        
        # Map Twist logic
        if msg.linear.x > 0.0:
            command = "F"
        elif msg.linear.x < 0.0:
            command = "B"
        elif msg.angular.z > 0.0:
            command = "L"
        elif msg.angular.z < 0.0:
            command = "R"
        else:
            command = "S" # Now it actually stops when keys are released

        # 3. Centralized Serial Send
        try:
            data_to_send = f"{command}\n"
            self.ser.write(data_to_send.encode('utf-8'))
            # Flushes the buffer to ensure immediate transmission
            self.ser.flush() 
            self.get_logger().info(f'Sent to ESP: {command}')
        except Exception as e:
            self.get_logger().error(f"Write failed: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser is not None:
            # Send Stop command one last time before closing
            try:
                self.ser.write(b"S\n")
            except:
                pass
            self.ser.close()
            self.get_logger().info("Serial port closed.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    move = Move()
    try:
        rclpy.spin(move)
    except KeyboardInterrupt:
        pass
    finally:
        move.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

