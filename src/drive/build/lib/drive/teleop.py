import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class TeleopTwist(Node):
    def __init__(self, settings):
        super().__init__('teleop_twist_node')
        self.settings = settings
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.get_logger().info("--- Twist Teleop Active ---")
        self.get_logger().info("Use Arrow Keys to move. Space to stop. Ctrl+C to quit.")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        # Wait up to 0.1s for a key press
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
            if key == '\x1b': # Handle multi-byte Arrow keys
                key += sys.stdin.read(2)
            return key
        return ''

    def run(self):
        try:
            while True:
                key = self.get_key()
                twist = Twist()

                if key == '\x1b[A':   # UP
                    twist.linear.x = 0.5
                elif key == '\x1b[B': # DOWN
                    twist.linear.x = -0.5
                elif key == '\x1b[D': # LEFT
                    twist.angular.z = 1.0
                elif key == '\x1b[C': # RIGHT
                    twist.angular.z = -1.0
                elif key == ' ':      # SPACE to stop
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == '\x03':   # Ctrl+C
                    break
                else:
                    # Optional: uncomment to stop if no key is held
                    # twist.linear.x = 0.0
                    continue 

                self.publisher_.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        finally:
            # Stop the robot before exiting
            self.publisher_.publish(Twist())
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = TeleopTwist(settings)
    
    # We use run() instead of spin() because we are handling the loop manually
    node.run()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

