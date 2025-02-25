import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from interfaces.msg import KeyInfo


class RemoteControllerNode(Node):
    def __init__(self):
        """
        Initialize the controller node, subscriber, publisher.
        """
        super().__init__("remote_controller_node")

        # create internal variables
        # key map is used to map the button name to the buttons binary
        # Example: when pressing Y, buttons = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0], therefore equal to 8
        self.KEY_MAP = {
            2**0: "A",
            2**1: "B",
            2**2: "X",
            2**3: "Y",
            2**4: "LB",
            2**5: "RB",
            2**6: "BACK",
            2**7: "START",
            2**8: "LOGITECH",
        }
        self.last_key_value = "NONE"
        self.key_value = "NONE"
        self.start_timer = False
        self.timer_counter = 0

        # Create a subscriber to the joystick
        self.joystick_sub = self.create_subscription(
            Joy,
            "joy",
            self.joystick_callback,
            10,
        )

        # Create key infor publisher
        self.key_info_pub = self.create_publisher(KeyInfo, "key_info", 10)

        # Create timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    def joystick_callback(self, msg: Joy):
        """
        Callback function for the joystick subscriber.
        """
        # Get the joy stick states
        buttons = msg.buttons
        # axes = msg.axes

        # # Publish the axis cmd
        # left_x = -axes[0]
        # left_y = axes[1]
        # right_x = -axes[3]
        # # right_y = axes[4]
        # twist_msg = Twist()
        # # twist_msg.header.stamp = self.get_clock().now().to_msg()
        # twist_msg.linear.x = left_y
        # twist_msg.angular.z = right_x
        # twist_msg.linear.y = left_x
        # self.cmd_vel_pub.publish(twist_msg)

        # Calculate the key value
        key_value = 0
        for index, value in enumerate(buttons):
            key_value = key_value + 2**index if value == 1 else key_value
        key_value = self.KEY_MAP.get(key_value, "NONE")

        # Check key event
        if self.last_key_value != key_value:
            key_info_msg = KeyInfo()
            if self.last_key_value == "NONE":  # Key is pressed
                # Publish the key event
                key_info_msg.key_event = "KEY_PRESSED"
                key_info_msg.key_value = key_value
                self.key_value = key_value
                self.key_info_pub.publish(key_info_msg)

            else:  # Key is released
                key_info_msg.key_value = self.key_value
                key_info_msg.key_event = "KEY_RELEASED"
                self.key_info_pub.publish(key_info_msg)
                if self.start_timer is True:
                    self.start_timer = False
                    self.timer_counter = 0

            # Key is holding, enable timer
            if self.start_timer is False and key_info_msg.key_event == "KEY_PRESSED":
                self.start_timer = True
                self.timer_counter = 0

            # Cache the last key value
            self.last_key_value = key_value

    def timer_callback(self):
        """
        Timer callback function.
        """
        if self.start_timer is False:
            return

        self.timer_counter += 1
        if self.timer_counter == 30:  # 3 second
            key_info_msg = KeyInfo()
            key_info_msg.key_value = self.key_value
            key_info_msg.key_event = "KEY_HOLDING_3S"
            self.key_info_pub.publish(key_info_msg)
        elif self.timer_counter == 50:  # 3 second
            key_info_msg = KeyInfo()
            key_info_msg.key_value = self.key_value
            key_info_msg.key_event = "KEY_HOLDING_5S"
            self.key_info_pub.publish(key_info_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RemoteControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
