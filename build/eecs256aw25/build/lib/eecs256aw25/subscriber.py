# ROS Client library for doing things
import rclpy

# Import the Node object for creating the Subscriber Node
from rclpy.node import Node

# String message that is going to be subscribed to
from std_msgs.msg import String

class SubscriberNode(Node):
    """
    SubscriberNode class that inherits from the ROS Node class.
    """

    def __init__(self):

        # Create and initialize the Node - this is required
        super().__init__('subscriber_node')

        # Subscribe to the topic "chatter", where the
        # publisher is publishing to. When a message arrives
        # to the topic, another "callback" function will be
        # executed. In this case, it's the listener_callback
        # member function of this class.

        # NOTE: You can have multiple publishers and subscribers
        # in the same class, depending on your application.
        # This is just an example.
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

        # prevent unused variable warning
        self.subscription

    def listener_callback(self, msg):
        """
        Listener callback function for the subscriber.

        @param msg - Message received by the callback function, which was
                     also published by the publisher.
        """

        # Log the message received from the publisher to the command line
        self.get_logger().info("Message received from publisher: {}".format(msg.data))


def subscriber_entry_point_function(args=None):
    """
    Entry point function for the node.

    @param args: Additional arguments that can be passed to the node. For example,
                 some arguments can also be passed using launch files.
    """
    # Initialize the client library - this is required
    rclpy.init(args=args)

    # Create the subscriber node
    subscriber_node = SubscriberNode()

    # Spin so that the node is busy processing
    # while no exceptions are encountered and ROS
    # is executing
    rclpy.spin(subscriber_node)

    # Destroy the node explicitly
    subscriber_node.destroy_node()

    # Shut down this context
    rclpy.shutdown()


if __name__ == '__main__':

    # Just in case you have a setup where you execute the script directly
    subscriber_entry_point_function()