# ROS Client library for doing things
import rclpy

# Import the Node object for creating the Publisher Node
from rclpy.node import Node

# String message that is going to be published
from std_msgs.msg import String


class PublisherNode(Node):
    """
    PublisherNode class that inherits from the ROS Node class.
    """

    def __init__(self):

        # Create and initialize the Node - this is required
        super().__init__('publisher_node')

        # Once the node has been initialized, the rest
        # of the class functionality can be implemented

        # Creates a publisher that publishes to the topic 'chatter'
        self.publisher_ = self.create_publisher(String, 'chatter', 10)

        # Create a timer object that calls back the "callback" function
        # every 0.5 seconds (2 Hz). In this case, the callback function
        # is the member function timer_callback.
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        """
        Callback function that the timer object will callback.
        It is passed as an argument when creating the timer object.
        """

        # Create the message and populate it with some information
        msg = String()
        msg.data = 'Message from publisher: %d' % self.i

        # Publish to the topic
        self.publisher_.publish(msg)

        # Log to the console for debugging
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Increment counter for the next round of messages
        self.i += 1

def publisher_entry_point_function(args=None):
    """
    Entry point function for the node.

    @param args: Additional arguments that can be passed to the node. For example,
                 some arguments can also be passed using launch files.
    """

    # Initialize the client library - this is required
    rclpy.init(args=args)

    # Create the publisher node
    publisher_node = PublisherNode()

    # Spin so that the node is busy processing
    # while no exceptions are encountered and ROS
    # is executing
    rclpy.spin(publisher_node)

    # Destroy the node explicitly
    publisher_node.destroy_node()

    # Shut down this context
    rclpy.shutdown()

if __name__ == '__main__':

    # Just in case you have a setup where you execute the script directly
    publisher_entry_point_function()