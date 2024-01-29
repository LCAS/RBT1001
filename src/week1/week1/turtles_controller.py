import rclpy
from rclpy.node import Node

from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from geometry_msgs.msg import Twist


class TurtlesController(Node):

    def __init__(self):
        # Defines the name of the node
        super().__init__('turtles_controller')

        # Declare a new topic this node will publish to
        self.t1_cmd_pub = self.create_publisher(
            Twist, 
            '/turtle1/cmd_vel', 
            10
        )

        # Declare a topic this node will listen to; at every new message, 
        # the callback will be called.
        self.t1_pose_sub = self.create_subscription(
            Pose,
            "/turtle1/pose",
            self.t1_pose_cb,
            10
        )

        # Declare a service this node will provide to the ROS network
        self.reset_srv = self.create_service(Empty, 'reset_controller', self.reset_cb)

        # Declare a service this node will call
        self.t1_pen_cli = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.t1_pen_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        # this is the service request message 
        t1_pen_req = SetPen.Request()
        t1_pen_req.r = 255
        t1_pen_req.g = 0
        t1_pen_req.b = 0
        # Call the service
        self.future = self.t1_pen_cli.call_async(t1_pen_req)
        # wait until the request is completed
        rclpy.spin_until_future_complete(self, self.future)
        # log the result
        self.get_logger().info('Turtle1 color change result: "%s"' % self.future.result())

        # The timer_callback will be called continuously until the node is destroyed
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.t1_pose = None
        self.t1_rot_direction = 1

    ##
    #  This function will be called at a frequency of 1/time_period Hz
    #  Here you should put the code that needs to be executed continuously,
    #  regardless of any topic callback, i.e., the logic of your controller.
    ##
    def timer_callback(self):
        # Change rotation direction at every full revolution
        if self.t1_pose is not None and abs(self.t1_pose.theta) < 1e-1:
            self.t1_rot_direction *= -1

        msg = Twist()
        msg.angular.z = 0.7 * self.t1_rot_direction
        self.t1_cmd_pub.publish(msg)
        
        # writing ROS logs, can be info(), debug(), warning() or error().
        self.get_logger().info('Publishing: "%s"' % msg)
   
    ##
    #  This function will be called every time a pose message is published
    #  for turtle1.
    ##
    def t1_pose_cb(self, msg):
        self.get_logger().debug('I got turtle1 pose: "%s"' % msg)
        self.t1_pose = msg
   
    ##
    #  This function will be called every time the /reset_controller service
    #  is called.
    ##
    def reset_cb(self, request, response):
        # TODO implement, should return a response message
        self.get_logger().error('Reset method not implemented')
        raise NotImplementedError

def main(args=None):
    rclpy.init(args=args)

    turtles_controller = TurtlesController()

    rclpy.spin(turtles_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtles_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
