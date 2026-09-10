"""ROS transport adapter for Gazebo's body-frame VelocityControl system."""

from geometry_msgs.msg import Twist
from drone_swarm.motion.geometry import Vec3, world_to_body


class GazeboVelocityBackend:
    def __init__(self, node):
        self.publisher = node.create_publisher(Twist, "/drone_001/cmd_vel", 10)
        self.orientation = (0.0, 0.0, 0.0, 1.0)

    def send_velocity(self, world_velocity: Vec3):
        velocity = world_to_body(world_velocity, self.orientation)
        message = Twist()
        message.linear.x, message.linear.y, message.linear.z = velocity.x, velocity.y, velocity.z
        self.publisher.publish(message)

    def stop(self):
        self.publisher.publish(Twist())
