import rclpy
from rclpy.node import Node
from hri_fullbody.fullbody_detector import FullbodyDetector
import random
from hri_msgs.msg import IdsList


def generate_id():
    """This function generates a 5 chars ID.
    """

    return "".join(random.sample("abcdefghijklmnopqrstuvwxyz", 5))


class MultibodyManager(Node):
    
    def __init__(self):

        super().__init__('fullbody_manager')

        self.declare_parameter('fullbody_manager/use_depth', False)
        self.declare_parameter('fullbody_manager/stickman_debug', False)
        self.declare_parameter('fullbody_manager/single_body', True)
        self.declare_parameter('fullbody_manager/min_detection', 0.7)

        self.use_depth = self.get_parameter('fullbody_manager/use_depth').get_parameter_value().bool_value
        self.stickman_debug = self.get_parameter('fullbody_manager/stickman_debug').get_parameter_value().bool_value
        self.min_detection = self.get_parameter('fullbody_manager/min_detection').get_parameter_value().double_value
        self.single_body = self.get_parameter('fullbody_manager/single_body').get_parameter_value().bool_value

        self.get_logger().info("Using depth camera for body position estimation: %s "% str(self.use_depth))

        if not self.single_body:

            self.get_logger().info("Setting up for multibody pose estimation")
            # Dictionary for the detected people
            self.detected_bodies = {}
            # id = uni
            # self.detected_bodies[id][0] = personal id

            # Subscriber for the list of detected bodies
            self.bodies_list_sub = self.create_subscription(
                IdsList,
                "/humans/bodies/tracked",
                self.ids_list_cb, 1)
            self.get_logger().info("Waiting for ids on /humans/bodies/tracked")            
        else:
            self.get_logger().info("Setting up for single body pose estimation")
            self.get_logger().warning(
            "hri_fullbody running in single body mode:"
            + " only one skeleton will be detected")
            id = generate_id()
            self.single_detector = FullbodyDetector(
                        self,
                        self.use_depth,
                        self.stickman_debug,
                        id,
                        self.min_detection
                    )

            self.get_logger().info("Generated single person detector for body_%s"% id)
            self.get_logger().info("Waiting for frames on topic %s"% self.single_detector.get_image_topic())  

            

    def ids_list_cb(self, msg):

        current_bodies = {}

        for id in msg.ids:
            if id in self.detected_bodies:
                current_bodies[id] = (self.detected_bodies[id][0], 0)
            else:
                current_bodies[id] = (
                    FullbodyDetector(
                        self,
                        self.use_depth,
                        self.stickman_debug,
                        id,
                        self.min_detection
                    ),
                    0,
                )
                self.get_logger().info("Generated single person detector for body_%s" % id)
                self.get_logger().info(
                    "Waiting for frames on topic %s" %
                    current_bodies[id][0].get_image_topic(),
                )

        for id in self.detected_bodies:
            if not id in current_bodies:
                self.detected_bodies[id][0].unregister()

        self.detected_bodies = current_bodies

def main(args=None):
    rclpy.init(args=args)
    node = MultibodyManager()    
    rclpy.spin(node)

if __name__ == "__main__":
    main()
