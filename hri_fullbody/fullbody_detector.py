import io
import os
from ikpy import chain
from hri_fullbody.utils import quaternion_from_euler
from hri_fullbody.jointstate import compute_jointstate, \
    HUMAN_JOINT_NAMES, compute_jointstate
from hri_fullbody.rs_to_depth import rgb_to_xyz  # SITW
from hri_fullbody.urdf_generator import make_urdf_human
from hri_fullbody.protobuf_to_dict import protobuf_to_dict
from hri_fullbody.one_euro_filter import OneEuroFilter
from hri_fullbody.face_pose_estimation import face_pose_estimation
import math
import numpy as np
import sys
import tempfile
import copy
import subprocess
import launch
import xacro
from ros2launch.api import get_share_file_path_from_package
from launch_ros.actions import Node
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


import rclpy
from tf2_ros import TransformBroadcaster

from sensor_msgs.msg import Image, CameraInfo, RegionOfInterest
from sensor_msgs.msg import JointState
from hri_msgs.msg import Skeleton2D, NormalizedPointOfInterest2D, IdsList
from message_filters import ApproximateTimeSynchronizer, Subscriber
from geometry_msgs.msg import TwistStamped, PointStamped, TransformStamped
from cv_bridge import CvBridge
import cv2

import mediapipe as mp

mp_face_detection = mp.solutions.face_detection
mp_holistic = mp.solutions.holistic
mp_pose = mp.solutions.pose

# One Euro Filter parameters
BETA_POSITION=0.05 
D_CUTOFF_POSITION=0.5 
MIN_CUTOFF_POSITION=0.3
BETA_VELOCITY=0.2 
D_CUTOFF_VELOCITY=0.2 
MIN_CUTOFF_VELOCITY=0.5

# Mediapipe 2D keypoint order:
# ['nose', 'left_eye_inner', 'left_eye', 'left_eye_outer',
#   'right_eye_inner', 'right_eye', 'right_eye_outer',
#   'left_ear', 'right_ear', 'mouth_left', 'mouth_right',
#   'left_shoulder', 'right_shoulder', 'left_elbow', 'right_elbow',
#   'left_wrist', 'right_wrist', 'left_pinky', 'right_pinky',
#   'left_index', 'right_index', 'left_thumb', 'right_thumb',
#   'left_hip', 'right_hip', 'left_knee', 'right_knee',
#   'left_ankle', 'right_ankle', 'left_heel', 'right_heel',
#   'left_foot_index', 'right_foot_index']

# Mediapipe 2D skeleton indexing

MP_NOSE = 0
MP_LEFT_EYE = 2
MP_LEFT_EAR = 7
MP_LEFT_SHOULDER = 11
MP_LEFT_ELBOW = 13
MP_LEFT_WRIST = 15
MP_LEFT_HIP = 23
MP_LEFT_KNEE = 25
MP_LEFT_ANKLE = 27
MP_LEFT_FOOT = 31
MP_RIGHT_EYE = 5
MP_RIGHT_EAR = 8
MP_RIGHT_SHOULDER = 12
MP_RIGHT_ELBOW = 14
MP_RIGHT_WRIST = 16
MP_RIGHT_HIP = 24
MP_RIGHT_KNEE = 26
MP_RIGHT_ANKLE = 28
MP_RIGHT_FOOT = 32

# ROS4HRI to Mediapipe 2D skeleton indexing conversion table

ros4hri_to_mediapipe = [None] * 18

ros4hri_to_mediapipe[Skeleton2D.NOSE] = MP_NOSE
ros4hri_to_mediapipe[Skeleton2D.LEFT_EYE] = MP_LEFT_EYE
ros4hri_to_mediapipe[Skeleton2D.LEFT_EAR] = MP_LEFT_EAR
ros4hri_to_mediapipe[Skeleton2D.LEFT_SHOULDER] = MP_LEFT_SHOULDER
ros4hri_to_mediapipe[Skeleton2D.LEFT_ELBOW] = MP_LEFT_ELBOW
ros4hri_to_mediapipe[Skeleton2D.LEFT_WRIST] = MP_LEFT_WRIST
ros4hri_to_mediapipe[Skeleton2D.LEFT_HIP] = MP_LEFT_HIP
ros4hri_to_mediapipe[Skeleton2D.LEFT_KNEE] = MP_LEFT_KNEE
ros4hri_to_mediapipe[Skeleton2D.LEFT_ANKLE] = MP_LEFT_ANKLE
ros4hri_to_mediapipe[Skeleton2D.RIGHT_EYE] = MP_RIGHT_EYE
ros4hri_to_mediapipe[Skeleton2D.RIGHT_EAR] = MP_RIGHT_EAR
ros4hri_to_mediapipe[Skeleton2D.RIGHT_SHOULDER] = MP_RIGHT_SHOULDER
ros4hri_to_mediapipe[Skeleton2D.RIGHT_ELBOW] = MP_RIGHT_ELBOW
ros4hri_to_mediapipe[Skeleton2D.RIGHT_WRIST] = MP_RIGHT_WRIST
ros4hri_to_mediapipe[Skeleton2D.RIGHT_HIP] = MP_RIGHT_HIP
ros4hri_to_mediapipe[Skeleton2D.RIGHT_KNEE] = MP_RIGHT_KNEE
ros4hri_to_mediapipe[Skeleton2D.RIGHT_ANKLE] = MP_RIGHT_ANKLE

# Mediapipe face mesh indexing (partial)
FM_NOSE = 1 
FM_MOUTH_CENTER = 13 
FM_RIGHT_EYE = 159 
FM_RIGHT_EAR_TRAGION = 234 
FM_LEFT_EYE = 386 
FM_LEFT_EAR_TRAGION = 454


def _normalized_to_pixel_coordinates(
        normalized_x: float, normalized_y: float, image_width: int,
        image_height: int):

    x_px = min(math.floor(normalized_x * image_width), image_width - 1)
    y_px = min(math.floor(normalized_y * image_height), image_height - 1)
    return x_px, y_px


def _make_2d_skeleton_msg(header, pose_2d):
    skel = Skeleton2D()
    skel.header = header
    _ = NormalizedPointOfInterest2D()
    skel.skeleton = [_] * 18

    for idx, human_joint in enumerate(ros4hri_to_mediapipe):
        if human_joint is not None:
            skel.skeleton[idx] = NormalizedPointOfInterest2D(
                x=pose_2d[human_joint].get('x'),
                y=pose_2d[human_joint].get('y'),
                c=pose_2d[human_joint].get('visibility'))

    # There is no Neck landmark in Mediapipe pose estimation
    # However, we can think of the neck point as the average
    # point between left and right shoulder.
    msg = NormalizedPointOfInterest2D()
    msg.x = (skel.skeleton[Skeleton2D.LEFT_SHOULDER].x + skel.skeleton[Skeleton2D.RIGHT_SHOULDER].x)/2
    msg.y = (skel.skeleton[Skeleton2D.LEFT_SHOULDER].y + skel.skeleton[Skeleton2D.RIGHT_SHOULDER].y)/2
    msg.x = min(skel.skeleton[Skeleton2D.LEFT_SHOULDER].c, skel.skeleton[Skeleton2D.RIGHT_SHOULDER].c)

    skel.skeleton[Skeleton2D.NECK] = msg

    return skel


def _get_bounding_box_limits(landmarks, image_width, image_height):
    x_max = 0.0
    y_max = 0.0
    x_min = 1.0
    y_min = 1.0
    # for result in results:
    for data_point in landmarks:
        if x_max < data_point.x:
            x_max = data_point.x
        if y_max < data_point.y:
            y_max = data_point.y
        if x_min > data_point.x:
            x_min = data_point.x
        if y_min > data_point.y:
            y_min = data_point.y

    x_min, y_min = _normalized_to_pixel_coordinates(
        x_min, y_min, image_width, image_height)
    x_max, y_max = _normalized_to_pixel_coordinates(
        x_max, y_max, image_width, image_height)
    return x_min, y_min, x_max, y_max


class FullbodyDetector():

    def __init__(self,
                 node,
                 use_depth,
                 stickman_debug,
                 body_id,
                 single_body=False,
                 min_detection=0.7):

        self.node = node
        self.use_depth = use_depth
        self.stickman_debug = stickman_debug
        self.single_body = single_body
        self.multi_body = not single_body
        self.skeleton_to_set = single_body

        self.detector = mp_holistic.Holistic(
            min_detection_confidence=min_detection, static_image_mode=True)

        self.from_depth_image = False

        self.x_min_face = 1.00
        self.y_min_face = 1.00
        self.x_max_face = 0.00
        self.y_max_face = 0.00

        self.x_min_body = 1.00
        self.y_min_body = 1.00
        self.x_max_body = 0.00
        self.y_max_body = 0.00

        self.human_description = ''
        self.body_position_estimation = [None] * 3
        # trans_vec ==> vector representing the translational component
        # of the homoegenous transform obtained solving the PnP problem
        # between the camera optical frame and the face frame. 
        self.trans_vec = [None] * 3
        self.valid_trans_vec = False

        self.js_topic = "/humans/bodies/" + body_id + "/joint_states"
        skel_topic = "/humans/bodies/" + body_id + "/skeleton2d"

        self.skel_pub = self.node.create_publisher(Skeleton2D, skel_topic, 1) 
        self.js_pub = self.node.create_publisher(JointState, self.js_topic, 1)
        self.br = CvBridge()

        self.body_id = body_id

        if self.multi_body:
            # URDF model settings, kinematic chains generation and
            # robot_state_publisher initialization
            self.skeleton_generation()

        self.tb = TransformBroadcaster(self.node)
        self.one_euro_filter = [None] * 3
        self.one_euro_filter_dot = [None] * 3

        if self.multi_body:
            self.image_subscriber = Subscriber(
                                        self.node,
                                        Image,
                                        "/humans/bodies/"+self.body_id+"/cropped"                                        
                                        )
                                        # buff_size=2**24)
        else:
            self.image_subscriber = Subscriber(self.node,
                                        Image,
                                        "/image",                                        
                                        )
                                        # buff_size=2**24)

        if self.use_depth and self.multi_body:
            self.tss = ApproximateTimeSynchronizer(
                [
                    self.image_subscriber,
                    Subscriber(self.node,
                        CameraInfo,
                        "/camera_info"                        
                        ),
                    Subscriber(self.node,
                        RegionOfInterest,
                        "/humans/bodies/"+self.body_id+"/roi"
                        ),
                    Subscriber(self.node,
                        Image,
                        "/depth_image"                       
                        ),
                        # buff_size=2**24),
                    Subscriber(self.node,
                        CameraInfo,
                        "/depth_info"
                        )
                ],
                10,
                0.1,
                allow_headerless=True
            )
            self.tss.registerCallback(self.image_callback_depth)
        elif not self.use_depth and self.multi_body:
            self.tss = ApproximateTimeSynchronizer(
                [
                    self.image_subscriber,
                    Subscriber(self.node,
                        CameraInfo,
                        "/camera_info"
                        )
                ],
                10,
                0.2
            )
            self.tss.registerCallback(self.image_callback_rgb)
        elif self.use_depth and single_body:
            # Here the code to detect one person only with depth information
            self.tss = ApproximateTimeSynchronizer(
                [
                    self.image_subscriber,
                    Subscriber(self.node,
                        CameraInfo,
                        "/camera_info"
                        ),
                    Subscriber(self.node,
                        Image,
                        "/depth_image"                     
                        ),
                        # buff_size=2**24),
                    Subscriber(self.node,
                        CameraInfo,
                        "/depth_info"
                        )
                ],
                10,
                0.1,
                allow_headerless=True
            )
            self.tss.registerCallback(self.image_callback_depth_single_person)
        else:
            self.tss = ApproximateTimeSynchronizer(
                [
                    self.image_subscriber,
                    Subscriber(self.node,
                        CameraInfo,
                        "/camera_info"
                        )
                ],
                10,
                0.2
            )
            self.tss.registerCallback(self.image_callback_rgb)

        if single_body:
            self.ids_pub = self.node.create_publisher(
                IdsList,
                "/humans/bodies/tracked",
                1
                )
            self.roi_pub = self.node.create_publisher(
                RegionOfInterest,
                "/humans/bodies/"+body_id+"/roi",
                1)

        self.body_filtered_position = [None] * 3  # x, y ,z
        self.body_filtered_position_prev = [None] * 3 # x, y, z
        self.body_vel_estimation = [None] * 3
        self.body_vel_estimation_filtered = [None] * 3

        self.position_msg = PointStamped()
        filtered_position_topic = "/humans/bodies/"+body_id+"/position"
        self.body_filtered_position_pub = self.node.create_publisher( 
            PointStamped,
            filtered_position_topic,
            1,)
        self.velocity_msg = TwistStamped()
        self.velocity_msg.header.frame_id = "body_"+body_id
        twist_topic = "/humans/bodies/"+body_id+"/velocity"
        self.velocity_pub = self.node.create_publisher(
            TwistStamped,
            twist_topic,
            1)
           

        self.image_info_sub = self.node.create_subscription(
            CameraInfo,
            "camera_info",
            self.camera_info_callback,1)
        

    def skeleton_generation(self):
        """ Generate a URDF model for this body, set it on the 
            ROS parameter server and spawn a new robot_state_publisher,
            which will publish the TF frames for this body"""
        self.urdf = make_urdf_human(self.body_id)
        self.node.get_logger().info("Setting URDF description for body"
                      "<%s> (param name: human_description_%s)" % (
                          self.body_id, self.body_id))
        self.human_description = "human_description_%s" % self.body_id

        human_param = rclpy.parameter.Parameter(
            self.human_description,
            rclpy.Parameter.Type.STRING,
            self.urdf
        )
        self.node.declare_parameter(self.human_description, self.urdf)
        self.node.set_parameters([human_param])
 

        self.urdf_file = io.StringIO(self.urdf)
        self.r_arm_chain = chain.Chain.from_urdf_file(
            self.urdf_file,
            base_elements=[
                "r_y_shoulder_%s" % self.body_id],
            base_element_type="joint",
            active_links_mask=[False, True, True, True, True, False])
        self.urdf_file.seek(0)
        self.l_arm_chain = chain.Chain.from_urdf_file(
            self.urdf_file,
            base_elements=[
                "l_y_shoulder_%s" % self.body_id],
            base_element_type="joint",
            active_links_mask=[False, True, True, True, True, False])
        self.urdf_file.seek(0)
        self.r_leg_chain = chain.Chain.from_urdf_file(
            self.urdf_file,
            base_elements=[
                "r_y_hip_%s" % self.body_id],
            base_element_type="joint",
            active_links_mask=[False, True, True, True, True, False])
        self.urdf_file.seek(0)
        self.l_leg_chain = chain.Chain.from_urdf_file(
            self.urdf_file,
            base_elements=[
                "l_y_hip_%s" % self.body_id],
            base_element_type="joint",
            active_links_mask=[False, True, True, True, True, False])
        
        with tempfile.NamedTemporaryFile(mode='w',suffix='.xacro', delete=False) as f:
            f.write(self.urdf_file.getvalue())
            urdf_file_path = f.name

        self.ik_chains = {}  # maps a body id to the IKpy chains
        self.ik_chains[self.body_id] = [
            self.r_arm_chain,
            self.l_arm_chain,
            self.r_leg_chain,
            self.l_leg_chain
        ]

        self.node.get_logger().info(
            "Spawning a instance of robot_state_publisher for this body...")

        # robot state publisher has a frame prefix now
        # todo (juandpenan) change tfs to just add a tf and thats it
        cmd = ["ros2", "run", "robot_state_publisher", "robot_state_publisher",
                urdf_file_path,
                "--ros-args",
                "-r",
                "__ns:=/humans/bodies/%s" % self.body_id,
                "-r",
                "joint_states:=%s" % self.js_topic              
               ]

        self.node.get_logger().info("Executing: %s" % " ".join(cmd))
        self.proc = subprocess.Popen(cmd, text=True)
       

    def unregister(self):
        if self.node.has_parameter(self.human_description):
            self.node.undeclare_parameter(self.human_description)
            self.node.get_logger().info('Deleted parameter %s', self.human_description)
        self.proc.kill()
        self.node.get_logger().warning('unregistered %s', self.body_id)

    def camera_info_callback(self, cameraInfo):
        """ This callback gets called only once, the first time
            a message arrives on the camera info topic. Here, 
            the node stores the instric matrix parameters that 
            will later use to solve face PnP problem and estimate
            body position """

        if not hasattr(self, 'cameraInfo'):
            self.K = np.zeros((3, 3), np.float32)
            self.K[0][0:3] = cameraInfo.k[0:3]
            self.K[1][0:3] = cameraInfo.k[3:6]
            self.K[2][0:3] = cameraInfo.k[6:9]

            self.f_x = self.K[0][0]
            self.f_y = self.K[1][1]
            self.c_x = self.K[0][2]
            self.c_y = self.K[1][2]


    def face_to_body_position_estimation(self, skel_msg):

        body_px = [(skel_msg.skeleton[Skeleton2D.LEFT_HIP].x \
                    + skel_msg.skeleton[Skeleton2D.RIGHT_HIP].x) \
                    / 2,
                   (skel_msg.skeleton[Skeleton2D.LEFT_HIP].y \
                    + skel_msg.skeleton[Skeleton2D.RIGHT_HIP].y) \
                    / 2]
        body_px = _normalized_to_pixel_coordinates(body_px[0], 
                                                   body_px[1], 
                                                   self.img_width,
                                                   self.img_height)
        if body_px == [0, 0]:
            return [0, 0, 0]
        else:
            d_x = np.sqrt((self.trans_vec[0]/1000)**2 \
                          +(self.trans_vec[1]/1000)**2 \
                          +(self.trans_vec[2]/1000)**2)

            x = body_px[0]-self.c_x
            y = body_px[1]-self.c_y

            Z = self.f_x*d_x/(np.sqrt(x**2 + self.f_x**2))
            X = x*Z/self.f_x
            Y = y*Z/self.f_y
            return [X, Y, Z]
 
    def stickman_debugging(self,
                           theta, 
                           torso, 
                           torso_res,
                           l_shoulder,
                           r_shoulder,
                           l_elbow,
                           r_elbow,
                           l_wrist,
                           r_wrist,
                           l_ankle,
                           r_ankle, 
                           header):
        """ Stickman debugging: publishing body parts tf frames directly
            using the estimation obtained from Mediapipe, or the one
            used as an input for the IK/FK process """

        t = TransformStamped()
        t.header = header
        t.child_frame_id = "mediapipe_torso_" + self.body_id
        t.transform.translation.x = -torso[1] + torso_res[0]
        t.transform.translation.y = torso[2]
        t.transform.translation.z = torso[0] + torso_res[2]

        q = quaternion_from_euler(np.pi/2, -theta, 0.0)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tb.sendTransform(t)

        t.header.frame_id = "our_torso_" + self.body_id
        t.child_frame_id = "mediapipe_torso_" + self.body_id
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.605

        q = quaternion_from_euler(0.0, 0.0, 0.0)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tb.sendTransform(t)

        t.header.frame_id = "left_shoulder_"+self.body_id
        t.child_frame_id = "our_torso_"+self.body_id
        t.transform.translation.x = l_shoulder[0]
        t.transform.translation.y = l_shoulder[1]
        t.transform.translation.z = l_shoulder[2]

        self.tb.sendTransform(t)
        
        t.header.frame_id = "right_shoulder_"+self.body_id
        t.child_frame_id = "our_torso_"+self.body_id
        t.transform.translation.x = r_shoulder[0]
        t.transform.translation.y = r_shoulder[1]
        t.transform.translation.z = r_shoulder[2]

        self.tb.sendTransform(t)

        t.header.frame_id = "left_elbow_"+self.body_id
        t.child_frame_id = "left_shoulder_"+self.body_id
        t.transform.translation.x = l_elbow[0]-l_shoulder[0]
        t.transform.translation.y = l_elbow[1]-l_shoulder[1]
        t.transform.translation.z = l_elbow[2]-l_shoulder[2]

        self.tb.sendTransform(t)
        
        t.header.frame_id = "right_elbow_"+self.body_id
        t.child_frame_id = "right_shoulder_"+self.body_id
        t.transform.translation.x = r_elbow[0]-r_shoulder[0]
        t.transform.translation.y = r_elbow[1]-r_shoulder[1]
        t.transform.translation.z = r_elbow[2]-r_shoulder[2]

        self.tb.sendTransform(t)
        
        t.header.frame_id = "left_wrist_"+self.body_id
        t.child_frame_id = "left_elbow_"+self.body_id
        t.transform.translation.x = l_wrist[0]-l_elbow[0]
        t.transform.translation.y = l_wrist[1]-l_elbow[1]
        t.transform.translation.z = l_wrist[2]-l_elbow[2]

        self.tb.sendTransform(t)

        t.header.frame_id = "right_wrist_"+self.body_id
        t.child_frame_id = "right_elbow_"+self.body_id
        t.transform.translation.x = r_wrist[0]-r_elbow[0]
        t.transform.translation.y = r_wrist[1]-r_elbow[1]
        t.transform.translation.z = r_wrist[2]-r_elbow[2]

        self.tb.sendTransform(t)

        t.header.frame_id = "left_ankle_"+self.body_id
        t.child_frame_id = "mediapipe_torso_"+self.body_id
        t.transform.translation.x = l_ankle[0]
        t.transform.translation.y = l_ankle[1]
        t.transform.translation.z = l_ankle[2]
        
        self.tb.sendTransform(t)

        t.header.frame_id = "right_ankle_"+self.body_id
        t.child_frame_id = "mediapipe_torso_"+self.body_id
        t.transform.translation.x = r_ankle[0]
        t.transform.translation.y = r_ankle[1]
        t.transform.translation.z = r_ankle[2]

        self.tb.sendTransform(t)

    def make_jointstate(
            self,
            body_id,
            pose_3d,
            pose_2d,
            header):

        js = JointState()
        js.header = copy.copy(header)
        js.name = [jn + "_%s" % body_id for jn in HUMAN_JOINT_NAMES]

        torso = np.array([
            -(
                pose_3d[MP_LEFT_HIP].get('z')
                + pose_3d[MP_RIGHT_HIP].get('z')
            )
            / 2,
            (
                pose_3d[MP_LEFT_HIP].get('x')
                + pose_3d[MP_RIGHT_HIP].get('x')
            )
            / 2,
            -(
                pose_3d[MP_LEFT_HIP].get('y')
                + pose_3d[MP_RIGHT_HIP].get('y')
            )
            / 2
        ])
        l_shoulder = np.array([
            -pose_3d[MP_LEFT_SHOULDER].get('z'),
            pose_3d[MP_LEFT_SHOULDER].get('x'),
            -pose_3d[MP_LEFT_SHOULDER].get('y')-0.605
        ])
        l_elbow = np.array([
            -pose_3d[MP_LEFT_ELBOW].get('z'),
            pose_3d[MP_LEFT_ELBOW].get('x'),
            -pose_3d[MP_LEFT_ELBOW].get('y')-0.605
        ])
        l_wrist = np.array([
            -pose_3d[MP_LEFT_WRIST].get('z'),
            pose_3d[MP_LEFT_WRIST].get('x'),
            -pose_3d[MP_LEFT_WRIST].get('y')-0.605
        ])
        l_ankle = np.array([
            -pose_3d[MP_LEFT_ANKLE].get('z'),
            pose_3d[MP_LEFT_ANKLE].get('x'),
            -pose_3d[MP_LEFT_ANKLE].get('y')
        ])
        r_shoulder = np.array([
            -pose_3d[MP_RIGHT_SHOULDER].get('z'),
            pose_3d[MP_RIGHT_SHOULDER].get('x'),
            -pose_3d[MP_RIGHT_SHOULDER].get('y')-0.605
        ])
        r_elbow = np.array([
            -pose_3d[MP_RIGHT_ELBOW].get('z'),
            pose_3d[MP_RIGHT_ELBOW].get('x'),
            -pose_3d[MP_RIGHT_ELBOW].get('y')-0.605
        ])
        r_wrist = np.array([
            -pose_3d[MP_RIGHT_WRIST].get('z'),
            pose_3d[MP_RIGHT_WRIST].get('x'),
            -pose_3d[MP_RIGHT_WRIST].get('y')-0.605
        ])
        r_ankle = np.array([
            -pose_3d[MP_RIGHT_ANKLE].get('z'),
            pose_3d[MP_RIGHT_ANKLE].get('x'),
            -pose_3d[MP_RIGHT_ANKLE].get('y')
        ])
        nose = np.array([
            -pose_3d[MP_NOSE].get('z'),
            pose_3d[MP_NOSE].get('x'),
            -pose_3d[MP_NOSE].get('y')
        ])
        feet = np.array([
            -(
                pose_3d[MP_RIGHT_FOOT].get('z')
                + pose_3d[MP_LEFT_FOOT].get('z')
            )
            / 2,
            (
                pose_3d[MP_RIGHT_FOOT].get('x')
                + pose_3d[MP_LEFT_FOOT].get('x')
            )
            / 2,
            -(
                pose_3d[MP_RIGHT_FOOT].get('y')
                + pose_3d[MP_LEFT_FOOT].get('y')
            )
            / 2
        ])

        ### depth and rotation ###

        theta = np.arctan2(pose_3d[MP_RIGHT_HIP].get('x'), -pose_3d[MP_RIGHT_HIP].get('z'))
        torso_res_prev = np.array([None, None, None])
        if self.use_depth:
            torso_px = _normalized_to_pixel_coordinates(
                (pose_2d[MP_LEFT_HIP].get('x')+pose_2d[MP_RIGHT_HIP].get('x'))/2,
                (pose_2d[MP_LEFT_HIP].get('y')+pose_2d[MP_RIGHT_HIP].get('y'))/2,
                self.img_width,
                self.img_height)
            torso_res = rgb_to_xyz(
                torso_px[0],
                torso_px[1],
                self.rgb_info,
                self.depth_info,
                self.image_depth,
                self.roi.x_offset,
                self.roi.y_offset
            )            
            self.node.get_logger().debug(f'torso_res {torso_res}')
            if torso_res.any() == None:
                if torso_res_prev.all() != None:
                    torso_res = torso_res_prev
                elif self.body_position_estimation[0]:
                    torso_res = self.body_position_estimation
                else:                
                    torso_res = np.array([0, 0, 0])
            else:
                torso_res_prev = torso_res

        elif self.body_position_estimation[0]:
            torso_res = self.body_position_estimation
        else:
            torso_res = np.array([0.0, 0.0, 0.0])

        ### Publishing tf transformations ###

        t = header.stamp.nanosec / 1e9

        self.node.get_logger().debug(f'time:  {header.stamp.nanosec}')

        if not self.one_euro_filter[0] and self.use_depth:
            self.node.get_logger().debug(f'torso res 2 {torso_res[2]}')
            self.one_euro_filter[0] = OneEuroFilter(
                t, 
                torso_res[0], 
                beta=BETA_POSITION, 
                d_cutoff=D_CUTOFF_POSITION, 
                min_cutoff=MIN_CUTOFF_POSITION)
            self.node.get_logger().debug(f'torso res 0 {torso_res[0]}')

            self.one_euro_filter[1] = OneEuroFilter(
                t, 
                torso_res[1], 
                beta=BETA_POSITION, 
                d_cutoff=D_CUTOFF_POSITION, 
                min_cutoff=MIN_CUTOFF_POSITION)
            
            self.one_euro_filter[2] = OneEuroFilter(
                t, 
                torso_res[2], 
                beta=BETA_POSITION, 
                d_cutoff=D_CUTOFF_POSITION, 
                min_cutoff=MIN_CUTOFF_POSITION)
            
            self.node.get_logger().debug('got here')
            self.node.get_logger().debug(f'time res {t}')
            self.node.get_logger().debug(f'torso res {torso_res[0]}')
            self.node.get_logger().debug(f'torso res {torso_res[2]}')
            self.body_filtered_position[0] = torso_res[1]
            self.body_filtered_position[1] = torso_res[1]
            self.body_filtered_position[2] = torso_res[2]

        elif self.use_depth:
            self.body_filtered_position_prev[0] = self.body_filtered_position[0]
            self.body_filtered_position_prev[1] = self.body_filtered_position[1]
            self.body_filtered_position_prev[2] = self.body_filtered_position[2]

            self.body_filtered_position[0], t_e = self.one_euro_filter[0](t, torso_res[0])
            self.body_filtered_position[1], _ = self.one_euro_filter[1](t, torso_res[1])
            self.body_filtered_position[2], _ = self.one_euro_filter[2](t, torso_res[2])

            self.node.get_logger().debug(f'one euro {(t, torso_res[2])}')
            self.node.get_logger().debug(f'body filtered {self.body_filtered_position}')

            self.position_msg.point.x = self.body_filtered_position[0]
            self.position_msg.point.y = 0.0
            self.position_msg.point.z = self.body_filtered_position[2]

            self.position_msg.header.stamp = self.node.get_clock().now().to_msg()
            self.position_msg.header.frame_id = header.frame_id
            self.body_filtered_position_pub.publish(self.position_msg)
            
            self.node.get_logger().debug(f't_e {t_e}')

            self.body_vel_estimation[0] = (self.body_filtered_position[0] - self.body_filtered_position_prev[0]) / t_e
            self.body_vel_estimation[1] = (self.body_filtered_position[1] - self.body_filtered_position_prev[1]) / t_e
            self.body_vel_estimation[2] = (self.body_filtered_position[2] - self.body_filtered_position_prev[2]) / t_e

            if not self.one_euro_filter_dot[0]:
                self.node.get_logger().debug(f'body vel {self.body_vel_estimation}')

                self.one_euro_filter_dot[0] = OneEuroFilter(
                    t, 
                    self.body_vel_estimation[0], 
                    beta=BETA_VELOCITY, 
                    d_cutoff=D_CUTOFF_VELOCITY, 
                    min_cutoff=MIN_CUTOFF_VELOCITY)
                
                self.one_euro_filter_dot[1] = OneEuroFilter(
                    t, 
                    self.body_vel_estimation[1], 
                    beta=BETA_VELOCITY, 
                    d_cutoff=D_CUTOFF_VELOCITY, 
                    min_cutoff=MIN_CUTOFF_VELOCITY)
                self.one_euro_filter_dot[2] = OneEuroFilter(
                    t, 
                    self.body_vel_estimation[2], 
                    beta=BETA_VELOCITY, 
                    d_cutoff=D_CUTOFF_VELOCITY, 
                    min_cutoff=MIN_CUTOFF_VELOCITY)
            else:
                self.body_vel_estimation_filtered[0], _ = \
                    self.one_euro_filter_dot[0](t, self.body_vel_estimation[0])
                self.body_vel_estimation_filtered[1], _ = \
                    self.one_euro_filter_dot[1](t, self.body_vel_estimation[1])
                self.body_vel_estimation_filtered[2], _ = \
                    self.one_euro_filter_dot[2](t, self.body_vel_estimation[2])
                
                self.velocity_msg.twist.linear.x = \
                    -self.body_vel_estimation_filtered[0]
                self.velocity_msg.twist.linear.y = \
                    self.body_vel_estimation_filtered[1]
                self.velocity_msg.twist.linear.y = \
                    self.body_vel_estimation_filtered[2]
                
                self.velocity_pub.publish(self.velocity_msg)

        if not self.use_depth:
            # todo(juandpenan) uncomment:
            # translation = (torso_res[0], 0.0, torso_res[2])
            translation = (torso_res[0], torso_res[1], torso_res[2])
        else:   
            translation = (self.body_filtered_position[0], 
                           self.body_filtered_position[1], 
                           self.body_filtered_position[2])
        t = TransformStamped()
        
        t.header.stamp = header.stamp
        t.header.frame_id = header.frame_id                    
        t.child_frame_id = "body_%s" % body_id

        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]
        
        q = quaternion_from_euler(np.pi/2, -theta, 0.0)

        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.node.get_logger().debug(f'publishing tf msg: {t}')
        self.tb.sendTransform(t)

        if self.stickman_debug:
            self.stickman_debugging(theta, 
                                    torso, 
                                    torso_res,
                                    l_shoulder,
                                    r_shoulder,
                                    l_elbow,
                                    r_elbow,
                                    l_wrist,
                                    r_wrist,
                                    l_ankle,
                                    r_ankle, 
                                    header)
            
        js.position = compute_jointstate(
            self.ik_chains[body_id], 
            torso,
            l_wrist,
            l_ankle,
            r_wrist,
            r_ankle
        )

        return js

    def check_bounding_box_consistency(self, bb):
        return bb.x_offset >= 0 \
            and bb.y_offset >= 0 \
            and bb.width > 0 \
            and bb.height > 0 \
            and (bb.x_offset + bb.width < self.img_width) \
            and (bb.y_offset + bb.height < self.img_height)

    def detect(self, image_rgb, header):

        img_height, img_width, _ = image_rgb.shape
        self.img_height, self.img_width = img_height, img_width

        image_rgb.flags.writeable = False
        image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB) # ok
        results = self.detector.process(image_rgb)
        image_rgb.flags.writeable = True
        self.image = image_rgb

        self.x_min_person = img_width
        self.y_min_person = img_height
        self.x_max_person = 0
        self.y_max_person = 0

        ######## Face Detection Process ########

        if hasattr(results.face_landmarks, 'landmark'):
            (self.x_min_face,
             self.y_min_face,
             self.x_max_face,
             self.y_max_face) = _get_bounding_box_limits(
                results.face_landmarks.landmark,
                img_width,
                img_height
            )

            self.x_min_person = int(min(
                self.x_min_person, 
                self.x_min_face))
            self.y_min_person = int(min(
                self.y_min_person, 
                self.y_min_face))
            self.x_max_person = int(max(
                self.x_max_person, 
                self.x_max_face))
            self.y_max_person = int(max(
                self.y_max_person, 
                self.y_max_face))

            if not self.use_depth and hasattr(self, "K"):
                # K = camera intrisic matrix. See method camera_info_callback 
                #     to understand more about it
                for idx, landmark in enumerate(results.face_landmarks.landmark):
                    if idx == FM_NOSE:
                        nose_tip = [landmark.x, landmark.y]
                    if idx == FM_MOUTH_CENTER:
                        mouth_center = [landmark.x, landmark.y]
                    if idx == FM_RIGHT_EYE:
                        right_eye = [landmark.x, landmark.y]
                    if idx == FM_RIGHT_EAR_TRAGION:
                        right_ear_tragion = [landmark.x, landmark.y]
                    if idx == FM_LEFT_EYE:
                        left_eye = [landmark.x, landmark.y]
                    if idx == FM_LEFT_EAR_TRAGION:
                        left_ear_tragion = [landmark.x, landmark.y]

                points_2D = np.array([
                    _normalized_to_pixel_coordinates(
                        nose_tip[0],
                        nose_tip[1],
                        self.img_width,
                        self.img_height),
                    _normalized_to_pixel_coordinates(
                        right_eye[0],
                        right_eye[1],
                        self.img_width,
                        self.img_height),
                    _normalized_to_pixel_coordinates(
                        left_eye[0],
                        left_eye[1],
                        self.img_width,
                        self.img_height),
                    _normalized_to_pixel_coordinates(
                        mouth_center[0],
                        mouth_center[1],
                        self.img_width,
                        self.img_height),
                    _normalized_to_pixel_coordinates(
                        right_ear_tragion[0],
                        right_ear_tragion[1],
                        self.img_width,
                        self.img_height),
                    _normalized_to_pixel_coordinates(
                        left_eye[0],
                        left_eye[1],
                        self.img_width,
                        self.img_height)], 
                    dtype="double")

                self.trans_vec, self.angles = \
                    face_pose_estimation(points_2D, self.K)

                if not self.trans_vec[0] \
                  or not self.trans_vec[1] \
                  or not self.trans_vec[2]:
                    self.valid_trans_vec = False
                elif np.isnan(self.trans_vec).any():
                    self.valid_trans_vec = False
                else:
                    self.valid_trans_vec = True

                if self.valid_trans_vec:
                    t = TransformStamped()
                    
                    t.header.stamp = self.node.get_clock().now().to_msg()
                    t.header.frame_id = header.frame_id                    
                    t.child_frame_id = "face_"+self.body_id

                    t.transform.translation.x = self.trans_vec[0]/1000
                    t.transform.translation.y = self.trans_vec[1]/1000
                    t.transform.translation.z = self.trans_vec[2]/1000
                    
                    q = quaternion_from_euler(self.angles[0]/180*np.pi, self.angles[1]/180*np.pi, self.angles[2]/180*np.pi)

                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]

                    self.tb.sendTransform(t)
                    
                    t.header.frame_id = "gaze_" + self.body_id
                    t.child_frame_id = "face_" + self.body_id

                    t.transform.translation.x = 0.0
                    t.transform.translation.y = 0.0
                    t.transform.translation.z = 0.0

                    q = quaternion_from_euler(-np.pi/2, 0.0, -np.pi/2)

                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]                 

                    self.tb.sendTransform(t)
                    
        ########################################

        ######## Introducing Hand Landmarks ########

        if hasattr(results.left_hand_landmarks, 'landmark'):
            pose_keypoints = protobuf_to_dict(results.pose_landmarks)
            pose_kpt = pose_keypoints.get('landmark')
            landmarks = [None] * 21
            for i in range(0, 21):
                msg = NormalizedPointOfInterest2D()
                msg.x = pose_kpt[i].get('x')
                msg.y = pose_kpt[i].get('y')
                msg.c = pose_kpt[i].get('visibility')
                landmarks[i] = msg
            (self.x_min_hand_left,
             self.y_min_hand_left,
             self.x_max_hand_left,
             self.y_max_hand_left) = _get_bounding_box_limits(landmarks,
                                                              img_width,
                                                              img_height)
            self.x_min_person = int(min(
                self.x_min_person, 
                self.x_min_hand_left))
            self.y_min_person = int(min(
                self.y_min_person, 
                self.y_min_hand_left))
            self.x_max_person = int(max(
                self.x_max_person, 
                self.x_max_hand_left))
            self.y_max_person = int(max(
                self.y_max_person, 
                self.y_max_hand_left))

        if hasattr(results.right_hand_landmarks, 'landmark'):
            pose_keypoints = protobuf_to_dict(results.pose_landmarks)
            pose_kpt = pose_keypoints.get('landmark')
            landmarks = [None] * 21
            for i in range(0, 21):
                msg =  NormalizedPointOfInterest2D()
                msg.x = pose_kpt[i].get('x')
                msg.y = pose_kpt[i].get('y')
                msg.c =  pose_kpt[i].get('visibility')

                landmarks[i] = msg

            (self.x_min_hand_right,
             self.y_min_hand_right,
             self.x_max_hand_right,
             self.y_max_hand_right) = _get_bounding_box_limits(landmarks,
                                                               img_width,
                                                               img_height)
            self.x_min_person = int(min(
                self.x_min_person, 
                self.x_min_hand_right))
            self.y_min_person = int(min(
                self.y_min_person, 
                self.y_min_hand_right))
            self.x_max_person = int(max(
                self.x_max_person, 
                self.x_max_hand_right))
            self.y_max_person = int(max(
                self.y_max_person, 
                self.y_max_hand_right))

        ############################################

        ######## Body Detection Process ########

        if hasattr(results.pose_landmarks, 'landmark'):
            pose_keypoints = protobuf_to_dict(results.pose_landmarks)
            pose_world_keypoints = protobuf_to_dict(
                results.pose_world_landmarks)
            pose_kpt = pose_keypoints.get('landmark')
            pose_world_kpt = pose_world_keypoints.get('landmark')
            skel_msg = _make_2d_skeleton_msg(header, pose_kpt)
            
            if self.valid_trans_vec and not self.use_depth:
                self.body_position_estimation = \
                    self.face_to_body_position_estimation(skel_msg)
                
            elif not self.valid_trans_vec and not self.use_depth:
                self.node.get_logger().error("It was not possible to estimate body position.")
            if self.use_depth or self.valid_trans_vec:
                js = self.make_jointstate(
                    self.body_id,
                    pose_world_kpt,
                    pose_kpt,
                    header
                )
                self.js_pub.publish(js)
            self.skel_pub.publish(skel_msg)
            if self.single_body:
                landmarks = [None]*32
                for i in range(0, 32):
                    msg = NormalizedPointOfInterest2D()
                    msg.x = pose_kpt[i].get('x')
                    msg.y = pose_kpt[i].get('y')
                    msg.c = pose_kpt[i].get('visibility')

                    landmarks[i] = msg

                
                (self.x_min_body,
                 self.y_min_body,
                 self.x_max_body,
                 self.y_max_body) = _get_bounding_box_limits(landmarks,
                                                                img_width,
                                                                img_height)
                self.x_min_person = int(min(
                    self.x_min_person, 
                    self.x_min_body))
                self.y_min_person = int(min(
                    self.y_min_person, 
                    self.y_min_body))
                self.x_max_person = int(max(
                    self.x_max_person, 
                    self.x_max_body))
                self.y_max_person = int(max(
                    self.y_max_person, 
                    self.y_max_body))

        if self.single_body:
            ids_list = IdsList()
            if self.x_min_person < self.x_max_person \
                and self.y_min_person < self.y_max_person:
                self.x_min_person = max(0, self.x_min_person)
                self.y_min_person = max(0, self.y_min_person)
                self.x_max_person = min(img_width, self.x_max_person)
                self.y_max_person = min(img_height, self.y_max_person)
                ids_list.ids = [self.body_id]
                roi = RegionOfInterest()
                roi.x_offset = self.x_min_person
                roi.y_offset = self.y_min_person
                roi.width = self.x_max_person - self.x_min_person
                roi.height = self.y_max_person - self.y_min_person
                self.roi_pub.publish(roi)
            self.ids_pub.publish(ids_list)

        ########################################

    def image_callback_depth(self, 
                rgb_img, 
                rgb_info, 
                roi,
                depth_img, 
                depth_info):

        rgb_img = self.br.imgmsg_to_cv2(rgb_img)
        image_depth = self.br.imgmsg_to_cv2(depth_img, "16UC1")
        self.image_depth = image_depth
        if depth_info.header.stamp > rgb_info.header.stamp:
            header = copy.copy(depth_info.header)
            header.frame_id = rgb_info.header.frame_id # to check 
        else:
            header = copy.copy(rgb_info.header)
        self.depth_info = depth_info
        self.rgb_info = rgb_info
        self.x_offset = roi.x_offset
        self.y_offset = roi.y_offset
        self.roi = roi
        self.detect(rgb_img, header)

    def image_callback_depth_single_person(self, 
                rgb_img, 
                rgb_info,
                depth_img, 
                depth_info):

        if self.skeleton_to_set:            
            self.skeleton_generation()
            self.skeleton_to_set = False

        rgb_img = self.br.imgmsg_to_cv2(rgb_img)
        image_depth = self.br.imgmsg_to_cv2(depth_img, "16UC1")
        self.image_depth = image_depth
        if depth_info.header.stamp.nanosec > rgb_info.header.stamp.nanosec:
            header = copy.copy(depth_info.header)
            header.frame_id = rgb_info.header.frame_id # to check 
        else:
            header = copy.copy(rgb_info.header)
        
        self.node.get_logger().debug(f'Header we are working with: {header}')
        self.depth_info = depth_info
        self.rgb_info = rgb_info
        self.x_offset = 0
        self.y_offset = 0
        self.roi = RegionOfInterest()
        self.roi.x_offset = 0
        self.roi.y_offset = 0
        self.detect(rgb_img, header)

    def image_callback_rgb(self, rgb_img, rgb_info):

        if self.skeleton_to_set:
            self.skeleton_generation()
            self.skeleton_to_set = False

        if not self.node.has_parameter(self.human_description):
            self.node.get_logger().error(f'URDF model of the human not yet available on the ROS parameter server. {self.human_description}')
            # todo(juandpenan) uncomment return
            return
        
        rgb_img = self.br.imgmsg_to_cv2(rgb_img)        

        header = copy.copy(rgb_info.header)
        self.rgb_info = rgb_info
        self.detect(rgb_img, header)

    def get_image_topic(self):
        return self.image_subscriber.topic
