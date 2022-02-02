#!/usr/bin/env python2

from concurrent import futures

import rospy
import roslib
from rosgraph_msgs.msg import Clock

import geometry_msgs.msg as geomsgs

import tf2_ros
import tf.transformations as tftrans

import std_msgs.msg
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2 # added this
from ros_adapter.msg import RadarSpoke

from cv_bridge import CvBridge, CvBridgeError

import grpc
import cv2

from sensor_streaming import sensor_streaming_pb2
from sensor_streaming import sensor_streaming_pb2_grpc
from navigation import navigation_pb2
from navigation import navigation_pb2_grpc

import numpy as np
from math import pi


class SensorStreaming(sensor_streaming_pb2_grpc.SensorStreamingServicer):
    def __init__(self, camera_pubs, lidar_pub, radar_pub, clock_pub):
        print("creating")
        self.bridge = CvBridge()
        self.camera_pubs = camera_pubs
        self.lidar_pub = lidar_pub
        self.radar_pub = radar_pub
        self.clock_pub = clock_pub

    def StreamCameraSensor(self, request, context):
        """
        Takes in a gRPC SensorStreamingRequest containing
        all the data needed to create and publish a sensor_msgs/Image
        ROS message.
        """
        img_string = request.data

        cv_image = np.fromstring(img_string, np.uint8)

        # NOTE, the height is specifiec as a parameter before the width
        cv_image = cv_image.reshape(request.height, request.width, 3)
        cv_image = cv2.flip(cv_image, 0)

        bgr_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        msg = Image()
        header = std_msgs.msg.Header()
        try:
            msg = self.bridge.cv2_to_imgmsg(bgr_image, 'bgr8')

            header.stamp = rospy.Time.from_sec(request.timeStamp)
            msg.header = header
        except CvBridgeError as e:
            print(e)

        camera_pubs[request.frame_id].publish(msg)

        return sensor_streaming_pb2.CameraStreamingResponse(success=True)

    def StreamLidarSensor(self, request, context):
        """
        Takes in a gRPC LidarStreamingRequest containing
        all the data needed to create and publish a PointCloud2
        ROS message.
        """

        pointcloud_msg = PointCloud2()
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.from_sec(request.timeInSeconds)

        header.frame_id = "velodyne"
        pointcloud_msg.header = header

        pointcloud_msg.height = request.height
        pointcloud_msg.width = request.width

        fields = request.fields

        # Set PointCloud[] fields in pointcloud_msg
        for i in range(len(fields)):
            pointcloud_msg.fields.append(PointField())
            pointcloud_msg.fields[i].name = fields[i].name
            pointcloud_msg.fields[i].offset = fields[i].offset
            pointcloud_msg.fields[i].datatype = fields[i].datatype
            pointcloud_msg.fields[i].count = fields[i].count

        pointcloud_msg.is_bigendian = request.isBigEndian
        pointcloud_msg.point_step = request.point_step
        pointcloud_msg.row_step = request.row_step

        pointcloud_msg.data = request.data

        pointcloud_msg.is_dense = request.is_dense

        # filter points that are on the boat ------------------------------------
        assert isinstance(pointcloud_msg, PointCloud2)
        gen = pc2.read_points(pointcloud_msg, field_names=('x','y','z'), skip_nans=True)
        points = []
        for p in gen:
            if not(p[0]>-3.0 and p[0]<3.0 and p[1]>-4.0 and p[1]<10.0):  # remove points that belong to the boat
                points.append(p)
        points = np.reshape(points, (len(points), 3))
        points = np.array([points[:,0], points[:,1], points[:,2], points[:,2]])
        points = points.reshape(4,-1).T

        fields2 = [PointField('x', 0, PointField.FLOAT32, 1),
              PointField('y', 4, PointField.FLOAT32, 1),
              PointField('z', 8, PointField.FLOAT32, 1),
              PointField('intensity', 12, PointField.FLOAT32, 1)]
        pointcloud_msg2 = pc2.create_cloud(header, fields2, points)

        # ----------------------------------------------------------------------

        self.lidar_pub.publish(pointcloud_msg2)

        # TODO: This does not belong in this RPC implementation, should be
        # moved to own or something like that.
        sim_clock = Clock()
        sim_clock.clock = rospy.Time.from_sec(request.timeInSeconds)
        self.clock_pub.publish(sim_clock)

        return sensor_streaming_pb2.LidarStreamingResponse(success=True)

    def StreamRadarSensor(self, request, context):
        """
        Takes in a gRPC RadarStreamingRequest containing
        all the data needed to create and publish a RadarSpoke
        ROS message.
        """

        number_of_spokes = request.numSpokes

        for i in range(number_of_spokes):

            radar_spoke_msg = RadarSpoke()

            # Header
            header = std_msgs.msg.Header()
            header.frame_id = "milliampere_radar"
            header.stamp = rospy.Time.from_sec(request.timeInSeconds[i])
            radar_spoke_msg.azimuth = request.azimuth[i]
            radar_spoke_msg.intensity = request.radarSpokes[i * request.numSamples : i * request.numSamples + request.numSamples]

            radar_spoke_msg.range_start = request.rangeStart
            radar_spoke_msg.range_increment = request.rangeIncrement
            radar_spoke_msg.min_intensity = request.minIntensity
            radar_spoke_msg.max_intensity = request.maxIntensity
            radar_spoke_msg.num_samples = request.numSamples

            self.radar_pub.publish(radar_spoke_msg)

        return sensor_streaming_pb2.RadarStreamingResponse(success=True)


class Navigation(navigation_pb2_grpc.NavigationServicer):
    def __init__(self, pose_pub, twist_pub, tf_pub):
        self.pose_pub = pose_pub
        self.twist_pub = twist_pub
        self.tf_pub = tf_pub

    def SendNavigationMessage(self, request, context):

        quat = tftrans.quaternion_from_euler(0, 0, 0)

        # TODO: This frame_id should be dynamically set from a config file.
        nav_header = std_msgs.msg.Header(
            frame_id="fosenkaia_NED",
            stamp=rospy.Time.from_sec(request.timeStamp)
        )

        position = geomsgs.Point()
        position.x = -request.position.x
        position.y = -request.position.y
        position.z = 0 #request.position.z

        orientation = geomsgs.Quaternion()
        orientation.x = quat[0] #request.orientation.x
        orientation.y = quat[1] #request.orientation.y
        orientation.z = quat[2] #request.orientation.z
        orientation.w = quat[3] #request.orientation.w

        #orientation in rpy is (0.0, -0.0, 2.6179938162270884) = (0,0,150) in degrees

        #print(orientation)
        #cenas = [orientation.x, orientation.y, orientation.z, orientation.w]
        #print(cenas)
        #print(tftrans.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w]))

        pose_msg = geomsgs.PoseStamped(
            header=nav_header,
            pose=geomsgs.Pose(
                position=position,
                orientation=orientation
            )
        )

        pose_pub.publish(pose_msg)

        linear_vel = geomsgs.Vector3()
        linear_vel.x = request.linearVelocity.x
        linear_vel.y = request.linearVelocity.y
        linear_vel.z = request.linearVelocity.z

        angular_vel = geomsgs.Vector3()
        angular_vel.x = request.angularVelocity.x
        angular_vel.y = request.angularVelocity.y
        angular_vel.z = request.angularVelocity.z

        twist_msg = geomsgs.TwistStamped(
            header=nav_header,
            twist=geomsgs.Twist(
                linear=linear_vel,
                angular=angular_vel
            )
        )

        twist_pub.publish(twist_msg)

        transform = geomsgs.TransformStamped(
            header=nav_header,
            child_frame_id="vessel_center",
            transform=geomsgs.Transform(
                translation=position,
                rotation=orientation
            )
        )

        tf_pub.sendTransform(transform)

        # criei aqui uma transformacao do velodyne para vessel_center idk
        header2 = std_msgs.msg.Header(
            frame_id="vessel_center",
            stamp=rospy.Time.from_sec(request.timeStamp)
        )
        position2 = geomsgs.Point()
        position2.x = 0
        position2.y = -2.525 -0.487
        position2.z = 3.073

        orientation2 = geomsgs.Quaternion()
        orientation2.x = quat[0]
        orientation2.y = quat[1]
        orientation2.z = quat[2]
        orientation2.w = quat[3]

        transform2 = geomsgs.TransformStamped(
            header=header2,
            child_frame_id="velodyne",
            transform=geomsgs.Transform(
                translation=position2,
                rotation=orientation2
            )
        )
        tf_pub.sendTransform(transform2)

        return navigation_pb2.NavigationResponse(success=True)


def serve(server_ip, server_port, camera_pubs,
          lidar_pub, radar_pub, clock_pub,
          pose_pub, twist_pub, tf_pub):

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))

    sensor_streaming_pb2_grpc.add_SensorStreamingServicer_to_server(
            SensorStreaming(camera_pubs, lidar_pub, radar_pub, clock_pub),
            server)

    navigation_pb2_grpc.add_NavigationServicer_to_server(
        Navigation(pose_pub, twist_pub, tf_pub),
        server)

    server.add_insecure_port(server_ip + ':' + str(server_port))
    print(server_ip + ":" + str(server_port))
    server.start()
    server.wait_for_termination()


if __name__ == '__main__':

    rospy.init_node('syntetic_data', anonymous=True)
    server_params = rospy.get_param('~')
    server_ip = server_params["server_ip"]
    server_port = server_params["server_port"]

    cam_ids = ["Front", "Back", "Starboard", "Port"]
    camera_pubs = dict()
    for cam_id in cam_ids:
        camera_pubs[cam_id] = rospy.Publisher('optical/' + cam_id + '/image_raw',
                                              Image, queue_size=10)

    lidar_pub = rospy.Publisher('lidar',
                                PointCloud2,
                                queue_size=10)

    # TODO: Change the message type to be published
    radar_pub = rospy.Publisher('radar/driver/spokes',
                                RadarSpoke,
                                queue_size=10)

    clock_pub = rospy.Publisher('clock', Clock, queue_size=10)

    pose_pub = rospy.Publisher('nav/pose', geomsgs.PoseStamped, queue_size=10)

    twist_pub = rospy.Publisher('nav/twist', geomsgs.TwistStamped, queue_size=10)

    tf_pub = tf2_ros.TransformBroadcaster()

    serve(server_ip, server_port, camera_pubs,
          lidar_pub, radar_pub, clock_pub,
          pose_pub, twist_pub, tf_pub)
