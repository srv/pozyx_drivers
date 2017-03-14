#!/usr/bin/env python

import pypozyx
import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import *
from sensor_msgs.msg import Imu
from pozyx_drivers.msg import AnchorsRange


class ReadyToLocalize(object):

    def __init__(self, pozyx, anchors, algorithm=POZYX_POS_ALG_LS, dimension=POZYX_3D, height=1000):
        self.pozyx = pozyx
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.range_error_counts = [0 for i in xrange(len(self.anchors))]
        self.frame_id = 'pozyx_tag'

    def setup(self):
        self.setAnchorsManual()
        self.printPublishConfigurationResult()

    def loop(self):

        #Topic 1: PoseWitchCovariance
        pwc = PoseWithCovarianceStamped()
        pwc.header.stamp = rospy.get_rostime()
        pwc.header.frame_id = self.frame_id
        pwc.pose.pose.position = Coordinates()
        pwc.pose.pose.orientation =  pypozyx.Quaternion()
        cov = pypozyx.PositionError()

        status = self.pozyx.doPositioning(pwc.pose.pose.position, self.dimension, self.height, self.algorithm)
        pozyx.getQuaternion(pwc.pose.pose.orientation)
        pozyx.getPositionError(cov)

        cov_row1 =[cov.x, cov.xy, cov.xz, 0, 0, 0]
        cov_row2 =[cov.xy, cov.y, cov.yz, 0, 0, 0]
        cov_row3 =[cov.xz, cov.yz, cov.z, 0, 0, 0]
        cov_row4 =[0, 0, 0, 0, 0, 0]
        cov_row5 =[0, 0, 0, 0, 0, 0]
        cov_row6 =[0, 0, 0, 0, 0, 0]

        pwc.pose.covariance = cov_row1 + cov_row2 + cov_row3 + cov_row4 + cov_row5 + cov_row6

        #Convert from mm to m
        pwc.pose.pose.position.x = pwc.pose.pose.position.x * 0.001
        pwc.pose.pose.position.y = pwc.pose.pose.position.y * 0.001
        pwc.pose.pose.position.z = pwc.pose.pose.position.z * 0.001

        if status == POZYX_SUCCESS:
            pub_pose_with_cov.publish(pwc)

        #Topic 2: IMU
        imu = Imu()
        imu.header.stamp = rospy.get_rostime()
        imu.header.frame_id = self.frame_id
        imu.orientation =  pypozyx.Quaternion()
        imu.orientation_covariance = [0,0,0,0,0,0,0,0,0]
        imu.angular_velocity = pypozyx.AngularVelocity()
        imu.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]
        imu.linear_acceleration = pypozyx.LinearAcceleration()
        imu.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]

        pozyx.getQuaternion(imu.orientation)
        pozyx.getAngularVelocity_dps(imu.angular_velocity)
        pozyx.getLinearAcceleration_mg(imu.linear_acceleration)

        #Convert from mg to m/s2
        imu.linear_acceleration.x = imu.linear_acceleration.x * 0.0098
        imu.linear_acceleration.y = imu.linear_acceleration.y * 0.0098
        imu.linear_acceleration.z = imu.linear_acceleration.z * 0.0098

        #Convert from Degree/second to rad/s
        imu.angular_velocity.x = imu.angular_velocity.x * 0.01745
        imu.angular_velocity.y = imu.angular_velocity.y * 0.01745
        imu.angular_velocity.z = imu.angular_velocity.z * 0.01745

        pub_imu.publish(imu)

        #Topic 3: Anchors Infor
        for i in range(len(anchors)):
            dr = AnchorsRange()
            dr.header.stamp = rospy.get_rostime()
            dr.header.frame_id = self.frame_id
            dr.id = hex(anchors[i].network_id)

            device_range = DeviceRange()
            status = self.pozyx.doRanging(self.anchors[i].network_id, device_range)
            dr.distance = device_range.distance
            dr.RSS = device_range.RSS

            if status == POZYX_SUCCESS:
                dr.status = True
                self.range_error_counts[i] = 0
            else:
                dr.status = False
                self.range_error_counts[i] += 1
                if self.range_error_counts[i] > 9:
                    self.range_error_counts[i] = 0
                    rospy.logerr("Anchor %s lost", dr.id)
            pub_anchors_info.publish(dr)

        #Topic 4: PoseStamped
        ps = PoseStamped()
        ps.header.stamp = rospy.get_rostime()
        ps.header.frame_id = self.frame_id
        ps.pose.position = pwc.pose.pose.position
        ps.pose.orientation =  pwc.pose.pose.orientation

        pub_pose.publish(ps)

    def setAnchorsManual(self):
        status = self.pozyx.clearDevices()
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor)
        if len(anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(anchors))
        return status

    def printPublishConfigurationResult(self):
        list_size = SingleRegister()
        status = self.pozyx.getDeviceListSize(list_size)
        device_list = DeviceList(list_size=list_size[0])
        status = self.pozyx.getDeviceIds(device_list)

        print("Anchors configuration:")
        print("Anchors found: {0}".format(list_size[0]))

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            status = self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates)
            print("ANCHOR,0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))


if __name__ == "__main__":

    serial_port = get_serial_ports()[0].device

    rospy.init_node('pozyx_node')

    # Reading parameters
    anchor0_id = int(rospy.get_param('~anchor0_id'), 16)
    anchor1_id = int(rospy.get_param('~anchor1_id'), 16)
    anchor2_id = int(rospy.get_param('~anchor2_id'), 16)
    anchor3_id = int(rospy.get_param('~anchor3_id'), 16)

    anchor0_coordinates = eval(rospy.get_param('~anchor0_coordinates'))
    anchor1_coordinates = eval(rospy.get_param('~anchor1_coordinates'))
    anchor2_coordinates = eval(rospy.get_param('~anchor2_coordinates'))
    anchor3_coordinates = eval(rospy.get_param('~anchor3_coordinates'))

    algorithm = int(rospy.get_param('~algorithm'))
    dimension = int(rospy.get_param('~dimension'))
    height    = int(rospy.get_param('~height'))
    frequency = int(rospy.get_param('~frequency'))

    # Creating publishers
    pub_pose_with_cov = rospy.Publisher('~pose_with_cov', PoseWithCovarianceStamped, queue_size=1)
    pub_imu = rospy.Publisher('~imu', Imu, queue_size=1)
    pub_anchors_info = rospy.Publisher('~device_range', AnchorsRange, queue_size=1)
    pub_pose = rospy.Publisher('~pose', PoseStamped , queue_size=1)

    anchors = [DeviceCoordinates(anchor0_id, 1, Coordinates(anchor0_coordinates[0], anchor0_coordinates[1], anchor0_coordinates[2])),
               DeviceCoordinates(anchor1_id, 1, Coordinates(anchor1_coordinates[0], anchor1_coordinates[1], anchor1_coordinates[2])),
               DeviceCoordinates(anchor2_id, 1, Coordinates(anchor2_coordinates[0], anchor2_coordinates[1], anchor2_coordinates[2])),
               DeviceCoordinates(anchor3_id, 1, Coordinates(anchor3_coordinates[0], anchor3_coordinates[1], anchor3_coordinates[2]))]

    rate = rospy.Rate(frequency)

    # Starting communication with Pozyx
    pozyx = PozyxSerial(serial_port)
    rdl = ReadyToLocalize(pozyx, anchors, algorithm, dimension, height)
    rdl.setup()
    while not rospy.is_shutdown():
        rdl.loop()
        rate.sleep()
