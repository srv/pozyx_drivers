#!/usr/bin/env python

import rospy
import pypozyx
from pypozyx import *
from std_msgs.msg import Header
from geometry_msgs.msg import *
from sensor_msgs.msg import Imu
from pozyx_drivers.msg import AnchorInfo
from sensor_msgs.msg import FluidPressure


class ReadyToLocalize(object):

    #def __init__(self, pozyx, anchors, algorithm=POZYX_POS_ALG_LS, dimension=POZYX_3D, height=1000):
    def __init__(self, pozyx, anchors, do_ranging_attempts, world_frame_id, tag_frame_id, algorithm=POZYX_POS_ALG_TRACKING, dimension=POZYX_3D, height=1000):
        self.pozyx = pozyx
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.range_error_counts = [0 for i in xrange(len(self.anchors))]
        self.world_frame_id = world_frame_id
        self.tag_frame_id = tag_frame_id
        self.do_ranging_attempts = do_ranging_attempts

    def setup(self):
        self.setAnchorsManual()
        self.printPublishConfigurationResult()

    def loop(self):

        #Topic 1: PoseWitchCovariance
        pwc = PoseWithCovarianceStamped()
        pwc.header.stamp = rospy.get_rostime()
        pwc.header.frame_id = self.world_frame_id
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
        imu.header.frame_id = self.tag_frame_id
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

        #Topic 3: Anchors Info
        for i in range(len(anchors)):
            dr = AnchorInfo()
            dr.header.stamp = rospy.get_rostime()
            dr.header.frame_id = self.world_frame_id
            dr.id = hex(anchors[i].network_id)
            dr.position.x = (float)(anchors[i].pos.x) * 0.001
            dr.position.y = (float)(anchors[i].pos.y) * 0.001
            dr.position.z = (float)(anchors[i].pos.z) * 0.001

            iter_ranging = 0
            while iter_ranging < self.do_ranging_attempts:

                device_range = DeviceRange()
                status = self.pozyx.doRanging(self.anchors[i].network_id, device_range)
                dr.distance = (float)(device_range.distance) * 0.001
                dr.RSS = device_range.RSS

                if status == POZYX_SUCCESS:
                    dr.status = True
                    self.range_error_counts[i] = 0
                    iter_ranging = self.do_ranging_attempts
                else:
                    dr.status = False
                    self.range_error_counts[i] += 1
                    if self.range_error_counts[i] > 9:
                        self.range_error_counts[i] = 0
                        rospy.logerr("Anchor %d (%s) lost", i, dr.id)
                iter_ranging += 1



            # device_range = DeviceRange()
            # status = self.pozyx.doRanging(self.anchors[i].network_id, device_range)
            # dr.distance = (float)(device_range.distance) * 0.001
            # dr.RSS = device_range.RSS

            # if status == POZYX_SUCCESS:
            #     dr.status = True
            #     self.range_error_counts[i] = 0
            # else:
            #     status = self.pozyx.doRanging(self.anchors[i].network_id, device_range)
            #     dr.distance = (float)(device_range.distance) * 0.001
            #     dr.RSS = device_range.RSS
            #     if status == POZYX_SUCCESS:
            #         dr.status = True
            #         self.range_error_counts[i] = 0
            #     else:
            #         dr.status = False
            #         self.range_error_counts[i] += 1
            #         if self.range_error_counts[i] > 9:
            #             self.range_error_counts[i] = 0
            #             rospy.logerr("Anchor %d (%s) lost", i, dr.id)

            dr.child_frame_id = "anchor_" + str(i)
            pub_anchor_info[i].publish(dr)


        #Topic 4: PoseStamped
        ps = PoseStamped()
        ps.header.stamp = rospy.get_rostime()
        ps.header.frame_id = self.world_frame_id
        ps.pose.position = pwc.pose.pose.position
        ps.pose.orientation =  pwc.pose.pose.orientation

        pub_pose.publish(ps)

        #Topic 5: Pressure
        pr = FluidPressure()
        pr.header.stamp = rospy.get_rostime()
        pr.header.frame_id = self.tag_frame_id
        pressure = pypozyx.Pressure()
        
        pozyx.getPressure_Pa(pressure)
        pr.fluid_pressure = pressure.value
        pr.variance = 0

        pub_pressure.publish(pr)

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

    # serial_port = get_serial_ports()[0].device

    rospy.init_node('pozyx_node')

    # Reading parameters

    serial_port = rospy.get_param('~serial_port', '/dev/ttyACM0')

    num_anchors = int(rospy.get_param('~num_anchors', 4))

    anchor_id = []
    anchor_coordinates = []

    for i in range(num_anchors):
        param_name = "~anchor" + str(i) + "_id"
        anchor_id.append(int(rospy.get_param(param_name), 16))

        param_name2 = "~anchor" + str(i) + "_coordinates"
        anchor_coordinates.append(eval(rospy.get_param(param_name2)))


    algorithm = int(rospy.get_param('~algorithm'))
    dimension = int(rospy.get_param('~dimension'))
    height    = int(rospy.get_param('~height'))
    frequency = int(rospy.get_param('~frequency'))

    world_frame_id = rospy.get_param('~world_frame_id', 'world')
    tag_frame_id = rospy.get_param('~tag_frame_id', 'pozyx_tag')

    do_ranging_attempts = rospy.get_param('~do_ranging_attempts', 1)
    if do_ranging_attempts < 1:
        do_ranging_attempts = 1

    # Creating publishers
    pub_pose_with_cov = rospy.Publisher('~pose_with_cov', PoseWithCovarianceStamped, queue_size=1)
    pub_imu = rospy.Publisher('~imu', Imu, queue_size=1)

    anchors = []
    for i in range(num_anchors):
        anchors.append(DeviceCoordinates(anchor_id[i], 1, Coordinates(anchor_coordinates[i][0], anchor_coordinates[i][1], anchor_coordinates[i][2])))

    pub_anchor_info = []

    for i in range(len(anchors)):
        topic_name = "~anchor_info_" + str(i)
        pub_anchor_info.append(rospy.Publisher(topic_name, AnchorInfo, queue_size=1))

    pub_pose = rospy.Publisher('~pose', PoseStamped , queue_size=1)
    pub_pressure = rospy.Publisher('~pressure', FluidPressure , queue_size=1)

    rate = rospy.Rate(frequency)

    # Starting communication with Pozyx
    pozyx = PozyxSerial(serial_port)
    rdl = ReadyToLocalize(pozyx, anchors, do_ranging_attempts, world_frame_id, tag_frame_id, algorithm, dimension, height)
    rdl.setup()
    while not rospy.is_shutdown():
        rdl.loop()
        rate.sleep()
