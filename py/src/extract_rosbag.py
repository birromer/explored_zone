#!/usr/bin/env python3

import sensor_msgs.point_cloud2 as pc2
import numpy as np
import rosbag
import os
import yaml
import sys


if __name__ == "__main__":

##### ROSBAG SETTINGS
    DATA_DIR = "../data/bags"
#    bag_filename = 'bag_2021-10-07-11-09-50.bag'
    bag_filename = 'bag_2021-10-07-11-09-50.bag'
    extracted_dir = "../data/extracted/"+bag_filename[:-4]
#    os.mkdir(extracted_dir)

    bag_path = os.path.join(DATA_DIR, bag_filename)

    print("Input bag:", bag_path)
    print("Output folder: ", extracted_dir)
#    bag_path =  sys.argv[1]

    bag = rosbag.Bag(bag_path, "r")

    # load the rosbag data
    info_dict = yaml.load(rosbag.Bag(bag_path, 'r')._get_yaml_info())

    # print the information contained in the info_dict
    info_dict.keys()
    for topic in info_dict["topics"]:
        print("-"*50)
        for k, v in topic.items():
            print(k.ljust(20), v)

    op = input("Do you with to continue? (yes/no) ")
    if op == "n" or op == "no":
        sys.exit(0)

##### EXTRACT DATA
    # create iterator for each topic
    heading_msg = bag.read_messages(topics=['/heading'])  # Float64
    imu_msg = bag.read_messages(topics=['/imu/data'])  # Imu
    gps_msg = bag.read_messages(topics=['/fix'])  # NavSatFix
    using_mag_msg = bag.read_messages(topics=['/using_mag'])  # Bool
#    twist_msg = bag.read_messages(topics=['/overGround'])  # TwistWithCovariance, from DVL (?)
#    str_msg = bag.read_messages(topics=['/str_data'])  # String, from the DVL (?)

    # get number of messages
    n_heading_msg = bag.get_message_count(topic_filters=["/heading"])
    n_imu_msg = bag.get_message_count(topic_filters=["/imu/data"])
    n_gps_msg = bag.get_message_count(topic_filters=["/fix"])
    n_using_mag_msg = bag.get_message_count(topic_filters=["/using_mag"])
#    n_twist_msg = bag.get_message_count(topic_filters=["/overGround"])
#    n_str_msg = bag.get_message_count(topic_filters=["/str_data"])

##### READ AND STORE DATA
    # Heading
    print("\nReading heading data.")
    heading = np.zeros((0,1), dtype=np.float64)

    for i in range(n_heading_msg):
        topic, msg, t = next(heading_msg)
        theta = np.array([[msg.data]])  # transform msg in numpy array
        heading = np.concatenate((heading, theta), axis=0)

    print("Finished reading heading. Shape: ", heading.shape)

    # IMU
    # data structure definition here: https://docs.ros.org/en/api/sensor_msgs/html/msg/Imu.html
    print("\nReading IMU.")
    imu_t = np.zeros((0,2), dtype=np.float64)  # secs, nsecs
    imu_orient = np.zeros((0,4), dtype=np.float64)  # x, y, z, w
    imu_orient_cov = np.zeros((0,9), dtype=np.float64)  # covariance matrix
    imu_ang_vel = np.zeros((0,3), dtype=np.float64)  # x, y, z
    imu_ang_vel_cov = np.zeros((0,9), dtype=np.float64)  # covariance matrix
    imu_lin_acc = np.zeros((0,3), dtype=np.float64)  # x, y, z
    imu_lin_acc_cov = np.zeros((0,9), dtype=np.float64)  # covariance matrix

    for i in range(n_imu_msg):
        topic, msg, t = next(imu_msg)

        # imu msg has the following available:
        # - header, with timestamps
        t = np.array([[msg.header.stamp.secs, msg.header.stamp.nsecs]])
        imu_t = np.concatenate((imu_t, t), axis=0)

        # - orientation
        orient = np.array([[msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]])
        imu_orient = np.concatenate((imu_orient, orient), axis=0)

        # - orientation covariance
        orient_cov = np.array([msg.orientation_covariance])
        imu_orient_cov = np.concatenate((imu_orient_cov, orient_cov), axis=0)

        # - angular velocity
        ang_vel = np.array([[msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]])
        imu_ang_vel = np.concatenate((imu_ang_vel, ang_vel), axis=0)

        # - angular velocity covariance
        ang_vel_cov = np.array([msg.angular_velocity_covariance])
        imu_ang_vel_cov = np.concatenate((imu_ang_vel_cov, ang_vel_cov), axis=0)

        # - linear acceleration
        lin_acc = np.array([[msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]])
        imu_lin_acc = np.concatenate((imu_lin_acc, lin_acc), axis=0)

        # - linear acceleration covariance
        lin_acc_cov = np.array([msg.linear_acceleration_covariance])
        imu_lin_acc_cov = np.concatenate((imu_lin_acc_cov, lin_acc_cov), axis=0)

    print("Finished reading. Shapes:")
    print("IMU time ->", imu_t.shape)
    print("IMU orientation ->", imu_orient.shape)
    print("IMU orientation covariance ->", imu_orient_cov.shape)
    print("IMU angular velocity ->", imu_ang_vel.shape)
    print("IMU angular velocity covariance ->", imu_ang_vel_cov.shape)
    print("IMU linear acceleration ->", imu_lin_acc.shape)
    print("IMU linear acceleration  covariance ->", imu_lin_acc_cov.shape)

    # GPS
    # data structure definition here: https://docs.ros.org/en/api/sensor_msgs/html/msg/NavSatFix.html
    print("\nReading GPS.")
    gps_t = np.zeros((0,2), dtype=np.float64)  # secs, nsecs
    gps_status = np.zeros((0, 3), dtype=np.int32)  # status, service, type
    gps_pos = np.zeros((0,3), dtype=np.float64)  # lon, lat, alt
    gps_pos_cov = np.zeros((0,9), dtype=np.float64)  # covariance matrix

    for i in range(n_gps_msg):
        topic, msg, t = next(gps_msg)

        # gps msg has the following available:
        # - header, with timestamps
        t = np.array([[msg.header.stamp.secs, msg.header.stamp.nsecs]])
        gps_t = np.concatenate((gps_t, t), axis=0)

        # - status, service and covariance type, where the possible types are:
        #   - 0: unkown
        #   - 1: approximated
        #   - 2: diagonal known
        #   - 3: known
        status = np.array([[msg.status.status, msg.status.service, msg.position_covariance_type]])
        gps_status = np.concatenate((gps_status, status), axis=0)

        # - position, with longitude, latitute and altitude
        pos = np.array([[msg.longitude, msg.latitude, msg.altitude]])
        gps_pos = np.concatenate((gps_pos, pos), axis=0)

        # - position covariance
        pos_cov = np.array([msg.position_covariance])
        gps_pos_cov = np.concatenate((gps_pos_cov, pos_cov), axis=0)
    print("Finished reading. Shapes:")
    print("GPS time ->", gps_t.shape)
    print("GPS status ->", gps_status.shape)
    print("GPS position ->", gps_pos.shape)
    print("GPS position covariance ->", gps_pos_cov.shape)

    # using mag flag
    print("\nReading using magnetometer flag.")
    using_mag = np.zeros((0,1), dtype=np.int32)  # bool equivalent
    for i in range(n_using_mag_msg):
        topic, msg, t = next(using_mag_msg)

        flag = np.array([[msg.data]])
        using_mag = np.concatenate((using_mag, flag), axis=0)
    print("Finished reading. Shape:", using_mag.shape)

    print("\nFinished reading rosbag", bag_filename)

    print("\nSaving data.")
    np.savez(os.path.join(extracted_dir,"heading.npz"), heading=heading)

    np.savez_compressed(os.path.join(extracted_dir,"imu.npz"), imu_t=imu_t,
                        imu_orient=imu_orient, imu_orient_cov=imu_orient_cov,
                        imu_ang_vel=imu_ang_vel, imu_ang_vel_cov=imu_ang_vel_cov,
                        imu_lin_acc=imu_lin_acc, imu_lin_acc_cov=imu_lin_acc_cov)

    np.savez_compressed(os.path.join(extracted_dir,"gps.npz"), gps_t=gps_t,
                        gps_status=gps_status, gps_pos=gps_pos, gps_pos_cov=gps_pos_cov)

    np.savez_compressed(os.path.join(extracted_dir,"mag_flag.npz"), using_mag=using_mag)

    np.savetxt(os.path.join(extracted_dir,"pos.txt"), gps_pos)
    np.savetxt(os.path.join(extracted_dir,"imu_orient.txt"), imu_orient)
    np.savetxt(os.path.join(extracted_dir,"heading.txt"), heading)

    print("\nFinished saving all data.")
