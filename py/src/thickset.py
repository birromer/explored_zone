#!/usr/bin/env python3

import os
import numpy as np
from vibes import *
from codac import *

import matplotlib.pyplot as plt

DATA_DIR = "../data/extracted/bag_2021-10-07-11-09-50"

# adapted from https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
# qx, qy, qz, qw ->  phi (roll/bank), theta (pitch), psi (heading/yaw)
def quaternion_to_euler(quaternion):
    qx, qy, qz, qw = quaternion

    test = qx * qy + qz + qw

    if test > 0.499:  # singularity at north pole
        yaw = 2 * np.arctan2(qx, qw)
        pitch = np.pi / 2
        roll = 0
    elif test < -0.499:  # singularity at south pole
        yaw = -2 * np.arctan2(qx, qw)
        pitch = -np.pi / 2
        roll = 0
    else:
        yaw = np.arctan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * qy * qy - 2 * qz * qz)
        pitch = np.arcsin(2 * test)
        roll = np.arctan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * qx * qx - 2 * qz * qz)

    return roll, pitch, yaw


if __name__ == "__main__":
    ##### LOAD DATA
    print("Reading data")
    # load heading
    hd = np.load(os.path.join(DATA_DIR, "heading.npz"))
    heading = hd["heading"]

    # load imu
    imu = np.load(os.path.join(DATA_DIR, "imu.npz"))
    t_imu = imu["imu_t"]
    imu_orient, imu_orient_cov = imu["imu_orient"], imu["imu_orient_cov"]
    imu_orient = np.apply_along_axis(quaternion_to_euler, 1, imu_orient)
    imu_ang_vel, imu_ang_vel_cov = imu["imu_ang_vel"], imu["imu_ang_vel_cov"]
    imu_lin_acc, imu_lin_acc_cov = imu["imu_lin_acc"], imu["imu_lin_acc_cov"]

    err_orient = np.array(
        [imu_orient_cov[:, 0], imu_orient_cov[:, 4], imu_orient_cov[:, 8]]
    )
    err_ang_vel = np.array(
        [imu_ang_vel_cov[:, 0], imu_ang_vel_cov[:, 4], imu_ang_vel_cov[:, 8]]
    )
    err_lin_acc = np.array(
        [imu_lin_acc_cov[:, 0], imu_lin_acc_cov[:, 4], imu_lin_acc_cov[:, 8]]
    )

    # Load gnss
    gps = np.load(os.path.join(DATA_DIR, "gps.npz"))
    t_gps, gps_status = gps["gps_t"], gps["gps_status"]
    gps_pos, gps_pos_cov = gps["gps_pos"], gps["gps_pos_cov"]
    err_gps_pos = np.array([gps_pos_cov[:, 0], gps_pos_cov[:, 4], gps_pos_cov[:, 8]])

    # load magnetometer flag
    mag = np.load(os.path.join(DATA_DIR, "mag_flag.npz"))
    mag_flag = mag["using_mag"]

    print("Finished reading data")

    print("Heading: ", heading.shape)
    print("GPS time ->", t_gps.shape)
    print("GPS status ->", gps_status.shape)
    print("GPS position ->", gps_pos.shape)
    print("GPS position covariance ->", gps_pos_cov.shape)
    print("IMU time ->", t_imu.shape)
    print("IMU orientation ->", imu_orient.shape)
    print("IMU orientation covariance ->", imu_orient_cov.shape)
    print("IMU angular velocity ->", imu_ang_vel.shape)
    print("IMU angular velocity covariance ->", imu_ang_vel_cov.shape)
    print("IMU linear acceleration ->", imu_lin_acc.shape)
    print("IMU linear acceleration  covariance ->", imu_lin_acc_cov.shape)

    ##### CREATE TUBES
    # fix absolute time to starting in 0 sec
    t_gps[:, 0] = t_gps[:, 0] - t_gps[0, 0]
    t_gps[:, 1] = t_gps[:, 1] / 10**9
    t_gps = t_gps[:, 0] + t_gps[:, 1]

    #    t_imu[:,0] = t_imu[:,0] - t_imu[0,0]
    #    t_imu[:,1] = t_imu[:,1]/10**9
    #    t_imu = t_imu[:,0] + t_imu[:,1]

    # will be using the gps as reference, but very close to imu
    t0, tf, dt = t_gps[0], t_gps[-1], t_gps[1] - t_gps[0]
    print("T0={}, Tf={}, dt={}".format(t0, tf, dt))

    # create tubes x and v
    tdomain = Interval(t0, tf)
    x = TubeVector(tdomain, dt, IntervalVector(12))  # pos, orient, lin vel, ang vel
    v = TubeVector(tdomain, dt, IntervalVector(12))
    u = TubeVector(tdomain, dt, IntervalVector(2))

    ##### ADD DATA TO TUBES
    # add heading
    x.put(0,  Trajectory(dict(zip(t_gps, gps_pos[:,0]))))
    x.put(1,  Trajectory(dict(zip(t_gps, gps_pos[:,1]))))
    x.put(2,  Trajectory(dict(zip(t_gps, gps_pos[:,2]))))
    x.put(3,  Trajectory(dict(zip(t_gps, imu_orient[:,0]))))
    x.put(4,  Trajectory(dict(zip(t_gps, imu_orient[:,1]))))
    x.put(5,  Trajectory(dict(zip(t_gps, imu_orient[:,2]))))
    x.put(6,  Trajectory(dict(zip(t_gps, imu_lin_acc[:,0]))))
    x.put(7,  Trajectory(dict(zip(t_gps, imu_lin_acc[:,1]))))
    x.put(8,  Trajectory(dict(zip(t_gps, imu_lin_acc[:,2]))))
    x.put(9,  Trajectory(dict(zip(t_gps, imu_ang_vel[:,0]))))
    x.put(10, Trajectory(dict(zip(t_gps, imu_ang_vel[:,1]))))
    x.put(11, Trajectory(dict(zip(t_gps, imu_ang_vel[:,2]))))

    #    x[3] &= traj_orient[0]
    #    for idx,dim in enumerate(x[3]):
    #        x[3].inflate(err_orient[0][idx])
    #
    #    x[4] &= Trajectory(dict(zip(t_gps, imu_orient[:,1])))
    #    x[4].inflate(err_orient[1])
    #
    #    x[5] &= Trajectory(dict(zip(t_gps, imu_orient[:,2])))
    #    x[5].inflate(err_orient[2])

    #    start = 2575
    #    for i in np.arange(start, start+25):
    #        print("heading {} | imu_orient {}".format(heading[i], imu_orient[i]))

    fig, axs = plt.subplots(2, 2)
    axs[0, 0].plot(t_gps, heading, "r", label="heading")
    axs[0, 0].plot(t_gps, imu_orient[:, 0], "b", label="roll")

    axs[0, 1].plot(t_gps, heading, "r", label="heading")
    axs[0, 1].plot(t_gps, imu_orient[:, 1], "g", label="pitch")

    axs[1, 0].plot(t_gps, heading, "r", label="heading")
    axs[1, 0].plot(t_gps, imu_orient[:, 2], "y", label="yaw")

    plt.legend()
    plt.show()
    input()
