#!/usr/bin/env python3

import os
import numpy as np
from vibes import *
from codac import *

DATA_DIR = "../data/extracted/bag_2021-10-07-11-09-50"

if __name__ == "__main__":
##### LOAD DATA
    print("Reading data")
    # loading data (heading)
    hd = np.load(os.path.join(DATA_DIR, "heading.npz"))
    heading = hd['heading']
#    accuracy_heading = 0.1

    # loading  (imu)
    imu = np.load(os.path.join(DATA_DIR, "imu.npz"))
    t_imu = imu['imu_t']
    imu_orient, imu_orient_cov = imu['imu_orient'], imu['imu_orient_cov']
    imu_ang_vel, imu_ang_vel_cov = imu['imu_ang_vel'], imu['imu_ang_vel_cov']
    imu_lin_acc, imu_lin_acc_cov = imu['imu_lin_acc'], imu['imu_lin_acc_cov']

    # Loading  (gnss)
    gps = np.load(os.path.join(DATA_DIR, "gps.npz"))
    t_gps, gps_status = gps['gps_t'], gps['gps_status']
    gps_pos, gps_pos_cov = gps['gps_pos'], gps['gps_pos_cov']

    # loading  (mag flag)
    mag = np.load(os.path.join(DATA_DIR, "mag_flag.npz"))
    mag_flag = mag['using_mag']

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
    t_gps[:,0] = t_gps[:,0] - t_gps[0,0]
    t_gps[:,1] = t_gps[:,1]/10**9
    t_gps = t_gps[:,0] + t_gps[:,1]

    t_imu[:,0] = t_imu[:,0] - t_imu[0,0]
    t_imu[:,1] = t_imu[:,1]/10**9
    t_imu = t_imu[:,0] + t_imu[:,1]

    # will be using the gps as reference, but very close to imu
    t0, tf, dt = t_gps[0], t_gps[-1], t_gps[1]-t_gps[0]
    print(t0, tf, dt)

    # create tubes x and v
    tdomain = Interval(t0, tf)
    x = TubeVector(tdomain, dt, IntervalVector(4))
    v = TubeVector(tdomain, dt, IntervalVector(4))
    u = TubeVector(tdomain, dt, IntervalVector(2))
