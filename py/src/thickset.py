#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
from vibes import *
from codac import *
from pyibex.thickset import *
import utm


# guerledan referential points
lonlat_refs = [
    [48.200100, -3.016789],
    [48.200193, -3.015264],
    [48.198639, -3.015064],
    [48.198763, -3.016315]
]

DATA_DIR = "../data/extracted/bag_2021-10-07-11-09-50"

init_set = False
x_init, y_init = 0,0

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


def latlon_to_xy(latlon):
    global init_set, x_init, y_init
    lat, lon = latlon

    x, y, _, _ = utm.from_latlon(lat, lon)

    if not init_set:
        init_set = True
        x_init, y_init = x, y
        return 0, 0

    return x, y

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
    ).T
    err_ang_vel = np.array(
        [imu_ang_vel_cov[:, 0], imu_ang_vel_cov[:, 4], imu_ang_vel_cov[:, 8]]
    ).T
    err_lin_acc = np.array(
        [imu_lin_acc_cov[:, 0], imu_lin_acc_cov[:, 4], imu_lin_acc_cov[:, 8]]
    ).T

    # Load gnss
    gps = np.load(os.path.join(DATA_DIR, "gps.npz"))
    t_gps, gps_status = gps["gps_t"], gps["gps_status"]
    gps_pos, gps_pos_cov = gps["gps_pos"], gps["gps_pos_cov"]
    gps_pos = np.concatenate((np.apply_along_axis(latlon_to_xy, 1, gps_pos[:,:2]), gps_pos[:,2]), axis=0)
    err_gps_pos = np.array([gps_pos_cov[:, 0], gps_pos_cov[:, 4], gps_pos_cov[:, 8]]).T

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

    print("Error GPS position ->", err_gps_pos.shape)
    print("Error orientation ->", err_orient.shape)
    print("Error angular velocity ->", err_ang_vel.shape)
    print("Error linear acceleration ->", err_lin_acc.shape)

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

    # TODO: find why of the difference
    fig, axs = plt.subplots(2, 2)
    axs[0, 0].plot(t_gps, heading, "r", label="heading")
    axs[0, 0].plot(t_gps, imu_orient[:, 0], "b", label="roll")

    axs[0, 1].plot(t_gps, heading, "r", label="heading")
    axs[0, 1].plot(t_gps, imu_orient[:, 1], "g", label="pitch")

    axs[1, 0].plot(t_gps, heading, "r", label="heading")
    axs[1, 0].plot(t_gps, imu_orient[:, 2], "y", label="yaw")

    plt.legend()
    plt.show()

    # create tubes x and v
    tdomain = Interval(t0, tf)
    x = TubeVector(tdomain, dt, IntervalVector(6))  # position, orientation
    v = TubeVector(tdomain, dt, IntervalVector(6))  # linear acceleration, angular velocity
    u = TubeVector(tdomain, dt, IntervalVector(2))  # inputs given to the boat
    m = TubeVector(tdomain, dt, IntervalVector(2))  # explored zone

    ##### ADD DATA TO TUBES
    # create trajectories with correct tdomain
    traj_gps = TrajectoryVector(
        [
            dict(zip(t_gps, gps_pos[:, 0])),
            dict(zip(t_gps, gps_pos[:, 1])),
            dict(zip(t_gps, gps_pos[:, 2])),
        ]
    )
    traj_gps.truncate_tdomain(tdomain)

    traj_orient = TrajectoryVector(
        [
            dict(zip(t_gps, imu_orient[:, 0])),
            dict(zip(t_gps, imu_orient[:, 1])),
            dict(zip(t_gps, imu_orient[:, 2])),
        ]
    )
    traj_orient.truncate_tdomain(tdomain)

    traj_lin_acc = TrajectoryVector(
        [
            dict(zip(t_gps, imu_lin_acc[:, 0])),
            dict(zip(t_gps, imu_lin_acc[:, 1])),
            dict(zip(t_gps, imu_lin_acc[:, 2])),
        ]
    )
    traj_lin_acc.truncate_tdomain(tdomain)

    traj_ang_vel = TrajectoryVector(
        [
            dict(zip(t_gps, imu_ang_vel[:, 0])),
            dict(zip(t_gps, imu_ang_vel[:, 1])),
            dict(zip(t_gps, imu_ang_vel[:, 2])),
        ]
    )
    traj_ang_vel.truncate_tdomain(tdomain)

    # update tubes with known data
    x[0] &= traj_gps[0]
    x[1] &= traj_gps[1]
    x[2] &= traj_gps[2]
    x[3] &= traj_orient[0]
    x[4] &= traj_orient[1]
    x[5] &= traj_orient[2]

    v[0] &= traj_lin_acc[0]
    v[1] &= traj_lin_acc[1]
    v[2] &= traj_lin_acc[2]
    v[3] &= traj_ang_vel[0]
    v[4] &= traj_ang_vel[1]
    v[5] &= traj_ang_vel[2]


    beginDrawing()
    fig_map = VIBesFigMap("Motorboat")
    fig_map.set_properties(100, 100, 600, 300)
#    fig_map.smooth_tube_drawing(True)
    fig_map.add_tube(x, "x", 0, 1)
    fig_map.axis_limits(-2.5,2.5,-0.1,0.1, True)
    fig_map.show()
    endDrawing()

    input("pause after vibes")

    # inflate with uncertainties
    for idx, t in enumerate(t_gps):
        if idx % 100 == 0:
            print(idx, "/", len(t_gps))
        x[0].slice(t).inflate(err_gps_pos[idx][0])
        x[1].slice(t).inflate(err_gps_pos[idx][1])
        x[2].slice(t).inflate(err_gps_pos[idx][2])

        x[3].slice(t).inflate(err_orient[idx][0])
        x[4].slice(t).inflate(err_orient[idx][1])
        x[5].slice(t).inflate(err_orient[idx][2])

        v[0].slice(t).inflate(err_lin_acc[idx][0])
        v[1].slice(t).inflate(err_lin_acc[idx][1])
        v[2].slice(t).inflate(err_lin_acc[idx][2])

        v[3].slice(t).inflate(err_ang_vel[idx][0])
        v[4].slice(t).inflate(err_ang_vel[idx][1])
        v[5].slice(t).inflate(err_ang_vel[idx][2])


    ##### ADD CONTRACTOR NETWORK CONSTRAINTS
#    cn = ContractorNetwork()
#
#    # derivative constraint between x and v
#    ctc_deriv = CtcDeriv()
#    cn.add(ctc_deriv, [x, v])
#
#    # position constrains from gnss
#    ctc_eval = CtcEval()  # yi = x(ti); xdot(.) = v(.)
#    ctc_eval.enable_time_propag(False)
#
#    for i in range(len(t_gps)):
#        # p = x[t_gps_i]
#        # xdot(.) = v(.)
#        p = IntervalVector([Interval(gps_pos[i, 0]), Interval(gps_pos[i, 1])])
#        p.inflate(err_gps_pos)
#
#        cn.add(ctc_eval, [Interval(t_gps[i]), p[0], x[0], v[0]])
#        cn.add(ctc_eval, [Interval(t_gps[i]), p[1], x[1], v[1]])
#
#    cn.contract(verbose=True)

##### COMPUTE EXPLORED ZONE OF THE MAP
    # thick function
    f_dist = Function("x1", "x2", "p1", "p2", "r", "(x1-p1)^2+(x2-p2)^2-r^2")

    p = IntervalVector(3, Interval())
    p[0] = m[0](0)
    p[1] = m[1](0)
    p[2] = Interval(10)

    S_dist = ThickSep_from_function(f_dist, p, y)

    n = int((tf-t0)/dt) + 1

    for i in range(1, n):
        p = IntervalVector(3, Interval())
        p[0] = m[0](i)
        p[1] = m[1](i)
        p[2] = Interval(10)
        S_dist = S_dist | ThickSep_from_function(f_dist, p, y)

    paving = ThickPaving(self.m_map, ThickTest_from_ThickSep(S_dist), 0.1)

    print(paving)

    beginDrawing()
    fig_map = VIBesFigMap("Saturne")
    fig_map.set_properties(100, 100, 600, 300)
    fig_map.smooth_tube_drawing(True)
    fig_map.add_tube(x, "x*", 0, 1)
    fig_map.axis_limits(-2.5,2.5,-0.1,0.1, True)
    fig_map.show()

#    vibes.beginDrawing()
#    P = m.process_coverage()
#    vibes.setFigureSize(m.m_map.max_diam(), m.m_map.max_diam())
#    vibes.newFigure("Mapping Coverage")
#    vibes.setFigureProperties({"x": 700, "y": 430, "width": 600, "height": 600})
#    P.visit(ToVibes(figureName="Mapping Coverage", color_map=My_CMap))
#    vibes.endDrawing()

    endDrawing()
