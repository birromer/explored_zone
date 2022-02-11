#!/usr/bin/env python3

import os
import numpy as np
import matplotlib.pyplot as plt
from vibes import *
from pyibex import *
from pyibex.thickset import *
from codac import *
import utm
from pyproj import Proj

class ThickSep:
    def __init__(self, Ssub, Ssup): self.Ssub, self.Ssup = Ssub, Ssup
    def separate(self, X):
       xin_sub, xout_sub = X.copy(), X.copy()
       xin_sup, xout_sup = X.copy(), X.copy()
       self.Ssub.separate(xin_sub, xout_sub)
       self.Ssup.separate(xin_sup, xout_sup)
       return (xin_sup, xin_sub | xout_sup, xout_sub)
    def __and__(self, thickSep_y): return ThickSep(self.Ssub & thickSep_y.Ssub, self.Ssup & thickSep_y.Ssup)
    def __invert__(self): return ThickSep(~self.Ssup, ~self.Ssub)
    def __or__(self, thickSep_y): return ThickSep(self.Ssub | thickSep_y.Ssub, self.Ssup | thickSep_y.Ssup)

def ThickSep_from_function(f, p, y):
    S = SepFwdBwd(f, y)
    Ssub = ~SepProj(~S, p)
    Ssup = SepProj(S, p)
    return ThickSep(Ssup, Ssub)

def ThickTest_from_ThickSep(S):
    def test(X):
       Xin, Xu, Xout = S.separate(X)
       if Xin.is_empty(): return IN
       elif Xout.is_empty(): return OUT
       elif Xu.is_empty(): return MAYBE
       return UNK
    return test


# referential points from guerledan
#lonlat_refs = [
#    [48.200100, -3.016789],
#    [48.200193, -3.015264],
#    [48.198639, -3.015064],
#    [48.198763, -3.016315],
#]

pp = Proj(proj="utm", zone=30, ellps="WGS84", preserve_units=False)

#DATA_DIR = "../data/extracted/bag_2021-10-07-11-09-50"
DATA_DIR = "../data/extracted/bag_2022-02-09-13-22-09"

init_set = False
x_init, y_init = 0, 0


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


def acc_to_global_frame(lin_acc, angles):
    la_x, la_y, la_z = lin_acc
    yaw, pitch, roll = angles

    R = np.array([
        [cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll)+sin(yaw)*sin(roll)],
        [sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll)],
        [        -sin(pitch),                             cos(pitch)*sin(roll),                             cos(pitch)*cos(roll)]
    ])

    rob_lin_acc_x = R[0,0]*la_x + R[0,1]*la_y + R[0,2]*la_z
    rob_lin_acc_y = R[1,0]*la_x + R[1,1]*la_y + R[1,2]*la_z
    rob_lin_acc_z = R[2,0]*la_x + R[2,1]*la_y + R[2,2]*la_z

    return TubeVector([rob_lin_acc_x, rob_lin_acc_y, rob_lin_acc_z])


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
        [3*np.sqrt(imu_orient_cov[:, 0]), 3*np.sqrt(imu_orient_cov[:, 4]), 3*np.sqrt(imu_orient_cov[:, 8])]
    ).T
    err_ang_vel = np.array(
        [3*np.sqrt(imu_ang_vel_cov[:, 0]), 3*np.sqrt(imu_ang_vel_cov[:, 4]), 3*np.sqrt(imu_ang_vel_cov[:, 8])]
    ).T
    err_lin_acc = np.array(
        [3*np.sqrt(imu_lin_acc_cov[:, 0]), 3*np.sqrt(imu_lin_acc_cov[:, 4]), 3*np.sqrt(imu_lin_acc_cov[:, 8])]
    ).T

    # Load gnss
    gps = np.load(os.path.join(DATA_DIR, "gps.npz"))

    t_gps, gps_status = gps["gps_t"], gps["gps_status"]

    gps_pos, gps_pos_cov = gps["gps_pos"], gps["gps_pos_cov"]
    xx, yy = pp(gps_pos[:, 0], gps_pos[:, 1])
    gps_pos[:, 0] = (xx - xx.min()) / (xx.max() - xx.min())
    gps_pos[:, 1] = (yy - yy.min()) / (yy.max() - yy.min())

    err_gps_pos = np.array([3*np.sqrt(gps_pos_cov[:, 0]), 3*np.sqrt(gps_pos_cov[:, 4]), 3*np.sqrt(gps_pos_cov[:, 8])]).T

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
#    t0, tf, dt = t_gps[0], t_gps[20000], t_gps[1] - t_gps[0]
    print("T0={}, Tf={}, dt={}".format(t0, tf, dt))

    # TODO: find why of the difference
    fig, axs = plt.subplots(2, 2)
    axs[0, 0].plot(t_gps, heading, "r", label="heading")
    axs[0, 0].plot(t_gps, imu_orient[:, 0], "b", label="roll")
    axs[0, 0].legend()

    axs[0, 1].plot(t_gps, heading, "r", label="heading")
    axs[0, 1].plot(t_gps, imu_orient[:, 1], "g", label="pitch")
    axs[0, 1].legend()

    axs[1, 0].plot(t_gps, heading, "r", label="heading")
    axs[1, 0].plot(t_gps, imu_orient[:, 2], "y", label="yaw")
    axs[1, 0].legend()
    plt.suptitle("comparisson heading and imu data")
    plt.show()

    fig, axs = plt.subplots(2, 2)
    axs[0, 0].plot(t_gps, err_gps_pos, label=["x", "y", "z"])
    axs[0, 0].set_title("error gps")
    axs[0, 0].legend()

    axs[0, 1].plot(t_gps, err_orient, label=["roll", "pitch", "yaw"])
    axs[0, 1].set_title("error orientation")
    axs[0, 1].legend()

    axs[1, 0].plot(t_gps, err_ang_vel, label=["roll", "pitch", "yaw"])
    axs[1, 0].set_title("error angular velocity")
    axs[1, 0].legend()

    axs[1, 1].plot(t_gps, err_lin_acc, label=["x", "y", "z"])
    axs[1, 1].set_title("error linear acceleration")
    axs[1, 1].legend()
    plt.show()

    # create tubes x and v
    tdomain = Interval(t0, tf)
    x = TubeVector(tdomain, dt, IntervalVector(6))  # position, orientation
    v = TubeVector(tdomain, dt, IntervalVector(6))  # linear acceleration, angular velocity
    u = TubeVector(tdomain, dt, IntervalVector(2))  # inputs given to the boat

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
    fig_map = VIBesFigMap("truth traj")
    fig_map.set_properties(100, 100, 600, 300)
    fig_map.smooth_tube_drawing(True)
    fig_map.add_tube(x, "x", 0, 1)
    fig_map.axis_limits(-2.5, 2.5, -0.1, 0.1, True)
    fig_map.show()
    endDrawing()

    plt.figure()

    # inflate with uncertainties
    err_imu = 0.005
#    for idx, t in enumerate(t_gps):
#        if idx % 100 == 0:
#            print(idx, "/", len(t_gps))
#        x[0].slice(t).inflate(err_gps_pos[idx][0]/100)
#        x[1].slice(t).inflate(err_gps_pos[idx][1]/100)
#        x[2].slice(t).inflate(err_gps_pos[idx][2]/100)

    x[0].inflate(0.01)
    x[1].inflate(0.01)
    x[2].inflate(0.01)

    x[3].inflate(err_imu)
    x[4].inflate(err_imu)
    x[5].inflate(err_imu)
    v[0].inflate(err_imu)
    v[1].inflate(err_imu)
    v[2].inflate(err_imu)
    v[3].inflate(err_imu)
    v[4].inflate(err_imu)
    v[5].inflate(err_imu)

    beginDrawing()
    fig_map = VIBesFigMap("Motorboat")
    fig_map.set_properties(100, 100, 600, 300)
    fig_map.smooth_tube_drawing(True)
    fig_map.add_tube(x, "x", 0, 1)
    fig_map.axis_limits(-2.5, 2.5, -0.1, 0.1, True)
    fig_map.show()
    endDrawing()

    input("pause after inflation")

    ##### ADD CONTRACTOR NETWORK CONSTRAINTS
    cn = ContractorNetwork()

    # derivative constraint between x and v
    ctc_deriv = CtcDeriv()

    T = TubeVector(tdomain, dt, IntervalVector(3))  # first derivative x_1:3
    T.set(IntervalVector(3, Interval(0)), tdomain.lb())

    C = TubeVector(x[:3])  # gps position x_1:3
#    C = x[:3]

    global_acc = acc_to_global_frame(v[:3], x[3:])

    ctc_deriv.contract(T, global_acc)  # acc -> /integ/ -> vel
    ctc_deriv.contract(C, T)           # vel -> /integ/ -> pos

    beginDrawing()
    fig_map = VIBesFigMap("Contraction X")
    fig_map.set_properties(100, 100, 600, 300)
    fig_map.smooth_tube_drawing(True)
    fig_map.add_tube(C, "C", 0, 1)
    fig_map.axis_limits(-2.5, 2.5, -0.1, 0.1, True)
    fig_map.show()
    endDrawing()

    input("pause after contraction")

    ##### COMPUTE EXPLORED ZONE OF THE MAP
    # thick function
    y = Interval(-oo, 0)
    f_dist = Function("x1", "x2", "p1", "p2", "r", "(x1-p1)^2+(x2-p2)^2-r^2")

    m = IntervalVector(2, Interval(-50,50))  # explored zone

    p = IntervalVector(3, Interval())
    p[0] = C[0](0)
    p[1] = C[1](0)
    p[2] = Interval(1)  # measurement range

    S_dist = ThickSep_from_function(f_dist, p, y)

    n = int((tf - t0) / dt) + 1

    for i in range(1, n):
        print(i, "/", n)
        p = IntervalVector(3, Interval())
        p[0] = C[0](i)
        p[1] = C[1](i)
        p[2] = Interval(1)
        S_dist = S_dist | ThickSep_from_function(f_dist, p, y)

    paving = ThickPaving(m, ThickTest_from_ThickSep(S_dist), 1.0, display=True)

    print(paving)

#    vibes.beginDrawing()
#    vibes.setFigureSize(m.max_diam(), m.max_diam())
#    vibes.newFigure("Mapping Coverage")
#    vibes.setFigureProperties({"x": 700, "y": 430, "width": 600, "height": 600})
#    paving.visit(ToVibes(figureName="Mapping Coverage", color_map=My_CMap))
#    vibes.endDrawing()
#    endDrawing()
