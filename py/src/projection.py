import numpy as np

angle_x = 70.0
angle_y = 60.0

angle_x *= 0.5 * np.pi / 180.0
angle_y *= 0.5 * np.pi / 180.0


def proj(pos, R, pos_img, dist_max=10):
    t = np.array(
        [[1], [-pos_img[0] * np.tan(angle_x)], [-pos_img[1] * np.tan(angle_y)]]
    )

    t = t / np.linalg.norm(t)
    t = R @ t

    if t[2][0] < 0:
        if -pos[2][0] / t[2][0] < dist_max:
            c = -pos[2][0] / t[2][0] * t + pos
            return np.array([c[0][0], c[1][0]])

    t[2, 0] = 0
    if np.linalg.norm(t) == 0:
        return np.array([pos[0][0], pos[1][0]])

    t = t / np.linalg.norm(t)
    c = dist_max * t + pos

    return np.array([c[0][0], c[1][0]])
