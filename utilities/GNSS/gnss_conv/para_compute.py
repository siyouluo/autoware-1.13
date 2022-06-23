import numpy as np
from rotation import *
# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector


def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]  # total points

    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    # centre the points
    AA = A - np.tile(centroid_A, (N, 1))
    BB = B - np.tile(centroid_B, (N, 1))

    H = np.matmul(np.transpose(AA),BB)

    U, S, Vt = np.linalg.svd(H)

    R = np.matmul(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        print("Reflection detected")
        Vt[2, :] *= -1
        R = np.matmul(Vt.T,U.T)

    t = -np.matmul(R, centroid_A) + centroid_B
    # err = B - np.matmul(A,R.T) - t.reshape([1, 3])
    return R, t


if __name__=='__main__':
    b = np.array([
        [584.930506718,1146.02136851,1254.358],
        [564.876415058,1158.53734232,1254.497],
        [585.120269678,1142.46884409,1254.47],
        [584.532873324,1152.08698355,1254.356],
        [583.966672515,1132.09527077,1254.565],
        [580.659197922,1114.21112193,1254.365],
        [583.916565201,1124.55375232,1254.492],
        [581.799987431,1156.98709209,1254.387],
        [576.732152303,1158.47273283,1254.393]
                  ])

    a = np.array([
        [-51.6991348267,12.9493875504,-0.021725833416],
        [-31.9058799744,0.0903624594212,0.194968044758],
        [-51.8822593689,16.61876297,-0.137042403221],
        [-51.1587867737,6.81911325455,0.182650506496],
        [-50.8071937561,26.71251297,-0.389379382133],
        [-47.4840316772,44.3386878967, -0.733607411385],
        [-51.0739746094,34.022403717,-0.505491673946],
        [-48.73595047,1.74229097366,0.290952682495],
        [-43.8022155762,0.0402972102165,0.187373787165]
    ])

    c = np.reshape(a[-2:], (2, 3))
    test_a1 = np.reshape(c[0], (1, 3))
    test_a2 = np.reshape(c[1], (1, 3))

    c = np.reshape(b[-2:], (2, 3))
    test_b1 = np.reshape(c[0], (1, 3))
    test_b2 = np.reshape(c[1], (1, 3))

    a = a[:-2]
    b = b[:-2]
    r, t = rigid_transform_3D(a, b)
    print('r:', r)
    print('t:', t)

    bb = np.matmul(a, r.T) + t.reshape([1, 3])
    print('b-bb:', b - bb)

    c = np.matmul(test_a1, r.T) + t.reshape([1, 3])
    print('c-test_b1:', c - test_b1)

    c = np.matmul(test_a2, r.T) + t.reshape([1, 3])
    print('c-test_b2:', c - test_b2)

    file_name = 'autoware-190927-velodyne_1.pcd'
    new_file_name = '200918_2.pcd'

    print('loadig points map')
    POINTS = np.loadtxt(file_name)

    INTEN = POINTS[:, 3].reshape((-1, 1))
    NEW_POINTS = np.matmul(POINTS[:, :3].reshape((-1, 3)), r.T) + t.reshape([1, 3])
    NEW_POINTS = np.hstack((NEW_POINTS, INTEN))

    out_file(NEW_POINTS, new_file_name)
