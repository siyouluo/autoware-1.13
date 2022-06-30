import sys
import rosbag
import rospy
import numpy as np
from para_compute import *


def compute_matrix(bag_name):
    topics = ['/gnss_pose', '/ndt_pose']

    bag = rosbag.Bag(bag_name)

    gnss_pose = []
    ndt_pose = []

    for topic, msg, t in bag.read_messages(topics=topics):
        stamp = msg.header.stamp
        t1 = stamp.secs * 1000000000 + stamp.nsecs
        data = {}
        if topic == str('/gnss_pose'):
            data['nsec'] = t1
            data['x'] = msg.pose.position.x
            data['y'] = msg.pose.position.y
            data['z'] = msg.pose.position.z
            gnss_pose.append(data)

        elif topic == str('/ndt_pose'):
            data['nsec'] = t1
            data['x'] = msg.pose.position.x
            data['y'] = msg.pose.position.y
            data['z'] = msg.pose.position.z
            ndt_pose.append(data)

    gnss_matrix = []
    ndt_matrix = []
    sample_num = 70

    for i in range(sample_num):
        n1 = (i + 1) * 5
        t1 = gnss_pose[n1]['nsec']
        index = 0
        for j in range(index, len(ndt_pose)):
            if -30000000 < t1 - ndt_pose[j]['nsec'] < 30000000:
                gnss_c = gnss_pose[n1]
                ndt_c = ndt_pose[j]
                gnss_matrix.append([gnss_c['x'], gnss_c['y'], gnss_c['z']])
                ndt_matrix.append([ndt_c['x'], ndt_c['y'], ndt_c['z']])
                index = j
                break
    #np.random.seed(111)

    gnss_matrix = np.array(gnss_matrix).reshape((-1, 3))
    ndt_matrix = np.array(ndt_matrix).reshape((-1, 3))
    data = np.hstack((gnss_matrix, ndt_matrix))
    np.random.shuffle(data)
    gnss_matrix = data[:, :3]
    ndt_matrix = data[:, 3:]

    c = np.reshape(ndt_matrix[-10:], (10, 3))
    test_a = c

    c = np.reshape(gnss_matrix[-10:], (10, 3))
    test_b = c

    a = ndt_matrix[:-10]
    b = gnss_matrix[:-10]
    r, t = rigid_transform_3D(b, a)
    print('r matrix:')
    print ""
    print r
    print('t matrix:')
    print ""
    print t
    print ""
    print ""
    aa = np.matmul(b, r.T) + t.reshape([1, 3])
    print('a-aa mean error:')
    print ""
    print np.mean(np.abs(a - aa), 0)

    c = np.matmul(test_b, r.T) + t.reshape([1, 3])
    print('c-test_b:\n', np.mean(np.abs(c - test_a), 0))


if __name__ == '__main__':
    # bn = 'autoware-20200918105939.bag'
    if(len(sys.argv)!=2):
        print("Usage: python geo_extract_bag.py [*].bag")
        exit()
    bn = sys.argv[1]
    print("extract bag: " + bn)
    compute_matrix(bn)
