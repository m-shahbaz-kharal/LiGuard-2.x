import numpy as np

def inverse_3x4_transform(transform):
    inv_transform = np.zeros_like(transform)
    inv_transform[0:3, 0:3] = np.transpose(transform[0:3, 0:3])
    inv_transform[0:3, 3] = np.dot(-np.transpose(transform[0:3, 0:3]), transform[0:3, 3])
    return inv_transform

def nx3_to_nx4(nx3):
    n = nx3.shape[0]
    nx4 = np.hstack((nx3, np.ones((n, 1))))
    return nx4