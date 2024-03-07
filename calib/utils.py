import numpy as np

def inverse_3x4_transform(transform: np.ndarray):
    """
    Calculates the inverse of a 3x4 transformation matrix.

    Parameters:
    transform (np.ndarray): The 3x4 transformation matrix.

    Returns:
    np.ndarray: The inverse of the input transformation matrix.
    """
    inv_transform = np.zeros_like(transform)
    inv_transform[0:3, 0:3] = np.transpose(transform[0:3, 0:3])
    inv_transform[0:3, 3] = np.dot(-np.transpose(transform[0:3, 0:3]), transform[0:3, 3])
    return inv_transform