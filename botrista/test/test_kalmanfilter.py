import numpy as np
from botrista.filtering.kalman_filter import KalmanFilter


def test_identity():

    # Test that the output to this kalman filter is the same as the input
    kalman_filter = KalmanFilter(
        A=np.identity(7),
        R=np.identity(7),
        Q=np.array([
            [1.0, 0, 0, 0, 0, 0, 0],
            [0, 1.0, 0, 0, 0, 0, 0],
            [0, 0, 1.0, 0, 0, 0, 0],
            [0, 0, 0, 1.0, 0, 0, 0],
            [0, 0, 0, 0, 1.0, 0, 0],
            [0, 0, 0, 0, 0, 1.0, 0],
            [0, 0, 0, 0, 0, 0, 1.0]
        ]),
        C=np.identity(7)
    )

    mean = np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]]).T
    sigma = np.identity(7)
    zt = np.array([[1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]]).T

    mean_t, sigma_t = kalman_filter.filter(mean, sigma, zt)
    assert np.allclose(mean_t, mean, 1e-5)
