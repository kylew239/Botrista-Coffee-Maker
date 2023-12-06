import numpy as np


class KalmanFilter:
    """Implements the base linear Kalman filter.

    Kalman filter equations taken from Probabilistic Robotics, Thrun et al,
    Chapter 3, Section 2.

    :param A: The state transition matrix.
    :type A: np.ndarray
    :param R: The state covariance matrix.
    :type R: np.ndarray
    :param C: The measurement prediction matrix.
    :type C: np.ndarray
    :param Q: The measurement covariance matrix.
    :type Q: np.ndarray
    """

    def __init__(self, A, R, Q, C):
        """
        Initialize the Kalman Filter.
        """
        self.A = A
        self.R = R
        self.Q = Q
        self.C = C

    def filter(self, mean, sigma, zt):
        """
        Filter the mean.

        :param mean: The previous mean.
        :type mean: np.ndarray
        :param sigma: The previous sigma.
        :type sigma: np.ndarray
        :param zt: The new measurement.
        :type zt: np.ndarray
        :return: A tuple of (mean_t, sigma_t).
        :rtype: tuple
        """
        mean_prediction = self.A @ mean
        sigma_t = self.A @ sigma @ self.A.T + self.R
        K = sigma_t @ self.C.T @ np.linalg.inv(self.C @
                                               sigma_t @ self.C.T + self.Q)
        u_t = mean_prediction + K @ (zt - self.C @ mean_prediction)
        sigma_t = (np.identity(np.shape(mean)[0]) - K @ self.C) @ sigma_t
        return u_t, sigma_t
