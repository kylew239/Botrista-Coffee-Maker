import numpy as np


class KalmanFilter:
    """Implements the base linear Kalman filter.

    Kalman filter equations taken from Probabilistic Robotics, Thrun et al,
    Chapter 3, Section 2.
    """

    def __init__(self, A, R, Q, C):
        """
        Initializes the Kalman Filter.

        Args:
            A (np.ndarray): The state transition matrix.
            R (np.ndarray): The state covariance matrix.
            Q (np.ndarray): The measurement covariance matrix.
            C (np.ndarray): The measurement prediction matrix.
        """
        self.A = A
        self.R = R
        self.Q = Q
        self.C = C

    def filter(self, mean, sigma, zt):
        """
        Filter the mean.

        Args:
            mean (np.ndarray): The previous mean.
            sigma (np.ndarray): The previous sigma.
            zt (np.ndarray): The new measurement.

        Returns:
            tuple: A tuple of (mean_t, sigma_t).

        """
        mean_prediction = self.A @ mean
        sigma_t = self.A @ sigma @ self.A.T + self.R
        K = sigma_t @ self.C.T @ np.linalg.inv(self.C @
                                               sigma_t @ self.C.T + self.Q)
        u_t = mean_prediction + K @ (zt - self.C @ mean_prediction)
        sigma_t = (np.identity(np.shape(mean)[0]) - K @ self.C) @ sigma_t
        return u_t, sigma_t
