import numpy as np


class KalmanFilter:
    """Implements the base linear kalman filter.

    Kalman filter equations taken from Probabilistic Robotics, Thrun et al,
    Chapter 3, Section 2.
    """

    def __init__(self,
                 A,
                 R,
                 Q,
                 C):
        """
        Initialize the Kalman Filter.

        Arguments:
            + A (np.ndarray): The state transition matrix.
            + R (np.ndarray): The state covariance matrix.
            + C (np.ndarray): The measurement prediction matrix.
            + Q (np.ndarray): The measurement covariance matrix.

        """
        self.A = A
        self.R = R
        self.Q = Q
        self.C = C

    def filter(self, mean, sigma, zt):
        """
        Filter the mean.

        Arguments:
            + mean (np.ndarray): The previous mean.
            + sigma (np.ndarray): The previous sigma.
            + zt (np.ndarray): The new measurement.

        Returns
        -------
            A tuple of the (mean_t, sigma_t).

        """
        mean_prediction = self.A@mean
        sigma_t = self.A @ sigma @ self.A.T + self.R
        K = sigma_t @ self.C.T @ np.linalg.inv(self.C @
                                               sigma_t @ self.C.T + self.Q)
        u_t = mean_prediction + K @ (zt - self.C @ mean_prediction)
        sigma_t = (np.identity(np.shape(mean)[0]) - K @ self.C) @ sigma_t
        return u_t, sigma_t
