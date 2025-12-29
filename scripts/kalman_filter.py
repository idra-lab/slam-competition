import numpy as np
import copy

def hat2(theta):
    """2D skew-symmetric matrix from angle."""
    return np.array([[0, -theta],
                     [theta, 0]], dtype=float)

def rot2(theta):
    """2D rotation matrix from angle."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s],
                     [s,  c]], dtype=float)

"""
We assume that the input is the 2D acceleration in body frame and the angular velocity from the IMU. 
The filter state will be the position, velocity, orientation and biases of the IMU."""

class ExtendedKalmanFilter:

    def __init__(self):

        self.state = np.zeros((8, 1))  # State vector: [x, y, vx, vy, theta, b_a_x, b_a_y, b_omega]
        self.imu_measurement = np.zeros((3, 1))  # Measurement vector: [a_x, a_y, omega]
        self.noise_imu = 0.1*np.ones((6, 1))  # Process noise vector: [na_x, na_y, ng, nb_a_x, nb_a_y, nb_omega]
        self.dt = 0.1  # Time step
        self.Q = np.eye(6)  # Process noise covariance

        self.P = np.eye(8)  # State covariance

        self.F = np.eye(8)  # State transition Jacobian
        self.G = np.zeros((8, 6)) # Process noise Jacobian
        self.R = 0.1 #  Pixel measurement noise covariance


        self.intrinsics = np.array([0, 1])  # Placeholder for camera intrinsics

        self.predicted_states_log = []  # Log of predicted states
        self.estimated_states_log = []  # Log of estimated states



    def set_jacobians(self):

        """
        Set the Jacobian matrices for the EKF.
        """

        th = self.state[4, 0] # orientation angle
        acc = self.imu_measurement[0:2] # acceleration in body frame
        b_a = self.state[5:7] # accelerometer bias
        b_omega = self.state[7,0] # gyroscope bias

        R_WB = rot2(th)  # Rotation matrix from body to world frame
        
        self.F = np.eye(8)  # State transition Jacobian

        self.F[0:2, 2:4] = np.eye(2)
        temp  = R_WB @ hat2(1) @ (acc -b_a)
        self.F[2:4, 4] = temp.flatten()  # dv/dtheta
        self.F[2:4, 5:7] = -R_WB
        self.F[4, -1] = -1  # dtheta/db_omega

        self.G = np.zeros((8, 6)) # Process noise Jacobian
        

        self.G[-4:, -4:] = np.eye(4)  # Process noise affects biases and orientation
        self.G[2:4, 0:2] = R_WB  # Process noise affects velocity

        self.previous_timestamp = 0.0
        self.current_timestamp = 0.0

        # print("Jacobian F:\n", self.F)
        # print("Jacobian G:\n", self.G)

    

        

    def predict(self, imu_measurement):

        """
        Predict the next state and estimate error covariance.
        """
        self.imu_measurement = copy.deepcopy(imu_measurement)

        self.set_jacobians()

        # Predict the next state
        v = self.state[2:4]  # velocity
        th = self.state[4, 0] # orientation angle
        a_b = self.imu_measurement[0:2]  # acceleration in body frame
        omega = self.imu_measurement[2, 0]  # angular velocity measurement
        b_a = self.state[5:7]  # accelerometer bias
        b_omega = self.state[7, 0]  # gyroscope bias
        R_WB = rot2(th)  # Rotation matrix from body to world frame

        dp = v
        dv = R_WB @ (a_b - b_a)
        dtheta = omega - b_omega
        db_a = np.zeros((2, 1))  # Assuming bias random walk is zero-mean
        db_omega = 0  # Assuming bias random walk is zero-mean

        self.d_state = np.vstack((dp, dv, dtheta, db_a, db_omega))

        # self.pred_state += d_state * self.dt

        # Evolution of state covariance with Lyapunov equation
        self.P_dot = self.F @ self.P + self.P @ self.F.T + self.G @ self.Q @ self.G.T

        # self.P += self.P_dot * self.dt

    


    

    def update(self, measurements, delta_t):
        """
        Update the state estimate with measurement z.
        x_pred: predicted state vector
        z: measurement vector [x_meas, y_meas, z_meas]
        """

        landmarks = measurements['landmarks']  # Extract landmarks from measurements
        z = measurements['pixels']  # Extract pixels from measurements


        

        self.pred_state = self.d_state * delta_t + self.state
        self.pred_state[4, 0] = (self.pred_state[4, 0] + np.pi) % (2 * np.pi) - np.pi  # wrap theta to [-pi, pi]
        self.predicted_states_log.append(copy.deepcopy(self.pred_state))
        self.P += self.P_dot * delta_t

        if len(z) == 0:
            self.state = self.pred_state
            self.estimated_states_log.append(copy.deepcopy(self.state))
            return copy.deepcopy(self.pred_state)

        z_predictions = np.zeros((len(z), 1))
        H = np.zeros((len(z), len(self.state)))  # Measurement Jacobian matrix

        self.R_matrix = np.eye(len(z)) * self.R  # Measurement noise covariance matrix

        for i, (z_i, landmark_i) in enumerate(zip(z, landmarks)):
            # Compute expected measurement
            z_pred, H_i = self.measurement_model(self.pred_state, landmark_i)
            z_predictions[i] = z_pred
            H[i,:] = H_i



        # Compute innovation covariance
        S = H @ self.P @ H.T + self.R_matrix

        # print("S:", S)

        # Compute Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # print("K:", K)

        # Update state estimate
        z_np = np.array(z).reshape(-1, 1)
        y = z_np - z_predictions  # Measurement residual
        self.state = self.pred_state + K @ y
        self.state[4, 0] = (self.state[4, 0] + np.pi) % (2 * np.pi) - np.pi  # wrap theta to [-pi, pi]

        # Update estimate error covariance
        I = np.eye(len(self.state))
        self.P = (I - K @ H) @ self.P

        self.estimated_states_log.append(copy.deepcopy(self.state))

        return copy.deepcopy(self.state)

    
    def measurement_model(self, pred_state, landmark):
        """
        Measurement model h(x) that maps the state to the measurement space.
        """
        # Placeholder for actual measurement model
        # This should compute the expected pixel coordinates given the state and landmark position

        l_w = landmark.reshape((2, 1)) # Landmark position in world frame
        p_w = pred_state[0:2]  # Position in world frame

        # print("L_w:\n", l_w)
        # print("P_w:\n", p_w)
        th = pred_state[4, 0]  # Orientation angle

        R_WB = rot2(th)  
        R_BW = R_WB.T  # Rotation matrix from world to body frame (transpose of rotation from body to world frame)

        l_b = R_BW @ (l_w - p_w)  # Landmark position in body frame

        # print("L_b:\n", l_b)
        

        # Find homogeneus landmark coordinates
        l_tilde_b = l_b / l_b[0]  # Normalize by x 

        # Project to pixel coordinates using camera intrinsics
        u = self.intrinsics @ l_tilde_b

        # print("Predicted measurement u:\n", u)

        H = self.measurement_jacobian(pred_state, l_w, l_b, R_BW, R_WB)

        return u, H
    
    def measurement_jacobian(self, pred_state, landmark_w, landmark_b, R_BW, R_WB):
        """
        Compute the measurement Jacobian H.
        """
        # Placeholder for actual measurement Jacobian computation

        H = np.zeros((1, 8))  # Adjust size based on measurement dimension

        du_dl_tilde_b = self.intrinsics # Derivative of pixel coordinates w.r.t. landmark homogenous coordinates in body frame
        
        dl_tilde_b_d_l_b = np.array([
            [0, 0], 
            [-landmark_b[1, 0]/(landmark_b[0, 0]**2), 1/landmark_b[0, 0]]
        ])
        # print("dl_tilde_b_d_l_b:\n", dl_tilde_b_d_l_b)

        dl_b_dtheta = -R_BW @ hat2(1) @ (landmark_w - pred_state[0:2]) 
        dl_b_d_p_w = -R_BW

        # print("dl_b_dtheta:\n", dl_b_dtheta)
        # print("dl_b_d_p_w:\n", dl_b_d_p_w)

        dl_b_d_state = np.zeros((2, 8))

        dl_b_d_state[:, 0:2] = dl_b_d_p_w
        dl_b_d_state[:, 4] = dl_b_dtheta.reshape((2,))

        du_d_state = du_dl_tilde_b @ dl_tilde_b_d_l_b @ dl_b_d_state

        # print(du_d_state)


        return du_d_state
