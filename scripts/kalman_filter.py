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

        self.F = np.zeros((8, 8))  # State transition Jacobian

        self.F[0:2, 2:4] = np.eye(2)
        temp  = R_WB @ hat2(1) @ (acc -b_a)
        self.F[2:4, 4] = temp.flatten()  # dv/dtheta
        self.F[2:4, 5:7] = -R_WB
        self.F[4, -1] = -1  # dtheta/db_omega

        self.G = np.zeros((8, 6)) # Process noise Jacobian
        

        self.G[-4:, -4:] = np.eye(4)  # Process noise affects biases and orientation
        self.G[2:4, 0:2] = R_WB  # Process noise affects velocity

    

        

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

        dp = v # position derivative
        dv = R_WB @ (a_b - b_a) # velocity derivative
        dtheta = omega - b_omega # orientation derivative
        db_a = np.zeros((2, 1))  # Assuming bias random walk is zero-mean
        db_omega = 0  # Assuming bias random walk is zero-mean

        #  Derivative of the state is just the concatenation of the derivatives we just computed in one vector 
        self.d_state = np.vstack((dp, dv, dtheta, db_a, db_omega)) # vstack concatenates arrays vertically


        # Evolution of state covariance with Lyapunov equation (See theory here)
        self.P_dot = self.F @ self.P + self.P @ self.F.T + self.G @ self.Q @ self.G.T

    


    
    def update(self, measurements, delta_t):
        """
        Update the state estimate with measurements.
        """

        landmarks = measurements['landmarks']  # Extract landmarks positions from measurements
        z = measurements['pixels']  # Extract pixels from measurements


        # Here we use the delta_t from when we called predict() to step the state forward
        self.pred_state = self.d_state * delta_t + self.state # Euler integration to get predicted state
        self.pred_state[4, 0] = (self.pred_state[4, 0] + np.pi) % (2 * np.pi) - np.pi  # wrap theta to [-pi, pi]
        self.predicted_states_log.append(copy.deepcopy(self.pred_state)) # Log predicted state

        self.P += self.P_dot * delta_t # Euler integration to get predicted covariance

        # Check if there are no measurements
        if len(z) == 0:

            # If we see no landmarks, we can only rely on the prediction
            self.state = self.pred_state
            self.estimated_states_log.append(copy.deepcopy(self.state))
            return copy.deepcopy(self.pred_state)

        # Prepare measurement prediction and Jacobian
        z_predictions = np.zeros((len(z), 1))
        H = np.zeros((len(z), len(self.state)))  # Measurement Jacobian matrix

        self.R_matrix = np.eye(len(z)) * self.R  # Measurement noise covariance matrix

        # Compute measurement predictions and Jacobians for each landmark
        for i, (z_i, landmark_i) in enumerate(zip(z, landmarks)):

            # Compute expected measurement
            z_pred, H_i = self.measurement_model(self.pred_state, landmark_i) # Considering our predicted state, we compute how we expect the measurement to be
            z_predictions[i] = z_pred # Store predicted measurement
            H[i,:] = H_i # Store measurement Jacobian

        # Compute innovation covariance
        S = H @ self.P @ H.T + self.R_matrix

        # print("S:", S)

        # Compute Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # print("K:", K)

        # Update state estimate
        z_np = np.array(z).reshape(-1, 1) # Convert measurements to numpy array
        y = z_np - z_predictions  # Measurement residual: difference between actual and predicted measurements
        self.state = self.pred_state + K @ y # Update state with Kalman gain and residual
        self.state[4, 0] = (self.state[4, 0] + np.pi) % (2 * np.pi) - np.pi  # wrap theta to [-pi, pi]

        # Update estimate error covariance
        I = np.eye(len(self.state))
        self.P = (I - K @ H) @ self.P # Update covariance that we will use in the next iteration


        self.estimated_states_log.append(copy.deepcopy(self.state)) # Log estimated state

        return copy.deepcopy(self.state)

    
    def measurement_model(self, pred_state, landmark):
        """
        Measurement model h(x) that maps the state to the measurement space.
        This is exactly the same projection model we use in the Camera class in the unicycle_estimation.py script.
        """
        # This computes the expected pixel coordinates given the state and landmark position

        l_w = landmark.reshape((2, 1)) # Landmark position in world frame
        p_w = pred_state[0:2]  # Robot predicted position in world frame

        th = pred_state[4, 0]  # Orientation angle

        R_WB = rot2(th)  
        R_BW = R_WB.T  # Rotation matrix from world to body frame (transpose of rotation from body to world frame)

        l_b = R_BW @ (l_w - p_w)  # Landmark position in body frame
        

        # Find homogeneus landmark coordinates
        l_tilde_b = l_b / l_b[0]  # Normalize by x 

        # Project to pixel coordinates using camera intrinsics
        u = self.intrinsics @ l_tilde_b

        # Compute measurement Jacobian
        H = self.measurement_jacobian(pred_state, l_w, l_b, R_BW, R_WB)

        return u, H
    
    def measurement_jacobian(self, pred_state, landmark_w, landmark_b, R_BW, R_WB):
        """
        Compute the measurement Jacobian H.
        """

        H = np.zeros((1, 8))  # Adjust size based on measurement dimension

        du_dl_tilde_b = self.intrinsics # Derivative of pixel coordinates w.r.t. landmark homogenous coordinates in body frame
        
        dl_tilde_b_d_l_b = np.array([
            [0, 0], 
            [-landmark_b[1, 0]/(landmark_b[0, 0]**2), 1/landmark_b[0, 0]]
        ]) # Derivative of homogenous landmark coordinates in body frame w.r.t. landmark position in body frame

        dl_b_dtheta = -R_BW @ hat2(1) @ (landmark_w - pred_state[0:2]) # Derivative of landmark in body coordinates wrt orientation theta
        dl_b_d_p_w = -R_BW # Derivative of landmark in body coordinates wrt robot position in world coordinates


        dl_b_d_state = np.zeros((2, 8)) # Preallocate derivative of landmark in body coordinates wrt state

        dl_b_d_state[:, 0:2] = dl_b_d_p_w # Fill the matrix in the correct spots
        dl_b_d_state[:, 4] = dl_b_dtheta.reshape((2,))

        du_d_state = du_dl_tilde_b @ dl_tilde_b_d_l_b @ dl_b_d_state # Chain rule to get derivative of pixel coordinates wrt state


        return du_d_state
