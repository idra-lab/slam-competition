from  slam_competition.slam_competition_lib import *

def cross_product_matrix(om:np.array) -> np.array:
    """
    Create a cross product matrix from a 3D vector.
    
    Args:
        om: A 3D vector (numpy array of shape (3,)).

    Returns:
        A 3x3 numpy array representing the cross product matrix.
    """
    return np.array([[0, -om[2], om[1]],
                     [om[2], 0, -om[0]],
                     [-om[1], om[0], 0]])

def quaternion_omega_matrix(om:np.array) -> np.array:

    """
    Create the quaternion omega matrix from angular velocity.
    
    Args:
        om: A 3D angular velocity vector (numpy array of shape (3,)).
    """

    Omega = np.zeros((4,4))
    Omega[:3, :3] = -cross_product_matrix(om)
    Omega[:3, 3] = om
    Omega[3, :3] = -om.transpose()
    return Omega

def integrate_imu_measurements(imu_measurements) -> Path:
    """
    Integrate IMU measurements to estimate pose changes over time.
    
    Args:
        imu_measurements: List of IMU messages in chronological order.
        
    Returns:
        pth: A Path object containing estimated poses over time.
    """

    _orientation = np.array([0, 0, 0, 1], dtype=np.float64)  # Initial orientation as a quaternion (x, y, z, w)
    _position = np.array([0, 0, 0], dtype=np.float64)      # Initial position (x, y, z)
    _velocity = np.array([0, 0, 0], dtype=np.float64)      # Initial velocity (vx, vy, vz)
    linear_acceleration = np.array([0, 0, 0], dtype=np.float64)  # Acceleration to be read from IMU message
    angular_velocity = np.array([0, 0, 0], dtype=np.float64)        # Angular velocity to be read from IMU message

    pth = Path() # Initialize the resulting Path
    
    ######################################################################################################################################################################################
    ######################################################################################################################################################################################
    ################################################################# YOUR CODE GOES HERE ################################################################################################
    ######################################################################################################################################################################################
    ######################################################################################################################################################################################

    # Iterate through each IMU measurement 

    for imu_msg in imu_measurements:  
        # ... your code to integrate IMU measurements ...
    
        """ 
         
          Pseudocode:

          1. Extract timestamp: 
            timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
          2. take care of the first measurement to initialize timestamps
          3. Compute delta_time (dt) between current and previous measurement
          4. Extract linear acceleration and angular velocity from imu_msg

                linear_acceleration = np.array([imu_msg.linear_acceleration.x,
                                        imu_msg.linear_acceleration.y,
                                        imu_msg.linear_acceleration.z], dtype=np.float64)

                angular_velocity = np.array([imu_msg.angular_velocity.x,
                                     imu_msg.angular_velocity.y,
                                     imu_msg.angular_velocity.z], dtype=np.float64)

          5. Rotate linear acceleration to the world frame using current orientation
          6. Subtract gravity from the world frame acceleration
          7. Update position and velocity using Euler integration
          8. Rotate angular velocity to the world frame: 

                rotmat = TF.Rotation.from_quat(_orientation).as_matrix()
                world_angular_velocity = rotmat @ angular_velocity

          9. Compute the derivative of the quaternion using the quaternion omega matrix

                big_omega = quaternion_omega_matrix(world_angular_velocity)
                dq_dt = 0.5 * big_omega @ _orientation
          10. Update orientation using Euler integration and normalize the quaternion
          11. Store the updated pose (position and orientation) in the Path object



        """

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = _position
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = _orientation
        pth.poses.append(pose)

    

    ######################################################################################################################################################################################
    ######################################################################################################################################################################################
    ############################################################################ END #####################################################################################################
    ######################################################################################################################################################################################
    ######################################################################################################################################################################################

    return pth


        
