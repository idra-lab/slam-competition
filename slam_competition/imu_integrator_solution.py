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
        integrated_positions: List of estimated positions over time.
        integrated_orientations: List of estimated orientations (as quaternions) over time.
    """

    _orientation = np.array([0, 0, 0, 1], dtype=np.float64)  # Initial orientation as a quaternion (x, y, z, w)
    _position = np.array([0, 0, 0], dtype=np.float64)      # Initial position (x, y, z)
    _velocity = np.array([0, 0, 0], dtype=np.float64)      # Initial velocity (vx, vy, vz)
    linear_acceleration = np.array([0, 0, 0], dtype=np.float64)  # Assuming gravity aligned with z-axis
    angular_velocity = np.array([0, 0, 0], dtype=np.float64)        # No initial angular velocity

    timestamp_prev = None

    pth = Path()
    
    first_imu = next(iter(imu_measurements))
    pth.header = first_imu.header

    pose = PoseStamped()
    timestamp_prev = first_imu.header.stamp.sec + first_imu.header.stamp.nanosec * 1e-9
    pose.header = first_imu.header
    pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = _position
    pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = _orientation
    pth.poses.append(pose)

    for imu_msg in imu_measurements: 

        timestamp = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9
        dt = timestamp - timestamp_prev
        if (dt<=0.0):
            continue

        print("Dtime between measurements:", dt)
        linear_acceleration = np.array([imu_msg.linear_acceleration.x,
                                        imu_msg.linear_acceleration.y,
                                        imu_msg.linear_acceleration.z], dtype=np.float64)
        
        print("Linear Acceleration:", linear_acceleration)

        angular_velocity = np.array([imu_msg.angular_velocity.x,
                                     imu_msg.angular_velocity.y,
                                     imu_msg.angular_velocity.z], dtype=np.float64)


        rotmat = TF.Rotation.from_quat(_orientation).as_matrix()
        world_acceleration = rotmat @ linear_acceleration - np.array([0, 0, 9.81], dtype=np.float64)


        _position += _velocity * dt + 0.5 * world_acceleration * dt**2
        _velocity += world_acceleration * dt

        print("DT:", dt)

        print("Position:", _position)

        world_angular_velocity = rotmat @ angular_velocity

        big_omega = quaternion_omega_matrix(world_angular_velocity)
        dq_dt = 0.5 * big_omega @ _orientation
        _orientation += dq_dt * dt
        _orientation /= np.linalg.norm(_orientation)  # Normalize quaternion

        pose = PoseStamped()
        # pose.header = imu_msg.header
        pose.header.frame_id = 'map'
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = _position
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = _orientation
        pth.poses.append(pose)

        timestamp_prev = timestamp

    return pth


        
