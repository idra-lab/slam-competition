
from  slam_competition.slam_competition_lib import *


from slam_competition.imu_integrator import integrate_imu_measurements



class SlamBagProcessor(Node):

    def __init__(self):
        super().__init__('slam_bag_processor_node')

        self.declare_parameter('bag_uri', 'src/slam-competition/assets/mapping')
        self.bag_uri = self.get_parameter('bag_uri').get_parameter_value().string_value

        self.declare_parameter('map_path', 'src/slam_competition/assets/map.pgm')

        self.reader = rosbag2_py.SequentialReader()

        self.bag_exhausted_flag = False

        # Topic containers
        self.odometry_list = []
        self.scan_list = []
        self.imu_list = []
        self.map = None

        self.frame_mappings ={
            'odometry_frame': 'odom',
            'base_frame': 'base_link',
            'imu_frame': 'imu_link',
            'map_frame': 'map'
        }

        self.topic_mappings = {
            'odometry': '/odom',
            'laser_scan': "/scan",
            'imu': '/imu',
            'map': '/map'
        }

        self.topic_types_mappings = {
            '/odom': 'Odometry',
            '/scan': 'LaserScan',
            '/imu': 'Imu',
            '/map': 'OccupancyGrid',
            '/tf': 'TFMessage',
            '/camera/color/image_raw': 'Image',
            '/camera/depth/points': 'PointCloud2',
            '/camera/depth/camera_info': 'CameraInfo',
        }


        self.transforms = {}
        self.transform_initialized_flag = False

        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_uri)
            # storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)


        self.timer = self.create_timer(0.005, self.timer_callback)


        # Republishers
        self.odom_publisher = self.create_publisher(globals()[self.topic_types_mappings['/odom']], '/odom', 10)
        self.scan_publisher = self.create_publisher(globals()[self.topic_types_mappings['/scan']], '/scan', 10)
        self.imu_publisher = self.create_publisher(globals()[self.topic_types_mappings['/imu']], '/imu', 10)
        self.map_publisher = self.create_publisher(globals()[self.topic_types_mappings['/map']], '/map', 10)
        self.image_publisher = self.create_publisher(globals()[self.topic_types_mappings['/camera/color/image_raw']], '/camera/color/image_raw', 10)
        self.pointcloud_publisher = self.create_publisher(globals()[self.topic_types_mappings['/camera/depth/points']], '/camera/depth/points', 10)
        self.camerainfo_publisher = self.create_publisher(globals()[self.topic_types_mappings['/camera/depth/camera_info']], '/camera/depth/camera_info', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # dict to hold publishers
        self.publisher_dict = {}

        self.publisher_dict['/odom'] = self.odom_publisher
        self.publisher_dict['/scan'] = self.scan_publisher
        self.publisher_dict['/imu'] = self.imu_publisher
        self.publisher_dict['/map'] = self.map_publisher
        self.publisher_dict['/tf'] = self.tf_broadcaster
        self.publisher_dict['/tf_static'] = self.tf_static_broadcaster

        self.publish_static_transform()

        # Results Publishers
        self.imu_path_publisher = self.create_publisher(Path, '/imu_integrated_path', 10)


    """  
    
    Callback Functions
    
    
    """

    def publish_static_transform(self):

        tf_base_to_laser = TransformStamped()

        tf_base_to_laser.header.stamp = self.get_clock().now().to_msg()
        tf_base_to_laser.header.frame_id = 'base_link'
        tf_base_to_laser.child_frame_id = 'laser_link'
        tf_base_to_laser.transform.translation.x = 0.18
        tf_base_to_laser.transform.translation.y = 0.0
        tf_base_to_laser.transform.translation.z = 0.02
        tf_base_to_laser.transform.rotation.x = 0.0
        tf_base_to_laser.transform.rotation.y = 0.0
        tf_base_to_laser.transform.rotation.z = 0.0
        tf_base_to_laser.transform.rotation.w = 1.0

        self.publisher_dict['/tf_static'].sendTransform([tf_base_to_laser])


        tf_base_to_camera = TransformStamped()
        tf_base_to_camera.header.stamp = self.get_clock().now().to_msg()
        tf_base_to_camera.header.frame_id = 'base_link'
        tf_base_to_camera.child_frame_id = 'camera_link'
        tf_base_to_camera.transform.translation.x = 0.18
        tf_base_to_camera.transform.translation.y = 0.0
        tf_base_to_camera.transform.translation.z = 0.18
        tf_base_to_camera.transform.rotation.x = 0.0
        tf_base_to_camera.transform.rotation.y = 0.0
        tf_base_to_camera.transform.rotation.z = 0.0
        tf_base_to_camera.transform.rotation.w = 1.0
        self.publisher_dict['/tf_static'].sendTransform([tf_base_to_camera])

        tf_base_to_imu = TransformStamped()
        tf_base_to_imu.header.stamp = self.get_clock().now().to_msg()
        tf_base_to_imu.header.frame_id = 'base_link'
        tf_base_to_imu.child_frame_id = 'imu_link'
        tf_base_to_imu.transform.translation.x = 0.0
        tf_base_to_imu.transform.translation.y = 0.0
        tf_base_to_imu.transform.translation.z = 0.0
        tf_base_to_imu.transform.rotation.x = 0.0
        tf_base_to_imu.transform.rotation.y = 0.0
        tf_base_to_imu.transform.rotation.z = 0.0
        tf_base_to_imu.transform.rotation.w = 1.0
        self.publisher_dict['/tf_static'].sendTransform([tf_base_to_imu])


        tf_cam_to_optical = TransformStamped()
        tf_cam_to_optical.header.stamp = self.get_clock().now().to_msg()
        tf_cam_to_optical.header.frame_id = 'camera_link'
        tf_cam_to_optical.child_frame_id = 'camera_depth_optical_frame'
        quat_opti = TF.Rotation.from_euler('xyz', [-90, 0, -90], degrees=True).as_quat()

        tf_cam_to_optical.transform.translation.x = 0.0
        tf_cam_to_optical.transform.translation.y = 0.0
        tf_cam_to_optical.transform.translation.z = 0.0
        tf_cam_to_optical.transform.rotation.x = quat_opti[0]
        tf_cam_to_optical.transform.rotation.y = quat_opti[1]
        tf_cam_to_optical.transform.rotation.z = quat_opti[2]
        tf_cam_to_optical.transform.rotation.w = quat_opti[3]
        self.publisher_dict['/tf_static'].sendTransform([tf_cam_to_optical])
        
        self.transform_initialized_flag = True

    def timer_callback(self):
        

        if self.reader.has_next():
            msg = self.reader.read_next()

            topic_type = self.topic_types_mappings.get(msg[0], None)

            if topic_type is None:
                # print("Unknown Topic Type")
                return
            
         
            msg_deserialized = deserialize_message(msg[1], globals()[topic_type])

            # build handler method name, e.g. "process_Odometry_msg"
            handler_name = f"process_{topic_type}_msg"

            handler = getattr(self, handler_name, None)
            if callable(handler):
                handler(msg_deserialized)
            else:
                print(f"No handler for message type: {topic_type}")
                # optionally handle missing handler: skip or warn
                pass

        else: 
            
            print("Bag Exhausted")
            self.evaluate_slam()
            self.timer.cancel()
            self.bag_exhausted_flag = True
            sys.exit()
        

    def process_Odometry_msg(self, odom_msg):
        # print("Publishing Odometry Message")
        self.publisher_dict['/odom'].publish(odom_msg)
        pass

    def process_LaserScan_msg(self, scan_msg):
        # print("Publishing Laser Scan Message")
        self.publisher_dict['/scan'].publish(scan_msg)
        pass

    def process_Imu_msg(self, imu_msg):
        # print("Publishing IMU Message")
        self.publisher_dict['/imu'].publish(imu_msg)

        self.imu_list.append(imu_msg)



    def process_TFMessage_msg(self, tf_msg):
        self.publisher_dict['/tf'].sendTransform(tf_msg.transforms)
        pass

    def process_OccupancyGrid_msg(self, occupancy_grid_msg):
        self.publisher_dict['/map'].publish(occupancy_grid_msg)
        pass

    def process_Image_msg(self, image_msg):
        self.image_publisher.publish(image_msg)
        pass

    def process_PointCloud2_msg(self, pointcloud2_msg):
        self.pointcloud_publisher.publish(pointcloud2_msg)
        pass

    def process_CameraInfo_msg(self, camera_info_msg):
        self.camerainfo_publisher.publish(camera_info_msg)
        pass


    """ 

    Evaluation Functions 
    
    """

    def integrate_all_imu(self):
        imu_path = integrate_imu_measurements(self.imu_list)

        imu_path.header.stamp = self.get_clock().now().to_msg()
        imu_path.header.frame_id = 'map'

        self.imu_path_publisher.publish(imu_path)


    def evaluate_slam(self):
        self.integrate_all_imu()

        print("Integrated all IMU measurements for evaluation.")
        

    

def main(args=None):

    rclpy.init(args=args)

    sbr = SlamBagProcessor()
    try: 
        rclpy.spin(sbr, executor=rclpy.executors.MultiThreadedExecutor())

    except SystemExit:                 # <--- process the exception 
        rclpy.logging.get_logger("Quitting").info('Done')


    sbr.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
