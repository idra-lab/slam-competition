
from  slam_competition.slam_competition_lib import *






class SlamBagProcessor(Node):

    def __init__(self):
        super().__init__('slam_bag_processor_node')

        self.declare_parameter('bag_uri', 'src/slam-competition/assets/mapping')
        self.bag_uri = self.get_parameter('bag_uri').get_parameter_value().string_value

        self.declare_parameter('map_path', 'src/slam_competition/assets/map.pgm')

        self.reader = rosbag2_py.SequentialReader()

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
            'camera/color/image_raw': 'Image',
            'camera/depth/points': 'PointCloud2',
            'camera/depth/camera_info': 'CameraInfo',
        }


        self.transforms = {}
        self.transform_initialized_flag = False

        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_uri)
            # storage_id='mcap')
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)


        self.timer = self.create_timer(0.01, self.timer_callback)


        # Republishers
        self.odom_publisher = self.create_publisher(globals()[self.topic_types_mappings['/odom']], '/odom', 10)
        self.scan_publisher = self.create_publisher(globals()[self.topic_types_mappings['/scan']], '/scan', 10)
        self.imu_publisher = self.create_publisher(globals()[self.topic_types_mappings['/imu']], '/imu', 10)
        self.map_publisher = self.create_publisher(globals()[self.topic_types_mappings['/map']], '/map', 10)
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

    def publish_static_transform(self):
        tf_static = TransformStamped()

        tf_static.header.stamp = self.get_clock().now().to_msg()
        tf_static.header.frame_id = 'base_link'
        tf_static.child_frame_id = 'laser_frame'
        tf_static.transform.translation.x = 0.2
        tf_static.transform.translation.y = 0.0
        tf_static.transform.translation.z = 0.02
        tf_static.transform.rotation.x = 0.0
        tf_static.transform.rotation.y = 0.0
        tf_static.transform.rotation.z = 0.0
        tf_static.transform.rotation.w = 1.0

        self.publisher_dict['/tf_static'].sendTransform(tf_static)

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


    def evaluate_slam(self):
        pass

    def initialize_transforms(self, tf_msg):
        pass

    def process_Odometry_msg(self, odom_msg):
        print("Publishing Odometry Message")
        self.publisher_dict['/odom'].publish(odom_msg)
        pass

    def process_LaserScan_msg(self, scan_msg):
        print("Publishing Laser Scan Message")
        scan_msg.header.frame_id = 'laser_frame'
        self.publisher_dict['/scan'].publish(scan_msg)
        pass

    def process_Imu_msg(self, imu_msg):
        print("Publishing IMU Message")
        self.publisher_dict['/imu'].publish(imu_msg)
        pass

    def process_TFMessage_msg(self, tf_msg):
        self.publisher_dict['/tf'].sendTransform(tf_msg.transforms)
        pass

    def process_OccupancyGrid_msg(self, occupancy_grid_msg):
        self.publisher_dict['/map'].publish(occupancy_grid_msg)
        pass

    def process_Image_msg(self, image_msg):
        pass

    def process_PointCloud2_msg(self, pointcloud2_msg):
        pass

    def process_CameraInfo_msg(self, camera_info_msg):
        pass

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
