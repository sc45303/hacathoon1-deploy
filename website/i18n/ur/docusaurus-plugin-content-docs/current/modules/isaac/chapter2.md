---
sidebar_position: 3
---

# باب 2: Isaac ROS اور VSLAM نیویگیشن

## سیکھنے کے مقاصد

- Isaac ROS ایکو سسٹم اور اس کے اجزاء کو سمجھیں
- Visual Simultaneous Localization and Mapping (VSLAM) کے بارے میں سیکھیں
- ہیومینائیڈ روبوٹ نیویگیشن کے لیے VSLAM سسٹمز نافذ کریں
- نیویگیشن سسٹمز کے ساتھ Isaac ROS ادراک پیکیجز کو مربوط کریں
- ہیومینائیڈ روبوٹ ایپلی کیشنز کے لیے Nav2 کنفیگر کریں

## Isaac ROS کا تعارف

Isaac ROS NVIDIA کا ہارڈویئر ایکسلریٹڈ، ادراک پر مبنی پیکیجز کا مجموعہ ہے جو روبوٹکس ایپلی کیشنز کے لیے ڈیزائن کیا گیا ہے۔ یہ پیکیجز NVIDIA کے GPUs کا فائدہ اٹھاتے ہوئے ادراک کے کاموں کو تیز کرتے ہیں، جو خاص طور پر ہیومینائیڈ روبوٹس کے لیے اہم ہے جو متعدد سینسر سٹریمز کی ریئل ٹائم پروسیسنگ کی ضرورت رکھتے ہیں۔

### اہم Isaac ROS پیکیجز

1. **Isaac ROS Image Pipeline**: GPU ایکسلریٹڈ امیج پروسیسنگ
2. **Isaac ROS Visual SLAM**: GPU ایکسلریٹڈ ویژول SLAM
3. **Isaac ROS Object Detection**: ریئل ٹائم آبجیکٹ ڈیٹیکشن
4. **Isaac ROS Apriltag**: AprilTag ڈیٹیکشن اور پوز اسٹیمیشن
5. **Isaac ROS Stereo Dense Reconstruction**: 3D ماحول کی تعمیر نو

### ہیومینائیڈ روبوٹس کے لیے فوائد

Isaac ROS ہیومینائیڈ روبوٹکس کے لیے مخصوص فوائد فراہم کرتا ہے:

- **GPU ایکسلریشن**: ریئل ٹائم میں متعدد سینسرز پروسیس کرنے کے لیے ضروری
- **ہائی پرفارمنس SLAM**: پیچیدہ ماحول میں لوکلائزیشن کے لیے لازمی
- **مضبوط ادراک**: انسانوں کے ارد گرد محفوظ نیویگیشن کے لیے اہم
- **ریئل ٹائم پروسیسنگ**: ڈائنامک بیلنس اور کنٹرول کے لیے ضروری

## Visual SLAM (VSLAM) کی بنیادیں

Visual SLAM (Simultaneous Localization and Mapping) ویژول ڈیٹا کو اوڈومیٹری کے ساتھ ملا کر نقشے تخلیق کرتا ہے جبکہ روبوٹ کی پوزیشن کو ان نقشوں میں ٹریک کرتا ہے۔ یہ خاص طور پر انسانی ماحول میں کام کرنے والے ہیومینائیڈ روبوٹس کے لیے قیمتی ہے جہاں روایتی LIDAR پر مبنی SLAM ناکافی ہو سکتا ہے۔

### VSLAM کیسے کام کرتا ہے

1. **فیچر ڈیٹیکشن**: ویژول ڈیٹا میں نمایاں پوائنٹس کی شناخت
2. **فیچر میچنگ**: مسلسل فریموں کے درمیان فیچرز کا میچ
3. **مووشن اسٹیمیشن**: فیچر موومنٹ کی بنیاد پر کیمرہ موشن کا اندازہ
4. **میپنگ**: ویژول فیچرز استعمال کر کے ماحول کا نقشہ بنانا
5. **آپٹمائزیشن**: نقشہ اور ٹریجیکٹری اسٹیمیٹس کو بہتر بنانا

### VSLAM بمقابلہ روایتی SLAM

Visual SLAM روایتی LIDAR پر مبنی طریقوں پر کئی فوائد پیش کرتا ہے:

- **امیر معلومات**: ویژول ڈیٹا میں زیادہ سیمینٹک معلومات ہوتی ہیں
- **کم لاگت**: مہنگے LIDAR سینسرز کی ضرورت نہیں
- **انڈور ماحول کے لیے بہتر**: ٹیکسچر سے بھرپور ماحول میں اچھا کام کرتا ہے
- **انسان جیسی ادراک**: انسانی نیویگیشن سے زیادہ مشابہت

تاہم، اس کے چیلنجز بھی ہیں:

- **لائٹنگ حساسیت**: خراب لائٹنگ میں کارکردگی کم ہو جاتی ہے
- **ڈائنامک آبجیکٹس**: حرکت کرنے والی اشیاء ٹریکنگ میں غلطیاں پیدا کر سکتی ہیں
- **کمپیوٹیشنل ضروریات**: زیادہ پروسیسنگ پاور کی ضرورت

## Isaac ROS Visual SLAM پیکیج

Isaac ROS Visual SLAM پیکیج NVIDIA GPUs استعمال کرتے ہوئے ہارڈویئر ایکسلریٹڈ ویژول SLAM فراہم کرتا ہے۔

### اہم خصوصیات

- **GPU ایکسلریشن**: فیچر ڈیٹیکشن اور میچنگ کے لیے CUDA کورز کا استعمال
- **ریئل ٹائم پرفارمنس**: ہائی فریم ریٹس پر ویڈیو پروسیس کرنے کی صلاحیت
- **مضبوط ٹریکنگ**: ویو پوائنٹ تبدیلیوں اور لائٹنگ ویری ایشنز کو ہینڈل کرتا ہے
- **ROS 2 مطابقت**: ROS 2 نیویگیشن سٹیک کے ساتھ بے رکاوٹ انٹیگریشن

### انسٹالیشن

Isaac ROS پیکیجز عام طور پر ڈاکر کنٹینرز کے ذریعے انسٹال کیے جاتے ہیں:

```bash
# Isaac ROS ڈاکر کنٹینر پل کریں
docker pull nvcr.io/nvidia/isaac-ros:ros-humble-visualslam-cu11.8.0-22.12.2

# GPU رسائی کے ساتھ چلائیں
docker run --gpus all -it --rm \
  --network=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  nvcr.io/nvidia/isaac-ros:ros-humble-visualslam-cu11.8.0-22.12.2
```

### Isaac ROS Visual SLAM لانچ کرنا

```xml
<!-- visual_slam.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='visual_slam_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
                name='visual_slam_node',
                parameters=[{
                    'enable_rectified_pose': True,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'enable_fisheye_distortion': False,
                }],
                remappings=[
                    ('/visual_slam/image_raw', '/camera/image_rect'),
                    ('/visual_slam/camera_info', '/camera/camera_info'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
```

## ہیومینائیڈ روبوٹس کے لیے Nav2

Nav2 ROS 2 کا معیاری نیویگیشن فریم ورک ہے۔ اگرچہ روایتی طور پر وہیلڈ روبوٹس کے لیے استعمال ہوتا ہے، لیکن کچھ تبدیلیوں کے ساتھ اسے ہیومینائیڈ روبوٹس کے لیے موافق بنایا جا سکتا ہے۔

### Nav2 آرکیٹیکچر

Nav2 سٹیک میں کئی اہم اجزاء شامل ہیں:

1. **گلوبل پلانر**: آغاز سے ہدف تک پاتھ تخلیق کرتا ہے
2. **لوکل پلانر**: رکاوٹوں سے بچتے ہوئے گلوبل پاتھ کی پیروی کرتا ہے
3. **کنٹرولر**: پاتھ فالو کمانڈز کو روبوٹ کنٹرولز میں تبدیل کرتا ہے
4. **ریکوری بیہیویئرز**: نیویگیشن ناکامیوں کو ہینڈل کرتا ہے

### ہیومینائیڈ روبوٹس کے لیے Nav2 لانچ فائل

```xml
<!-- humanoid_nav2.launch.py -->
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # ہیومینائیڈ مخصوص نقشوں کے لیے میپ سرور پیرامیٹرز
    map_server_params = {
        'yaml_filename': '/path/to/humanoid_map.yaml',
        'frame_id': 'map',
        'topic_name': 'map',
        'use_bag_pose': False
    }

    return LaunchDescription([
        # لانچ آرگیومنٹس کا اعلان
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start lifecycle nodes'),
        DeclareLaunchArgument(
            'params_file',
            default_value='/path/to/humanoid_nav2_params.yaml',
            description='Full path to the ROS2 parameters file'),

        # میپ سرور
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[map_server_params],
            output='screen'),

        # لوکل کوسٹ میپ
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='local_costmap',
            parameters=[params_file],
            output='screen'),

        # گلوبل کوسٹ میپ
        Node(
            package='nav2_costmap_2d',
            executable='costmap_2d_node',
            name='global_costmap',
            parameters=[params_file],
            output='screen'),

        # ہیومینائیڈ مخصوص موومنٹ کے لیے کنٹرولر سرور
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[params_file],
            output='screen'),

        # پلانر سرور
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            parameters=[params_file],
            output='screen')
    ])
```

## ہیومینائیڈ نیویگیشن کے غور طلب امور

ہیومینائیڈ روبوٹ کے ساتھ نیویگیشن کے لیے خصوصی غور کی ضرورت ہے:

### بائی پیڈل لوکوموشن

- **فٹ سٹیپ پلاننگ**: مسلسل پاتھوں کی بجائے، ہیومینائیڈ روبوٹس کو الگ الگ فٹ سٹیپس کی ضرورت ہوتی ہے
- **بیلنس مینٹیننس**: موومنٹ کے دوران کنٹرولرز کو بیلنس برقرار رکھنا چاہیے
- **سٹیبیلٹی**: چلنے کے گیٹس کو ڈائنامکلی سٹیبل ہونا چاہیے
- **ٹیرین ایڈاپٹیشن**: ناہموار زمین کو ہینڈل کرنے کی صلاحیت

### کنفیگریشن کی مثال

```yaml
# humanoid_nav2_params.yaml
amcl:
  ros__parameters:
    use_sim_time: False
    alpha1: 0.2
    alpha2: 0.2
    alpha3: 0.2
    alpha4: 0.2
    alpha5: 0.2
    base_frame_id: "base_footprint"
    beam_skip_distance: 0.5
    beam_skip_error_threshold: 0.9
    beam_skip_threshold: 0.3
    do_beamskip: false
    global_frame_id: "map"
    lambda_short: 0.1
    laser_likelihood_max_dist: 2.0
    laser_max_range: 100.0
    laser_min_range: -1.0
    laser_model_type: "likelihood_field"
    max_beams: 60
    max_particles: 2000
    min_particles: 500
    odom_frame_id: "odom"
    pf_err: 0.05
    pf_z: 0.99
    recovery_alpha_fast: 0.0
    recovery_alpha_slow: 0.0
    resample_interval: 1
    robot_model_type: "nav2_amcl::DifferentialMotionModel"
    save_pose_rate: 0.5
    sigma_hit: 0.2
    tf_broadcast: true
    transform_timeout: 1.0
    update_min_a: 0.2
    update_min_d: 0.25

controller_server:
  ros__parameters:
    use_sim_time: False
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # ہیومینائیڈ مخصوص کنٹرولر
    FollowPath:
      plugin: "nav2_mppi_controller::MppiController"
      time_steps: 24
      control_freq: 20.0
      horizon: 1.5
      Q: [2.0, 2.0, 0.8]
      R: [1.0, 1.0, 0.5]
      P: [0.02, 0.02, 0.02]
      collision_penalty: 100.0
      goal_angle_tolerance: 0.15
      goal_check_tolerance: 0.25
      inflation_radius: 0.15
      debug_cost_data_enabled: False
      motion_model: "DiffDrive"
      # ہیومینائیڈ روبوٹس کے لیے مناسب موشن ماڈل استعمال کریں

local_costmap:
  ros__parameters:
    use_sim_time: False
    update_frequency: 5.0
    publish_frequency: 2.0
    global_frame: odom
    robot_base_frame: base_link
    footprint: "[ [0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3] ]"
    resolution: 0.05
    inflation_radius: 0.55
    plugins: ["voxel_layer", "inflation_layer"]
    voxel_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: True
      voxel_size: 0.05
      max_voxels: 10000
      mark_threshold: 0
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.55
    always_send_full_costmap: True

global_costmap:
  ros__parameters:
    use_sim_time: False
    update_frequency: 1.0
    publish_frequency: 1.0
    global_frame: map
    robot_base_frame: base_link
    footprint: "[ [0.3, 0.3], [0.3, -0.3], [-0.3, -0.3], [-0.3, 0.3] ]"
    resolution: 0.05
    track_unknown_space: true
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
    obstacle_layer:
      plugin: "nav2_costmap_2d::VoxelLayer"
      enabled: True
      voxel_size: 0.05
      max_voxels: 10000
      mark_threshold: 0
      observation_sources: scan
      scan:
        topic: /scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"
    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0
      inflation_radius: 0.55

planner_server:
  ros__parameters:
    use_sim_time: False
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true
```

## Isaac ROS Visual SLAM کے ساتھ انٹیگریشن

Isaac ROS VSLAM کو Nav2 کے ساتھ مربوط کرنے کے لیے:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster
import tf_transformations

class IsaacVSLAMIntegrator(Node):
    def __init__(self):
        super().__init__('isaac_vslam_integrator')

        # Isaac ROS VSLAM کے لیے سبسکرائبرز
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/visual_slam/pose_graph/pose',
            self.pose_callback,
            10
        )

        # Nav2 کے لیے پبلشرز
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # VSLAM سے Nav2 ٹرانسفارم کے لیے TF براڈکاسٹر
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info('Isaac VSLAM Integrator initialized')

    def pose_callback(self, msg):
        # VSLAM پوز کو پروسیس کریں اور ممکنہ طور پر Nav2 کو بھیجیں
        self.get_logger().info(f'VSLAM pose: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}')

        # VSLAM ڈیٹا استعمال کر کے map سے odom ٹرانسفارم براڈکاسٹ کریں
        t = msg.pose.pose  # VSLAM سے پوزیشن اور اورینٹیشن

        # TF میسیج تخلیق کریں
        from geometry_msgs.msg import TransformStamped
        tf_msg = TransformStamped()

        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'

        tf_msg.transform.translation.x = t.position.x
        tf_msg.transform.translation.y = t.position.y
        tf_msg.transform.translation.z = t.position.z
        tf_msg.transform.rotation = t.orientation

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    integrator = IsaacVSLAMIntegrator()
    rclpy.spin(integrator)
    integrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## ہیومینائیڈ VSLAM کے لیے حقیقی دنیا کے غور طلب امور

### لائٹنگ حالات

- **انڈور ماحول**: عام طور پر مستقل لائٹنگ ہوتی ہے
- **ونڈوز**: لائٹنگ تبدیلیاں پیدا کر سکتی ہیں جو ٹریکنگ کو متاثر کرتی ہیں
- **مصنوعی لائٹنگ**: سائے پیدا کر سکتی ہے جو فیچر ڈیٹیکشن کو متاثر کرتے ہیں
- **ڈائنامک لائٹنگ**: روشن سے تاریک علاقوں میں منتقل ہونا

### موشن آرٹیفیکٹس

- **ہیڈ موومنٹ**: ہیومینائیڈ روبوٹس اکثر سر ہلاتے ہیں، جو کیمرہ پرسپیکٹیو کو متاثر کرتا ہے
- **باڈی ڈائنامکس**: چلنے کی موشن کیمرہ وائبریشن کا باعث بن سکتی ہے
- **تیز موومنٹس**: تیز سر موومنٹس موشن بلر کا باعث بن سکتے ہیں

## VSLAM مسائل کا ٹربل شوٹنگ

### ٹریکنگ لاس

- **حل**: ری لوکلائزیشن نافذ کریں یا IMU کے ساتھ سینسر فیوشن استعمال کریں
- **وجہ**: ناکافی ویژول فیچرز یا تیز موومنٹ

### ڈریفٹ

- **حل**: لوپ کلوژر ڈیٹیکشن اور پوز گراف آپٹمائزیشن استعمال کریں
- **وجہ**: پوز اسٹیمیشن میں جمع شدہ غلطیاں

### میپ کوالٹی

- **حل**: پیرامیٹرز کو آپٹمائز کریں اور مناسب سینسرز استعمال کریں
- **وجہ**: خراب لائٹنگ، تکراری ٹیکسچرز، یا ڈائنامک آبجیکٹس

## خلاصہ

Isaac ROS طاقتور GPU ایکسلریٹڈ ادراک کی صلاحیتیں فراہم کرتا ہے جو خاص طور پر ہیومینائیڈ روبوٹس کے لیے قیمتی ہیں۔ Visual SLAM امیر ماحولیاتی سمجھ پیش کرتا ہے جسے Nav2 کے ساتھ مربوط کر کے مضبوط نیویگیشن حاصل کی جا سکتی ہے۔ Isaac ROS اور Nav2 کا امتزاج ہیومینائیڈ روبوٹس کو پیچیدہ انسانی ماحول میں محفوظ اور موثر طریقے سے نیویگیٹ کرنے کے قابل بناتا ہے۔

## مشقیں

1. سیمولیشن ماحول میں Isaac ROS Visual SLAM سیٹ اپ کریں
2. ہیومینائیڈ روبوٹ ماڈل کے لیے Nav2 کنفیگر کریں
3. دونوں سسٹمز کو مربوط کر کے نیویگیشن ٹیسٹ کریں

## اگلے اقدامات

اگلے باب میں، ہم بائی پیڈل روبوٹس کے لیے خاص طور پر موافق Nav2 پاتھ پلاننگ کی دریافت کریں گے، جو ہیومینائیڈ لوکوموشن کے منفرد چیلنجز کو حل کرے گی۔