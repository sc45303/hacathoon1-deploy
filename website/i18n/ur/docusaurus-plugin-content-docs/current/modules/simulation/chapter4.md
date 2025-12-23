---
sidebar_position: 5
---

# باب 4: Gazebo اور Unity کو ضم کرنا

## سیکھنے کے مقاصد

- سمجھیں کہ ہیومینائیڈ روبوٹ سیمولیشن کے لیے Gazebo اور Unity کو کیسے ضم کرنا ہے
- ملٹی پلیٹ فارم سیمولیشن کے معماری کے نقطہ نظر سیکھیں
- Gazebo اور Unity کے درمیان ڈیٹا ہم وقت سازی نافذ کریں
- دونوں پلیٹ فارمز کی مضبوطیوں کا فائدہ اٹھانے والے بے داغ ورک فلو تخلیق کریں
- ہیومینائیڈ روبوٹس کے لیے ہائبرڈ سیمولیشن ماحول ڈیزائن کریں

## ملٹی پلیٹ فارم سیمولیشن کا تعارف

سیمولیشن پائپ لائن میں Gazebo اور Unity دونوں کا استعمال کرتے ہوئے ہم ہر پلیٹ فارم کی مضبوطیوں کا فائدہ اٹھا سکتے ہیں:

- **Gazebo**: بہترین فزکس سیمولیشن، قائم ROS انضمام، سینسر ماڈلنگ
- **Unity**: ہائی فائیڈلٹی رینڈرنگ، حقیقی وژولائزیشن، جدید گرافکس کی صلاحیتیں

دونوں پلیٹ فارمز کا انضمام ایک مکمل سیمولیشن پائپ لائن کو فعال کرتا ہے جہاں Gazebo میں فزکس کے اعتبار سے درست سیمولیشن ہوتی ہے جبکہ Unity میں ہائی فائیڈلٹی وژولائزیشن اور انسان-روبوٹ تعامل ہوتا ہے۔

## انضمام کے نقطہ نظر

### 1. ہم وقت سازی کے ساتھ متوازی سیمولیشن

دونوں سیمولیٹرز کو آزادانہ طور پر چلائیں لیکن ان کی حالتیں ہم وقت ساز رکھیں:

```
[روبوٹ کنٹرولر] → [Gazebo (فزکس)] ↔ [ہم وقت سازی لیئر] ↔ [Unity (رینڈرنگ)]
                      ↓                    ↓                           ↓
                [فزکس نتائج]    [حالت اپ ڈیٹ]              [وژولائزیشن]
```

### 2. مخصوص کام کی تقسیم

- **Gazebo**: فزکس سیمولیشن، سینسر ڈیٹا تخلیق، نیویگیشن منصوبہ بندی
- **Unity**: ہائی فائیڈلٹی رینڈرنگ، انسان-روبوٹ تعامل، حقیقی منظر وژولائزیشن

### 3. ہائبرڈ معماری

ڈیٹا فلو کو دونوں سیمولیٹرز کے درمیان منظم کرنے کے لیے ایک مڈل ویئر لیئر استعمال کریں:

```
[ROS 2 نوڈز]
     |
[مڈل ویئر لیئر]
     |---------|---------|
     |         |         |
[Gazebo]  [Unity]  [دیگر ٹولز]
   (فزکس) (رینڈرنگ)
```

## نافذ کاری معماری

### ہم وقت سازی لیئر

ہم وقت سازی لیئر دونوں سیمولیٹرز کے درمیان مطابقت برقرار رکھنے کے لیے انتہائی اہم ہے:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Twist
import time
import threading

class SimulationSynchronizer(Node):
    def __init__(self):
        super().__init__('simulation_synchronizer')

        # دونوں سیمولیٹرز کے لیے پبلشرز
        self.gazebo_joint_pub = self.create_publisher(JointState, '/gazebo/joint_commands', 10)
        self.unity_joint_pub = self.create_publisher(JointState, '/unity/joint_commands', 10)

        # دونوں سیمولیٹرز سے سبسکرائبرز
        self.gazebo_state_sub = self.create_subscription(
            JointState, '/gazebo/joint_states', self.gazebo_state_callback, 10)
        self.unity_state_sub = self.create_subscription(
            JointState, '/unity/joint_states', self.unity_state_callback, 10)

        # ہم وقت سازی ٹائمر
        self.sync_timer = self.create_timer(0.01, self.sync_states)  # 100 Hz ہم وقت سازی

        # حالت ذخیرہ کرنا
        self.gazebo_state = JointState()
        self.unity_state = JointState()
        self.last_sync_time = time.time()

        self.get_logger().info('سیمولیشن ہم وقت ساز شروع کیا گیا')

    def gazebo_state_callback(self, msg):
        """Gazebo سے جوائنٹ حالتیں ہینڈل کریں"""
        self.gazebo_state = msg
        # وژولائزیشن کے لیے Unity کو بھیجیں
        self.unity_joint_pub.publish(msg)

    def unity_state_callback(self, msg):
        """Unity وژولائزیشن سے جوائنٹ حالتیں ہینڈل کریں"""
        self.unity_state = msg
        # ضرورت ہونے پر Gazebo کو بھیج سکتے ہیں
        # عام سیٹ اپ میں، Unity "slave" ہے فزکس سیمولیشن کا

    def sync_states(self):
        """سیمولیٹرز کے درمیان حالتیں ہم وقت کریں"""
        # اس مثال میں، Gazebo فزکس کے لیے معتبر ہے
        # Unity Gazebo کی حالت کے مطابق ہے وژولائزیشن کے لیے

        # نمایاں عدم مطابقت کے لیے چیک کریں (مثلاً نیٹ ورک تاخیر کی وجہ سے)
        if self.gazebo_state.name and self.unity_state.name:
            if len(self.gazebo_state.position) == len(self.unity_state.position):
                max_diff = 0
                for i in range(len(self.gazebo_state.position)):
                    diff = abs(self.gazebo_state.position[i] - self.unity_state.position[i])
                    max_diff = max(max_diff, diff)

                if max_diff > 0.1:  # نمایاں عدم مطابقت کے لیے حد
                    self.get_logger().warn(f'نمایاں عدم مطابقت پائی گئی: {max_diff} rad')

        # Unity کو Gazebo حالت کے ساتھ ہم وقت رکھیں
        if self.gazebo_state.name and self.gazebo_state.position:
            sync_msg = JointState()
            sync_msg.header.stamp = self.get_clock().now().to_msg()
            sync_msg.name = self.gazebo_state.name
            sync_msg.position = self.gazebo_state.position
            sync_msg.velocity = self.gazebo_state.velocity
            sync_msg.effort = self.gazebo_state.effort

            # وژولائزیشن اپ ڈیٹ کے لیے Unity کو بھیجیں
            self.unity_joint_pub.publish(sync_msg)

def main(args=None):
    rclpy.init(args=args)
    synchronizer = SimulationSynchronizer()

    try:
        rclpy.spin(synchronizer)
    except KeyboardInterrupt:
        pass
    finally:
        synchronizer.destroy_node()
        rclpy.shutdown()
```

## جدید ہم وقت سازی کی تکنیکیں

### نیٹ ورک تاخیر کے لیے حالت کی پیشن گوئی

جب سیمولیٹرز کے درمیان نیٹ ورک تاخیر ہو، ہم پیشن گوئی کا استعمال کر سکتے ہیں:

```python
import numpy as np
from collections import deque

class PredictiveSynchronizer:
    def __init__(self):
        self.state_history = deque(maxlen=10)  # آخری 10 حالتیں محفوظ کریں
        self.network_delay_estimate = 0.05  # 50ms تاخیر کا تخمینہ

    def add_state(self, state, timestamp):
        """پیشن گوئی کے لیے حالت کو تاریخ میں شامل کریں"""
        self.state_history.append({
            'state': state,
            'timestamp': timestamp
        })

    def predict_state(self, target_time):
        """مستقبل کے وقت پر حالت کی پیشن گوئی کریں"""
        if len(self.state_history) < 2:
            return None

        # آخری دو حالتیں کے درمیان سادہ لکیری بیرون کشی
        recent_states = list(self.state_history)[-2:]

        if len(recent_states) < 2:
            return recent_states[0]['state']

        state1 = recent_states[0]
        state2 = recent_states[1]

        dt = state2['timestamp'] - state1['timestamp']
        if dt <= 0:
            return state2['state']

        # رفتار کا حساب لگائیں (سادہ کیا گیا جوائنٹ پوزیشنز کے لیے)
        dt_target = target_time - state2['timestamp']
        predicted_state = JointState()
        predicted_state.name = state2['state'].name
        predicted_state.position = []

        for i in range(len(state2['state'].position)):
            if i < len(state1['state'].position):
                velocity = (state2['state'].position[i] - state1['state'].position[i]) / dt
                predicted_pos = state2['state'].position[i] + velocity * dt_target
                predicted_state.position.append(predicted_pos)
            else:
                predicted_state.position.append(state2['state'].position[i])

        return predicted_state
```

## ڈیٹا تبادلہ کے نمونے

### 1. سینسر ڈیٹا تبادلہ

```python
from sensor_msgs.msg import LaserScan, Image, Imu
from nav_msgs.msg import Odometry

class SensorDataExchange:
    def __init__(self, node):
        self.node = node

        # دونوں سیمولیٹرز کے لیے پبلشرز
        self.gazebo_sensor_pub = node.create_publisher(LaserScan, '/gazebo/scanner', 10)
        self.unity_sensor_pub = node.create_publisher(Image, '/unity/camera', 10)

        # دونوں سیمولیٹرز سے سبسکرائبرز
        self.gazebo_odom_sub = node.create_subscription(
            Odometry, '/gazebo/odom', self.gazebo_odom_callback, 10)
        self.unity_imu_sub = node.create_subscription(
            Imu, '/unity/imu', self.unity_imu_callback, 10)

        # ہم وقت سازی کا پرچم
        self.odom_sync_enabled = True
        self.imu_sync_enabled = True

    def gazebo_odom_callback(self, msg):
        """Gazebo اودومیٹر کو Unity کے لیے وژولائزیشن کے لیے بھیجیں"""
        if self.odom_sync_enabled:
            # اودومیٹر کو Unity کے لیے ٹرانسفارم میں تبدیل کریں
            # یہ عام طور پر Unity وژولائزیشن کے لیے پوز ڈیٹا بھیجے گا
            pass

    def unity_imu_callback(self, msg):
        """Unity IMU کو Gazebo کے لیے سیمولیشن کے لیے بھیجیں"""
        if self.imu_sync_enabled:
            # ایک حقیقی نافذ کاری میں، Unity IMU ڈیٹا کو Gazebo کے لیے
            # ویژول اور فزکل سیمولیشن کے درمیان مطابقت کے لیے بھیج سکتا ہے
            pass
```

### 2. کنٹرول کمانڈ تقسیم

```python
class ControlDistribution:
    def __init__(self, node):
        self.node = node

        # بلند سطح کمانڈز کے لیے سبسکرائبر
        self.cmd_sub = node.create_subscription(
            Twist, '/cmd_vel', self.command_callback, 10)

        # دونوں سیمولیٹرز کے لیے پبلشرز
        self.gazebo_cmd_pub = node.create_publisher(Twist, '/gazebo/cmd_vel', 10)
        self.unity_cmd_pub = node.create_publisher(Twist, '/unity/cmd_vel', 10)

        # کنٹرول پیرامیٹرز
        self.distribution_mode = "duplicate"  # ڈوپلیکیٹ یا فنکشن کے ذریعے تقسیم

    def command_callback(self, msg):
        """کمانڈز کو دونوں سیمولیٹرز میں تقسیم کریں"""
        if self.distribution_mode == "duplicate":
            # دونوں سیمولیٹرز کو ایک ہی کمانڈ بھیجیں
            self.gazebo_cmd_pub.publish(msg)
            self.unity_cmd_pub.publish(msg)
        elif self.distribution_mode == "function_split":
            # کمانڈ کے مختلف حصے مختلف سیمولیٹرز کو جاتے ہیں
            # مثلاً، لوکوموشن Gazebo کو، سر کی حرکت Unity کو
            self.distribute_by_function(msg)

    def distribute_by_function(self, cmd):
        """فنکشن کے مطابق کمانڈ تقسیم کریں"""
        # ترجمان حرکت کو Gazebo (فزکس) کو بھیجیں
        gazebo_cmd = Twist()
        gazebo_cmd.linear.x = cmd.linear.x
        gazebo_cmd.linear.y = cmd.linear.y
        gazebo_cmd.angular.z = cmd.angular.z  # موڑنا
        self.gazebo_cmd_pub.publish(gazebo_cmd)

        # سر کی حرکت کو Unity (وژولائزیشن) کو بھیجیں
        unity_cmd = Twist()
        unity_cmd.angular.x = cmd.angular.x  # گردن کا جھکاؤ
        unity_cmd.angular.y = cmd.angular.y  # گردن کا موڑ
        self.unity_cmd_pub.publish(unity_cmd)
```

## حقیقی دنیا کی نافذ کاری کی مثال

### ضم سیمولیشن کے لیے لانچ فائل

```xml
<!-- integrated_simulation.launch.py -->
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Gazebo سیمولیشن لانچ کریں
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': PathJoinSubstitution([
                FindPackageShare('humanoid_simulation'),
                'worlds',
                'humanoid_world.world'
            ])
        }.items()
    )

    # Unity سیمولیشن لانچ کریں (TCP کے ذریعے منسلک ہو گا)
    unity_node = Node(
        package='unity_simulation',
        executable='unity_bridge',
        name='unity_bridge',
        parameters=[
            {'unity_ip': '127.0.0.1'},
            {'unity_port': 10000}
        ]
    )

    # ہم وقت سازی نوڈ لانچ کریں
    sync_node = Node(
        package='simulation_integration',
        executable='simulation_synchronizer',
        name='simulation_synchronizer'
    )

    # سینسر ڈیٹا ایکسچینج لانچ کریں
    sensor_exchange_node = Node(
        package='simulation_integration',
        executable='sensor_exchange',
        name='sensor_exchange'
    )

    # کنٹرول تقسیم لانچ کریں
    control_dist_node = Node(
        package='simulation_integration',
        executable='control_distribution',
        name='control_distribution'
    )

    return LaunchDescription([
        gazebo_launch,
        unity_node,
        sync_node,
        sensor_exchange_node,
        control_dist_node
    ])
```

## وژولائزیشن ڈیٹا پائپ لائن

### Gazebo ڈیٹا کو Unity کے لیے تبدیل کرنا

```csharp
// Unity C# اسکرپٹ Gazebo سے ڈیٹا کو ROS-TCP-Connector کے ذریعے موصول کرنے کے لیے
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;

public class GazeboDataReceiver : MonoBehaviour
{
    ROSConnection ros;

    public Transform robotRoot;
    public Transform[] jointTransforms;
    public string[] jointNames;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<JointStateMsg>("/gazebo/joint_states", JointStateCallback);
    }

    void JointStateCallback(JointStateMsg jointState)
    {
        for (int i = 0; i < jointState.name.Length; i++)
        {
            string jointName = jointState.name[i];
            double position = jointState.position[i];

            // Unity ماڈل میں متعلقہ جوائنٹ تلاش کریں
            for (int j = 0; j < jointNames.Length; j++)
            {
                if (jointNames[j] == jointName && j < jointTransforms.Length)
                {
                    // Unity جوائنٹ پر پوزیشن لاگو کریں (ریڈینز کو ڈگریز میں تبدیل کریں)
                    jointTransforms[j].localEulerAngles = new Vector3(
                        0,
                        (float)(position * Mathf.Rad2Deg),
                        0
                    );
                    break;
                }
            }
        }

        // روبوٹ کی پوزیشن اور اورینٹیشن اپ ڈیٹ کریں
        // یہ ایک مکمل نافذ کاری میں الگ پوز ٹاپک سے آئے گا
    }
}
```

## کارکردگی کے غور

### بہتری کی حکمت عملیاں

1. **انتخابی ہم وقت سازی**: صرف ان حالتیں کو ہم وقت کریں جو ویژول مطابقت کے لیے اہم ہیں
2. **ڈیٹا کمپریشن**: کوائف کاری یا نمونہ دہی کے ذریعے ڈیٹا کا سائز کم کریں
3. **غیر ہم وقت پروسیسنگ**: بلاک کرنے والے عملوں کو روکنے کے لیے تھریڈنگ استعمال کریں
4. **کیش**: اکثر استعمال ہونے والے ٹرانسفارمیشن کو محفوظ کریں

```python
class OptimizedSynchronizer:
    def __init__(self):
        self.last_sync_values = {}  # آخری ہم وقت اقدار کو محفوظ کریں
        self.sync_threshold = 0.01  # صرف اس سے زیادہ تبدیلی ہونے پر ہم وقت کریں
        self.compression_factor = 4  # اپ ڈیٹ کی شرح کم کریں

    def should_sync_joint(self, joint_name, new_value):
        """تعین کریں کہ کیا جوائنٹ کی ہم وقت سازی کی ضرورت ہے"""
        if joint_name not in self.last_sync_values:
            return True

        old_value = self.last_sync_values[joint_name]
        return abs(new_value - old_value) > self.sync_threshold

    def compress_state(self, joint_state):
        """جوائنٹ حالت ڈیٹا کو کمپریس کریں"""
        compressed = JointState()
        compressed.header = joint_state.header

        # صرف ان جوائنٹس کو شامل کریں جن میں نمایاں تبدیلی آئی ہے
        for i, name in enumerate(joint_state.name):
            if i < len(joint_state.position):
                if self.should_sync_joint(name, joint_state.position[i]):
                    compressed.name.append(name)
                    compressed.position.append(joint_state.position[i])
                    self.last_sync_values[name] = joint_state.position[i]

        return compressed
```

## ملٹی پلیٹ فارم سیمولیشن کی ڈیبگنگ

### تشخیص کے ٹولز

```python
class SimulationDiagnostics(Node):
    def __init__(self):
        super().__init__('simulation_diagnostics')

        # تشخیصی ڈیٹا کے لیے پبلشرز
        self.diag_pub = self.create_publisher(String, '/simulation_diagnostics', 10)

        # تشخیصی ٹائمر
        self.diag_timer = self.create_timer(5.0, self.publish_diagnostics)

        # حالت ٹریکنگ
        self.gazebo_active = True
        self.unity_active = True
        self.sync_deviation = 0.0
        self.network_latency = 0.0

    def publish_diagnostics(self):
        """تشخیصی معلومات شائع کریں"""
        diag_msg = String()
        diag_msg.data = f"""
        سیمولیشن تشخیص:
        - Gazebo فعال: {self.gazebo_active}
        - Unity فعال: {self.unity_active}
        - ہم وقت کا انحراف: {self.sync_deviation:.3f} rad
        - نیٹ ورک لیٹنسی: {self.network_latency:.3f} ms
        - آخری ہم وقت: {time.time()}
        """

        self.diag_pub.publish(diag_msg)
```

## ضم سیمولیشن کے لیے استعمال کے معاملات

### 1. انسان-روبوٹ تعامل کے مطالعات

- Gazebo میں فزکس کے اعتبار سے درست روبوٹ رویہ
- Unity میں حقیقی انسانی اوتارز اور ماحول
- ادراک کے لیے Gazebo میں درست سینسر سیمولیشن
- انسانی شرکاء کے لیے Unity میں ہائی فائیڈلٹی رینڈرنگ

### 2. AI ماڈلز کی تربیت

- کنٹرول تربیت کے لیے فزکس کے اعتبار سے درست سیمولیشن (Gazebo)
- ادراک تربیت کے لیے ہائی فائیڈلٹی رینڈرنگ (Unity)
- دونوں پلیٹ فارمز سے حقیقی حقائق کے ساتھ مصنوعی ڈیٹا تخلیق

### 3. سسٹم انضمام ٹیسٹنگ

- دونوں سیمولیٹرز کے ساتھ مکمل روبوٹ سافٹ ویئر اسٹیک
- ادراک-ایکشن لوپ کی توثیق
- حقیقی انٹرفیسز کے ساتھ انسان-لوپ میں ٹیسٹنگ

## عام مسائل کا حل

1. **غیر مطابقت**: حالت کی پیشن گوئی اور اصلاح نافذ کریں
2. **نیٹ ورک لیٹنسی**: ڈیٹا ٹرانسمیشن کو بہتر بنائیں اور بفرنگ شامل کریں
3. **کارکردگی**: انتخابی ہم وقت سازی اور ڈیٹا کمپریشن استعمال کریں
4. **ڈیٹا فارمیٹ عدم مطابقت**: یقینی بنائیں کہ یونٹس اور کوآرڈینیٹ فریم مطابق ہیں

## خلاصہ

اس باب میں ہیومینائیڈ روبوٹ سیمولیشن کے لیے Gazebo اور Unity کا انضمام دیکھا گیا:

- ملٹی پلیٹ فارم سیمولیشن کے لیے معماری نقطہ نظر
- سیمولیٹرز کے درمیان ہم وقت سازی لیئرز کی نافذ کاری
- ڈیٹا تبادلہ اور کارکردگی کی بہتری کے لیے جدید تکنیکیں
- حقیقی دنیا کی نافذ کاری کی مثالیں اور استعمال کے معاملات
- ضم سیمولیشنز کے لیے ڈیبگنگ اور مسئلہ حل کرنے کی حکمت عملیاں

Gazebo اور Unity کا انضمام ہیومینائیڈ روبوٹ کی ترقی کے لیے ایک طاقتور پلیٹ فارم فراہم کرتا ہے، درست فزکس سیمولیشن کو ہائی فائیڈلٹی وژولائزیشن کے ساتھ جوڑتا ہے۔ یہ نقطہ نظر حقیقی مجازی ماحول میں ہیومینائیڈ روبوٹس کی مزید جامع ٹیسٹنگ اور ترقی کو فعال کرتا ہے۔

## مشقیں

1. دو سیمولیٹڈ روبوٹس کے درمیان ایک سادہ ہم وقت سازی نوڈ سیٹ کریں
2. Gazebo اور Unity کو ہم وقت سازی کے ساتھ شروع کرنے والی لانچ فائل تخلیق کریں
3. جوائنٹ حالت ڈیٹا کے لیے کمپریشن الگورتھم نافذ کریں

## اگلے اقدامات

فزکس سیمولیشن، سینسر ماڈلنگ، ہائی فائیڈلٹی رینڈرنگ، اور سیمولیشن انضمام کی مکمل سمجھ کے ساتھ، آپ اب اگلے ماڈیول میں NVIDIA Isaac ایکو سسٹم کی تلاش کے لیے تیار ہیں۔ Isaac GPU ایکسلریٹڈ ادراک اور AI صلاحیتیں فراہم کرتا ہے جو آپ کے تعمیر کردہ سیمولیشن بنیاد کو ساتھ دیتا ہے۔