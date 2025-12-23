---
sidebar_position: 3
---

# باب 2: سینسرز اور ماحول کی تعمیر

## سیکھنے کے مقاصد

- Gazebo میں مختلف روبوٹ سینسرز کو کیسے شبیہ کرنا سمجھیں
- روبوٹ ٹیسٹنگ کے لیے حقیقی ماحول تخلیق کریں
- مناسب پیرامیٹرز کے ساتھ سینسر ماڈلز کو تشکیل دیں
- شبیہ کاری شدہ سینسرز کو ROS 2 سسٹمز کے ساتھ ضم کریں
- ہیومینائیڈ روبوٹ کے اطلاق کے لیے سینسر ڈیٹا کی توثیق کریں

## روبوٹکس میں سینسر شبیہ کاری

سینسر شبیہ کاری روبوٹ کی ترقی کا ایک اہم جزو ہے، جو ادراک الگورتھم کی محفوظ اور قیمت مؤثر ٹیسٹنگ کی اجازت دیتا ہے۔ ہیومینائیڈ روبوٹکس میں، درست سینسر شبیہ کاری خاص طور پر اہم ہے کیونکہ روبوٹ اور اس کے ماحول کے درمیان پیچیدہ تعامل ہوتا ہے۔

### ہیومینائیڈ روبوٹس میں سینسرز کی اقسام

ہیومینائیڈ روبوٹس عام طور پر ان سینسر اقسام کا استعمال کرتے ہیں:

1. **پروپریوسیپٹو سینسرز**: روبوٹ کی اندرونی حالت کو ناپیں
   - جوائنٹ انکوڈرز: جوائنٹس کی پوزیشن، رفتار
   - IMUs: اورینٹیشن، زاویہ وار رفتار، تیزی
   - فورس/ٹورک سینسرز: جوائنٹس اور اینڈ ایفیکٹرز پر قوتیں

2. **ایکسٹروپریوسیپٹو سینسرز**: ماحول کو ناپیں
   - کیمرز: وژول ادراک
   - LIDAR: نیویگیشن کے لیے فاصلہ پیمائش
   - سونار: اضافی فاصلہ سینسنگ
   - ٹیکٹائل سینسرز: رابطہ کا پتہ لگانا

## Gazebo سینسر پلگ ان

Gazebo مختلف سینسرز کی شبیہ کاری کے لیے پلگ ان فراہم کرتا ہے۔ یہ پلگ ان روبوٹ الگورتھم کے ذریعے پروسیس کیے جانے والے ROS ٹاپکس پر ڈیٹا شائع کرتے ہیں۔

### کیمرہ سینسرز

Gazebo میں کیمرہ سینسرز RGB کیمرز کی شبیہ کاری کرتے ہیں اور ROS ٹاپکس پر تصاویر شائع کرتے ہیں:

```xml
<!-- URDF/SDF میں کیمرہ سینسر کی مثال -->
<sensor name="camera" type="camera">
  <camera name="head">
    <horizontal_fov>1.089</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R8G8B8</format>
    </image>
    <clip>
      <near>0.1</near>
      <far>100</far>
    </clip>
  </camera>
  <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
    <frame_name>camera_frame</frame_name>
    <topic_name>image_raw</topic_name>
  </plugin>
</sensor>
```

### LIDAR سینسرز

LIDAR (لائٹ ڈیٹیکشن اینڈ رینجنگ) سینسرز لیزر رینج فائنڈرز کی شبیہ کاری کرتے ہیں:

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_laser.so">
    <topic_name>scan</topic_name>
    <frame_name>lidar_frame</frame_name>
  </plugin>
</sensor>
```

### IMU سینسرز

IMU (انرٹیل میزورمینٹ یونٹ) سینسرز اورینٹیشن اور تیزی کا ڈیٹا فراہم کرتے ہیں:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </x>
      <y>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </y>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>1.7e-2</stddev>
        </noise>
      </z>
    </linear_acceleration>
  </imu>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <topic_name>imu</topic_name>
    <body_name>imu_link</body_name>
    <frame_name>imu_link</frame_name>
  </plugin>
</sensor>
```

## Gazebo میں ماحول کی تعمیر

حقیقی ماحول تخلیق کرنا معنی خیز روبوٹ ٹیسٹنگ کے لیے انتہائی اہم ہے۔ Gazebo ماحول تخلیق کرنے کے لیے کئی طریقے فراہم کرتا ہے:

### دنیا کی فائلیں

دنیا کی فائلیں مکمل سیمولیشن ماحول کی وضاحت کرتی ہیں:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="small_room">
    <!-- سورج کو شامل کریں -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- زمین کے پلیٹ فارم کو شامل کریں -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- فرنیچر شامل کریں -->
    <model name="table">
      <pose>-1 0 0 0 0 0</pose>
      <include>
        <uri>model://table</uri>
      </include>
    </model>

    <model name="chair">
      <pose>-1.5 0.5 0 0 0 1.57</pose>
      <include>
        <uri>model://chair</uri>
      </include>
    </model>

    <!-- مینوپولیشن کے لیے اشیاء شامل کریں -->
    <model name="box">
      <pose>-0.8 0.3 0.5 0 0 0</pose>
      <link name="box_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.1 0.1 0.1</size>
            </box>
          </geometry>
          <material>
            <ambient>1 0 0 1</ambient>
            <diffuse>1 0 0 1</diffuse>
          </material>
        </visual>
        <inertial>
          <mass>0.1</mass>
          <inertia>
            <ixx>0.0001</ixx>
            <iyy>0.0001</iyy>
            <izz>0.0001</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

### پیچیدہ ماحول تخلیق کرنا

ہیومینائیڈ روبوٹس کے لیے، ماحول میں شامل ہونا چاہیے:

- **نیویگیشن علاقے**: چلنے کے لیے کھلی جگہیں، راستے
- **رکاوٹیں**: فرنیچر، دیواریں، دیگر اشیاء جن کے گرد نیویگیٹ کرنا ہو
- **تعامل کی اشیاء**: مینوپولیشن ٹاسکس کے لیے اشیاء
- **مارکرز/لینڈ مارکس**: لوکلائزیشن اور میپنگ کے لیے اشیاء
- **متغیر زمین**: مختلف فرش کے مواد، ہلکی ڈھلوان، سیڑھیاں (اعلیٰ درجے کے روبوٹس کے لیے)

## ROS 2 کے ساتھ سینسر انضمام

ایک بار جب سینسرز Gazebo میں تشکیل دیے جاتے ہیں، انہیں ROS 2 سسٹمز کے ساتھ ضم کرنے کی ضرورت ہوتی ہے:

### کیمرہ ڈیٹا پروسیسنگ نوڈ

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraProcessor(Node):
    def __init__(self):
        super().__init__('camera_processor')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.subscription  # غیر استعمال شدہ متغیر کی وارننگ سے بچنے کے لیے
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # ROS تصویر کا پیغام OpenCV تصویر میں تبدیل کریں
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # تصویر کو پروسیس کریں (مثال: کنارے ڈیٹیکٹ کریں)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 50, 150)

        # نتیجہ ڈسپلے کریں
        cv2.imshow("Camera View", cv_image)
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_processor = CameraProcessor()
    rclpy.spin(camera_processor)
    cv2.destroyAllWindows()
    camera_processor.destroy_node()
    rclpy.shutdown()
```

### LIDAR پروسیسنگ

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # غیر استعمال شدہ متغیر کی وارننگ سے بچنے کے لیے

    def scan_callback(self, msg):
        # LIDAR ڈیٹا پروسیس کریں
        # زیادہ آسان پروسیسنگ کے لیے numpy ارے میں تبدیل کریں
        ranges = np.array(msg.ranges)

        # کم از کم فاصلہ تلاش کریں (قریب ترین رکاوٹ)
        valid_ranges = ranges[np.isfinite(ranges)]  # غلط (inf) ویلیوز ہٹا دیں
        if len(valid_ranges) > 0:
            min_distance = np.min(valid_ranges)
            self.get_logger().info(f'قریب ترین رکاوٹ: {min_distance:.2f}م')

        # سادہ رکاوٹ ڈیٹیکشن
        threshold = 1.0  # میٹر
        obstacles = valid_ranges < threshold
        obstacle_count = np.sum(obstacles)
        if obstacle_count > 0:
            self.get_logger().info(f'{threshold}م کے اندر {obstacle_count} رکاوٹیں ملیں')

def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    rclpy.spin(lidar_processor)
    lidar_processor.destroy_node()
    rclpy.shutdown()
```

## ہیومینائیڈ روبوٹ مخصوص سینسرز

ہیومینائیڈ روبوٹس کے لیے منفرد سینسر کی ضروریات ہوتی ہیں:

### بیلنس سینسرز

- **ZMP (زیرو مومینٹ پوائنٹ) سینسرز**: بائی پیڈل سٹیبیلٹی کے لیے انتہائی اہم
- **فورس پلیٹس**: زمین کی ردعمل کی قوتوں کو ناپیں
- **فٹ کنٹیکٹ سینسرز**: جب پاؤں زمین سے رابطہ کریں تو ڈیٹکٹ کریں

### مینوپولیشن سینسرز

- **ٹیکٹائل سینسرز**: اشیاء کو مینوپولیٹ کرنے کے لیے انگلیوں پر
- **فورس/ٹورک سینسرز**: رابطے کی قوتوں کو ناپنے کے لیے ہتھیلیوں میں
- **سٹیریو کیمرز**: مینوپولیشن کے دوران گہرائی کے ادراک کے لیے

## سینسر کی توثیق

شبیہ کاری شدہ سینسرز کی توثیق انتہائی اہم ہے:

1. **حقیقی سینسرز کے ساتھ موازنہ کریں**: جب ممکن ہو، شبیہ کاری شدہ سینسر ڈیٹا کو حقیقی ہارڈ ویئر کے ساتھ موازنہ کریں
2. **فزکس مطابقت**: یقینی بنائیں کہ سینسر کے مطالعات شبیہ کاری شدہ فزکس کے دی گئی منطق پر مبنی ہیں
3. **وقت کی درستی**: تصدیق کریں کہ سینسر صحیح شرح پر شائع کرتے ہیں
4. **شور کی خصوصیات**: حقیقی شور ماڈلز کو یقینی بنائیں

## مثال: ہیومینائیڈ روبوٹ کے لیے مکمل سینسر سیٹ اپ

```xml
<!-- متعدد سینسرز والے لنک کی مثال -->
<link name="head">
  <visual>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <sphere radius="0.1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.0083" ixy="0" ixz="0" iyy="0.0083" iyz="0" izz="0.0083"/>
  </inertial>

  <!-- سر کا کیمرہ -->
  <sensor name="head_camera" type="camera">
    <pose>0.05 0 0 0 0 0</pose>
    <camera name="head">
      <horizontal_fov>1.089</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>head_camera_frame</frame_name>
      <topic_name>head_camera/image_raw</topic_name>
    </plugin>
  </sensor>

  <!-- سر میں IMU -->
  <sensor name="head_imu" type="imu">
    <pose>0 0 0 0 0 0</pose>
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <topic_name>imu/head</topic_name>
      <frame_name>head_imu_frame</frame_name>
    </plugin>
  </sensor>
</link>
```

## عام مسائل کا حل

### سینسر شائع نہیں کر رہا
- چیک کریں کہ کیا پلگ ان صحیح طریقے سے لوڈ ہوا ہے
- ٹاپک کے ناموں اور نیمسپیس کی تصدیق کریں
- یقینی بنائیں کہ ماڈل میں سینسر کو پاور/کنیکشنز ہیں

### غلط سینسر ڈیٹا
- ماڈل میں سینسر کی جگہ کی تصدیق کریں
- کوآرڈینیٹ فریم ٹرانسفارمیشن چیک کریں
- سینسر پیرامیٹرز (FOV، رینج، وغیرہ) کی توثیق کریں

### کارکردگی کے مسائل
- اگر ضرورت نہ ہو تو سینسر اپ ڈیٹ کی شرح کم کریں
- کیمرز کے لیے امیج ریزولوشن کم کریں
- اگر درستی اجازت دیتی ہے تو کم LIDAR ریز استعمال کریں

## خلاصہ

سینسر شبیہ کاری ہیومینائیڈ روبوٹس کی ترقی اور ٹیسٹنگ کو محفوظ اور کارآمد طریقے سے کرنے کے لیے انتہائی ضروری ہے۔ سینسر ماڈلز کی مناسب تشکیل، ROS 2 سسٹمز کے ساتھ انضمام، اور سینسر ڈیٹا کی توثیق اہم اقدامات ہیں حقیقی سیمولیشنز تخلیق کرنے کے لیے۔ ماحول جو آپ تخلیق کریں گے وہ اس پیچیدگی کا مطابق ہونا چاہیے جس کا سامنا آپ کا ہیومینائیڈ روبوٹ حقیقی دنیا کے منظر ناموں میں کرے گا۔

## مشقیں

1. اپنے شبیہ کاری شدہ روبوٹ میں ایک کیمرہ سینسر شامل کریں اور آؤٹ پٹ وژولائز کریں
2. نیویگیشن ٹیسٹنگ کے لیے رکاوٹوں والے سادہ ماحول تخلیق کریں
3. بنیادی LIDAR رکاوٹ ڈیٹیکشن نوڈ نافذ کریں

## اگلے اقدامات

اگلے باب میں، ہم Unity کا استعمال کرتے ہوئے ہائی فائیڈلٹی رینڈرنگ اور انسان-روبوٹ تعامل کی تلاش کریں گے، روبوٹ سیمولیشن اور وژولائزیشن کے بارے میں ایک مختلف نظریہ فراہم کرتے ہوئے۔