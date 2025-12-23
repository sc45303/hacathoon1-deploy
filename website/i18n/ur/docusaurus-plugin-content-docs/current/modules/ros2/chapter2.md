---
sidebar_position: 3
---

# باب 2: ROS 2 نوڈز، ٹاپکس، اور سروسز

## سیکھنے کے مقاصد

- ROS 2 میں بنیادی رابطے کے نمونوں کو سمجھیں
- مخصوص فعالیت کے لیے ROS 2 نوڈز تخلیق کریں اور نافذ کریں
- ٹاپکس کا استعمال کرتے ہوئے پبلش/سبسکرائب رابطے کا ماڈل ماسٹر کریں
- سروسز کا استعمال کرتے ہوئے درخواست/جواب رابطے کو نافذ کریں
- ہیومینائیڈ روبوٹ سسٹمز پر رابطے کے نمونے لاگو کریں

## ROS 2 میں نوڈز

ایک نوڈ ایک ایگزیکیوٹیبل ہے جو دیگر نوڈز کے ساتھ رابطہ کرنے کے لیے ROS 2 کا استعمال کرتا ہے۔ نوڈز ROS 2 پروگرام کے بنیادی بلڈنگ بلاکس ہیں۔ ایک واحد سسٹم میں بہت سارے نوڈز ایک وقت میں چل سکتے ہیں، ہر ایک مخصوص کام انجام دیتا ہے۔

### نوڈ تخلیق کرنا

پائی تھون میں، ایک نوڈ کو `rclpy` سے `Node` کلاس کو بڑھا کر تخلیق کیا جاتا ہے:

```python
import rclpy
from rclpy.node import Node

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # سیکنڈ
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'ہیلو ورلڈ: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('شائع کرنا: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### نوڈ لائف سائیکل

ROS 2 نوڈز کی ایک اچھی طرح سے وضاحت شدہ لائف سائیکل ہوتی ہے جس میں شامل ہیں:
- **غیر ترتیب شدہ**: تخلیق کے بعد ابتدائی حالت
- **غیر فعال**: ترتیب شدہ لیکن فعال نہیں
- **فعال**: مکمل طور پر کارگر اور چل رہا ہے
- **حتمی**: حذف ہونے سے پہلے صاف کاری کا مرحلہ

## ٹاپکس اور میسیجز

ٹاپکس نامزد بسز ہیں جن کے ذریعے نوڈز میسیجز کا تبادلہ کرتے ہیں۔ میسیجز وہ ڈیٹا پیکٹس ہیں جو پبلشر نوڈز سے سبسکرائبر نوڈز تک ٹاپکس پر بھیجے جاتے ہیں۔ پبلش/سبسکرائب پیٹرن ROS 2 میں ایک بنیادی رابطے کا نمونہ ہے۔

### میسیج اقسام

ROS 2 `std_msgs` پیکیج میں معیاری میسیج کی اقسام کا امیر سیٹ فراہم کرتا ہے:
- `String`: متن کا ڈیٹا
- `Int32`, `Float32`: عددی ڈیٹا
- `Bool`: بولین ویلیوز
- `Header`: ٹائم اسٹیمپ اور فریم کی معلومات

علاوہ ازیں، `sensor_msgs`, `geometry_msgs`, اور `nav_msgs` جیسے پیکیجز میں مزید مخصوص میسیج کی اقسام ہیں۔

### ٹاپک پر شائع کرنا

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):

    def __init__(self):
        super().__init__('talker')
        self.publisher = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # سیکنڈ
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'ہیلو ورلڈ: %d' % self.i
        self.publisher.publish(msg)
        self.get_logger().info('شائع کرنا: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()
```

### ٹاپک کو سبسکرائب کرنا

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):

    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # غیر استعمال شدہ متغیر کی وارننگ کو روکنے کے لیے

    def listener_callback(self, msg):
        self.get_logger().info('میں نے سنا: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()
```

## ROS 2 میں سروسز

سسیز ROS 2 میں ایک درخواست/جواب رابطے کا نمونہ فراہم کرتی ہیں۔ ایک سروس کلائنٹ ایک درخواست کو سروس سرور پر بھیجتا ہے، جو درخواست کو پروسیس کرتا ہے اور جواب واپس بھیجتا ہے۔

### سروس اقسام

سسیز کی اقسام `.srv` فائلوں کا استعمال کرتے ہوئے وضاحت کی جاتی ہیں، جو درخواست اور جواب کے پیغامات کی وضاحت کرتی ہیں:

```
# درخواست کا پیغام
string name
int32 age
---
# جواب کا پیغام
bool success
string message
```

### سروس سرور تخلیق کرنا

```python
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('آنے والی درخواست\na: %d b: %d' % (request.a, request.b))
        return response

def main():
    rclpy.init()
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()
```

### سروس کلائنٹ تخلیق کرنا

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):

    def __init__(self):
        super().__init__('minimal_client')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('سروس دستیاب نہیں ہے، دوبارہ انتظار کر رہا ہے...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main():
    rclpy.init()
    minimal_client = MinimalClient()
    response = minimal_client.send_request(1, 2)
    minimal_client.get_logger().info(
        'add_two_ints کا نتیجہ: %d + %d = %d' %
        (1, 2, response.sum))
    minimal_client.destroy_node()
    rclpy.shutdown()
```

## ROS 2 میں کوالٹی آف سروس (QoS)

QoS پروفائلز آپ کو یہ کنفیگر کرنے کی اجازت دیتے ہیں کہ پبلشرز اور سبسکرائبرز کے درمیان میسیجز کیسے ترسیل کیے جاتے ہیں۔ یہ ریئل ٹائم سسٹمز اور قابل اعتماد رابطے کے لیے اہم ہے:

```python
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy

# ریئل ٹائم کارکردگی کے لیے QoS پروفائل تخلیق کریں
qos_profile = QoSProfile(
    depth=10,
    history=QoSHistoryPolicy.RMW_QOS_HISTORY_POLICY_KEEP_LAST,
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE
)

publisher = self.create_publisher(String, 'topic', qos_profile)
```

## ہیومینائیڈ روبوٹکس میں اطلاق

ہیومینائیڈ روبوٹکس میں، ROS 2 رابطے کے نمونوں کا وسیع استعمال کیا جاتا ہے:

### جوائنٹ کنٹرول

- **ٹاپکس**: جوائنٹ اسٹیٹس زیادہ فریکوئنسی پر شائع کیے جاتے ہیں
- **سسیز**: کیلبریشن رoutines، موڈ سوئچنگ
- **ایکشنز**: پیچیدہ حرکتیں جن میں مکمل ہونے میں وقت لگتا ہے

### سینسر انضمام

- **ٹاپکس**: کیمرہ امیجز، IMU ڈیٹا، فورس/ٹورک سینسرز
- **سسیز**: سینسر کنفیگریشن، کیلبریشن
- **ایکشنز**: نقشہ کاری جیسے طویل وقت تک چلنے والے سینسر ٹاسکس

### نیویگیشن

- **ٹاپکس**: اوڈومیٹری، لیزر اسکینز، کوسٹ میپس
- **سسیز**: گلوبل منصوبہ بندی، کوسٹ میپ اپ ڈیٹس
- **ایکشنز**: راستہ فالو کرنا، نیویگیشن گولز

## رابطے کے لیے ROS 2 ٹولز

### ros2 topic

```bash
# تمام ٹاپکس کو فہرست بند کریں
ros2 topic list

# ٹاپک پر میسیجز کو ایکو کریں
ros2 topic echo /chatter std_msgs/msg/String

# ٹاپک پر ایک میسیج شائع کریں
ros2 topic pub /chatter std_msgs/msg/String "data: ہیلو"
```

### ros2 service

```bash
# تمام سروسز کو فہرست بند کریں
ros2 service list

# ایک سروس کو کال کریں
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 1, b: 2}"
```

## مثال: سادہ ہیومینائیڈ کنٹرول نوڈ

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class HumanoidController(Node):

    def __init__(self):
        super().__init__('humanoid_controller')

        # پبلشرز
        self.joint_cmd_publisher = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        # سبسکرائبرز
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # کمانڈ سبسکرائبر
        self.command_subscriber = self.create_subscription(
            String,
            '/humanoid_commands',
            self.command_callback,
            10
        )

        # موجودہ جوائنٹ اسٹیٹس کو محفوظ کریں
        self.current_joint_states = JointState()

        self.get_logger().info('ہیومینائیڈ کنٹرولر شروع کیا گیا')

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f'کمانڈ موصول ہوئی: {command}')

        if command == 'wave':
            self.execute_wave_motion()
        elif command == 'stand':
            self.move_to_standing_position()

    def execute_wave_motion(self):
        # ہاتھ ہلانے کے لیے ایک جوائنٹ ٹریجکٹری تخلیق کریں
        trajectory = JointTrajectory()
        trajectory.joint_names = ['right_shoulder_roll', 'right_elbow_pitch']

        # ٹریجکٹری پوائنٹس تخلیق کریں
        point1 = JointTrajectoryPoint()
        point1.positions = [0.0, 0.0]  # غیر جانبدار پوزیشن
        point1.time_from_start.sec = 1
        trajectory.points.append(point1)

        point2 = JointTrajectoryPoint()
        point2.positions = [0.5, -0.5]  # ہاتھ ہلانے کی پوزیشن
        point2.time_from_start.sec = 2
        trajectory.points.append(point2)

        point3 = JointTrajectoryPoint()
        point3.positions = [0.0, 0.0]  # غیر جانبدار پوزیشن پر واپس
        point3.time_from_start.sec = 3
        trajectory.points.append(point3)

        # ٹریجکٹری شائع کریں
        self.joint_cmd_publisher.publish(trajectory)

    def move_to_standing_position(self):
        # ایک متعین کردہ کھڑے ہونے کی پوزیشن پر جائیں
        trajectory = JointTrajectory()
        trajectory.joint_names = [
            'left_hip_pitch', 'left_knee_pitch', 'left_ankle_pitch',
            'right_hip_pitch', 'right_knee_pitch', 'right_ankle_pitch'
        ]

        point = JointTrajectoryPoint()
        point.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # کھڑے ہونے کی پوزیشن
        point.time_from_start.sec = 2

        trajectory.points.append(point)
        self.joint_cmd_publisher.publish(trajectory)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## خلاصہ

نوڈز، ٹاپکس، اور سروسز ROS 2 سسٹمز میں رابطے کی بنیاد تشکیل دیتے ہیں۔ ان تصورات کو سمجھنا ہیومینائیڈ روبوٹس جیسے پیچیدہ روبوٹک سسٹمز تیار کرنے کے لیے ضروری ہے۔ پبلش/سبسکرائب ماڈل سینسر ڈیٹا اور اسٹیٹ اپ ڈیٹس کے لیے بہترین ہے، جبکہ سروسز درخواست/جواب بات چیت کے لیے بہترین ہیں۔ کوالٹی آف سروس کی ترتیبات کا مناسب استعمال وقت کے لحاظ سے اہم ایپلی کیشنز کے لیے قابل اعتماد رابطہ یقینی بناتا ہے۔

## مشقیں

1. ایک پبلشر نوڈ تخلیق کریں جو 50Hz پر جوائنٹ کمانڈز شائع کرتا ہے
2. ایک سبسکرائبر نوڈ تخلیق کریں جو IMU ڈیٹا سنتا ہے اور اورینٹیشن لاگ کرتا ہے
3. ایک سروس نافذ کریں جو ہدف کی پوزیشن لیتا ہے اور ایک ٹریجکٹری منصوبہ بندی کرتا ہے

## اگلے اقدامات

اگلے باب میں، ہم rclpy کا استعمال کرتے ہوئے پائی تھون AI ایجنٹس کو ROS کنٹرولرز سے جوڑنے کی تلاش کریں گے، AI سسٹمز کو جسمانی روبوٹ کنٹرول کے ساتھ جوڑنا۔