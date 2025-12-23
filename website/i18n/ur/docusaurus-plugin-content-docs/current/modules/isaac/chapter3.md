---
sidebar_position: 4
---

# باب 3: بائی پیڈل روبوٹس کے لیے Nav2 راستہ منصوبہ بندی

## سیکھنے کے مقاصد

- بائی پیڈل ہیومینائیڈ روبوٹس کے لیے راستہ منصوبہ بندی کے منفرد چیلنجز کو سمجھیں
- ہیومینائیڈ مخصوص نیویگیشن کے لیے Nav2 کنفیگریشن سیکھیں
- بائی پیڈل گیٹ کنٹرینٹس کو مدنظر رکھتے ہوئے متحرک راستہ منصوبہ بندی نافذ کریں
- گلوبل اور مقامی پلانرز کو بائی پیڈل لوکوموشن کے لیے اڈاپٹ کریں
- Nav2 نیویگیشن کے ساتھ فٹ سٹیپ منصوبہ بندی کو مربوط کریں

## بائی پیڈل نیویگیشن چیلنجز کا تعارف

ہیومینائیڈ روبوٹس کے لیے نیویگیشن کے مقابلے میں وہیلڈ روبوٹس کے لیے نیویگیشن کے منفرد چیلنجز ہیں۔ بائی پیڈل لوکوموشن کی ضرورت ہے:

1. **متحرک بیلنس**: دو ٹانگوں پر چلتے ہوئے بیلنس برقرار رکھنا
2. **فٹ سٹیپ منصوبہ بندی**: مسلسل راستوں کے بجائے الگ الگ قدم
3. **ZMP (Zero Moment Point) غور**: ڈائنامک سٹیبیلٹی کو یقینی بنانا
4. **ٹیرین اڈاپٹیشن**: ناہموار سطح، سیڑھیاں، رکاوٹیں کو ہینڈل کرنا
5. **کولیژن ایوائیڈنس**: بیلنس برقرار رکھتے ہوئے رکاوٹوں سے بچنا

### وہیلڈ روبوٹ نیویگیشن سے فرق

روایتی نیویگیشن سسٹمز مسلسل، ہموار موومنٹ کا ا assumption کرتے ہیں۔ بائی پیڈل روبوٹس کی ضرورت ہے:

- **ڈسکریٹ راستہ نمائندگی**: راستے کو قدم کی ترتیب کے طور پر
- **سٹیبیلٹی کنٹرینٹس**: ہر قدم پر بیلنس برقرار رکھنا
- **متحرک رکاوٹ سے بچنا**: حقیقی وقت میں گیٹ ایڈجسٹ کرنا
- **ٹیرین کلاسیفکیشن**: مختلف سطحوں کے لیے مختلف چلنے کے نمونے

## Nav2 آرکیٹیکچر کا جائزہ

Nav2 میں کئی اہم اجزاء ہیں جنہیں بائی پیڈل نیویگیشن کے لیے اڈاپٹ کرنے کی ضرورت ہے:

```
[گلوبل پلانر] → [کنٹرولر سرور] → [مقامی پلانر] → [روبوٹ کنٹرولر]
      ↑                    ↑                   ↑
[کوسٹ میپ] ← → [کوسٹ میپ] ← → [کوسٹ میپ]
```

بائی پیڈل روبوٹس کے لیے، ان اجزاء کو مخصوص اڈاپٹیشن کی ضرورت ہے:

### گلوبل پلانر اڈاپٹیشن

گلوبل پلانر کو مندرجہ ذیل کو مدنظر رکھنا چاہیے:

- **قدم سے قدم کنکٹیویٹی**: مسلسل راستوں کے بجائے
- **سٹیبیلٹی زونز**: علاقوں جہاں روبوٹ محفوظ طریقے سے قدم رکھ سکے
- **گیٹ کنٹرینٹس**: مختلف چلنے کی رفتار کے لیے مخصوص نمونے
- **ٹیرین کلاسیفکیشن**: مختلف سطح کی خصوصیات کے لیے اڈاپٹ کرنا

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import numpy as np

class BipedalGlobalPlanner(Node):
    def __init__(self):
        super().__init__('bipedal_global_planner')

        # راستہ کمپیوٹیشن کے لیے ایکشن سرور
        self.action_server = self.create_action_server(
            'compute_path_to_pose',
            ComputePathToPose,
            self.execute_path_request
        )

        # کوسٹ میپ سبسکرپشن
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,  # سادہ نوعیت کی نمائندگی
            '/global_costmap/costmap',
            self.costmap_callback,
            10
        )

        self.get_logger().info('Bipedal گلوبل پلانر شروع کیا گیا')

    def execute_path_request(self, goal_handle):
        """بائی پیڈل کنٹرینٹس کے ساتھ راستہ منصوبہ بندی کی درخواست کریں"""
        start = goal_handle.request.start
        goal = goal_handle.request.goal

        # بائی پیڈل کنٹرینٹس کو مدنظر رکھتے ہوئے راستہ منصوبہ بندی کریں
        path = self.plan_path_with_constraints(start, goal)

        if path is not None:
            # منصوبہ بندی شدہ راستہ واپس کریں
            result = ComputePathToPose.Result()
            result.path = path
            goal_handle.succeed()
            return result
        else:
            # منصوبہ بندی ناکام ہو گئی
            goal_handle.abort()
            return ComputePathToPose.Result()

    def plan_path_with_constraints(self, start, goal):
        """بائی پیڈل لوکوموشن کنٹرینٹس کو مدنظر رکھتے ہوئے راستہ منصوبہ بندی کریں"""
        # یہ ایک سادہ نافذ کاری ہے
        # حقیقت میں، یہ فٹ سٹیپ پلینرز کے ساتھ انٹرفیس کرے گا
        path = Path()
        path.header = Header()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        # بائی پیڈل روبوٹس کے لیے، ہمیں فٹ سٹیپ مقامات کی منصوبہ بندی کرنے کی ضرورت ہے
        # یہ ایک سادہ مثال ہے جو ایک سیدھا راستہ تخلیق کرتا ہے
        steps = self.generate_footsteps(start.pose, goal.pose)

        for step in steps:
            pose_stamped = PoseStamped()
            pose_stamped.header = path.header
            pose_stamped.pose = step
            path.poses.append(pose_stamped)

        return path

    def generate_footsteps(self, start_pose, goal_pose):
        """بائی پیڈل نیویگیشن کے لیے ڈسکریٹ فٹ سٹیپس تخلیق کریں"""
        # سیدھی لکیر راستہ کا حساب لگائیں
        dx = goal_pose.position.x - start_pose.position.x
        dy = goal_pose.position.y - start_pose.position.y
        distance = np.sqrt(dx*dx + dy*dy)

        # روبوٹ کے پیرامیٹرز کی بنیاد پر قدم کا سائز متعین کریں
        step_length = 0.3  # میٹر - ہیومینائیڈ کے لیے عام
        step_width = 0.2   # میٹر - بیلنس کے لیے قدم کی چوڑائی

        # فٹ سٹیپس تخلیق کریں
        footsteps = []
        num_steps = max(1, int(distance / step_length))

        for i in range(num_steps + 1):
            ratio = i / num_steps if num_steps > 0 else 0

            step_pose = Pose()
            step_pose.position.x = start_pose.position.x + dx * ratio
            step_pose.position.y = start_pose.position.y + dy * ratio
            step_pose.position.z = start_pose.position.z  # اونچائی برقرار رکھیں

            # الٹرنیٹ فٹ کے لیے چھوٹا آفسیٹ شامل کریں
            if i % 2 == 0:  # بائیں فٹ
                step_pose.position.y += step_width / 2
            else:  # دائیں فٹ
                step_pose.position.y -= step_width / 2

            # اورینٹیشن - گول کی طرف منہ
            yaw = np.arctan2(dy, dx)
            step_pose.orientation = self.yaw_to_quaternion(yaw)

            footsteps.append(step_pose)

        return footsteps

    def yaw_to_quaternion(self, yaw):
        """yaw اینگل کو کوارٹرنین میں تبدیل کریں"""
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        from geometry_msgs.msg import Quaternion
        quat_msg = Quaternion()
        quat_msg.x = q[0]
        quat_msg.y = q[1]
        quat_msg.z = q[2]
        quat_msg.w = q[3]
        return quat_msg
```

## بائی پیڈل روبوٹس کے لیے مقامی پلانر

بائی پیڈل روبوٹس کا مقامی پلانر:

- **مسلسل رفتار کمانڈز** کے بجائے **فٹ سٹیپ ٹریجکٹریز** تیار کرنا
- **رکاوٹوں سے بچاؤ** کے دوران **متحرک بیلنس** برقرار رکھنا
- **حقیقی وقت میں گیٹ پیٹرنز** ایڈجسٹ کرنا
- **اچانک رکاوٹوں** کے لیے **ری ایکٹو سٹیپنگ** ہینڈل کرنا

```python
from geometry_msgs.msg import Twist, Pose, Point
from nav_msgs.msg import Path
import math

class BipedalLocalPlanner(Node):
    def __init__(self):
        super().__init__('bipedal_local_planner')

        # سبسکرائبرز
        self.path_sub = self.create_subscription(
            Path,
            '/global_plan',
            self.global_path_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,  # سادہ نوعیت
            '/odom',
            self.odom_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,  # سادہ نوعیت
            '/scan',
            self.scan_callback,
            10
        )

        # فٹ سٹیپ کمانڈز کے لیے پبلشرز
        self.footstep_pub = self.create_publisher(
            String,  # حقیقت میں، یہ ایک کسٹم فٹ سٹیپ میسیج ہوگا
            '/footstep_commands',
            10
        )

        # انٹرنل اسٹیٹ
        self.current_path = None
        self.current_pose = None
        self.current_velocity = Twist()
        self.path_index = 0

        # بائی پیڈل مخصوص پیرامیٹرز
        self.step_duration = 0.8  # سیکنڈ فی قدم
        self.max_step_adjustment = 0.1  # میٹر
        self.balance_margin = 0.3  # بیلنس کے لیے سیفٹی مارجن

        self.get_logger().info('Bipedal مقامی پلانر شروع کیا گیا')

    def global_path_callback(self, path_msg):
        """نئے گلوبل راستہ کو ہینڈل کریں"""
        self.current_path = path_msg
        self.path_index = 0
        self.get_logger().info(f'نیا راستہ موصول ہوا {len(path_msg.poses)} قدم کے ساتھ')

    def odom_callback(self, odom_msg):
        """موجودہ روبوٹ پوز اپ ڈیٹ کریں"""
        self.current_pose = odom_msg.pose.pose
        self.current_velocity = odom_msg.twist.twist

        # موجودہ اسٹیٹ کی بنیاد پر اگلے قدموں کی منصوبہ بندی کریں
        if self.current_path:
            self.plan_next_footsteps()

    def scan_callback(self, scan_msg):
        """مقامی رکاوٹ کا پتہ لگانے کے لیے لیزر اسکین ہینڈل کریں"""
        # راستہ میں رکاوٹوں کی جانچ کریں
        min_range = min(scan_msg.ranges)
        if min_range < self.balance_margin:
            self.handle_obstacle_avoidance()

    def plan_next_footsteps(self):
        """گلوبل راستہ کی بنیاد پر اگلے چند قدموں کی منصوبہ بندی کریں"""
        if not self.current_path or self.path_index >= len(self.current_path.poses):
            return

        # موجودہ پوزیشن حاصل کریں
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y

        # اگلے فٹ سٹیپس کا تعین کریں
        footsteps = []
        for i in range(self.path_index, min(self.path_index + 3, len(self.current_path.poses))):
            target_pose = self.current_path.poses[i].pose

            # رکاوٹوں سے بچاؤ کے ساتھ مطلوبہ قدم کا حساب لگائیں
            adjusted_pose = self.adjust_footstep_for_obstacles(
                current_x, current_y,
                target_pose.position.x, target_pose.position.y
            )

            # چیک کریں کہ قدم ڈائنامکلی ممکن ہے
            if self.is_step_feasible(adjusted_pose):
                footsteps.append(adjusted_pose)

        # فٹ سٹیپ کمانڈز شائع کریں
        if footsteps:
            self.publish_footsteps(footsteps)

    def adjust_footstep_for_obstacles(self, current_x, current_y, target_x, target_y):
        """رکاوٹوں سے بچاؤ کے لیے فٹ سٹیپ ایڈجسٹ کریں"""
        # مطلوبہ قدم کا حساب لگائیں
        dx = target_x - current_x
        dy = target_y - current_y

        # راستہ کے ساتھ رکاوٹوں کی جانچ کریں
        # یہ ایک سادہ ورژن ہے
        step_pose = Pose()
        step_pose.position.x = target_x
        step_pose.position.y = target_y
        step_pose.position.z = 0.0  # زمین کی سطح

        # اورینٹیشن شامل کریں
        yaw = math.atan2(dy, dx)
        step_pose.orientation = self.yaw_to_quaternion(yaw)

        return step_pose

    def is_step_feasible(self, footstep_pose):
        """چیک کریں کہ کیا ایک فٹ سٹیپ ڈائنامکلی ممکن ہے"""
        # چیک کریں کہ قدم بیلنس حدود کے اندر ہے
        # فٹ سٹیپ مقام پر رکاوٹوں کی جانچ کریں
        # زمین کی سٹیبیلٹی کی جانچ کریں

        # سادہ قابلیت کی جانچ
        # حقیقت میں، یہ ZMP کیلکولیشنز اور زمین کے تجزیہ میں شامل ہوگا
        return True

    def handle_obstacle_avoidance(self):
        """مقامی رکاوٹ سے بچاؤ کو ہینڈل کریں"""
        # بائی پیڈل روبوٹس کے لیے، اس میں یہ شامل ہو سکتا ہے:
        # - اگلے چند قدموں کو ایڈجسٹ کرنا
        # - گیٹ پیرامیٹرز کو تبدیل کرنا
        # - ضرورت پڑنے پر عارضی طور پر رکنا

        self.get_logger().info('رکاوٹ کا پتہ چلا، راستہ ایڈجسٹ کیا جا رہا ہے')
        # نافذ کاری مخصوص روبوٹ اور سینسر سیٹ اپ پر منحصر ہوگی

    def publish_footsteps(self, footsteps):
        """منصوبہ بندی شدہ فٹ سٹیپس کو روبوٹ کنٹرولر میں شائع کریں"""
        for i, step in enumerate(footsteps):
            step_cmd = String()
            step_cmd.data = f"قدم {self.path_index + i}: ({step.position.x:.2f}, {step.position.y:.2f})"
            self.footstep_pub.publish(step_cmd)

        # راستہ اشاریہ اپ ڈیٹ کریں
        self.path_index += len(footsteps)
```

## کنٹرولر سرور اڈاپٹیشنز

بائی پیڈل روبوٹس کے لیے کنٹرولر سرور کو:

- **راستوں کو فٹ سٹیپ سیکوئنسز** میں تبدیل کرنا
- **راستہ فالو کرنے کی ضروریات** کی بنیاد پر **گیٹ پیٹرنز** تیار کرنا
- **انجام دہی** کے دوران **بیلنس** برقرار رکھنا
- **متغیرات** کو ہینڈل کرنا اور **بیلنس** بحال کرنا

```python
class BipedalController(Node):
    def __init__(self):
        super().__init__('bipedal_controller')

        # روبوٹ کو فٹ سٹیپ کمانڈز بھیجنے کے لیے ایکشن کلائنٹ
        self.footstep_client = ActionClient(
            self,
            FollowFootstepPath,  # کسٹم ایکشن نوعیت
            'follow_footstep_path'
        )

        # کنٹرولر فریکوئنسی
        self.controller_freq = 10  # Hz ہائی لیول فٹ سٹیپ منصوبہ بندی کے لیے
        self.controller_timer = self.create_timer(
            1.0 / self.controller_freq,
            self.controller_callback
        )

        # راستہ ٹریکنگ
        self.current_path = None
        self.path_progress = 0.0
        self.control_loop_counter = 0

        # بائی پیڈل مخصوص پیرامیٹرز
        self.step_length = 0.3  # میٹر
        self.step_height = 0.02  # میٹر (کلیئرنس)
        self.step_duration = 0.8  # سیکنڈ فی قدم

        # بیلنس پیرامیٹرز
        self.balance_threshold = 0.05  # میٹر زیادہ سے زیادہ انحراف
        self.recovery_enabled = True

        self.get_logger().info('Bipedal کنٹرولر شروع کیا گیا')

    def controller_callback(self):
        """مرکزی کنٹرول کال بیک"""
        if self.current_path is None or len(self.current_path.poses) == 0:
            return

        # راستہ فالو کرنے کے لیے اگلے سیگمینٹ کا تعین کریں
        next_waypoints = self.get_next_waypoints()

        if next_waypoints:
            # اگلے سیگمینٹ کے لیے فٹ سٹیپ منصوبہ تیار کریں
            footstep_plan = self.plan_footsteps(next_waypoints)

            # روبوٹ کو فٹ سٹیپ کمانڈز بھیجیں
            self.execute_footstep_plan(footstep_plan)

    def get_next_waypoints(self):
        """گلوبل راستہ سے اگلے ویزی ویجز حاصل کریں"""
        if self.path_progress >= len(self.current_path.poses):
            return []

        # فٹ سٹیپس کی منصوبہ بندی کے لیے اگلے چند ویزی ویجز واپس کریں
        start_idx = int(self.path_progress)
        end_idx = min(start_idx + 5, len(self.current_path.poses))

        waypoints = []
        for i in range(start_idx, end_idx):
            waypoints.append(self.current_path.poses[i].pose)

        return waypoints

    def plan_footsteps(self, waypoints):
        """ویزی ویجز کی ترتیب کے لیے تفصیلی فٹ سٹیپس کی منصوبہ بندی کریں"""
        # یہ ایک فٹ سٹیپ پلینر کے ساتھ انٹرفیس کرے گا
        # اس مثال کے لیے، ہم ایک سادہ فٹ سٹیپ سیکوئنس تخلیق کریں گے

        footsteps = []

        for i, waypoint in enumerate(waypoints):
            # اس مقام پر ایک فٹ سٹیپ تخلیق کریں
            footstep = Footstep()  # کسٹم میسیج نوعیت
            footstep.pose = waypoint
            footstep.step_type = "walk"  # چلنا، قدم، موڑ سکتا ہے
            footstep.duration = self.step_duration
            footstep.foot = "left" if i % 2 == 0 else "right"

            footsteps.append(footstep)

        return footsteps

    def execute_footstep_plan(self, footstep_plan):
        """فٹ سٹیپس کی منصوبہ بندی شدہ ترتیب کو انجام دیں"""
        if not footstep_plan:
            return

        # روبوٹ کے فٹ سٹیپ انجام دہی سسٹم کو منصوبہ بھیجیں
        goal = FollowFootstepPath.Goal()
        goal.footstep_path = footstep_plan

        # سرور کا انتظار کریں اور گوئل بھیجیں
        self.footstep_client.wait_for_server()
        future = self.footstep_client.send_goal_async(goal)

        # نتیجہ ہینڈل کریں
        future.add_done_callback(self.footstep_execution_callback)

    def footstep_execution_callback(self, future):
        """فٹ سٹیپ انجام دہی کے مکمل ہونے کو ہینڈل کریں"""
        goal_handle = future.result()
        if goal_handle.accepted:
            self.get_logger().info('فٹ سٹیپ منصوبہ قبول کیا گیا')
        else:
            self.get_logger().error('فٹ سٹیپ منصوبہ مسترد کر دیا گیا')
```

## بائی پیڈل نیویگیشن کے لیے کسٹم کوسٹ میپس

بائی پیڈل روبوٹس کو ان چیزوں کو مدنظر رکھنے والے مخصوص کوسٹ میپس کی ضرورت ہے:

- **بیلنس زونز**: جہاں فٹس محفوظ طریقے سے رکھے جا سکیں
- **زمین کی سٹیبیلٹی**: مختلف زمین کی اقسام کے لیے مختلف قیمتیں
- **رکاوٹ کلیئرنس**: فٹ پلیسمنٹ کے لیے کافی جگہ
- **متحرک سٹیبیلٹی**: علاقوں کو ZMP کو محفوظ حدود کے اندر رکھنا

```python
class BipedalCostmapGenerator(Node):
    def __init__(self):
        super().__init__('bipedal_costmap_generator')

        # مخصوص کوسٹ میپس کے لیے پبلشرز
        self.balance_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/bipedal_balance_costmap',
            10
        )

        self.foot_placement_costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/bipedal_foot_placement_costmap',
            10
        )

        # سینسر ڈیٹا کے لیے سبسکرائبرز
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,  # سادہ نوعیت
            '/imu',
            self.imu_callback,
            10
        )

        # کوسٹ میپ اپ ڈیٹ ٹائمر
        self.costmap_timer = self.create_timer(0.5, self.update_costmaps)

        # انٹرنل ڈیٹا
        self.base_map = None
        self.current_terrain_type = "flat"
        self.balance_state = "stable"

        self.get_logger().info('Bipedal کوسٹ میپ جنریٹر شروع کیا گیا')

    def map_callback(self, map_msg):
        """نیا میپ ڈیٹا ہینڈل کریں"""
        self.base_map = map_msg
        self.get_logger().info('کوسٹ میپ جنریشن کے لیے نیا میپ موصول ہوا')

    def imu_callback(self, imu_msg):
        """IMU سے بیلنس اسٹیٹ اپ ڈیٹ کریں"""
        # یہ طے کرنے کے لیے کہ روبوٹ فی الحال مستحکم حالت میں ہے
        # یہ سادہ ہے - حقیقی نافذ کاری ZMP کیلکولیشنز استعمال کرے گی
        orientation = imu_msg.orientation
        # اورینٹیشن ڈیٹا کو پروسیس کریں تاکہ بیلنس اسٹیٹ کا تعین ہو

        # IMU ڈیٹا کی بنیاد پر اپ ڈیٹ کریں
        self.update_balance_state(imu_msg)

    def update_balance_state(self, imu_msg):
        """IMU کی بنیاد پر انٹرنل بیلنس اسٹیٹ اپ ڈیٹ کریں"""
        # اورینٹیشن سے رول اور پچ نکالیں
        import tf_transformations
        orientation_list = [imu_msg.orientation.x, imu_msg.orientation.y,
                           imu_msg.orientation.z, imu_msg.orientation.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(orientation_list)

        # جھکاؤ کی بنیاد پر بیلنس اسٹیٹ کا تعین کریں
        tilt_magnitude = math.sqrt(roll*roll + pitch*pitch)
        if tilt_magnitude > 0.5:  # عدم استحکام کے لیے تھریشولڈ
            self.balance_state = "unstable"
        else:
            self.balance_state = "stable"

    def update_costmaps(self):
        """بائی پیڈل نیویگیشن کے لیے مخصوص کوسٹ میپس اپ ڈیٹ کریں"""
        if self.base_map is None:
            return

        # بیلنس مبنی کوسٹ میپ تیار کریں
        balance_costmap = self.generate_balance_costmap()
        self.balance_costmap_pub.publish(balance_costmap)

        # فٹ پلیسمنٹ کوسٹ میپ تیار کریں
        foot_placement_costmap = self.generate_foot_placement_costmap()
        self.foot_placement_costmap_pub.publish(foot_placement_costmap)

        self.get_logger().debug('کوسٹ میپس اپ ڈیٹ ہو گئے')

    def generate_balance_costmap(self):
        """بیلنس کنٹرینٹس کو مدنظر رکھتے ہوئے کوسٹ میپ تیار کریں"""
        # بیس میپ کے ساتھ شروع کریں
        costmap = OccupancyGrid()
        costmap.header = self.base_map.header
        costmap.info = self.base_map.info
        costmap.data = list(self.base_map.data)  # بیس ڈیٹا کا کاپی

        # بیلنس مخصوص قیمتیں شامل کریں
        for i in range(len(costmap.data)):
            if costmap.data[i] > 0:  # اگر پہلے سے رکاوٹ کی قیمت ہے
                continue

            # زمین کی سٹیبیلٹی، ڈھلوان وغیرہ کی بنیاد پر قیمتیں کیلکولیٹ کریں
            x, y = self.grid_to_world(i % costmap.info.width, i // costmap.info.width)
            cost = self.calculate_balance_cost(x, y)

            # موجودہ قیمت میں شامل کریں
            if cost > 0:
                costmap.data[i] = min(100, costmap.data[i] + int(cost * 100))

        return costmap

    def generate_foot_placement_costmap(self):
        """بہترین فٹ پلیسمنٹ کے لیے کوسٹ میپ تیار کریں"""
        costmap = OccupancyGrid()
        costmap.header = self.base_map.header
        costmap.info = self.base_map.info
        costmap.data = list(self.base_map.data)

        # فٹ پلیسمنٹ کے لیے زمین کی قسم اور استحکام کو مدنظر رکھیں
        for i in range(len(costmap.data)):
            if costmap.data[i] > 0:  # اگر رکاوٹ ہے
                continue

            x, y = self.grid_to_world(i % costmap.info.width, i // costmap.info.width)
            cost = self.calculate_foot_placement_cost(x, y)

            if cost > 0:
                costmap.data[i] = min(100, costmap.data[i] + int(cost * 100))

        return costmap

    def calculate_balance_cost(self, x, y):
        """ایک پوزیشن کے لیے بیلنس متعلقہ قیمت کیلکولیٹ کریں"""
        # یہ پیچیدہ بیلنس کیلکولیشنز نافذ کرے گا
        # فی الحال، ایک سادہ نافذ کاری
        return 0.0  # سادہ

    def calculate_foot_placement_cost(self, x, y):
        """ایک پوزیشن کے لیے فٹ پلیسمنٹ قیمت کیلکولیٹ کریں"""
        # زمین کی قسم، استحکام وغیرہ کو مدنظر رکھیں
        return 0.0  # سادہ

    def grid_to_world(self, grid_x, grid_y):
        """گرڈ کوآرڈینیٹس کو ورلڈ کوآرڈینیٹس میں تبدیل کریں"""
        x = self.base_map.info.origin.position.x + grid_x * self.base_map.info.resolution
        y = self.base_map.info.origin.position.y + grid_y * self.base_map.info.resolution
        return x, y
```

## Isaac ROS اجزاء کے ساتھ انضمام

Isaac ROS GPU ایکسلریٹڈ ادراک فراہم کرتا ہے جو بائی پیڈل نیویگیشن کو بہتر بناسکتا ہے:

```python
class IsaacBipedalIntegration(Node):
    def __init__(self):
        super().__init__('isaac_bipedal_integration')

        # Isaac ROS ادراک نوڈس انٹرفیس
        self.depth_sub = self.create_subscription(
            Image,  # Isaac ڈیپتھ امیج
            '/isaac_ros/depth/image',
            self.depth_callback,
            10
        )

        self.segmentation_sub = self.create_subscription(
            Image,  # Isaac سیگمینٹیشن ماسک
            '/isaac_ros/segmentation/mask',
            self.segmentation_callback,
            10
        )

        # بہتر نیویگیشن کے لیے پبلشرز
        self.terrain_classification_pub = self.create_publisher(
            String,  # زمین کی قسم کے لیے کسٹم میسیج
            '/terrain_classification',
            10
        )

        self.foot_placement_analysis_pub = self.create_publisher(
            String,  # فٹ پلیسمنٹ تجزیہ کے نتائج
            '/foot_placement_analysis',
            10
        )

        self.get_logger().info('Isaac Bipedal انضمام شروع کیا گیا')

    def depth_callback(self, depth_msg):
        """زمین کے تجزیہ کے لیے ڈیپتھ ڈیٹا پروسیس کریں"""
        # ڈیپتھ ڈیٹا کا تجزیہ کریں تاکہ زمین کی خصوصیات کا تعین ہو
        # - بیلنس منصوبہ بندی کے لیے سطح کا جھکاؤ
        # - فٹ پلیسمنٹ کے لیے رکاوٹ کا پتہ لگانا
        # - سیڑھی نیویگیشن کے لیے قدم کی اونچائی

        # یہ Isaac کے GPU ایکسلریٹڈ پروسیسنگ کا استعمال کرے گا
        terrain_info = self.analyze_terrain_from_depth(depth_msg)
        self.terrain_classification_pub.publish(terrain_info)

    def segmentation_callback(self, seg_msg):
        """فٹ پلیسمنٹ کے لیے سیگمینٹیشن ڈیٹا پروسیس کریں"""
        # سیگمینٹیشن کا تجزیہ کریں:
        # - ٹریورس ایبل سطحیں vs رکاوٹیں
        # - مختلف زمین کی اقسام (گھاس، کنکریٹ وغیرہ)
        # - خطرناک علاقوں (پانی، گڑھے وغیرہ)

        foot_placement_analysis = self.analyze_foot_placement_area(seg_msg)
        self.foot_placement_analysis_pub.publish(foot_placement_analysis)

    def analyze_terrain_from_depth(self, depth_msg):
        """ڈیپتھ ڈیٹا سے زمین کی خصوصیات کا تجزیہ کریں"""
        # Isaac ٹولز کا استعمال کرتے ہوئے GPU ایکسلریٹڈ زمین کا تجزیہ
        # یہ پیچیدہ زمین کی کلاسیفکیشن نافذ کرے گا
        result = String()
        result.data = "terrain_analysis_result"
        return result

    def analyze_foot_placement_area(self, seg_msg):
        """فٹ پلیسمنٹ کے لیے مناسب علاقوں کا تجزیہ کریں"""
        # سیگمینٹیشن کا استعمال کرتے ہوئے محفوظ فٹ پلیسمنٹ زونز کی شناخت کریں
        result = String()
        result.data = "foot_placement_analysis_result"
        return result
```

## فٹ سٹیپ منصوبہ بندی انضمام

ایک مکمل بائی پیڈل نیویگیشن سسٹم ہائی لیول راستہ منصوبہ بندی کو فٹ سٹیپ جنریشن کے ساتھ مربوط کرتا ہے:

```python
class IntegratedBipedalNavigator(Node):
    def __init__(self):
        super().__init__('integrated_bipedal_navigator')

        # Nav2 اجزاء کے ساتھ انٹرفیس
        self.path_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose'
        )

        # فٹ سٹیپ پلینر کے ساتھ انٹرفیس
        self.footstep_planner = ActionClient(
            self,
            PlanFootsteps,
            'plan_footsteps'
        )

        # نیویگیشن کمانڈ انٹرفیس
        self.nav_goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.navigation_goal_callback,
            10
        )

        # اجزاء کو شروع کریں
        self.bipedal_costmap_generator = BipedalCostmapGenerator(self)
        self.bipedal_local_planner = BipedalLocalPlanner(self)

        self.get_logger().info('مکمل بائی پیڈل نیویگیٹر شروع کیا گیا')

    def navigation_goal_callback(self, goal_msg):
        """نیویگیشن گول کو ہینڈل کریں"""
        # پہلے ہائی لیول راستہ منصوبہ بندی کریں
        self.plan_global_path(goal_msg.pose)

    def plan_global_path(self, goal_pose):
        """بائی پیڈل کنٹرینٹس کو مدنظر رکھتے ہوئے گلوبل راستہ منصوبہ بندی کریں"""
        # راستہ منصوبہ بندی کی درخواست تخلیق کریں
        path_goal = ComputePathToPose.Goal()
        path_goal.goal = goal_pose
        # بائی پیڈل مخصوص پیرامیٹرز شامل کریں
        path_goal.planner_id = "bipedal_global_planner"

        # گلوبل پلینر کو بھیجیں
        self.path_client.wait_for_server()
        future = self.path_client.send_goal_async(path_goal)
        future.add_done_callback(self.global_path_callback)

    def global_path_callback(self, future):
        """گلوبل راستہ منصوبہ بندی کا نتیجہ ہینڈل کریں"""
        goal_handle = future.result()
        if goal_handle.accepted:
            path_result = goal_handle.result()
            # راستہ کے لیے تفصیلی فٹ سٹیپس کی منصوبہ بندی کریں
            self.plan_detailed_footsteps(path_result.path)
        else:
            self.get_logger().error('گلوبل راستہ منصوبہ بندی ناکام ہو گئی')

    def plan_detailed_footsteps(self, path):
        """ہائی لیول راستہ کے لیے تفصیلی فٹ سٹیپس کی منصوبہ بندی کریں"""
        footstep_goal = PlanFootsteps.Goal()
        footstep_goal.path = path
        footstep_goal.robot_properties = self.get_robot_properties()

        self.footstep_planner.wait_for_server()
        future = self.footstep_planner.send_goal_async(footstep_goal)
        future.add_done_callback(self.footstep_plan_callback)

    def footstep_plan_callback(self, future):
        """فٹ سٹیپ منصوبہ بندی کا نتیجہ ہینڈل کریں"""
        goal_handle = future.result()
        if goal_handle.accepted:
            footstep_result = goal_handle.result()
            # منصوبہ بندی شدہ فٹ سٹیپس انجام دیں
            self.execute_navigation(footstep_result.footsteps)
        else:
            self.get_logger().error('فٹ سٹیپ منصوبہ بندی ناکام ہو گئی')

    def execute_navigation(self, footsteps):
        """منصوبہ بندی شدہ نیویگیشن انجام دیں"""
        # روبوٹ کے واکنگ کنٹرولر کا انٹرفیس
        # انجام دہی کو مانیٹر کریں اور ضرورت پڑنے پر ری پلیننگ ہینڈل کریں
        self.get_logger().info(f' {len(footsteps)} قدم کے ساتھ نیویگیشن انجام دیا جا رہا ہے')

    def get_robot_properties(self):
        """منصوبہ بندی کے لیے روبوٹ مخصوص خصوصیات حاصل کریں"""
        # قدم کی لمبائی، چوڑائی، اونچائی وغیرہ جیسی خصوصیات واپس کریں
        from builtin_interfaces.msg import Duration
        props = RobotProperties()
        props.max_step_length = 0.3  # میٹر
        props.max_step_width = 0.2   # میٹر
        props.step_height_clearance = 0.05
        props.balance_margin = 0.1   # میٹر
        props.zmp_stability_threshold = 0.05  # میٹر
        props.walk_period = Duration(sec=0, nanosec=800000000)  # 0.8 سیکنڈ

        return props
```

## کارکردگی کی بہتری

ریئل ٹائم بائی پیڈل نیویگیشن کے لیے، بہتری انتہائی ضروری ہے:

```python
class OptimizedBipedalPlanner:
    def __init__(self):
        # عام گیٹ پیٹرنز کے لیے پیش احتساب یافتہ لوک اپ ٹیبلز
        self.gait_patterns = self.precompute_gait_patterns()

        # کیش شدہ انورس کنیمیٹکس حل
        self.ik_cache = {}

        # کثیر ریزولوشن راستہ منصوبہ بندی
        self.coarse_planner = None
        self.fine_planner = None

    def precompute_gait_patterns(self):
        """عام گیٹ پیٹرنز کا پیش احتساب کریں تاکہ فاسٹ لوک اپ کیا جا سکے"""
        # اس میں عام موومنٹ پیٹرنز کے لیے پیش احتساب شدہ فٹ سٹیپ ترتیبیں ہوں گی
        # (آگے، پیچھے، موڑ وغیرہ)
        patterns = {}
        # آگے کے قدم
        patterns['forward'] = self.compute_standard_gait(0.0, 0.3, 0.0)  # x, y, theta
        # موڑ کے قدم
        patterns['turn_left'] = self.compute_standard_gait(0.1, 0.0, 0.2)  # چھوٹا آگے + موڑ
        patterns['turn_right'] = self.compute_standard_gait(0.1, 0.0, -0.2)

        return patterns

    def compute_standard_gait(self, dx, dy, dtheta):
        """دی گئی موومنٹ کے لیے ایک معیاری گیٹ پیٹرن کمپیوٹ کریں"""
        # سادہ گیٹ پیٹرن کمپیوٹیشن
        # حقیقت میں، یہ پیچیدہ بائی میکینیکل ماڈلز استعمال کرے گا
        footsteps = []
        # نافذ کاری اہل فٹ سٹیپ ترتیب پیدا کرے گی
        return footsteps
```

## سیفٹی اور ریکوری میکنزمز

بائی پیڈل نیویگیشن سسٹمز کو مضبوط سیفٹی میکنزمز کی ضرورت ہوتی ہے:

```python
class BipedalSafetyManager(Node):
    def __init__(self):
        super().__init__('bipedal_safety_manager')

        # سسٹم ہیلتھ مانیٹر کریں
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # ایمرجنسی اسٹاپ پبلشر
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)

        # ریکوری ایکشن کلائنٹ
        self.recovery_client = ActionClient(self, ExecuteRecovery, 'execute_recovery')

        # سیفٹی ٹائمرز
        self.safety_timer = self.create_timer(0.1, self.check_safety)

        # سیفٹی تھریشولڈز
        self.tilt_threshold = 0.785  # 45 ڈگری
        self.velocity_threshold = 1.0  # میٹر/سیکنڈ
        self.joint_limit_threshold = 0.1  # حد سے ریڈینز

        self.current_tilt = 0.0
        self.current_velocity = 0.0
        self.in_safe_state = True

        self.get_logger().info('Bipedal سیفٹی مینیجر شروع کیا گیا')

    def imu_callback(self, imu_msg):
        """روبوٹ کی بیلنس اسٹیٹ مانیٹر کریں"""
        # IMU سے جھکاؤ اینگل کا حساب لگائیں
        import tf_transformations
        orientation_list = [imu_msg.orientation.x, imu_msg.orientation.y,
                           imu_msg.orientation.z, imu_msg.orientation.w]
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(orientation_list)
        self.current_tilt = math.sqrt(roll*roll + pitch*pitch)

    def joint_state_callback(self, joint_state_msg):
        """سیفٹی کے لیے جوائنٹ پوزیشنز مانیٹر کریں"""
        # جوائنٹ حدود اور رفتار کی جانچ کریں
        pass

    def check_safety(self):
        """چیک کریں کہ روبوٹ محفوظ حالت میں ہے یا نہیں"""
        if self.current_tilt > self.tilt_threshold:
            self.trigger_recovery("balance_loss")
        elif self.current_velocity > self.velocity_threshold:
            self.trigger_recovery("speed_violation")

    def trigger_recovery(self, reason):
        """ریکوری کی کارروائی شروع کریں"""
        self.get_logger().warn(f'سیفٹی خلاف ورزی: {reason}, ریکوری شروع کی جا رہی ہے')

        # ایمرجنسی اسٹاپ بھیجیں
        stop_msg = Bool()
        stop_msg.data = True
        self.emergency_stop_pub.publish(stop_msg)

        # ریکوری ایکشن متحرک کریں
        recovery_goal = ExecuteRecovery.Goal()
        recovery_goal.recovery_type = reason
        self.recovery_client.wait_for_server()
        self.recovery_client.send_goal_async(recovery_goal)
```

## خلاصہ

اس باب میں ہم نے بائی پیڈل ہیومینائیڈ روبوٹس کے لیے Nav2 کی اڈاپٹیشن پر بات کی:

- چکر لگانے والے روبوٹس کے مقابلے میں بائی پیڈل نیویگیشن کے منفرد چیلنجز
- Nav2 اجزاء (گلوبل پلانر، مقامی پلانر، کنٹرولر) کی بائی پیڈل لوکوموشن کے لیے تبدیلیاں
- روایتی راستہ منصوبہ بندی کے ساتھ فٹ سٹیپ منصوبہ بندی کا انضمام
- بیلنس اور فٹ پلیسمنٹ کے لیے مخصوص کوسٹ میپس
- بائی پیڈل روبوٹس کے لیے سیفٹی اور ریکوری میکنزمز
- کارکردگی کی بہتری کی تکنیکیں

بائی پیڈل نیویگیشن کو روایتی موبائل روبوٹکس سے بنیادی طور پر مختلف نقطہ نظر کی ضرورت ہوتی ہے، جس میں جاری رفتار کنٹرول کے بجائے ڈسکریٹ فٹ سٹیپس، بیلنس برقرار رکھنا، اور گیٹ اڈاپٹیشن پر توجہ مرکوز کی جاتی ہے۔

## مشقیں

1. دو پوائنٹس کے درمیان فٹ سٹیپس تخلیق کرنے والا ایک سادہ فٹ سٹیپ پلینر تخلیق کریں
2. فٹ پلیسمنٹ کے لیے زمین کی استحکام کو مدنظر رکھنے والا ایک کوسٹ میپ نافذ کریں
3. بیلنس کے نقصان کا پتہ لگانے اور ریکوری متحرک کرنے والا ایک سیفٹی میکنزم ڈیزائن کریں

## اگلے اقدامات

اگلے باب میں، ہم اعلیٰ درجے کی AI-روبوٹ دماغ تکنیکوں کی تلاش کریں گے جو اس ادراک اور نیویگیشن کے قابلیت کا فائدہ اٹھاتی ہیں جو ہم نے تیار کی، ہیومینائیڈ روبوٹس کے لیے اعلیٰ سطحی شعوری فنکشنز اور لرننگ پر توجہ مرکوز کرتے ہوئے۔