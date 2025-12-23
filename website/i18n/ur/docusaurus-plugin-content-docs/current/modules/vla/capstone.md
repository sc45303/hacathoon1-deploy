---
sidebar_position: 5
---

# کیپسٹون: خودکار ہیومینائیڈ

## سیکھنے کے مقاصد

- چاروں کورس ماڈیولز کے تمام تصورات کو ایک مکمل سسٹم میں ضم کریں
- ایک اینڈ ٹو اینڈ آٹونومس ہیومینائیڈ روبوٹ نافذ کریں
- ووائس کمانڈ → شعوری منصوبہ بندی → نیویگیشن → مینوپولیشن کا مظاہرہ کریں
- سیمولیشن میں مکمل سسٹم کی توثیق کریں

## کیپسٹون کا جائزہ

کیپسٹون پروجیکٹ کورس کے دوران سیکھے گئے تمام تصورات کو ایک مکمل آٹونومس ہیومینائیڈ روبوٹ سسٹم کو نافذ کرنے کے لیے اکٹھا کرتا ہے۔ یہ روبوٹ ووائس کمانڈز وصول کرنے، پیچیدہ کاموں کو سمجھنے اور منصوبہ بندی کرنے، مقامات پر جانے، اور اپنے ماحول میں اشیاء کو ہینڈل کرنے کے قابل ہوگا۔

### سسٹم معماری

مکمل آٹونومس ہیومینائیڈ سسٹم ضم کرتا ہے:

1. **ROS 2** - تمام اجزاء کو جوڑنے والا کمیونیکیشن مڈل ویئر
2. **Gazebo/Unity** - فزکس سیمولیشن اور وژولائزیشن
3. **NVIDIA Isaac** - AI ادراک اور نیویگیشن
4. **VLA پائپ لائن** - ووائس ٹو ایکشن اور کوگنیٹو پلاننگ

```
[صارف ووائس کمانڈ] → [اسپیچ ریکوگنیشن (Whisper)] → [NLU/LM] → [ٹاسک پلاننگ] → [نیویگیشن (Nav2)] → [مینوپولیشن] → [روبوٹ ایکشنز]
```

## نافذ کاری کے اقدامات

### 1. سسٹم انضمام

سب سے پہلے، ہم ایک مرکزی آرکیسٹریٹر نوڈ تخلیق کریں گے جو پورے سسٹم کا نظم کرتا ہے:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import whisper
import openai

class AutonomousHumanoidNode(Node):
    def __init__(self):
        super().__init__('autonomous_humanoid_node')

        # پبلشرز اور سبسکرائبرز
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.voice_feedback_publisher = self.create_publisher(String, 'voice_feedback', 10)
        self.voice_command_subscriber = self.create_subscription(
            String, 'voice_command', self.voice_command_callback, 10
        )

        # وہیسپر ماڈل شروع کریں
        self.whisper_model = whisper.load_model("base")

        # اسٹیٹ شروع کریں
        self.current_state = "idle"

    def voice_command_callback(self, msg):
        command_text = msg.data
        self.process_command(command_text)

    def process_command(self, command_text):
        # VLA پائپ لائن کے ذریعے کمانڈ کو پروسیس کریں
        self.get_logger().info(f"کمانڈ کو پروسیس کر رہا ہے: {command_text}")

        # LLM کا استعمال کرتے ہوئے ٹاسک پلاننگ
        planned_actions = self.plan_actions(command_text)

        # ایکشنز کو مسلسل طور پر انجام دیں
        for action in planned_actions:
            self.execute_action(action)

    def plan_actions(self, command_text):
        # کمانڈ کو روبوٹ ایکشنز میں ڈیکومپوز کرنے کے لیے LLM کا استعمال کریں
        prompt = f"""
        مندرجہ ذیل انسانی کمانڈ کو روبوٹ ایکشنز کی ترتیب میں ڈیکومپوز کریں:

        کمانڈ: "{command_text}"

        دستیاب ہائی لیول ایکشنز:
        1. navigate_to(location) - کسی مخصوص مقام پر جائیں
        2. recognize_objects() - ماحول میں اشیاء کو پہچانیں
        3. grasp_object(object_name) - کسی مخصوص چیز کو تھامیں
        4. place_object(object_name, location) - کسی مقام پر چیز رکھیں
        5. speak_text(text) - روبوٹ کو بولنے دیں
        6. wave_gesture() - ہاتھ ہلانے کا اشارہ کریں
        7. dance() - ڈانس کی روتین انجام دیں
        8. follow_person(person_id) - کسی مخصوص شخص کو فالو کریں
        9. turn_around() - ماحول اسکین کرنے کے لیے گھومیں

        ہر ایکشن کے لیے پیرامیٹرز کے ساتھ ایکشنز کی ترتیب کو JSON ارے کے طور پر فراہم کریں۔
        """

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}]
        )

        # LLm کے جواب کو پارس کریں تاکہ ایکشن سیکوئس حاصل ہو
        # ایک حقیقی سسٹم میں یہ زیادہ جامع ہوگا
        # اس کورس کی مثال کے لیے، ہم ایک سادہ مثال لوٹائیں گے
        return self.parse_action_sequence(response.choices[0].message.content)

    def parse_action_sequence(self, llm_response):
        # ایک حقیقی نفاذ میں، یہ JSON جواب کو مناسب طریقے سے پارس کرے گا
        # اس کورس کی مثال کے لیے، ہم ایک سادہ ترتیب لوٹائیں گے
        import json
        import re

        # جواب سے JSON نکالیں
        json_match = re.search(r'\[.*\]', llm_response, re.DOTALL)
        if json_match:
            try:
                action_list = json.loads(json_match.group())
                return [Action(action["type"], action.get("parameters", {})) for action in action_list]
            except json.JSONDecodeError:
                # اگر پارسنگ ناکام ہو جائے، تو ایک سادہ مثال لوٹائیں
                pass

        # ڈیفالٹ مثال کی ترتیب
        return [
            Action("speak_text", {"text": "میں آپ کے حکم کو انجام دوں گا"}),
            Action("navigate_to", {"location": "kitchen"}),
            Action("recognize_objects", {}),
            Action("grasp_object", {"object_name": "bottle"}),
            Action("navigate_to", {"location": "table"}),
            Action("place_object", {"object_name": "bottle", "location": "table"}),
            Action("speak_text", {"text": "کام مکمل ہوا"})
        ]

    def execute_action(self, action):
        # اس کے قسم کی بنیاد پر ایکشن انجام دیں
        self.get_logger().info(f"ایکشن انجام دے رہا ہے: {action.type} پیرامیٹرز کے ساتھ: {action.parameters}")

        if action.type == "navigate_to":
            self.navigate_to_location(action.parameters["location"])
        elif action.type == "recognize_objects":
            self.recognize_objects()
        elif action.type == "grasp_object":
            self.grasp_object(action.parameters["object_name"])
        elif action.type == "place_object":
            self.place_object(action.parameters["object_name"], action.parameters["location"])
        elif action.type == "speak_text":
            self.speak_text(action.parameters["text"])
        elif action.type == "turn_around":
            self.turn_around()
        else:
            self.get_logger().warn(f"نامعلوم ایکشن قسم: {action.type}")

    def navigate_to_location(self, location):
        # نیویگیشن کے لیے Nav2 کا استعمال کریں
        self.get_logger().info(f"{location} کی طرف جا رہا ہے")
        # ایک حقیقی نفاذ میں، یہ Nav2 کے لیے نیویگیشن گولز بھیجے گا
        # سیمولیشن کے مقاصد کے لیے، ہم Twist کمانڈز شائع کریں گے
        twist = Twist()
        # جگہ کا نمونہ نیویگیشن منطق
        self.cmd_vel_publisher.publish(twist)

    def recognize_objects(self):
        # Isaac ROS ادراک پیکیجز کا استعمال کریں
        self.get_logger().info("ماحول میں اشیاء کو پہچان رہا ہے")
        # ایک حقیقی نفاذ میں، یہ کیمرہ ڈیٹا اور ڈیٹیکشن ماڈلز کا استعمال کرے گا
        # فی الحال، یہ ایک جگہ کا نمونہ ہے

    def grasp_object(self, object_name):
        # مینوپولیشن پیکیجز کا استعمال کریں
        self.get_logger().info(f"{object_name} کو تھامنے کی کوشش کر رہا ہے")
        # ایک حقیقی نفاذ میں، یہ مینوپولیشن کمانڈز بھیجے گا
        # فی الحال، یہ ایک جگہ کا نمونہ ہے

    def place_object(self, object_name, location):
        # مینوپولیشن پیکیجز کا استعمال کریں
        self.get_logger().info(f"{object_name} کو {location} پر رکھ رہا ہے")
        # ایک حقیقی نفاذ میں، یہ مینوپولیشن کمانڈز بھیجے گا
        # فی الحال، یہ ایک جگہ کا نمونہ ہے

    def speak_text(self, text):
        # اس بات کی فیڈ بیک شائع کریں کہ بول رہا ہے
        feedback_msg = String()
        feedback_msg.data = f"بول رہا ہے: {text}"
        self.voice_feedback_publisher.publish(feedback_msg)
        self.get_logger().info(f"بول رہا ہے: {text}")

    def turn_around(self):
        # ماحول اسکین کرنے کے لیے روبوٹ کو گھومائیں
        self.get_logger().info("ماحول اسکین کرنے کے لیے گھوم رہا ہے")
        twist = Twist()
        twist.angular.z = 0.5  # 0.5 rad/s کے ساتھ گھومیں
        self.cmd_vel_publisher.publish(twist)
        # ایک حقیقی نفاذ میں، ہم گرد کو مکمل کرنے کے لیے مدت کو کنٹرول کریں گے

from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class Action:
    type: str
    parameters: Dict[str, Any]

## مکمل سسٹم انضمام

### لانچ فائل

مکمل سسٹم کو اٹھانے کے لیے، ہمیں ایک لانچ فائل کی ضرورت ہے جو تمام ضروری نوڈز شروع کرے:

```xml
<!-- autonomous_humanoid.launch.py -->
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # آٹونومس ہیومینائیڈ نوڈ شروع کریں
        Node(
            package='autonomous_humanoid',
            executable='autonomous_humanoid_node',
            name='autonomous_humanoid',
            output='screen'
        ),

        # ووائس ریکوگنیشن نوڈ شروع کریں
        Node(
            package='voice_recognition',
            executable='voice_recognition_node',
            name='voice_recognition',
            output='screen'
        ),

        # نیویگیشن کے لیے Nav2 شروع کریں
        Node(
            package='nav2_bringup',
            executable='nav2_launch.py',
            name='navigation',
            output='screen'
        ),

        # Isaac ROS ادراک نوڈز شروع کریں
        Node(
            package='isaac_ros_perceptor',
            executable='perceptor_node',
            name='perceptor',
            output='screen'
        )
    ])
```

### ٹیسٹنگ کے لیے ورلڈ سیٹ اپ

کیپسٹون ڈیمو کے لیے عناصر پر مشتمل سیمولیشن ورلڈ تخلیق کریں:

```xml
<!-- capstone_world.world -->
<sdf version="1.6">
  <world name="capstone_world">
    <!-- بنیادی ورلڈ عناصر شامل کریں -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- مینوپولیشن ٹاسکس کے لیے فرنیچر اور اشیاء شامل کریں -->
    <model name="table">
      <pose>2 0 0 0 0 0</pose>
      <include>
        <uri>model://table</uri>
      </include>
    </model>

    <model name="kitchen_counter">
      <pose>-2 1 0 0 0 0</pose>
      <include>
        <uri>model://counter</uri>
      </include>
    </model>

    <model name="bottle">
      <pose>-1.5 1.5 1 0 0 0</pose>
      <include>
        <uri>model://coke_can</uri>
      </include>
    </model>

    <!-- ایک ہیومینائیڈ روبوٹ شامل کریں -->
    <include>
      <uri>model://humanoid_robot</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## مکمل سسٹم کی ٹیسٹنگ

### ٹیسٹ منظر نامے

1. **سادہ نیویگیشن**: "کچن کاؤنٹر پر جائیں"
2. **آبجیکٹ مینوپولیشن**: "بائیل کو اٹھائیں اور ٹیبل پر رکھ دیں"
3. **پیچیدہ کام**: "کچن میں جائیں، ایک بائیل تلاش کریں، اسے اٹھائیں، اسے ٹیبل پر لائیں، اور اسے نیچے رکھ دیں"

### جوازیت کے معیار

1. **کام مکمل کرنے کی شرح**: کامیابی کے ساتھ مکمل کیے گئے کاموں کا فیصد
2. **نیویگیشن کی درستگی**: روبوٹ مقصود مقامات کے کتنے قریب جاتا ہے
3. **مینوپولیشن کی کامیابی**: چیزیں اٹھانے اور جگہ دینے کی کامیابی کی شرح
4. **ریسپانس ٹائم**: حکم سے کارروائی کے انجام تک کا وقت
5. **مضبوطی**: خامیوں سے بحالی کی صلاحیت

## ڈپلائمنٹ کے غور طلب امور

### سیمولیشن سے حقیقی روبوٹ

سیمولیشن سے حقیقی ہارڈویئر پر منتقلی کے لیے ضرورت ہے:

1. **کیلیبریشن**: یقینی بنائیں کہ سینسرز اور ایکچوایٹرز مناسب طریقے سے کیلیبریٹڈ ہیں
2. **سسٹم آئیڈنٹیفکیشن**: حقیقی دنیا کے مطابق کنٹرول پیرامیٹرز کو ٹیون کرنا
3. **سیفٹی کے تحفظات**: انسان-روبوٹ تعامل کے لیے سیفٹی سسٹم نافذ کرنا
4. **کارکردگی کی ایڈاپٹیشن**: سیمولیشن اور حقیقت کے درمیان کمپیوٹیشنل فرق کے لیے ایڈجسٹ کرنا

### ہارڈویئر کی ضروریات

مکمل سسٹم کو ضرورت ہے:
- Whisper، LLMs، ادراک، اور کنٹرول چلانے کے لیے کافی کمپیوٹیشنل طاقت
- مناسب سینسرز (کیمرے، IMU، وغیرہ)
- لوکوموشن اور مینوپولیشن کے لیے ایکچوایٹرز
- ووائس ان پٹ کے لیے مائیکروفونز
- کمیونیکیشن سسٹم (وائی فائی، وغیرہ)

## خلاصہ

کیپسٹون پروجیکٹ اس کورس میں سیکھے گئے تمام اجزاء کو ایک مکمل آٹونومس ہیومینائیڈ سسٹم میں ضم کرتا ہے۔ یہ پروجیکٹ مظاہرہ کرتا ہے:

- ROS 2 کا انضمام سسٹم کمیونیکیشن کے لیے
- محفوظ ترقی اور ٹیسٹنگ کے لیے فزکس سیمولیشن
- Isaac کا استعمال AI ادراک اور نیویگیشن کے لیے
- قدرتی انسان-روبوٹ تعامل کے لیے ویژن-زبان-ایکشن پائپ لائن

سسٹم ووائس کمانڈز سے روبوٹ ایکشنز تک کی مکمل پائپ لائن کی نمائندگی کرتا ہے، جو ہیومینائیڈ روبوٹکس کی ترقی کے مکمل اسپیکٹرم کا مظاہرہ کرتا ہے۔

## اعلیٰ درجے کی توسیعات

طلباء کیپسٹون کو مندرجہ ذیل کے ساتھ وسعت دے سکتے ہیں:

1. **ڈیموسٹریشن سے سیکھنا**: انسانی ڈیموسٹریشن کے ذریعے نئے رویے سکھانا
2. **ملٹی ماڈل انٹرایکشن**: ووائس، گیسچر، اور وژول ہدایات کو جوڑنا
3. **کولیبریٹو روبوٹکس**: مشترکہ ماحول میں انسانوں کے ساتھ کام کرنا
4. **طویل المدتی آٹونومی**: کم مداخلت کے ساتھ جاری کام

## اگلے اقدامات

فزیکل AI اور ہیومینائیڈ روبوٹکس کورس مکمل کرنے پر مبارکباد! ان ماڈیولز میں حاصل کردہ علم کے ساتھ، آپ کو درج ذیل کے لیے اچھی طرح تیار کیا گیا ہے:

1. اعلیٰ درجے کی روبوٹکس اور AI کے موضوعات کی مزید تلاش کریں
2. اوپن سورس روبوٹکس پروجیکٹس میں شراکت کریں
3. اپنی ہیومینائیڈ روبوٹکس ایپلی کیشنز تیار کریں
4. ایمبیڈڈ AI اور روبوٹکس میں تحقیق کریں

سیکھے گئے تصورات کے ساتھ تجربہ کرنا جاری رکھیں، اور یاد رکھیں کہ روبوٹکس کی ترقی ڈیزائن، ٹیسٹ، اور بہتری کا ایک مسلسل عمل ہے۔