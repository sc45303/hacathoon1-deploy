---
sidebar_position: 3
---

# باب 2: LLMs کا استعمال کرتے ہوئے شعوری منصوبہ بندی

## سیکھنے کے مقاصد

- سمجھیں کہ بڑے زبان کے ماڈلز (LLMs) شعوری منصوبہ بندی کے لیے کیسے استعمال کیے جا سکتے ہیں
- ہیومینائیڈ روبوٹس کے لیے شعوری منصوبہ بندی کے سسٹم نافذ کریں
- LLM-مبنی منصوبہ بندی کو ایکشن ایگزیکیوشن کے ساتھ ضم کریں
- کام کی تقسیم اور سلسلہ وار منصوبہ بندی کے بارے میں سیکھیں
- مضبوط منصوبہ بندی کے سسٹم تخلیق کریں جو ابہام اور غلطیوں کو سنبھال سکیں

## شعوری منصوبہ بندی کا تعارف

روبوٹکس میں شعوری منصوبہ بندی سے مراد اعلیٰ سطح کا فیصلہ کرنا جو یہ تعین کرتا ہے کہ روبوٹ کو اپنے اہداف کو حاصل کرنے کے لیے کون سے ایکشنز اٹھانے چاہئیں۔ ہیومینائیڈ روبوٹس کے لیے، اس میں قدرتی زبان کے احکامات کو سمجھنا، پیچیدہ کاموں کو آسان کاموں میں تقسیم کرنا، اور متحرک ماحول کے مطابق ایڈجسٹ کرنا شامل ہے۔

### شعوری منصوبہ بندی میں LLMs کا کردار

بڑے زبان کے ماڈلز نے شعوری منصوبہ بندی میں انقلاب برپا کر دیا ہے:

1. **قدرتی زبان کی سمجھ**: انسانی کمانڈز کو روبوٹ ایکشنز میں تبدیل کرنا
2. **کام کی تقسیم**: پیچیدہ کاموں کو قابل عمل ترتیبات میں توڑنا
3. **سیاق و سباق کا نظم**: ماحول اور اہداف کی سمجھ کو برقرار رکھنا
4. **ایڈاپٹیشن**: نئی معلومات یا رکاوٹوں کے مطابق منصوبوں کو ایڈجسٹ کرنا

### شعوری منصوبہ بندی بمقابلہ کم سطح کی منصوبہ بندی

- **شعوری منصوبہ بندی**: اعلیٰ سطح کا ہدف-مشرق کی طرف متوجہ رویہ، کام کی تقسیم، ابہامی کمانڈز کو سنبھالنا
- **کم سطح کی منصوبہ بندی**: راستہ منصوبہ بندی، ٹریجکٹری جنریشن، موشن کنٹرول

## LLM-مبنی شعوری منصوبہ بندی کی معماری

شعوری منصوبہ بندی کا سسٹم عام طور پر اس معماری کو follow کرتا ہے:

```
[انسانی کمانڈ] → [LLM کے ساتھ NLU] → [کام کی تقسیم] → [منصوبہ کی تکمیل] → [ایکشن ایگزیکیوشن] → [فیڈ بیک لوپ]
```

### LLMs کے ساتھ قدرتی زبان کی سمجھ

LLMs پیچیدہ، قدرتی زبان کے احکامات کو سمجھ سکتے ہیں:

```python
import openai

class CognitivePlanner:
    def __init__(self, openai_api_key):
        openai.api_key = openai_api_key
        self.robot_capabilities = [
            "move_forward", "turn_left", "turn_right", "stop",
            "pick_up_object", "place_object", "speak_text",
            "wave_gesture", "navigate_to", "find_object"
        ]

    def parse_command(self, command_text):
        """قدرتی زبان کو منظم ایکشن منصوبہ میں تبدیل کریں"""

        prompt = f"""
        آپ ایک ہیومینائیڈ روبوٹ کے لیے شعوری منصوبہ بندی کا سسٹم ہیں۔
        درج ذیل انسانی کمانڈ کو روبوٹ ایکشنز کی ترتیب میں تبدیل کریں۔
        مندرجہ ذیل صلاحیتوں میں سے منتخب کریں: {self.robot_capabilities}

        انسانی کمانڈ: "{command_text}"

        منصوبہ کو ایک JSON ارے کے طور پر واپس کریں، جہاں ہر ایکشن میں ہوگا:
        - action: ایکشن کا نام (صلاحیتوں کی فہرست سے)
        - parameters: ایکشن کے لیے ضروری پیرامیٹرز کا حامل آبجیکٹ

        جواب کی مثال کی شکل:
        [
            {{"action": "navigate_to", "parameters": {{"location": "kitchen"}}}},
            {{"action": "find_object", "parameters": {{"object": "bottle"}}}},
            {{"action": "pick_up_object", "parameters": {{"object": "bottle"}}}},
            {{"action": "navigate_to", "parameters": {{"location": "table"}}}},
            {{"action": "place_object", "parameters": {{"object": "bottle", "location": "table"}}}}
        ]

        کمانڈ:
        """

        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.1,  # مسلسل آؤٹ پٹس کے لیے کم درجہ حرارت
            max_tokens=500
        )

        import json
        try:
            # جواب سے JSON نکالیں
            content = response.choices[0].message.content
            # جواب میں JSON تلاش کریں
            import re
            json_match = re.search(r'\[.*\]', content, re.DOTALL)
            if json_match:
                plan = json.loads(json_match.group())
                return plan
        except (json.JSONDecodeError, AttributeError):
            # اگر پارسنگ ناکام ہوتا ہے، تو ایک ڈیفالٹ منصوبہ لوٹائیں
            return [{"action": "speak_text", "parameters": {"text": f"میں کمانڈ کو سمجھ نہیں سکتا: {command_text}"}}]

        return [{"action": "speak_text", "parameters": {"text": f"میں کمانڈ کو سمجھ نہیں سکتا: {command_text}"}}]
```

## کام کی تقسیم اور سلسلہ وار منصوبہ بندی

پیچیدہ کمانڈز کو سادہ، قابل عمل ایکشنز میں توڑنا ضروری ہے:

### مثال: "میرے لیے کچن سے کوئی مشروب لائیں"

یہ اعلیٰ سطح کی کمانڈ مندرجہ ذیل میں تقسیم ہوتی ہے:

1. **ہدف**: صارف کو مشروب فراہم کرنا
2. **ذیلی اہداف**:
   - کچن کی طرف جانا
   - مشروب کی شناخت کرنا
   - مشروب اٹھانا
   - صارف کی طرف واپس جانا
   - مشروب پیش کرنا

### سلسلہ وار منصوبہ بند کار کا نفاذ

```python
class HierarchicalPlanner:
    def __init__(self, llm_planner):
        self.llm_planner = llm_planner
        self.known_locations = {
            "kitchen": [1.0, 2.0, 0.0],
            "living_room": [0.0, 0.0, 0.0],
            "bedroom": [-2.0, 1.0, 0.0],
            "dining_room": [-1.0, -1.0, 0.0]
        }

        self.known_objects = {
            "drink": ["water_bottle", "soda_can", "juice_box"],
            "snack": ["cookies", "apple", "chips"],
            "tool": ["screwdriver", "wrench", "hammer"]
        }

    def create_plan(self, high_level_command):
        """LLM اور ڈومین نالج کا استعمال کرتے ہوئے ایک اعلیٰ سطح کی کمانڈ کے لیے ایک منصوبہ تخلیق کریں"""

        # پہلا، LLM کا استعمال کریں تاکہ عام منصوبہ کی ساخت حاصل کی جا سکے
        general_plan = self.llm_planner.parse_command(high_level_command)

        # پھر، ڈومین نالج کا استعمال کرتے ہوئے منصوبہ کو تکمیل دیں
        refined_plan = self.refine_plan(general_plan, high_level_command)

        return refined_plan

    def refine_plan(self, plan, command):
        """سیاق و سباق اور ڈومین نالج کا استعمال کرتے ہوئے منصوبہ کو تکمیل دیں"""
        refined = []

        for action in plan:
            if action["action"] == "navigate_to" and "location" in action["parameters"]:
                location = action["parameters"]["location"]

                # مقام کو کوآرڈینیٹس میں حل کریں
                if location in self.known_locations:
                    coords = self.known_locations[location]
                    refined.append({
                        "action": "navigate_to_coordinates",
                        "parameters": {"x": coords[0], "y": coords[1], "theta": coords[2]}
                    })
                else:
                    # اگر مقام نامعلوم ہے، تو ایک تلاش کا مرحلہ شامل کریں
                    refined.append({
                        "action": "search_for_location",
                        "parameters": {"location": location}
                    })

            elif action["action"] == "find_object" and "object" in action["parameters"]:
                obj_type = action["parameters"]["object"]

                # چیز کی قسم کو مخصوص چیزوں میں بڑھائیں
                if obj_type in self.known_objects:
                    possible_objects = self.known_objects[obj_type]
                    refined.append({
                        "action": "search_for_objects",
                        "parameters": {"objects": possible_objects}
                    })
                else:
                    refined.append(action)  # اصل ایکشن برقرار رکھیں

            else:
                refined.append(action)

        return refined
```

## سیاق و سباق کا نظم اور یاداشت

شعوری منصوبہ بندی کے سسٹم کو متعدد تعاملات کے دوران سیاق و سباق کو برقرار رکھنے کی ضرورت ہے:

### یاداشت کے سسٹم

```python
import datetime
from typing import List, Dict, Any

class ContextManager:
    def __init__(self):
        self.episodic_memory = []  # حالیہ تعاملات
        self.semantic_memory = {}  # دنیا کے بارے میں عام نالج
        self.procedural_memory = {}  # کاموں کو انجام دینے کا طریقہ

    def update_context(self, event: Dict[str, Any]):
        """نئے واقعہ کے ساتھ سیاق و سباق کو اپ ڈیٹ کریں"""
        event_with_timestamp = {
            "timestamp": datetime.datetime.now(),
            "event": event
        }
        self.episodic_memory.append(event_with_timestamp)

        # صرف حالیہ واقعات رکھیں (آخری 100)
        if len(self.episodic_memory) > 100:
            self.episodic_memory = self.episodic_memory[-100:]

    def get_recent_context(self, timeframe_minutes=30):
        """آخری timeframe_minutes سے سیاق و سباق حاصل کریں"""
        cutoff_time = datetime.datetime.now() - datetime.timedelta(minutes=timeframe_minutes)
        recent_events = [
            event for event in self.episodic_memory
            if event["timestamp"] > cutoff_time
        ]
        return recent_events

    def infer_state(self):
        """سیاق و سباق سے دنیا کی موجودہ حالت کا اندازہ لگائیں"""
        recent_events = self.get_recent_context()

        # مثال: نیویگیشن واقعات سے روبوٹ کا مقام سمجھنا
        current_location = "unknown"
        for event in reversed(recent_events):
            if event["event"]["type"] == "navigation" and event["event"]["status"] == "completed":
                current_location = event["event"]["destination"]
                break

        # مثال: کام کی پیشرفت کا تعین
        current_task = "idle"
        for event in reversed(recent_events):
            if event["event"]["type"] == "task":
                current_task = event["event"]["task_name"]
                break

        return {
            "location": current_location,
            "task": current_task,
            "recent_events": recent_events[-10:]  # آخری 10 واقعات
        }
```

## ابہام اور وضاحت کو سنبھالنا

حقیقی دنیا کی کمانڈز اکثر ابہام ہوتی ہیں اور وضاحت کی ضرورت ہوتی ہے:

```python
class AmbiguityResolver:
    def __init__(self, context_manager):
        self.context_manager = context_manager

    def resolve_ambiguity(self, command):
        """تعین کریں کہ کیا کمانڈ ابہام ہے اور کون سی وضاحت کی ضرورت ہے"""
        context = self.context_manager.infer_state()

        # چیک کریں کہ کیا 'it' یا 'that' سیاق و سباق میں کسی چیز کو حوالہ دیتے ہیں
        if "it" in command.lower() or "that" in command.lower():
            if not self.resolve_pronoun(command, context):
                return {
                    "ambiguous": True,
                    "clarification_needed": "'it' یا 'that' کس چیز کو حوالہ دیتے ہیں؟",
                    "options": self.get_possible_referents(context)
                }

        if "there" in command.lower():
            # غیر واضح مقام کا حوالہ
            return {
                "ambiguous": True,
                "clarification_needed": "آپ کس مقام کا خیال رکھتے ہیں؟",
                "options": ["kitchen", "living room", "bedroom", "dining room"]
            }

        # مبہم چیز کے حوالوں کی جانچ
        import re
        object_patterns = re.findall(r'the (\w+)', command.lower())
        for obj in object_patterns:
            if self.is_ambiguous_object(obj, context):
                return {
                    "ambiguous": True,
                    "clarification_needed": f"آپ کون سا {obj} مراد لے رہے ہیں؟",
                    "options": self.get_specific_objects(obj, context)
                }

        return {"ambiguous": False}

    def resolve_pronoun(self, command, context):
        """سیاق و سباق کا استعمال کرتے ہوئے 'it' یا 'that' جیسے ضمائر کو حل کریں"""
        # نفاذ اس بات کے لیے کہ سیاق و سباق کا استعمال ضمائر کو حل کرنے کے لیے کیا جائے
        # آسانی کے لیے، وضاحت کو متحرک کرنے کے لیے False لوٹائیں
        return False

    def is_ambiguous_object(self, obj, context):
        """چیک کریں کہ کیا چیز کا حوالہ مبہم ہے"""
        # نفاذ چیک کرے گا کہ کیا متعدد مثالیں موجود ہیں
        return obj in ["object", "item", "thing", "one"]

    def get_specific_objects(self, obj, context):
        """چیز کی قسم کی مخصوص مثالیں حاصل کریں"""
        return [f"{obj}_1", f"{obj}_2", f"{obj}_3"]

    def get_possible_referents(self, context):
        """ضمائر کے لیے ممکنہ حوالہ جات حاصل کریں"""
        return ["the bottle", "the chair", "the table", "the person"]
```

## منصوبہ ایگزیکیوشن اور مانیٹرنگ

ایک بار جب منصوبہ تخلیق کر لیا جاتا ہے، اسے انجام دینے اور مانیٹر کرنے کی ضرورت ہوتی ہے:

```python
class PlanExecutor:
    def __init__(self, robot_interface, context_manager):
        self.robot_interface = robot_interface
        self.context_manager = context_manager
        self.current_plan = None
        self.current_step = 0

    def execute_plan(self, plan):
        """ہر قدم کے مطابق ایک منصوبہ انجام دیں"""
        self.current_plan = plan
        self.current_step = 0

        for i, action in enumerate(plan):
            self.current_step = i
            success = self.execute_action(action)

            if not success:
                return self.handle_failure(action, i)

        return True

    def execute_action(self, action):
        """ایک واحد ایکشن انجام دیں"""
        action_type = action["action"]
        parameters = action.get("parameters", {})

        try:
            if action_type == "navigate_to_coordinates":
                return self.robot_interface.navigate_to(
                    parameters["x"],
                    parameters["y"],
                    parameters.get("theta", 0.0)
                )
            elif action_type == "pick_up_object":
                return self.robot_interface.pick_up_object(
                    parameters["object"]
                )
            elif action_type == "place_object":
                return self.robot_interface.place_object(
                    parameters["object"],
                    parameters["location"]
                )
            elif action_type == "speak_text":
                return self.robot_interface.speak_text(
                    parameters["text"]
                )
            else:
                self.robot_interface.speak_text(f"نامعلوم ایکشن: {action_type}")
                return False

        except Exception as e:
            self.robot_interface.speak_text(f"ایکشن انجام دینے میں خامی: {str(e)}")
            return False

    def handle_failure(self, failed_action, step_index):
        """منصوبہ ایگزیکیوشن کی ناکامی کو سنبھالیں"""
        self.context_manager.update_context({
            "type": "failure",
            "action": failed_action,
            "step": step_index,
            "reason": "action_execution_failed"
        })

        # اس نفاذ کے لیے، ناکامی کی نشاندہی کے لیے False لوٹائیں
        # ایک زیادہ جامع سسٹم میں بازیابی کے ایکشنز ہو سکتے ہیں
        return False
```

## ROS 2 کے ساتھ انضمام

ROS 2 کے ساتھ شعوری منصوبہ بندی کو ضم کرنے کے لیے مناسب میسج پاسنگ کی ضرورت ہوتی ہے:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from cognitive_planning_msgs.msg import Plan, PlanStep  # کسٹم میسج

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')

        # پبلشرز
        self.plan_publisher = self.create_publisher(Plan, 'robot_plan', 10)
        self.feedback_publisher = self.create_publisher(String, 'planner_feedback', 10)

        # سبسکرائبرز
        self.command_subscriber = self.create_subscription(
            String,
            'high_level_command',
            self.command_callback,
            10
        )

        # منصوبہ بندی کے اجزاء کو شروع کریں
        self.context_manager = ContextManager()
        self.ambiguity_resolver = AmbiguityResolver(self.context_manager)
        self.hierarchical_planner = HierarchicalPlanner(CognitivePlanner(openai_api_key="YOUR_KEY"))
        self.plan_executor = PlanExecutor(RobotInterface(), self.context_manager)

        self.get_logger().info('شعوری منصوبہ بندی نوڈ شروع کیا گیا')

    def command_callback(self, msg):
        """اعلیٰ سطح کی کمانڈ کو پروسیس کریں"""
        command_text = msg.data
        self.get_logger().info(f'کمانڈ موصول ہوئی: {command_text}')

        # ابہام کی جانچ
        ambiguity_check = self.ambiguity_resolver.resolve_ambiguity(command_text)
        if ambiguity_check["ambiguous"]:
            feedback_msg = String()
            feedback_msg.data = f"وضاحت کی ضرورت ہے: {ambiguity_check['clarification_needed']}"
            self.feedback_publisher.publish(feedback_msg)
            return

        # منصوبہ تخلیق کریں اور انجام دیں
        plan = self.hierarchical_planner.create_plan(command_text)

        # منصوبہ شائع کریں
        plan_msg = self.convert_to_ros_plan(plan)
        self.plan_publisher.publish(plan_msg)

        # منصوبہ انجام دیں
        success = self.plan_executor.execute_plan(plan)

        # نتائج کی اطلاع دیں
        result_msg = String()
        if success:
            result_msg.data = f"کامیابی کے ساتھ کمانڈ انجام دی: {command_text}"
        else:
            result_msg.data = f"کمانڈ انجام دینے میں ناکامی: {command_text}"

        self.feedback_publisher.publish(result_msg)

        # سیاق و سباق کو اپ ڈیٹ کریں
        self.context_manager.update_context({
            "type": "command_execution",
            "command": command_text,
            "result": "success" if success else "failure"
        })

    def convert_to_ros_plan(self, plan):
        """اندرونی منصوبہ کی نمائندگی کو ROS میسج میں تبدیل کریں"""
        plan_msg = Plan()

        for i, step in enumerate(plan):
            step_msg = PlanStep()
            step_msg.id = i
            step_msg.action = step["action"]

            # سادگی کے لیے پیرامیٹرز کو سٹرنگ شکل میں تبدیل کریں
            import json
            step_msg.parameters = json.dumps(step.get("parameters", {}))

            plan_msg.steps.append(step_msg)

        return plan_msg

# مثال RobotInterface کلاس (سادہ)
class RobotInterface:
    def __init__(self):
        pass

    def navigate_to(self, x, y, theta):
        # نفاذ Nav2 کو نیویگیشن گوئلز بھیجنے کا کام کرے گا
        print(f"({x}, {y}, {theta}) کی طرف جا رہا ہے")
        return True  # محاکاتی کامیابی

    def pick_up_object(self, obj_name):
        # نفاذ مینوپولیٹر کو کنٹرول کرے گا
        print(f"{obj_name} اٹھانے کی کوشش کر رہا ہے")
        return True  # محاکاتی کامیابی

    def place_object(self, obj_name, location):
        # نفاذ مینوپولیٹر کو کنٹرول کرے گا
        print(f"{obj_name} کو {location} پر رکھ رہا ہے")
        return True  # محاکاتی کامیابی

    def speak_text(self, text):
        # نفاذ ٹیکسٹ ٹو اسپیچ کا استعمال کرے گا
        print(f"روبوٹ کہتا ہے: {text}")
        return True  # محاکاتی کامیابی

def main(args=None):
    rclpy.init(args=args)
    planner_node = CognitivePlannerNode()

    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## خامیوں کا سامنا اور بحالی

مضبوط شعوری منصوبہ بندی کے سسٹم کو خامیوں کو نرمی سے سنبھالنے کی ضرورت ہوتی ہے:

```python
class RecoverySystem:
    def __init__(self):
        self.recovery_strategies = {
            "navigation_failure": [
                "try_alternative_path",
                "request_human_help",
                "wait_and_retry"
            ],
            "object_not_found": [
                "expand_search_area",
                "ask_for_help",
                "substitute_alternative"
            ],
            "grasp_failure": [
                "adjust_grasp_approach",
                "request_assistance",
                "try_different_object"
            ]
        }

    def suggest_recovery(self, failure_type):
        """ایک دی گئی ناکامی کی قسم کے لیے بحالی کی حکمت عملیاں تجویز کریں"""
        if failure_type in self.recovery_strategies:
            return self.recovery_strategies[failure_type]
        else:
            return ["request_human_help"]

    def execute_recovery(self, strategy, context):
        """ایک بحالی کی حکمت عملی انجام دیں"""
        if strategy == "try_alternative_path":
            # نفاذ Nav2 کے ساتھ دوبارہ منصوبہ بندی میں شامل ہو گا
            print("متبادل نیویگیشن راستہ کی کوشش کر رہا ہے...")
            return True
        elif strategy == "request_human_help":
            # انسانی مدد کی درخواست دیں
            print("انسانی مدد کی درخواست کر رہا ہے...")
            return False  # انسانی مداخلت کی ضرورت ہے
        elif strategy == "expand_search_area":
            # چیز تلاش کرنے کے لیے علاقہ بڑھائیں
            print("تلاش کا علاقہ بڑھا رہا ہے...")
            return True
        else:
            print(f"نامعلوم بحالی کی حکمت عملی: {strategy}")
            return False
```

## خلاصہ

LLMs کے ساتھ شعوری منصوبہ بندی ہیومینائیڈ روبوٹس کو پیچیدہ، قدرتی زبان کے احکامات کو سمجھنے اور انجام دینے کے قابل بناتی ہے۔ قدرتی زبان کی سمجھ اور کام کی تقسیم کے لیے LLMs کو جوڑ کر اور مضبوط ایگزیکیوشن مانیٹرنگ اور خامی کے انتظام کے ساتھ، ہم ایسے جامع روبوٹک سسٹم تخلیق کر سکتے ہیں جو انسانوں کے ساتھ قدرتی طور پر تعامل کریں۔ کلیدی اجزاء میں قدرتی زبان کی سمجھ، سلسلہ وار منصوبہ بندی، سیاق و سباق کا نظم، ابہام کا حل، اور خامی کی بحالی شامل ہیں۔

## مشقیں

1. ایک سادہ شعوری منصوبہ بند تخلیق کریں جو بنیادی کمانڈز کو سنبھال سکے
2. روبوٹ کی حالت کو تعاملات کے درمیان ٹریک کرنے کے لیے سیاق و سباق کا نظم شامل کریں
3. ایک سسٹم تخلیق کریں جو کمانڈز مبہم ہونے پر وضاحت کے لیے کہے

## اگلے اقدامات

اگلے باب میں، ہم یہ دیکھیں گے کہ ویژن، زبان، اور ایکشن سسٹم کو ایک مکمل پائپ لائن میں کیسے ضم کیا جائے، اس کورس میں سیکھے گئے تمام اجزاء کو ہیومینائیڈ روبوٹس کے لیے ایک مربوط سسٹم میں لاتے ہوئے۔