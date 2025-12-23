---
sidebar_position: 4
---

# باب 3: ویژن، زبان، اور ایکشن کو ضم کرنا

## سیکھنے کے مقاصد

- یکجا معماری میں ویژن، زبان، اور ایکشن سسٹمز کو ضم کرنے کا طریقہ سمجھیں
- ہیومینائیڈ روبوٹس کے لیے متعدد ماڈلز کا ادراک نافذ کریں
- ایسے سسٹم تخلیق کریں جو زبانی ماڈلز کا استعمال کرتے ہوئے بصری معلومات کی وضاحت کر سکیں
- متعدد ماڈلز کی سمجھ کی بنیاد پر ایکشن ایگزیکیوشن نافذ کریں
- ادراک سے ایکشن تک مکمل پائپ لائن تخلیق کریں

## ویژن-زبان-ایکشن (VLA) سسٹم کا تعارف

ویژن-زبان-ایکشن (VLA) سسٹم ادراک (ویژن)، شعور (زبان)، اور ایگزیکیوشن (ایکشن) کو ایک یکجا معماری میں ضم کاری کی نمائندگی کرتے ہیں۔ یہ ضم کاری قدرتی زبان کے احکامات کو سمجھنے اور حقیقی دنیا کے ماحول میں انہیں انجام دینے والے ہیومینائیڈ روبوٹس تخلیق کرنے کے لیے بنیادی ہے۔

### VLA پائپ لائن

ویژن-زبان-ایکشن پائپ لائن عام طور پر اس فلو کو فالو کرتی ہے:

```
[بصری ادراک] → [زبانی تشریح] → [ایکشن منصوبہ بندی] → [ایکشن ایگزیکیوشن] → [فیڈ بیک لوپ]
```

ہر کمپوننٹ پچھلے کمپوننٹ پر تعمیر ہوتا ہے، ادراک سے ایکشن تک ایک بے داغ سسٹم تخلیق کرتا ہے۔

## متعدد ماڈلز کا ادراک

متعدد ماڈلز کا ادراک ماحول کو سمجھنے کے لیے متعدد حسی ان پٹس کو جوڑتا ہے۔

### بصری-متنی انضمام

```python
import torch
import clip  # CLIP ماڈل برائے ویژن-زبان سمجھ
from transformers import BlipProcessor, BlipForConditionalGeneration
from PIL import Image

class متعدد_ماڈلز_ادراک:
    def __init__(self):
        # پیش تربیت یافتہ ماڈلز لوڈ کریں
        self.clip_model, self.clip_preprocess = clip.load("ViT-B/32")
        self.blip_processor = BlipProcessor.from_pretrained("Salesforce/blip-image-captioning-base")
        self.blip_model = BlipForConditionalGeneration.from_pretrained("Salesforce/blip-image-captioning-base")

    def generate_caption(self, image):
        """تصویر میں موجود چیزوں کی متنی تفصیل تخلیق کریں"""
        inputs = self.blip_processor(image, return_tensors="pt")
        out = self.blip_model.generate(**inputs)
        caption = self.blip_processor.decode(out[0], skip_special_tokens=True)
        return caption

    def classify_objects(self, image, object_list):
        """چیزوں کو متنی تفصیلات کے استعمال سے ترتیب دیں"""
        # CLIP کے لیے تصویر کو پروسیس کریں
        image_input = self.clip_preprocess(image).unsqueeze(0)

        # ٹیکسٹ تفصیلات کو ٹوکنائز کریں
        text_inputs = clip.tokenize(object_list)

        # مماثلت کے اسکور حاصل کریں
        with torch.no_grad():
            logits_per_image, logits_per_text = self.clip_model(image_input, text_inputs)
            probs = logits_per_image.softmax(dim=-1).cpu().numpy()

        # سب سے زیادہ امکان والی چیز لوٹائیں
        best_match_idx = probs[0].argmax()
        return object_list[best_match_idx], float(probs[0][best_match_idx])

    def find_object_by_description(self, image, description):
        """وہ چیزیں تلاش کریں جو متنی تفصیل سے مماثلت رکھتی ہوں"""
        # تصویر اور تفصیل کو پروسیس کریں
        image_input = self.clip_preprocess(image).unsqueeze(0)
        text_input = clip.tokenize([description])

        with torch.no_grad():
            logits_per_image, logits_per_text = self.clip_model(image_input, text_input)
            prob = logits_per_image.softmax(dim=-1).cpu().numpy()[0][0]

        return float(prob)
```

### ROS 2 کے ساتھ انضمام

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from PIL import Image as PILImage
import io

class VLAIntegrationNode(Node):
    def __init__(self):
        super().__init__('vla_integration_node')

        # پبلشرز اور سبسکرائبرز سیٹ اپ کریں
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.command_subscriber = self.create_subscription(
            String,
            '/high_level_command',
            self.command_callback,
            10
        )

        self.action_publisher = self.create_publisher(
            String,  # اسٹریٹجی میں، یہ ایک کسٹم ایکشن میسج ہو سکتا ہے
            '/robot_actions',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/vla_feedback',
            10
        )

        # ادراک کمپوننٹس شروع کریں
        self.perceptor = MultimodalPerceptor()
        self.bridge = CvBridge()

        # موجودہ حالت
        self.current_image = None
        self.pending_command = None

        self.get_logger().info('VLA انضمام نوڈ شروع کیا گیا')

    def image_callback(self, msg):
        """آنے والی کیمرہ کی تصاویر کو پروسیس کریں"""
        try:
            # ROS تصویر کو PIL تصویر میں تبدیل کریں
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            pil_image = PILImage.fromarray(cv_image)

            # بعد کی پروسیسنگ کے لیے محفوظ کریں
            self.current_image = pil_image

            # اگر ہمارے پاس کمانڈ ہے، تو دونوں کو ایک ساتھ پروسیس کریں
            if self.pending_command:
                self.process_command_with_image(self.pending_command, pil_image)
                self.pending_command = None

        except Exception as e:
            self.get_logger().error(f'تصویر کی پروسیسنگ میں خامی: {str(e)}')

    def command_callback(self, msg):
        """آنے والے اعلیٰ سطح کے احکامات کو پروسیس کریں"""
        command = msg.data
        self.get_logger().info(f'کمانڈ موصول ہوئی: {command}')

        # اگر ہمارے پاس تصویر ہے، تو فوری طور پر پروسیس کریں؛ بصورت دیگر، کمانڈ محفوظ کریں
        if self.current_image:
            self.process_command_with_image(command, self.current_image)
        else:
            self.pending_command = command
            self.get_logger().info('کمانڈ محفوظ کی گئی، تصویر کا انتظار ہے')

    def process_command_with_image(self, command, image):
        """کمانڈ کو موجودہ تصویر کے ساتھ پروسیس کریں"""
        self.get_logger().info(f'"{command}" کمانڈ کو تصویر کے ساتھ پروسیس کر رہے ہیں')

        # متعدد ماڈلز کا ادراک استعمال کرتے ہوئے منظر کو سمجھیں
        caption = self.perceptor.generate_caption(image)
        self.get_logger().info(f'تصویر کی کیپشن: {caption}')

        # کمانڈ اور منظر کی سمجھ کے مطابق ایکشنز منصوبہ بند کریں
        actions = self.plan_vla_actions(command, caption, image)

        # منصوبہ بند کردہ ایکشنز انجام دیں یا شائع کریں
        for action in actions:
            self.publish_action(action)

        # فیڈ بیک فراہم کریں
        feedback_msg = String()
        feedback_msg.data = f'کمانڈ کے لیے {len(actions)} ایکشنز منصوبہ بند کیے گئے: {command}'
        self.feedback_publisher.publish(feedback_msg)

    def plan_vla_actions(self, command, caption, image):
        """کمانڈ، کیپشن، اور تصویر کی بنیاد پر ایکشنز منصوبہ بند کریں"""
        # یہ ایک سادہ مثال ہے - عمل میں، اس میں زیادہ پیچیدہ دلیلیں ہوں گی

        actions = []

        # مثال: اگر کمانڈ میں چیز تلاش کرنا شامل ہو، تو تصویر میں اسے تلاش کریں
        if "find" in command.lower() or "locate" in command.lower():
            # کمانڈ سے چیز نکالیں (سادہ)
            import re
            words = command.lower().split()
            potential_objects = [w for w in words if w in ["bottle", "cup", "box", "chair", "table"]]

            if potential_objects:
                target_object = potential_objects[0]

                # چیک کریں کہ چیز تصویر میں موجود ہے یا نہیں
                object_prob = self.perceptor.find_object_by_description(image, f"an image of a {target_object}")

                if object_prob > 0.5:  # "چیز کا پتہ چلا" کے لیے تھریشولڈ
                    # چیز کی طرف نیویگیشن منصوبہ بند کریں
                    actions.append(f"navigate_to_object({target_object})")
                    actions.append(f"approach_object({target_object})")
                else:
                    # چیز نظروں میں نہیں، ممکنہ طور پر منتقل ہونے کی ضرورت ہو سکتی ہے
                    actions.append(f"search_for_object({target_object})")

        # مثال: اگر کمانڈ میں مینوپولیشن شامل ہو
        if "pick up" in command.lower() or "grasp" in command.lower():
            actions.append("plan_grasp_approach()")
            actions.append("execute_grasp()")

        # ڈیفالٹ ایکشن اگر کوئی مخصوص کمانڈ پیٹرن میچ نہ ہو
        if not actions:
            actions.append(f"speak_text(Received command: {command})")

        return actions

    def publish_action(self, action):
        """ایکشن ایگزیکیوشن کے لیے ایکشن شائع کریں"""
        action_msg = String()
        action_msg.data = action
        self.action_publisher.publish(action_msg)
        self.get_logger().info(f'ایکشن شائع کیا گیا: {action}')

def main(args=None):
    rclpy.init(args=args)
    node = VLAIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## چیز کا پتہ لگانا اور منظر کی سمجھ

ہیومینائیڈ روبوٹس کے لیے، 3D ماحول کو سمجھنا انتہائی ضروری ہے:

### 3D چیز کا پتہ لگانا

```python
import numpy as np
import open3d as o3d

class SceneUnderstanding:
    def __init__(self):
        # 3D ادراک ماڈلز شروع کریں (سادہ)
        pass

    def detect_objects_3d(self, point_cloud):
        """3D پوائنٹ کلاؤڈ میں چیزوں کا پتہ لگائیں اور سیگمینٹ کریں"""
        # پوائنٹ کلاؤڈ کو Open3D فارمیٹ میں تبدیل کریں
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(point_cloud[:, :3])

        # گراؤنڈ پلین سیگمینٹ کریں
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=0.01,
            ransac_n=3,
            num_iterations=1000
        )

        # بقیہ چیزوں کو نکالیں
        objects_cloud = pcd.select_by_index(inliers, invert=True)

        # باقی پوائنٹس کو چیزوں میں کلسٹر کریں
        labels = np.array(objects_cloud.cluster_dbscan(eps=0.02, min_points=10))

        segmented_objects = []
        for i in range(labels.max() + 1):
            object_points = np.asarray(objects_cloud.select_by_index(np.where(labels == i)[0]))
            if len(object_points) > 10:  # چیز سمجھے جانے کے لیے کم از کم 10 پوائنٹس
                segmented_objects.append(object_points)

        return segmented_objects

    def estimate_object_properties(self, object_points):
        """سیگمینٹ کی گئی چیز کی خصوصیات کا تخمینہ لگائیں"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(object_points)

        # باؤنڈنگ باکس کمپیوٹ کریں
        aabb = pcd.get_axis_aligned_bounding_box()
        obb = pcd.get_oriented_bounding_box()

        # مرکز اور سائز کا تخمینہ لگائیں
        center = np.array(obb.get_center())
        size = np.array(obb.get_extent())

        return {
            "center": center,
            "size": size,
            "bbox": obb,
            "point_count": len(object_points)
        }
```

## زبان-ہدایت کار ایکشن منصوبہ بندی

زبانی سمجھ کو ایکشن منصوبہ بندی کے ساتھ ضم کرنا روبوٹس کو پیچیدہ، قدرتی زبان کے احکامات انجام دینے کی اجازت دیتا ہے:

```python
class LanguageGuidedPlanner:
    def __init__(self, robot_capabilities):
        self.capabilities = robot_capabilities
        self.location_map = {
            "kitchen": [1.0, 2.0, 0.0],
            "living_room": [0.0, 0.0, 0.0],
            "bedroom": [-2.0, 1.0, 0.0],
            "dining_room": [-1.0, -1.0, 0.0]
        }
        self.object_map = {
            "drink": ["water_bottle", "soda_can", "juice_box"],
            "snack": ["cookies", "apple", "chips"],
            "tool": ["screwdriver", "wrench", "hammer"]
        }

    def parse_and_plan(self, command, current_context):
        """قدرتی زبان کے حکم کو تبدیل کریں اور قابل عمل منصوبہ تخلیق کریں"""
        import openai
        import json
        import re

        # LLM کے لیے ایک پروموٹ تخلیق کریں
        prompt = f"""
        آپ ایک ہیومینائیڈ روبوٹ کے لیے زبان-ہدایت کار ایکشن پلینر ہیں۔ انسانی کمانڈ کو قابل عمل ایکشنز کی ترتیب میں تبدیل کریں۔

        موجودہ سیاق و سباق: {current_context}
        روبوٹ صلاحیات: {self.capabilities}

        انسانی کمانڈ: "{command}"

        منصوبہ کو JSON ارے کے طور پر فراہم کریں ان ایکشن قسیم کے ساتھ:
        - navigate_to_location: کسی نامزد مقام پر جائیں
        - find_object: کسی مخصوص چیز کو تلاش کریں
        - pick_up_object: کوئی چیز اٹھائیں
        - place_object: کسی مقام پر چیز رکھیں
        - speak_text: انسان کو کچھ کہیں
        - wave_gesture: ہاتھ ہلانے کا اشارہ کریں
        - wait: کسی مخصوص واقعہ کا انتظار کریں

        ہر ایکشن میں کوئی ضروری پیرامیٹرز شامل کریں۔

        مثال:
        [
            {{"action": "navigate_to_location", "parameters": {{"location": "kitchen"}}}},
            {{"action": "find_object", "parameters": {{"object": "water_bottle"}}}},
            {{"action": "pick_up_object", "parameters": {{"object": "water_bottle"}}}},
            {{"action": "navigate_to_location", "parameters": {{"location": "living_room"}}}},
            {{"action": "place_object", "parameters": {{"object": "water_bottle", "location": "table"}}}},
            {{"action": "speak_text", "parameters": {{"text": "میں نے بوتل کو ٹیبل پر رکھ دیا ہے"}}}}
        ]

        منصوبہ:
        """

        try:
            # اصل نفاذ میں، یہ OpenAI API کال کرے گا
            # اس مثال کے لیے، ہم جواب کی شبیہہ سازی کریں گے
            return self.simulate_llm_response(command, current_context)
        except Exception as e:
            self.get_logger().error(f'LLM منصوبہ بندی میں خامی: {str(e)}')
            return [{"action": "speak_text", "parameters": {"text": f"میں کمانڈ کو سمجھ نہیں سکا: {command}"}}]

    def simulate_llm_response(self, command, context):
        """ڈیمو کے مقاصد کے لیے LLM جواب کی شبیہہ سازی کریں"""
        # یہ ایک سادہ شبیہہ سازی ہے - ایک حقیقی نفاذ LLM API کال کرے گا

        command_lower = command.lower()

        if "bring" in command_lower or "get" in command_lower:
            # کمانڈ میں چیز کی قسم تلاش کریں
            obj_type = None
            for obj_class, obj_list in self.object_map.items():
                for obj in obj_list:
                    if obj in command_lower:
                        obj_type = obj
                        break
                if obj_type:
                    break

            if obj_type:
                # منزل کو نکالیں اگر ذکر کیا گیا ہو
                destination = "living_room"  # ڈیفالٹ
                for loc_name in self.location_map.keys():
                    if loc_name in command_lower:
                        destination = loc_name
                        break

                return [
                    {"action": "navigate_to_location", "parameters": {"location": "kitchen"}},
                    {"action": "find_object", "parameters": {"object": obj_type}},
                    {"action": "pick_up_object", "parameters": {"object": obj_type}},
                    {"action": "navigate_to_location", "parameters": {"location": destination}},
                    {"action": "place_object", "parameters": {"object": obj_type, "location": "table"}},
                    {"action": "speak_text", "parameters": {"text": f"میں نے {obj_type} کو {destination} میں لایا ہے"}}
                ]

        elif "go to" in command_lower or "navigate to" in command_lower:
            # منزل نکالیں
            destination = "living_room"  # ڈیفالٹ
            for loc_name in self.location_map.keys():
                if loc_name in command_lower:
                    destination = loc_name
                    break

            return [
                {"action": "navigate_to_location", "parameters": {"location": destination}},
                {"action": "speak_text", "parameters": {"text": f"میں {destination} پہنچ گیا ہوں"}}
            ]

        else:
            return [
                {"action": "speak_text", "parameters": {"text": f"میں یقینی طور پر یہ سمجھ نہیں سکتا کہ کیسے انجام دوں: {command}"}}
            ]
```

## ایکشن ایگزیکیوشن اور کنٹرول

ایک بار جب منصوبے تیار کر لیے جاتے ہیں، انہیں روبوٹ کے کنٹرول سسٹمز کے ذریعے انجام دینے کی ضرورت ہوتی ہے:

```python
class ActionExecutor:
    def __init__(self):
        self.current_task = None
        self.is_executing = False

    def execute_plan(self, plan):
        """ایکشنز کی ترتیب انجام دیں"""
        for i, action in enumerate(plan):
            self.get_logger().info(f'ایکشن انجام دے رہے ہیں {i+1}/{len(plan)}: {action["action"]}')
            success = self.execute_single_action(action)

            if not success:
                self.get_logger().error(f'ایکشن ناکام: {action}')
                return False

        return True

    def execute_single_action(self, action):
        """ایک ایکشن انجام دیں"""
        action_type = action["action"]
        params = action.get("parameters", {})

        try:
            if action_type == "navigate_to_location":
                return self.execute_navigation(params["location"])
            elif action_type == "find_object":
                return self.execute_object_search(params["object"])
            elif action_type == "pick_up_object":
                return self.execute_grasping(params["object"])
            elif action_type == "place_object":
                return self.execute_placement(params["object"], params["location"])
            elif action_type == "speak_text":
                return self.execute_speech(params["text"])
            elif action_type == "wave_gesture":
                return self.execute_wave()
            else:
                self.get_logger().error(f'نامعلوم ایکشن قسم: {action_type}')
                return False

        except Exception as e:
            self.get_logger().error(f'ایکشن انجام دینے میں خامی {action_type}: {str(e)}')
            return False

    def execute_navigation(self, location):
        """مخصوص مقام پر نیویگیشن انجام دیں"""
        # ایک حقیقی نفاذ میں، یہ Nav2 کے ساتھ انٹرفیس کرے گا
        self.get_logger().info(f'{location} کی طرف جا رہے ہیں')
        # نیویگیشن کے لیے شبیہہ سازی
        import time
        time.sleep(1)  # نیویگیشن کے لیے وقت کی شبیہہ سازی
        return True

    def execute_object_search(self, obj_name):
        """مخصوص چیز کی تلاش انجام دیں"""
        self.get_logger().info(f'{obj_name} کی تلاش کر رہے ہیں')
        # ایک حقیقی نفاذ میں، یہ ادراک سسٹمز کو چالو کرے گا
        # شبیہہ سازی کے لیے، فرض کریں کہ چیز چند لمحوں کے بعد مل جاتی ہے
        import time
        time.sleep(0.5)
        return True

    def execute_grasping(self, obj_name):
        """چیز کو اٹھانے کا عمل انجام دیں"""
        self.get_logger().info(f'{obj_name} اٹھانے کی کوشش کر رہے ہیں')
        # ایک حقیقی نفاذ میں، یہ مینوپولیٹر کنٹرول کے ساتھ انٹرفیس کرے گا
        import time
        time.sleep(0.5)
        return True

    def execute_placement(self, obj_name, location):
        """چیز کو مقام پر رکھیں"""
        self.get_logger().info(f'{obj_name} کو {location} پر رکھ رہے ہیں')
        import time
        time.sleep(0.5)
        return True

    def execute_speech(self, text):
        """متن سے تقریر نافذ کریں"""
        self.get_logger().info(f'بول رہے ہیں: {text}')
        # ایک حقیقی نفاذ میں، یہ TTS سسٹم کا استعمال کرے گا
        return True

    def execute_wave(self):
        """ہاتھ ہلانے کا اشارہ انجام دیں"""
        self.get_logger().info('ہاتھ ہلانے کا اشارہ انجام دے رہے ہیں')
        # ایک حقیقی نفاذ میں، یہ روبوٹ کی بازو کو حرکت دے گا
        import time
        time.sleep(0.5)
        return True
```

## مکمل VLA سسٹم انضمام

اب، تمام اجزاء کو ایک مکمل سسٹم میں ڈالیں:

```python
class CompleteVLASystem(Node):
    def __init__(self):
        super().__init__('vla_system_node')

        # پبلشرز اور سبسکرائبرز سیٹ اپ کریں
        self.image_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.command_subscriber = self.create_subscription(
            String,
            '/high_level_command',
            self.command_callback,
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            '/vla_system_feedback',
            10
        )

        # کمپوننٹس شروع کریں
        self.perceptor = MultimodalPerceptor()
        self.scene_understanding = SceneUnderstanding()
        self.language_planner = LanguageGuidedPlanner([
            "navigate_to_location", "find_object", "pick_up_object",
            "place_object", "speak_text", "wave_gesture"
        ])
        self.action_executor = ActionExecutor()
        self.bridge = CvBridge()

        # حالت کا نظم
        self.current_image = None
        self.pending_command = None
        self.current_context = {
            "robot_location": "unknown",
            "carried_object": None,
            "last_action": "none",
            "timestamp": self.get_clock().now().to_msg()
        }

        self.get_logger().info('مکمل VLA سسٹم شروع کیا گیا')

    def image_callback(self, msg):
        """آنے والی تصاویر کو ہینڈل کریں"""
        try:
            # ROS تصویر کو PIL تصویر میں تبدیل کریں
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            pil_image = PILImage.fromarray(cv_image)

            # موجودہ تصویر کو اپ ڈیٹ کریں
            self.current_image = pil_image

            # کوئی زیر التوا کمانڈ کو نئی تصویر کے ساتھ پروسیس کریں
            if self.pending_command:
                self.process_command_with_context(self.pending_command)
                self.pending_command = None

        except Exception as e:
            self.get_logger().error(f'تصویر کی پروسیسنگ میں خامی: {str(e)}')

    def command_callback(self, msg):
        """اعلیٰ سطح کے احکامات کو ہینڈل کریں"""
        command = msg.data
        self.get_logger().info(f'اعلیٰ سطح کا حکم موصول ہوا: {command}')

        if self.current_image:
            # فوری طور پر پروسیس کریں اگر ہمیں تصویر مل جائے
            self.process_command_with_context(command)
        else:
            # تصویر ملنے تک کمانڈ محفوظ کریں
            self.pending_command = command
            self.get_logger().info('کمانڈ محفوظ کی گئی، تصویر کا انتظار ہے')

    def process_command_with_context(self, command):
        """سیاق و سباق کے ساتھ کمانڈ کو پروسیس کریں"""
        self.get_logger().info(f'سیاق و سباق کے ساتھ کمانڈ کو پروسیس کر رہے ہیں: {command}')

        try:
            # زبان کی ہدایت کے ساتھ ایکشنز منصوبہ بند کریں
            plan = self.language_planner.parse_and_plan(command, self.current_context)
            self.get_logger().info(f'{len(plan)} ایکشنز کے ساتھ منصوبہ تخلیق کیا گیا')

            # منصوبہ انجام دیں
            success = self.action_executor.execute_plan(plan)

            # ایگزیکیوشن کی بنیاد پر سیاق و سباق کو اپ ڈیٹ کریں
            if success:
                self.update_context_after_execution(plan)
                feedback = f'کامیابی کے ساتھ کمانڈ انجام دیا گیا: {command}'
            else:
                feedback = f'کمانڈ انجام دینے میں ناکامی: {command}'

            # فیڈ بیک شائع کریں
            feedback_msg = String()
            feedback_msg.data = feedback
            self.feedback_publisher.publish(feedback_msg)

        except Exception as e:
            error_msg = f'کمانڈ کی پروسیسنگ میں خامی: {str(e)}'
            self.get_logger().error(error_msg)
            feedback_msg = String()
            feedback_msg.data = error_msg
            self.feedback_publisher.publish(feedback_msg)

    def update_context_after_execution(self, plan):
        """منصوبہ ایگزیکیوشن کے بعد سسٹم سیاق و سباق کو اپ ڈیٹ کریں"""
        if plan:
            # آخری ایکشن کی بنیاد پر اپ ڈیٹ کریں
            last_action = plan[-1]
            self.current_context["last_action"] = last_action["action"]
            self.current_context["timestamp"] = self.get_clock().now().to_msg()

            # اٹھائی گئی چیز کو اپ ڈیٹ کریں اگر متعلقہ ہو
            if last_action["action"] == "pick_up_object":
                obj = last_action["parameters"]["object"]
                self.current_context["carried_object"] = obj
            elif last_action["action"] == "place_object":
                self.current_context["carried_object"] = None

    def get_logger(self):
        """نوڈ لاگر کے لیے وریپر"""
        return self.get_logger()

def main(args=None):
    rclpy.init(args=args)
    vla_system = CompleteVLASystem()

    try:
        rclpy.spin(vla_system)
    except KeyboardInterrupt:
        pass
    finally:
        vla_system.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## جائزہ اور کارکردگی کے معیار

### مقداری معیار

1. **کام مکمل کرنے کی شرح**: کامیابی کے ساتھ مکمل کیے گئے کاموں کا فیصد
2. **منصوبہ بندی کی درستگی**: منصوبہ مطلوبہ کام سے کتنی اچھی طرح مماثلت رکھتا ہے
3. **ایگزیکیوشن ٹائم**: حکم سے کام مکمل ہونے تک کا وقت
4. **ادراک کی درستگی**: چیزوں اور مناظر کو کتنی درست طور پر سمجھا جاتا ہے

### معیاری معیار

1. **قدرتی تعامل**: انسان-روبوٹ تعامل کتنا قدرتی لگتا ہے
2. **مضبوطی**: سسٹم غیر متوقع حالات کو کتنی اچھی طرح سنبھال سکتا ہے