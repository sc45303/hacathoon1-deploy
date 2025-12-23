---
sidebar_position: 2
---

# باب 1: وہیسپر کے ساتھ ووائس ٹو ایکشن

## سیکھنے کے مقاصد

- سمجھیں کہ روبوٹکس میں اسپیچ ریکگنیشن سسٹم کیسے کام کرتے ہیں
- OpenAI وہیسپر اور اس کی صلاحیتوں کے بارے میں سیکھیں
- روبوٹ کنٹرول کے لیے ایک ووائس کمانڈ پائپ لائن نافذ کریں
- اسپیچ ریکگنیشن کو ایکشن ایگزیکیوشن کے ساتھ ضم کریں
- ایک مکمل ووائس ٹو ایکشن سسٹم تخلیق کریں

## ووائس ٹو ایکشن سسٹم کا تعارف

ووائس ٹو ایکشن سسٹم قدرتی انسان-روبوٹ تعامل کو فعال کرتے ہیں جس سے صارف اسپیچ کمانڈز کا استعمال کرتے ہوئے روبوٹس کو کنٹرول کر سکیں۔ یہ سسٹم خاص طور پر ہیومینائیڈ روبوٹس کے لیے اہم ہیں، کیونکہ وہ انسانوں اور روبوٹک سسٹم کے درمیان قدرتی تعامل کو بہتر بناتے ہیں۔

### ووائس ٹو ایکشن سسٹم کے کلیدی اجزاء

1. **اسپیچ ریکگنیشن**: بولی گئی زبان کو ٹیکسٹ میں تبدیل کرنا
2. **قدرتی زبان کی سمجھ**: ٹیکسٹ کا مطلب سمجھنا
3. **ایکشن میپنگ**: سمجھی گئی کمانڈز کو روبوٹ ایکشنز سے میپ کرنا
4. **ایگزیکیوشن**: درخواست کردہ روبوٹ ایکشنز انجام دینا
5. **فیڈ بیک**: صارف کو ایکشنز کی تصدیق فراہم کرنا

## اسپیچ ریکگنیشن کے لیے OpenAI وہیسپر

OpenAI وہیسپر ایک جدید اسپیچ ریکگنیشن ماڈل ہے جو:

- متعدد زبانوں کی حمایت کرتا ہے
- مختلف لهجے اور پس منظر کی آواز کے درمیان مضبوط کارکردگی رکھتا ہے
- مخصوص اطلاق کے لیے فائن ٹیون کیا جا سکتا ہے
- محدود تربیتی ڈیٹا کے ساتھ اچھا کام کرتا ہے

### وہیسپر معماری

وہیسپر ایک ٹرانسفارمر مبنی ماڈل ہے جو:

- اینکوڈر-ڈیکوڈر معماری کا استعمال کرتا ہے
- آڈیو کو 30 سیکنڈ کے ٹکڑوں میں پروسیس کرتا ہے
- پہچانی گئی زبان میں ٹیکسٹ آؤٹ پٹ کرتا ہے
- مخصوص ڈومینز پر توجہ مرکوز کرنے کے لیے پُچھا جا سکتا ہے

### روبوٹکس کے تناظر میں وہیسپر

روبوٹکس کے اطلاق کے لیے، وہیسپر کا استعمال کیا جا سکتا ہے:

- NLP سسٹم کے ذریعے پروسیس کیے جانے کے لیے ووائس کمانڈز کو ٹیکسٹ میں تبدیل کرنے کے لیے
- روبوٹ کے ماحول میں عام پس منظر کی آواز کو سنبھالنے کے لیے
- بین الاقوامی اطلاق کے لیے متعدد زبانوں کی حمایت کرنے کے لیے
- مناسب کمپیوٹیشنل وسائل کے ساتھ ریل ٹائم میں کام کرنے کے لیے

## ووائس ٹو ایکشن پائپ لائن کا نفاذ

مکمل ووائس ٹو ایکشن پائپ لائن میں یہ شامل ہے:

```
[مائیکروفون] → [آڈیو پری پروسیسنگ] → [وہیسپر ASR] → [NLU] → [ایکشن میپنگ] → [روبوٹ ایگزیکیوشن]
```

### آڈیو پری پروسیسنگ

وہیسپر کو آڈیو بھیجنے سے پہلے، پری پروسیسنگ میں یہ شامل ہو سکتا ہے:

```python
import pyaudio
import numpy as np
import webrtcvad
from scipy.io import wavfile

# آڈیو سٹریم کو شروع کریں
audio = pyaudio.PyAudio()
stream = audio.open(
    format=pyaudio.paInt16,
    channels=1,
    rate=16000,  # وہیسپر 16kHz کی توقع کرتا ہے
    input=True,
    frames_per_buffer=1024
)

# آواز کی سرگرمی کا پتہ لگانے کے لیے وائس ایکٹیویٹی ڈیٹیکشن
vad = webrtcvad.Vad()
vad.set_mode(1)  # سرگرمی کا موڈ

# ٹکڑوں میں آڈیو کو پروسیس کریں
frames = []
for i in range(0, int(16000 / 1024 * 5)):  # 5 سیکنڈ کا آڈیو
    data = stream.read(1024)
    frames.append(data)
    # ضرورت پڑنے پر آواز کی سرگرمی کے لیے چیک کریں
```

### وہیسپر کو ضم کرنا

```python
import whisper

# ماڈل لوڈ کریں (ریل ٹائم اطلاق کے لیے 'بیس' یا 'چھوٹا' استعمال کریں)
model = whisper.load_model("base")

# آڈیو کو ٹرانسکرائب کریں
result = model.transcribe("audio_file.wav")
command_text = result["text"]
print(f"کمانڈ پہچانی گئی: {command_text}")
```

## قدرتی زبان کی سمجھ

ایک بار جب اسپیچ کو ٹیکسٹ میں تبدیل کر دیا جاتا ہے، ہمیں مقصد کو سمجھنے کی ضرورت ہوتی ہے:

### سادہ کمانڈ پہچان

```python
# کمانڈ پیٹرنز کی وضاحت کریں
COMMAND_PATTERNS = {
    "move_forward": ["move forward", "go forward", "walk forward"],
    "turn_left": ["turn left", "left turn", "rotate left"],
    "turn_right": ["turn right", "right turn", "rotate right"],
    "stop": ["stop", "halt", "freeze"],
    "wave": ["wave", "waving", "wave hello"],
    "dance": ["dance", "dancing", "perform dance"]
}

def extract_command(text):
    text_lower = text.lower()
    for action, patterns in COMMAND_PATTERNS.items():
        for pattern in patterns:
            if pattern in text_lower:
                return action
    return None
```

### سمجھ کے لیے LLMs کا استعمال

زیادہ پیچیدہ کمانڈز کے لیے، ہم بڑے زبان کے ماڈلز استعمال کر سکتے ہیں:

```python
import openai

def parse_complex_command(text):
    prompt = f"""
    انسانی کمانڈ کو ایک روبوٹ کے لیے پارس کریں اور مناسب ایکشن(ز) لوٹائیں:

    کمانڈ: "{text}"

    دستیاب ایکشنز: move_forward, turn_left, turn_right, stop, wave, dance, pickup_object, place_object, speak_text, navigate_to, follow_person

    جواب کی شکل:
    - action: <action_name>
    - parameters: <dict with any needed parameters>

    اگر کمانڈ کو پارس نہیں کیا جا سکتا، تو جواب دیں:
    - action: "unknown"
    - parameters: {{"text": "<original command>"}}
    """

    response = openai.ChatCompletion.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": prompt}]
    )

    return response.choices[0].message.content
```

## ووائس کمانڈ سسٹم معماری

### مکمل سسٹم نفاذ

```python
import asyncio
import threading
import queue
import time
from dataclasses import dataclass
from typing import Optional

@dataclass
class VoiceCommand:
    text: str
    timestamp: float
    confidence: float

class VoiceCommandSystem:
    def __init__(self, ros_node):
        self.ros_node = ros_node
        self.command_queue = queue.Queue()
        self.is_listening = False
        self.whisper_model = whisper.load_model("base")

    def start_listening(self):
        self.is_listening = True
        # آڈیو کیپچر تھریڈ شروع کریں
        audio_thread = threading.Thread(target=self._capture_audio)
        audio_thread.start()

        # پروسیسنگ تھریڈ شروع کریں
        processing_thread = threading.Thread(target=self._process_commands)
        processing_thread.start()

    def _capture_audio(self):
        # آڈیو کیپچر کا نفاذ یہاں ہوگا
        pass

    def _process_commands(self):
        while self.is_listening:
            if not self.command_queue.empty():
                command = self.command_queue.get()
                self._execute_robot_command(command)

    def _execute_robot_command(self, command: VoiceCommand):
        # کمانڈ کو روبوٹ ایکشن سے میپ کریں
        action = extract_command(command.text)

        if action == "move_forward":
            self.ros_node.move_robot_forward()
        elif action == "turn_left":
            self.ros_node.turn_robot_left()
        elif action == "wave":
            self.ros_node.perform_wave_action()
        # ... اضافی میپنگز
```

## ROS 2 کے ساتھ انضمام

ROS 2 کے ساتھ ضم کرنے کے لیے، ہمیں ووائس سسٹم کو ROS 2 نوڈز سے جوڑنے کی ضرورت ہے:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        # روبوٹ موومنٹ کمانڈز کے لیے پبلشر
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # ووائس فیڈ بیک کے لیے پبلشر
        self.voice_feedback_publisher = self.create_publisher(String, 'voice_feedback', 10)

        # ووائس کمانڈ سسٹم کو شروع کریں
        self.voice_system = VoiceCommandSystem(self)

    def move_robot_forward(self):
        twist = Twist()
        twist.linear.x = 0.5  # 0.5 میٹر/سیکنڈ کے ساتھ آگے بڑھیں
        self.cmd_vel_publisher.publish(twist)

    def turn_robot_left(self):
        twist = Twist()
        twist.angular.z = 0.5  # 0.5 ریڈین/سیکنڈ کے ساتھ بائیں مڑیں
        self.cmd_vel_publisher.publish(twist)

    def perform_wave_action(self):
        # روبوٹ کے ایکشن سرور پر شائع کریں
        # نفاذ مخصوص روبوٹ صلاحیتوں پر منحصر ہوگا
        feedback_msg = String()
        feedback_msg.data = "ویو ایکشن انجام دیا جا رہا ہے"
        self.voice_feedback_publisher.publish(feedback_msg)
```

## ووائس ٹو ایکشن سسٹم میں چیلنجز

### نوائز اور ماحول

- پس منظر کی آواز تشخیص کی درستگی کو متاثر کر سکتی ہے
- روبوٹ کی اپنی آوازیں تشخیص میں مداخلت کر سکتی ہیں
- کمرے کی ایکووسٹکس آڈیو کی کوالیٹی کو متاثر کرتی ہے

### زبان اور کمانڈ کی پیچیدگی

- قدرتی زبان کمانڈز کے اظہار میں بہت زیادہ مختلف ہوتی ہے
- مقصد کی سمجھ کے لیے مضبوط NLU سسٹم کی ضرورت ہوتی ہے
- مبہم کمانڈز کو وضاحت کی ضرورت ہوتی ہے

### ریل ٹائم کی ضروریات

- پروسیسنگ کی تاخیر صارف کے تجربے کو متاثر کرتی ہے
- روبوٹ کا ردعمل وقت انسانی امکانات کے مطابق ہونا چاہیے
- سسٹم کو مداخلتوں کو بروۓ طور پر سنبھالنا چاہیے

## خلاصہ

ووائس ٹو ایکشن سسٹم انسان-روبوٹ تعامل کے لیے ایک قدرتی انٹرفیس فراہم کرتے ہیں، جس سے روبوٹس کو کنٹرول کرنا زیادہ قابل رسائی اور جامع ہو جاتا ہے۔ ان سسٹم کو نافذ کرنے کے لیے اسپیچ ریکگنیشن، قدرتی زبان کی سمجھ، اور روبوٹ ایکشن ایگزیکیوشن کو ضم کرنے کی ضرورت ہوتی ہے۔

## مشقیں

1. Python میں ایک بنیادی آڈیو کیپچر سسٹم سیٹ کریں
2. اسپیچ ریکگنیشن کے لیے وہیسپر انسٹال کریں اور چلائیں
3. ایک سادہ کمانڈ میپنگ سسٹم تخلیق کریں

## اگلے اقدامات

اگلے باب میں، ہم ایسے شعوری منصوبہ بندی کے سسٹم کی تلاش کریں گے جو پیچیدہ کاموں کو قابل عمل ذیلی کاموں میں تقسیم کرنے کے لیے بڑے زبان کے ماڈلز (LLMs) کا استعمال کرتے ہیں۔