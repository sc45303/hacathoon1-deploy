---
sidebar_position: 4
---

# باب 3: پائی تھون ایجنٹس کو ROS کنٹرولرز سے جوڑنا (rclpy)

## سیکھنے کے مقاصد

- یہ سمجھیں کہ ROS 2 کے ساتھ انٹرفیس کرنے والے پائی تھون نوڈز کیسے تخلیق کیے جاتے ہیں
- ROS 2 سسٹمز کے ساتھ رابطہ کرنے کے لیے rclpy کا استعمال سیکھیں
- پائی تھون AI ایجنٹس اور ROS کنٹرولرز کے درمیان پل نافذ کریں
- AI اور کنٹرول سسٹمز کے درمیان مضبوط رابطے کے نمونے تخلیق کریں
- AI-کنٹرول پلوں میں غلطیوں اور استثنیٰ کا سامنا کریں

## rclpy کا تعارف

rclpy ROS 2 کے لیے پائی تھون کلائنٹ لائبریری ہے، جو ROS 2 مڈل ویئر کے لیے پائی تھون بائنڈنگس فراہم کرتی ہے۔ یہ پائی تھون ڈیولپرز کو ROS 2 نوڈز تخلیق کرنے اور ROS 2 ایکو سسٹم کے ساتھ بات چیت کرنے کی اجازت دیتی ہے۔ یہ ہیومینائیڈ روبوٹکس کے لیے خاص طور پر اہم ہے، جہاں AI اور مشین لرننگ ایپلی کیشنز کے لیے پائی تھون کا وسیع استعمال کیا جاتا ہے۔

### روبوٹکس میں AI کے لیے پائی تھون کیوں

AI اور مشین لرننگ کی ترقی کے لیے پائی تھون غالب زبان ہے کیونکہ:

- **امیر ایکو سسٹم**: TensorFlow، PyTorch، scikit-learn، اور OpenAI جیسی لائبریریز
- **تیز پروٹو ٹائپنگ**: AI الگورتھم کو تیار کرنے اور ٹیسٹ کرنے میں آسانی
- **کمیونٹی سپورٹ**: AI محققین اور عمل کاروں کی بڑی کمیونٹی
- **انضمام کی صلاحیتیں**: مختلف سسٹمز اور لائبریریز کو ملانے میں آسانی

## AI-کنٹرول پل کو سمجھنا

ہیومینائیڈ روبوٹکس میں، اکثر AI سسٹمز (پائی تھون میں چلنے والے) کو روبوٹک کنٹرول سسٹمز (زیادہ تر ROS 2 کا استعمال کرتے ہوئے) سے جوڑنے کی ضرورت ہوتی ہے۔ پل میں عام طور پر شامل ہوتا ہے:

1. **ROS 2 ٹاپکس** سے سینسر ڈیٹا وصول کرنا
2. **AI الگورتھم** کے ذریعے ڈیٹا کو پروسیس کرنا
3. **AI فیصلوں** کی بنیاد پر کمانڈز تخلیق کرنا
4. **ROS 2** کے ذریعے روبوٹ کنٹرولرز کو کمانڈز بھیجنا

### AI-کنٹرول پل کی آرکیٹیکچر

```
[ROS 2 سینسرز] → [پائی تھون پل نوڈ] → [AI ایجنٹ] → [پائی تھون پل نوڈ] → [ROS 2 کنٹرولرز]
```

## rclpy نوڈز کا سیٹ اپ

### بنیادی نوڈ ڈھانچہ

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import numpy as np

class AIBridgeNode(Node):
    def __init__(self):
        super().__init__('ai_bridge_node')

        # روبوٹ کو کمانڈز بھیجنے کے لیے پبلشرز
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)

        # سینسر ڈیٹا وصول کرنے کے لیے سبسکرائبرز
        self.sensor_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_subscriber = self.create_subscription(
            String,  # عمل میں، یہ sensor_msgs/Imu ہوگا
            '/imu_data',
            self.imu_callback,
            10
        )

        # AI کے ذریعے پروسیس کیے جانے والے اسٹیٹ ڈیٹا کو محفوظ کریں
        self.current_joint_states = JointState()
        self.imu_data = None

        # AI پروسیسنگ لوپ کے لیے ٹائمر
        self.processing_timer = self.create_timer(0.1, self.process_ai_step)  # 10 Hz

        self.get_logger().info('AI پل نوڈ شروع کیا گیا')

    def joint_state_callback(self, msg):
        """جوائنٹ اسٹیٹ میسیجز کو پروسیس کریں"""
        self.current_joint_states = msg
        self.get_logger().debug(f'{len(msg.name)} جوائنٹس کے لیے جوائنٹ اسٹیٹس موصول ہوئے')

    def imu_callback(self, msg):
        """IMU ڈیٹا کو پروسیس کریں"""
        self.imu_data = msg.data
        self.get_logger().debug(f'IMU ڈیٹا موصول ہوا: {msg.data}')

    def process_ai_step(self):
        """AI الگورتھم کا ایک قدم پروسیس کریں"""
        # اصل سسٹم میں، یہ آپ کے AI ایجنٹ کو کال کرے گا
        # فی الحال، ہم ایک سادہ بیلنس کنٹرولر نافذ کریں گے

        if self.imu_data is not None:
            # سادہ مثال: اگر روبوٹ جھک رہا ہے، تو درستگی کی کمانڈ بھیجیں
            try:
                # سٹرنگ IMU ڈیٹا کو عددی ویلیوز میں تبدیل کریں
                tilt_angle = float(self.imu_data)

                if abs(tilt_angle) > 0.5:  # اگر 0.5 ریڈین سے زیادہ جھک رہا ہے
                    # درستگی کے جوائنٹ کمانڈز بھیجیں
                    cmd_msg = JointState()
                    cmd_msg.header.stamp = self.get_clock().now().to_msg()
                    cmd_msg.name = ['left_ankle_pitch', 'right_ankle_pitch']
                    cmd_msg.position = [-tilt_angle * 0.5, -tilt_angle * 0.5]  # درستگی کا ٹورک

                    self.joint_cmd_publisher.publish(cmd_msg)
                    self.get_logger().info(f'جھکاؤ کے لیے درستگی کی کمانڈز بھیجی گئیں: {tilt_angle}')
            except ValueError:
                self.get_logger().error(f'IMU ڈیٹا کو پارس نہیں کیا جا سکا: {self.imu_data}')

def main(args=None):
    rclpy.init(args=args)
    ai_bridge_node = AIBridgeNode()

    try:
        rclpy.spin(ai_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## AI ایجنٹ انضمام نافذ کرنا

### سادہ AI ایجنٹ مثال

```python
import numpy as np
from sklearn.linear_model import LinearRegression

class SimpleAIAgent:
    def __init__(self):
        # اصل سسٹم میں، یہ ایک نیورل نیٹ ورک یا دیگر ML ماڈل ہو سکتا ہے
        self.model = LinearRegression()
        self.is_trained = False

        # ہیومینائیڈ بیلنس کنٹرول کے لیے
        self.balance_history = []
        self.max_history = 100  # آخری 100 نمونے محفوظ کریں

    def predict_control(self, sensor_data):
        """
        سینسر ڈیٹا کو دیکھتے ہوئے، مناسب کنٹرول ایکشنز کی پیشن گوئی کریں
        sensor_data: سینسر ریڈنگز پر مشتمل ڈکٹ
        """
        if not self.is_trained:
            # غیر تربیت یافتہ ماڈل کے لیے، سادہ تناسب کنٹرول لوٹائیں
            tilt = sensor_data.get('tilt', 0.0)
            return {'left_ankle_torque': -tilt * 0.5, 'right_ankle_torque': -tilt * 0.5}

        # تربیت یافتہ ماڈل کا استعمال کنٹرول کی پیشن گوئی کے لیے
        # یہ ایک سادہ مثال ہے
        features = np.array([sensor_data['tilt'], sensor_data['angular_velocity']]).reshape(1, -1)
        control_output = self.model.predict(features)

        return {
            'left_ankle_torque': float(control_output[0]),
            'right_ankle_torque': float(control_output[1])
        }

    def add_training_data(self, sensor_data, control_output):
        """مستقبل کی سیکھنے کے لیے تربیت کا ڈیٹا شامل کریں"""
        self.balance_history.append({
            'sensor': sensor_data.copy(),
            'control': control_output.copy()
        })

        # صرف حالیہ تاریخ کو رکھیں
        if len(self.balance_history) > self.max_history:
            self.balance_history.pop(0)

    def train_model(self):
        """جمع کردہ ڈیٹا کے ساتھ ماڈل کو تربیت دیں"""
        if len(self.balance_history) < 10:  # کم از کم ڈیٹا کی ضرورت ہے
            return False

        # تربیت کا ڈیٹا تیار کریں
        X = []  # سینسر ان پٹس
        y = []  # کنٹرول آؤٹ پٹس

        for sample in self.balance_history:
            sensor_data = sample['sensor']
            control_data = sample['control']

            X.append([sensor_data['tilt'], sensor_data['angular_velocity']])
            y.append([control_data['left_ankle_torque'], control_data['right_ankle_torque']])

        X = np.array(X)
        y = np.array(y)

        # ماڈل کو تربیت دیں
        self.model.fit(X, y)
        self.is_trained = True

        return True
```

## AI انضمام کے ساتھ اعلیٰ درجے کا پل نوڈ

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from builtin_interfaces.msg import Time
import numpy as np
import time

class AdvancedAIBridgeNode(Node):
    def __init__(self):
        super().__init__('advanced_ai_bridge_node')

        # پبلشرز
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)
        self.ai_feedback_publisher = self.create_publisher(Float32, '/ai_control_effort', 10)

        # سبسکرائبرز
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        self.force_torque_subscriber = self.create_subscription(
            String,  # عمل میں، یہ WrenchStamped یا اسی طرح کا استعمال کرے گا
            '/ft_sensors',
            self.ft_callback,
            10
        )

        # AI ایجنٹ کو شروع کریں
        self.ai_agent = SimpleAIAgent()

        # اسٹیٹ متغیرات
        self.current_joint_states = JointState()
        self.current_imu = None
        self.ft_data = None
        self.last_control_time = time.time()

        # پروسیسنگ ٹائمر
        self.processing_timer = self.create_timer(0.05, self.process_ai_step)  # 20 Hz

        # تربیت کا ٹائمر (آہستہ، مستقل سیکھنے کے لیے)
        self.training_timer = self.create_timer(5.0, self.train_model_if_needed)

        self.get_logger().info('اعلیٰ درجے کا AI پل نوڈ شروع کیا گیا')

    def joint_state_callback(self, msg):
        """جوائنٹ اسٹیٹ اپ ڈیٹس کو ہینڈل کریں"""
        self.current_joint_states = msg

    def imu_callback(self, msg):
        """IMU ڈیٹا اپ ڈیٹس کو ہینڈل کریں"""
        self.current_imu = msg

    def ft_callback(self, msg):
        """فورس/ٹورک سینسر اپ ڈیٹس کو ہینڈل کریں"""
        self.ft_data = msg.data  # سادہ سٹرنگ نمائندگی

    def process_ai_step(self):
        """مرکزی AI پروسیسنگ قدم"""
        # حالیہ سینسر ڈیٹا جمع کریں
        sensor_data = self.get_sensor_data()

        if sensor_data is None:
            # آگے بڑھنے کے لیے کافی ڈیٹا نہیں
            return

        try:
            # AI پیشن گوئی حاصل کریں
            control_output = self.ai_agent.predict_control(sensor_data)

            # کنٹرول کمانڈز انجام دیں
            self.execute_control_commands(control_output)

            # کنٹرول کوشش کا حساب لگائیں اور فیڈ بیک شائع کریں
            effort = self.calculate_control_effort(control_output)
            effort_msg = Float32()
            effort_msg.data = effort
            self.ai_feedback_publisher.publish(effort_msg)

            # اختیاری طور پر تربیت کا ڈیٹا محفوظ کریں
            self.store_training_data(sensor_data, control_output)

            # اگلے کنٹرول قدم کے لیے ٹائمنگ اپ ڈیٹ کریں
            self.last_control_time = time.time()

        except Exception as e:
            self.get_logger().error(f'AI پروسیسنگ میں خامی: {str(e)}')

    def get_sensor_data(self):
        """AI ایجنٹ کے لیے متعلقہ سینسر ڈیٹا نکالیں"""
        if self.current_imu is None:
            return None

        # سینسرز سے متعلقہ معلومات نکالیں
        sensor_data = {
            'tilt': self.current_imu.orientation.z,  # سادہ - اصل سسٹمز میں، مناسب اورینٹیشن استعمال کیا جائے گا
            'angular_velocity': self.current_imu.angular_velocity.z,
            'linear_acceleration': self.current_imu.linear_acceleration.x,
            'joint_positions': dict(zip(self.current_joint_states.name, self.current_joint_states.position)),
            'joint_velocities': dict(zip(self.current_joint_states.name, self.current_joint_states.velocity))
        }

        # ٹائم بیسڈ خصوصیات شامل کریں
        dt = time.time() - self.last_control_time
        sensor_data['dt'] = dt

        return sensor_data

    def execute_control_commands(self, control_output):
        """AI ایجنٹ سے کنٹرول کمانڈز انجام دیں"""
        # جوائنٹ کمانڈ میسیج تخلیق کریں
        joint_cmd_msg = JointState()
        joint_cmd_msg.header.stamp = self.get_clock().now().to_msg()

        # کمانڈ کردہ جوائنٹ پوزیشنز/ٹورکس شامل کریں
        for joint_name, torque_value in control_output.items():
            if 'torque' in joint_name:
                # یہ ایک ٹورک کمانڈ ہے
                joint_name_clean = joint_name.replace('_torque', '')
                joint_cmd_msg.name.append(joint_name_clean)
                joint_cmd_msg.effort.append(torque_value)
            elif 'position' in joint_name:
                # یہ ایک پوزیشن کمانڈ ہے
                joint_name_clean = joint_name.replace('_position', '')
                joint_cmd_msg.name.append(joint_name_clean)
                joint_cmd_msg.position.append(torque_value)

        # جوائنٹ کمانڈز شائع کریں
        if len(joint_cmd_msg.name) > 0:
            self.joint_cmd_publisher.publish(joint_cmd_msg)

    def calculate_control_effort(self, control_output):
        """کنٹرول کوشش کا پیمانہ حساب لگائیں"""
        effort = 0.0
        for value in control_output.values():
            effort += abs(value)
        return effort

    def store_training_data(self, sensor_data, control_output):
        """مستقبل کی تربیت کے لیے ڈیٹا محفوظ کریں"""
        self.ai_agent.add_training_data(sensor_data, control_output)

    def train_model_if_needed(self):
        """دورانیہ کے بعد اگر کافی ڈیٹا دستیاب ہو تو ماڈل کو تربیت دینے کی کوشش کریں"""
        success = self.ai_agent.train_model()
        if success:
            self.get_logger().info('AI ماڈل کامیابی سے دوبارہ تربیت یافتہ')
        else:
            self.get_logger().debug('AI ماڈل کو دوبارہ تربیت دینے کے لیے کافی ڈیٹا نہیں')

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedAIBridgeNode()

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

## حقیقی دنیا کے غور

### کارکردگی کی بہتری

پائی تھون AI ایجنٹس کو ROS 2 کنٹرولرز سے جوڑتے وقت، کارکردگی انتہائی اہم ہے:

```python
import threading
from queue import Queue, Empty
import numpy as np

class OptimizedAIBridgeNode(Node):
    def __init__(self):
        super().__init__('optimized_ai_bridge_node')

        # پبلشرز اور سبسکرائبرز (پچھلی مثال کی طرح)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joint_cmd_publisher = self.create_publisher(JointState, '/joint_commands', 10)

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # AI پروسیسنگ کے لیے الگ تھریڈ
        self.ai_input_queue = Queue(maxsize=10)  # قیو سائز کو محدود کریں
        self.ai_output_queue = Queue(maxsize=10)

        # AI پروسیسنگ تھریڈ شروع کریں
        self.ai_thread = threading.Thread(target=self.ai_processing_loop, daemon=True)
        self.ai_thread.start()

        # سینسر ڈیٹا پروسیسنگ کے لیے ٹائمر
        self.sensor_timer = self.create_timer(0.02, self.process_sensor_data)  # 50 Hz
        self.get_logger().info('بہتر شدہ AI پل نوڈ شروع کیا گیا')

    def joint_state_callback(self, msg):
        """غیر مسدود کن سینسر ڈیٹا پروسیسنگ"""
        try:
            # کارکردگی کے لیے نیمپائی ارے میں تبدیل کریں
            joint_pos = np.array(msg.position)
            joint_vel = np.array(msg.velocity)

            # سینسر ڈیٹا پیکٹ تیار کریں
            sensor_data = {
                'timestamp': time.time(),
                'joint_positions': joint_pos,
                'joint_velocities': joint_vel,
                'joint_names': msg.name
            }

            # AI ان پٹ قیو میں شامل کریں اگر جگہ ہو
            try:
                self.ai_input_queue.put_nowait(sensor_data)
            except:
                # قیو بھر گئی ہے، پرانے ڈیٹا کو چھوڑ دیں
                try:
                    self.ai_input_queue.get_nowait()
                    self.ai_input_queue.put_nowait(sensor_data)
                except:
                    pass  # اب بھی بھر گئی ہے، اس ڈیٹا پوائنٹ کو چھوڑ دیں
        except Exception as e:
            self.get_logger().error(f'جوائنٹ کال بیک میں خامی: {str(e)}')

    def ai_processing_loop(self):
        """AI پروسیسنگ کے لیے مخصوص تھریڈ"""
        while rclpy.ok():
            try:
                # سب سے حالیہ سینسر ڈیٹا حاصل کریں
                sensor_data = None
                while True:
                    try:
                        sensor_data = self.ai_input_queue.get_nowait()
                    except Empty:
                        break  # مزید حالیہ ڈیٹا نہیں

                if sensor_data is not None:
                    # AI ماڈل کے ساتھ پروسیس کریں (یہ کمپیوٹیشنل طور پر مہنگا ہو سکتا ہے)
                    control_output = self.process_with_ai(sensor_data)

                    # آؤٹ پٹ قیو میں شامل کریں
                    try:
                        self.ai_output_queue.put_nowait(control_output)
                    except:
                        # آؤٹ پٹ قیو بھر گئی ہے، نتیجہ چھوڑ دیں
                        pass

            except Exception as e:
                self.get_logger().error(f'AI تھریڈ میں خامی: {str(e)}')

            # مصروف انتظار کو روکنے کے لیے مختصر سلیپ
            time.sleep(0.001)

    def process_with_ai(self, sensor_data):
        """AI پروسیسنگ فنکشن (الگ تھریڈ میں چلتا ہے)"""
        # یہاں وہ بھاری AI کمپیوٹیشن ہوتی ہے
        # مثال کے طور پر، ایک نیورل نیٹ ورک چلانا
        pass

    def process_sensor_data(self):
        """سینسر ڈیٹا کو پروسیس کریں اور کمانڈز بھیجیں"""
        try:
            # سب سے حالیہ AI آؤٹ پٹ حاصل کریں
            ai_output = None
            while True:
                try:
                    ai_output = self.ai_output_queue.get_nowait()
                except Empty:
                    break  # مزید حالیہ آؤٹ پٹس نہیں

            if ai_output is not None:
                # AI کے ذریعے تیار کردہ کمانڈز انجام دیں
                self.execute_control_commands(ai_output)
        except Exception as e:
            self.get_logger().error(f'سینسر ٹائمر میں خامی: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedAIBridgeNode()

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

## خامیوں کا سامنا اور بحالی

 مضبوط AI-ROS پلز کو خامیوں کو نرمی سے سنبھالنا چاہیے:

```python
import traceback
from rclpy.qos import QoSProfile, ReliabilityPolicy

class RobustAIBridgeNode(Node):
    def __init__(self):
        super().__init__('robust_ai_bridge_node')

        # قابل اعتماد ترسیل کے لیے کسٹم QoS کے ساتھ پبلشرز
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile)

        # سبسکرائبرز (خامیوں کے سامنے)
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.safe_joint_state_callback,
            qos_profile
        )

        # اسٹیٹ ٹریکنگ
        self.system_mode = 'normal'  # normal, degraded, emergency
        self.error_count = 0
        self.max_errors_before_recovery = 5

        # بحالی ٹائمر
        self.recovery_timer = self.create_timer(1.0, self.monitor_system_health)

        self.get_logger().info('مضبوط AI پل نوڈ شروع کیا گیا')

    def safe_joint_state_callback(self, msg):
        """خامیوں کے سامنے محفوظ کال بیک"""
        try:
            self.joint_state_callback(msg)
        except Exception as e:
            self.error_count += 1
            error_msg = f'Joint state callback error: {str(e)}\n{traceback.format_exc()}'
            self.get_logger().error(error_msg)
            self.trigger_error_handling()

    def trigger_error_handling(self):
        """خامیوں کو مناسب طریقے سے ہینڈل کریں"""
        if self.error_count >= self.max_errors_before_recovery:
            self.get_logger().error('بہت زیادہ خامیاں، ہنگامی موڈ میں داخل ہو رہا ہے')
            self.system_mode = 'emergency'
            self.emergency_stop()
        elif self.error_count >= self.max_errors_before_recovery // 2:
            self.get_logger().warn('زیادہ خامی کی شرح، تنزل موڈ میں داخل ہو رہا ہے')
            self.system_mode = 'degraded'

    def emergency_stop(self):
        """تمام روبوٹ موشن کو روکیں"""
        # صفر رفتار کمانڈز شائع کریں
        stop_cmd = Twist()
        self.cmd_vel_publisher.publish(stop_cmd)

        # محفوظ پوزیشنز پر جوائنٹ کمانڈز ری سیٹ کریں
        safe_joint_cmd = JointState()
        safe_joint_cmd.header.stamp = self.get_clock().now().to_msg()
        # یہاں محفوظ جوائنٹ پوزیشنز شامل کریں
        self.joint_cmd_publisher.publish(safe_joint_cmd)

    def monitor_system_health(self):
        """سسٹم کی صحت کو مانیٹر کریں اور بحالی کی کوشش کریں"""
        if self.system_mode == 'emergency':
            self.get_logger().info('سسٹم بحالی کی کوشش کر رہا ہے...')
            # بحالی کی اجازت دینے کے لیے خامی کی تعداد ری سیٹ کریں
            self.error_count = 0
            self.system_mode = 'normal'
            self.get_logger().info('سسٹم بحالی کی کوشش کی گئی')
```

## AI-ROS انضمام کے لیے بہترین طریقے

### 1. AI پروسیسنگ کو الگ رکھیں

ROS مواصلت کو مسدود کرنے سے بچنے کے لیے الگ تھریڈز یا عملوں کا استعمال کریں AI کمپیوٹیشن کے لیے۔

### 2. مناسب QoS ترتیبات استعمال کریں

کنٹرول کمانڈز کے لیے قابل اعتماد ترسیل استعمال کریں۔ سینسر ڈیٹا کے لیے، بہترین کوشش کافی ہو سکتی ہے۔

### 3. مناسب خامی کا سامنا کریں

ہمیشہ AI پروسیسنگ کے ارد گرد try-catch بلاکس شامل کریں اور بحالی کی حکمت عملیاں نافذ کریں۔

### 4. کارکردگی کی نگرانی کریں

پروسیسنگ ٹائمز اور سسٹم لوڈ کو ٹریک کریں تاکہ حقیقی وقت کی کارکردگی یقینی بنائی جا سکے۔

### 5. تفصیلی طور پر لاگ کریں

AI-ROS تعامل سے پیدا ہونے والے مسائل کو ڈیبگ کرنے کے لیے تفصیلی لاگز رکھیں۔

## خلاصہ

پائی تھون AI ایجنٹس کو ROS 2 کنٹرولرز سے جوڑنا جدید ہیومینائیڈ روبوٹس کے لیے ایک اہم صلاحیت ہے۔ اس باب میں بنیادی تصورات پر بات کی گئی:

- AI سسٹمز اور ROS 2 کے درمیان انٹرفیس کرنے والے نوڈز تخلیق کرنے کے لیے rclpy کا استعمال
- AI اور کنٹرول سسٹمز کے درمیان مناسب رابطے کے نمونے نافذ کرنا
- تھریڈنگ اور قیو کے ساتھ کارکردگی کو بہتر بنانا
- خامی کا سامنا کرنا اور بحالی کی حکمت عملیاں نافذ کرنا
- مضبوط انضمام کے لیے بہترین طریقے کا پیرو کرنا

AI الگورتھم اور حقیقی روبوٹ کنٹرول کے درمیان پل وہ جگہ ہے جہاں روبوٹ کی ذہانت جسمانی دنیا سے ملتی ہے۔ اس انٹرفیس کا مناسب ڈیزائن ہیومینائیڈ روبوٹ کے محفوظ، قابل اعتماد، اور مؤثر آپریشن کے لیے انتہائی اہم ہے۔

## مشقیں

1. ایک سادہ پل نوڈ تخلیق کریں جو جوائنٹ اسٹیٹس پڑھتا ہے اور ایک سادہ کنٹرول پالیسی کمپیوٹ کرتا ہے
2. اپنے پل میں خامی کا سامنا کریں تاکہ سینسر کی ناکامیوں کو نرمی سے ہینڈل کیا جا سکے
3. ایک فیڈ بیک لوپ شامل کریں جو AI کے رویے کو کنٹرول کارکردگی کی بنیاد پر ایڈجسٹ کرتا ہے

## اگلے اقدامات

اگلے باب میں، ہم URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ) کا استعمال کرتے ہوئے ہیومینائیڈ روبوٹس کو ماڈل کرنے کی تلاش کریں گے، جو ROS 2 میں سیمولیشن اور وژولائزیشن کے لیے ضروری ہے۔ ہم دیکھیں گے کہ جو جوائنٹ سٹرکچرز ہم نے بات کی تھی، وہ روبوٹ کے جسمانی ماڈل سے کیسے جڑتے ہیں۔