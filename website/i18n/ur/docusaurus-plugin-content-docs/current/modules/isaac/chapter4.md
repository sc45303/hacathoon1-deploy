---
sidebar_position: 5
---

# باب 4: اعلیٰ درجے کی AI روبوٹ دماغ کی تکنیکیں

## سیکھنے کے مقاصد

- ہیومینائیڈ روبوٹ ادراک اور فیصلہ سازی کے لیے اعلیٰ درجے کی AI تکنیکوں کو سمجھیں
- ہیومینائیڈ لوکوموشن کے لیے مضبوط سیکھنے کی تکنیکوں کے بارے میں سیکھیں
- Isaac ROS ادراک کے ساتھ ڈیپ لرننگ کا انضمام تلاش کریں
- AI کا استعمال کرتے ہوئے ایڈاپٹیو کنٹرول سسٹمز نافذ کریں
- روبوٹکس ایپلی کیشنز کے لیے نیورل آرکیٹیکچر سرچ سمجھیں

## اعلیٰ درجے کی AI تکنیکوں کا تعارف

ہیومینائیڈ روبوٹس کو پیچیدہ ماحول میں ادراک، منطق اور عمل کرنے کے لیے ترقی یافتہ AI سسٹمز کی ضرورت ہوتی ہے۔ یہ باب اعلیٰ درجے کی تکنیکوں کو تلاش کرتا ہے جو بنیادی ادراک اور نیویگیشن سے آگے جاتی ہیں، ان سسٹمز پر توجہ مرکز کرتا ہے جو سیکھ سکتے ہیں، ایڈاپٹ کر سکتے ہیں، اور پیچیدہ فیصلے کر سکتے ہیں۔

### ہیومینائیڈ روبوٹکس کے لیے اعلیٰ درجے کی AI کے کلیدی علاقوں

1. **سیکھنے مبنی لوکوموشن**: ایڈاپٹیو چلنے کے نمونے تیار کرنے کے لیے AI کا استعمال
2. **ادراک ایکشن انضمام**: ڈیپ لرننگ سسٹمز جو ادراک کو ایکشن سے جوڑتے ہیں
3. **ایڈاپٹیو کنٹرول**: AI سسٹمز جو حقیقی وقت میں کنٹرول پیرامیٹرز ایڈجسٹ کرتے ہیں
4. **سلسلہ وار فیصلہ سازی**: پیچیدہ کاموں کے لیے ملٹی لیول AI سسٹمز
5. **سم ٹو ریل ٹرانسفر**: سیکھے ہوئے رویوں کو سیمولیشن سے حقیقی روبوٹس میں منتقل کرنے کی تکنیکیں

## ہیومینائیڈ لوکوموشن کے لیے مضبوط سیکھنا

مضبوط سیکھنا (RL) ہیومینائیڈ لوکوموشن کنٹرولرز کے ترقی دینے میں نمایاں کامیابی دکھائی ہے۔ روایتی کنٹرول طریقوں کے برعکس، RL پیچیدہ گیٹ پیٹرنز سیکھ سکتا ہے اور مختلف زمینوں کے مطابق ایڈاپٹ کر سکتا ہے۔

### ڈیپ مضبوط سیکھنا فریم ورک

```python
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from collections import deque
import random

class ActorNetwork(nn.Module):
    """ہیومینائیڈ کنٹرول پالیسی کے لیے ایکٹر نیٹ ورک"""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(ActorNetwork, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(state_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
            nn.Tanh()  # ایکشن [-1, 1] تک محدود ہیں
        )

    def forward(self, state):
        return self.network(state)

class CriticNetwork(nn.Module):
    """ویلیو اسٹیمیشن کے لیے کریٹک نیٹ ورک"""
    def __init__(self, state_dim, action_dim, hidden_dim=256):
        super(CriticNetwork, self).__init__()

        self.network = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1)
        )

    def forward(self, state, action):
        x = torch.cat([state, action], dim=1)
        return self.network(x)

class HumanoidRLAgent:
    def __init__(self, state_dim, action_dim, lr=3e-4, gamma=0.99, tau=0.005):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # نیٹ ورکس
        self.actor = ActorNetwork(state_dim, action_dim).to(self.device)
        self.critic = CriticNetwork(state_dim, action_dim).to(self.device)
        self.target_actor = ActorNetwork(state_dim, action_dim).to(self.device)
        self.target_critic = CriticNetwork(state_dim, action_dim).to(self.device)

        # آپٹیمائزرز
        self.actor_optimizer = optim.Adam(self.actor.parameters(), lr=lr)
        self.critic_optimizer = optim.Adam(self.critic.parameters(), lr=lr)

        # ہائپر پیرامیٹرز
        self.gamma = gamma  # ڈسکاؤنٹ فیکٹر
        self.tau = tau      # سافٹ اپ ڈیٹ پیرامیٹر
        self.action_dim = action_dim

        # ٹارگیٹ نیٹ ورکس کو شروع کریں
        self.hard_update(self.target_actor, self.actor)
        self.hard_update(self.target_critic, self.critic)

    def hard_update(self, target, source):
        """سورس پیرامیٹرز کے ساتھ ٹارگیٹ نیٹ ورک کو ہارڈ اپ ڈیٹ کریں"""
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(param.data)

    def soft_update(self, target, source):
        """سورس پیرامیٹرز کے ساتھ ٹارگیٹ نیٹ ورک کو سافٹ اپ ڈیٹ کریں"""
        for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(self.tau * param.data + (1.0 - self.tau) * target_param.data)

    def select_action(self, state, add_noise=False, noise_scale=0.1):
        """ایکسپلوریشن نوائز کے ساتھ ایکشن منتخب کریں"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0).to(self.device)

        with torch.no_grad():
            action = self.actor(state_tensor)

        if add_noise:
            noise = torch.randn_like(action) * noise_scale
            action = torch.clamp(action + noise, -1, 1)

        return action.cpu().numpy()[0]

    def update(self, replay_buffer, batch_size=100):
        """ریپلے بفر سے تجربات کے ساتھ نیٹ ورکس کو اپ ڈیٹ کریں"""
        if len(replay_buffer) < batch_size:
            return

        # بیچ نمونہ
        states, actions, rewards, next_states, dones = replay_buffer.sample(batch_size)

        states = torch.FloatTensor(states).to(self.device)
        actions = torch.FloatTensor(actions).to(self.device)
        rewards = torch.FloatTensor(rewards).unsqueeze(1).to(self.device)
        next_states = torch.FloatTensor(next_states).to(self.device)
        dones = torch.BoolTensor(dones).unsqueeze(1).to(self.device)

        # کریٹک اپ ڈیٹ کریں
        with torch.no_grad():
            next_actions = self.target_actor(next_states)
            next_q_values = self.target_critic(next_states, next_actions)
            target_q_values = rewards + (self.gamma * next_q_values * ~dones)

        current_q_values = self.critic(states, actions)
        critic_loss = nn.MSELoss()(current_q_values, target_q_values)

        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        # ایکٹر اپ ڈیٹ کریں
        predicted_actions = self.actor(states)
        actor_loss = -self.critic(states, predicted_actions).mean()

        self.actor_optimizer.zero_grad()
        actor_loss.backward()
        self.actor_optimizer.step()

        # سافٹ اپ ڈیٹ ٹارگیٹ نیٹ ورکس
        self.soft_update(self.target_actor, self.actor)
        self.soft_update(self.target_critic, self.critic)

class ReplayBuffer:
    """RL تربیت کے لیے تجربہ ریپلے بفر"""
    def __init__(self, capacity=1000000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        """بفر میں تجربہ شامل کریں"""
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        """بفر سے بیچ نمونہ لیں"""
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done

    def __len__(self):
        return len(self.buffer)
```

### RL تربیت کے لیے ہیومینائیڈ ماحول

```python
import gym
from gym import spaces
import numpy as np

class HumanoidLocomotionEnv(gym.Env):
    """ہیومینائیڈ لوکوموشن تربیت کے لیے کسٹم ماحول"""
    def __init__(self):
        super(HumanoidLocomotionEnv, self).__init__()

        # ایکشن اور مشاہدہ کی جگہوں کی وضاحت کریں
        # یہ ایک سادہ مثال ہے - حقیقی ماحولوں میں زیادہ پیچیدہ جگہیں ہوں گی
        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(19,), dtype=np.float32  # 19 جوائنٹس
        )

        # مشاہدہ جگہ: جوائنٹ پوزیشنز، رفتار، IMU ریڈنگز
        obs_dim = 48  # مثال طور پر جگہ
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_dim,), dtype=np.float32
        )

        # ہیومینائیڈ روبوٹ انٹرفیس (سادہ)
        self.robot = None  # Gazebo، PyBullet وغیرہ کے ساتھ انٹرفیس کرے گا

        # ایپی سوڈ پیرامیٹرز
        self.max_episode_steps = 1000
        self.current_step = 0
        self.target_velocity = 0.5  # میٹر/سیکنڈ

    def reset(self):
        """ماحول کو ری سیٹ کریں"""
        # روبوٹ کو ابتدائی پوز پر ری سیٹ کریں
        self._reset_robot()
        self.current_step = 0

        # ابتدائی مشاہدہ واپس کریں
        return self._get_observation()

    def step(self, action):
        """دیے گئے ایکشن کے ساتھ ایک قدم انجام دیں"""
        # روبوٹ پر ایکشن لاگو کریں
        self._apply_action(action)

        # سیمولیشن کا قدم
        self._step_simulation()

        # نیا مشاہدہ حاصل کریں
        observation = self._get_observation()

        # انعام کا حساب لگائیں
        reward = self._calculate_reward()

        # ختم ہونے کی جانچ کریں
        done = self._is_done()
        info = {}

        self.current_step += 1

        return observation, reward, done, info

    def _get_observation(self):
        """روبوٹ سے موجودہ مشاہدہ حاصل کریں"""
        # یہ روبوٹ کے سینسرز کے ساتھ انٹرفیس کرے گا
        # مثالی مشاہدات: جوائنٹ اینگلز، رفتار، IMU ڈیٹا وغیرہ
        observation = np.zeros(48, dtype=np.float32)  # سادہ
        return observation

    def _calculate_reward(self):
        """موجودہ حالت کی بنیاد پر انعام کا حساب لگائیں"""
        # فارورڈ رفتار کے لیے انعام
        forward_vel_reward = self._get_forward_velocity() * 0.1

        # توانائی کی کھپت کے لیے جرمانہ
        energy_penalty = self._get_energy_consumption() * 0.01

        # بیلنس برقرار رکھنے کے لیے انعام
        balance_reward = self._get_balance_score() * 0.5

        # جوائنٹ حدود کی خلاف ورزیوں کے لیے جرمانہ
        joint_limit_penalty = self._get_joint_limit_violations() * 1.0

        total_reward = forward_vel_reward - energy_penalty + balance_reward - joint_limit_penalty
        return max(total_reward, -10.0)  # انعام کو محدود کریں

    def _get_forward_velocity(self):
        """روبوٹ کی فارورڈ رفتار حاصل کریں"""
        # روبوٹ کے اوڈومیٹر کے ساتھ انٹرفیس کرے گا
        return 0.0  # سادہ

    def _get_energy_consumption(self):
        """توانائی کی کھپت حاصل کریں"""
        # جوائنٹ ٹارک اور رفتار کی بنیاد پر حساب لگائے گا
        return 0.0  # سادہ

    def _get_balance_score(self):
        """بیلنس اسکور حاصل کریں (زیادہ بہتر ہے)"""
        # COM پوزیشن، IMU ریڈنگز وغیرہ کی بنیاد پر حساب لگائیں
        return 0.0  # سادہ

    def _get_joint_limit_violations(self):
        """جوائنٹ حدود کی خلاف ورزیوں کو گنیں"""
        # موجودہ جوائنٹ پوزیشنز کو حدود کے خلاف چیک کریں
        return 0.0  # سادہ

    def _is_done(self):
        """چیک کریں کہ ایپی سوڈ ختم ہو گیا ہے"""
        # اگر گر گیا، زیادہ سے زیادہ اسٹیپس سے تجاوز کر گیا، یا دیگر ناکامی کی حالتیں
        return self.current_step >= self.max_episode_steps

    def _apply_action(self, action):
        """روبوٹ پر ایکشن لاگو کریں"""
        # نارملائزڈ ایکشن کو جوائنٹ کمانڈز میں تبدیل کریں
        # یہ روبوٹ کنٹرولر کے ساتھ انٹرفیس کرے گا
        pass

    def _step_simulation(self):
        """فزکس سیمولیشن کا قدم لگائیں"""
        # یہ فزکس انجن کے ساتھ انٹرفیس کرے گا
        pass

    def _reset_robot(self):
        """روبوٹ کو ابتدائی کنفیگریشن پر ری سیٹ کریں"""
        # روبوٹ پوز، رفتار وغیرہ کو ری سیٹ کریں
        pass
```

### ہیومینائیڈ لوکوموشن کے لیے تربیتی لوپ

```python
def train_humanoid_locomotion():
    """ہیومینائیڈ لوکوموشن پالیسی کے لیے تربیتی لوپ"""
    env = HumanoidLocomotionEnv()

    # ایجنٹ کو شروع کریں
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.shape[0]
    agent = HumanoidRLAgent(state_dim, action_dim)

    # ریپلے بفر کو شروع کریں
    replay_buffer = ReplayBuffer(capacity=1000000)

    # تربیتی پیرامیٹرز
    num_episodes = 2000
    max_steps_per_episode = 1000
    batch_size = 256
    update_every = 50

    scores = []
    avg_scores = []

    for episode in range(num_episodes):
        state = env.reset()
        episode_reward = 0
        episode_steps = 0

        for step in range(max_steps_per_episode):
            # ایکسپلوریشن کے ساتھ ایکشن منتخب کریں
            action = agent.select_action(state, add_noise=True, noise_scale=0.1)

            # ایکشن لیں
            next_state, reward, done, info = env.step(action)

            # تجربہ ذخیرہ کریں
            replay_buffer.push(state, action, reward, next_state, done)

            # ایجنٹ کو اپ ڈیٹ کریں
            if len(replay_buffer) > batch_size and step % update_every == 0:
                agent.update(replay_buffer, batch_size)

            state = next_state
            episode_reward += reward
            episode_steps += 1

            if done:
                break

        scores.append(episode_reward)

        # آخری 100 ایپی سوڈز پر اوسط اسکور کا حساب لگائیں
        if len(scores) >= 100:
            avg_score = sum(scores[-100:]) / 100
            avg_scores.append(avg_score)
        else:
            avg_scores.append(sum(scores) / len(scores))

        print(f"ایپی سوڈ {episode}, اسکور: {episode_reward:.2f}, "
              f"اوسط اسکور: {avg_scores[-1]:.2f}")

    # تربیت یافتہ ماڈل کو محفوظ کریں
    torch.save(agent.actor.state_dict(), "humanoid_locomotion_actor.pth")
    torch.save(agent.critic.state_dict(), "humanoid_locomotion_critic.pth")

    return agent, scores, avg_scores
```

## Isaac ROS ڈیپ لرننگ انضمام

Isaac ROS GPU ایکسلریٹڈ ڈیپ لرننگ کی صلاحیتیں فراہم کرتا ہے جو ہیومینائیڈ کنٹرول سسٹمز کے ساتھ مربوط کی جا سکتی ہیں:

### ادراک ایکشن انضمام

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, Imu
from geometry_msgs.msg import Twist
import torch
import torchvision.transforms as T
from PIL import Image as PILImage
import io
import cv2

class PerceptionActionNode(Node):
    def __init__(self):
        super().__init__('perception_action_node')

        # سبسکرپشنز
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)

        # ایکشن کمانڈز کے لیے پبلشر
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # پیش تربیت یافتہ ماڈلز لوڈ کریں
        self.perception_model = self.load_perception_model()
        self.action_model = self.load_action_model()

        # تصاویر کے لیے ٹرانسفارمیشن
        self.transform = T.Compose([
            T.Resize((224, 224)),
            T.ToTensor(),
            T.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        # اسٹیٹ بفرز
        self.current_img = None
        self.current_depth = None
        self.current_imu = None

        self.get_logger().info('ادراک-ایکشن نوڈ شروع کیا گیا')

    def load_perception_model(self):
        """پیش تربیت یافتہ ادراک ماڈل لوڈ کریں"""
        # حقیقت میں، یہ Isaac ROS یا دیگر ذرائع سے ماڈل لوڈ کرے گا
        # مثال کے طور پر، ایک آبجیکٹ ڈیٹیکشن یا سیگمینٹیشن ماڈل
        import torchvision.models as models
        model = models.resnet18(pretrained=True)
        model.eval()
        return model

    def load_action_model(self):
        """ایکشن منتخب کرنے والا ماڈل لوڈ کریں"""
        # یہ ادراک کو ایکشنز میں میپ کرنے والا ماڈل ہوگا
        # اوپر تربیت یافتہ RL پالیسی ہو سکتی ہے
        import torch.nn as nn

        class ActionModel(nn.Module):
            def __init__(self, perception_features_dim, action_dim):
                super(ActionModel, self).__init__()
                self.fc1 = nn.Linear(perception_features_dim, 256)
                self.relu = nn.ReLU()
                self.fc2 = nn.Linear(256, 128)
                self.action_head = nn.Linear(128, action_dim)

            def forward(self, perception_features):
                x = self.relu(self.fc1(perception_features))
                x = self.relu(self.fc2(x))
                action = torch.tanh(self.action_head(x))
                return action

        model = ActionModel(512, 2)  # 512 فیچرز، 2D ایکشن (vx, wz)
        model.eval()
        return model

    def image_callback(self, msg):
        """آنے والی تصویر کو پروسیس کریں"""
        try:
            # ROS تصویر کو PIL تصویر میں تبدیل کریں
            img_data = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, -1)
            pil_img = PILImage.fromarray(img_data)

            # ادراک ماڈل کے ساتھ تصویر کو پروسیس کریں
            with torch.no_grad():
                transformed_img = self.transform(pil_img).unsqueeze(0)
                features = self.extract_features(transformed_img)

                # ادراک کی بنیاد پر ایکشن کا تعین کریں
                action = self.action_model(features)

                # ایکشن انجام دیں
                self.publish_action(action)

        except Exception as e:
            self.get_logger().error(f'تصویر کو پروسیس کرنے میں خامی: {str(e)}')

    def extract_features(self, img_tensor):
        """ادراک ماڈل کا استعمال کرتے ہوئے فیچرز نکالیں"""
        # فیچرز نکالنے کے لیے بطور مثال
        # یہ مخصوص ماڈل آرکیٹیکچر پر منحصر ہوگا
        with torch.no_grad():
            # کنولوشنل لیئرز کے ذریعے چلائیں تاکہ فیچرز نکالے جا سکیں
            x = self.perception_model.conv1(img_tensor)
            x = self.perception_model.bn1(x)
            x = self.perception_model.relu(x)
            x = self.perception_model.maxpool(x)

            x = self.perception_model.layer1(x)
            x = self.perception_model.layer2(x)
            x = self.perception_model.layer3(x)
            x = self.perception_model.layer4(x)

            # گلوبل اوسط پولنگ
            x = torch.nn.functional.adaptive_avg_pool2d(x, (1, 1))
            features = torch.flatten(x, 1)

            return features

    def depth_callback(self, msg):
        """گہرائی کی معلومات کو پروسیس کریں"""
        # نیویگیشن کے لیے گہرائی ڈیٹا کو پروسیس کریں
        pass

    def imu_callback(self, msg):
        """بیلنس کے لیے IMU ڈیٹا کو پروسیس کریں"""
        # بیلنس بیداری کے لیے IMU کو پروسیس کریں
        pass

    def publish_action(self, action_tensor):
        """روبوٹ پر ایکشن انجام دینے کے لیے شائع کریں"""
        cmd_msg = Twist()
        cmd_msg.linear.x = float(action_tensor[0, 0]) * 0.5  # مناسب رفتار تک اسکیل کریں
        cmd_msg.angular.z = float(action_tensor[0, 1]) * 0.5  # مناسب زاویہ وار رفتار تک اسکیل کریں

        self.cmd_vel_pub.publish(cmd_msg)
```

## ایڈاپٹیو کنٹرول سسٹمز

ایڈاپٹیو کنٹرول سسٹمز تبدیل ہوتی حالت یا کارکردگی کے مطابق اپنا رویہ تبدیل کر سکتے ہیں:

### ماڈل ریفرنس ایڈاپٹیو کنٹرول (MRAC)

```python
class MRACController:
    """ہیومینائیڈ روبوٹس کے لیے ماڈل ریفرنس ایڈاپٹیو کنٹرولر"""
    def __init__(self, reference_model_params, plant_params):
        self.reference_model = self.initialize_reference_model(reference_model_params)
        self.plant_params = plant_params

        # ایڈاپٹیو پیرامیٹرز
        self.theta = np.zeros(plant_params.size)  # کنٹرولر پیرامیٹرز
        self.P = np.eye(plant_params.size) * 100  # کوویریئنس میٹرکس
        self.gamma = 1.0  # ایڈاپٹیشن گین

        # اسٹیٹ ٹریکنگ
        self.error = 0.0
        self.prev_error = 0.0
        self.integral_error = 0.0

    def initialize_reference_model(self, params):
        """مطلوبہ رویے کے لیے ریفرنس ماڈل شروع کریں"""
        # یہ ایک ریفرنس ڈائنامک سسٹم تخلیق کرے گا
        # ہیومینائیڈ کے لیے، یہ مثالی چلنے کے ڈائنامکس کی نمائندگی کر سکتا ہے
        class ReferenceModel:
            def __init__(self, params):
                self.params = params
                self.state = 0.0  # سادہ اسٹیٹ

            def update(self, input_signal):
                # ریفرنس ماڈل اسٹیٹ کو اپ ڈیٹ کریں
                # یہ مطلوبہ ڈائنامکس کو لاگو کرتا ہے
                self.state = self.state * 0.9 + input_signal * 0.1  # سادہ
                return self.state

        return ReferenceModel(params)

    def update(self, measured_output, reference_input):
        """نئے پیمائش کے ساتھ کنٹرولر کو اپ ڈیٹ کریں"""
        # ریفرنس آؤٹ پٹ حاصل کریں
        reference_output = self.reference_model.update(reference_input)

        # ٹریکنگ غلطی کا حساب لگائیں
        self.error = reference_output - measured_output

        # پیرامیٹر ایڈجسٹمنٹ کمپیوٹ کریں
        phi = self.get_regression_vector(measured_output, reference_input)
        adjustment = self.gamma * np.outer(self.P, phi) * self.error
        self.theta += adjustment.flatten()

        # کوویریئنس میٹرکس کو اپ ڈیٹ کریں
        denom = 1 + np.dot(phi, np.dot(self.P, phi))
        self.P = self.P - (np.outer(np.dot(self.P, phi), np.dot(phi, self.P))) / denom

        # کنٹرول سگنل کمپیوٹ کریں
        control_signal = np.dot(self.theta, phi)

        # اگلی دہرائی کے لیے اپ ڈیٹ کریں
        self.prev_error = self.error

        return control_signal

    def get_regression_vector(self, y, r):
        """ایڈاپٹیشن لا کے لیے ریگریشن ویکٹر حاصل کریں"""
        # یہ مخصوص پلانٹ ماڈل پر منحصر ہوگا
        # ہیومینائیڈ کے لیے، یہ جوائنٹ کنیمیٹکس/ڈائنامکس سے متعلق ہو سکتا ہے
        phi = np.array([y, r, y*r, y**2, r**2])  # مثالی فیچرز
        return phi[:self.theta.size]  # پیرامیٹر سائز کے مطابق ٹریم کریں
```

### نیورل ایڈاپٹیو کنٹرول

```python
import torch.nn as nn

class NeuralAdaptiveController(nn.Module):
    """پیچیدہ روبوٹک سسٹمز کے لیے نیورل ایڈاپٹیو کنٹرولر"""
    def __init__(self, state_dim, action_dim, hidden_dim=128):
        super(NeuralAdaptiveController, self).__init__()

        # کنٹرولر نیٹ ورک
        self.controller_network = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, action_dim),
        )

        # ایڈاپٹیشن نیٹ ورک (کنٹرولر پیرامیٹرز کو ایڈجسٹ کرنا سیکھتا ہے)
        self.adaptation_network = nn.Sequential(
            nn.Linear(state_dim + action_dim + hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
        )

        # غلطی کی پیشن گوئی نیٹ ورک
        self.error_predictor = nn.Sequential(
            nn.Linear(state_dim + action_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 1),
        )

        self.state_dim = state_dim
        self.action_dim = action_dim

    def forward(self, state, action, internal_state=None):
        """ایڈاپٹیشن کے ساتھ فارورڈ پاس"""
        if internal_state is None:
            internal_state = torch.zeros(state.size(0), self.hidden_dim)

        # کنٹرولر آؤٹ پٹ
        controller_output = self.controller_network(torch.cat([state, action], dim=1))

        # ایڈاپٹیشن
        adaptation_input = torch.cat([state, action, internal_state], dim=1)
        adaptation = self.adaptation_network(adaptation_input)

        # مجموعی آؤٹ پٹ
        adapted_output = controller_output + 0.1 * adaptation  # چھوٹا ایڈاپٹیشن اثر

        # سیکھنے کے لیے غلطی کی پیشن گوئی کریں
        error_prediction = self.error_predictor(torch.cat([state, action], dim=1))

        return adapted_output, adaptation, error_prediction

class AdaptiveControlSystem:
    """ہیومینائیڈ روبوٹس کے لیے مکمل ایڈاپٹیو کنٹرول سسٹم"""
    def __init__(self, state_dim, action_dim):
        self.neural_controller = NeuralAdaptiveController(state_dim, action_dim)
        self.optimizer = optim.Adam(self.neural_controller.parameters(), lr=1e-4)

        # کارکردگی میٹرکس
        self.performance_history = deque(maxlen=100)
        self.adaptation_activity = 0.0

    def compute_control(self, state, desired_action):
        """ایڈاپٹیشن کے ساتھ کنٹرول ایکشن کمپیوٹ کریں"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        action_tensor = torch.FloatTensor(desired_action).unsqueeze(0)

        with torch.no_grad():
            control_output, adaptation, error_pred = self.neural_controller(
                state_tensor, action_tensor)

        return control_output.numpy()[0]

    def update_adaptation(self, state, action, desired_output, actual_output):
        """کارکردگی کی بنیاد پر نیورل ایڈاپٹیشن کو اپ ڈیٹ کریں"""
        state_tensor = torch.FloatTensor(state).unsqueeze(0)
        action_tensor = torch.FloatTensor(action).unsqueeze(0)
        desired_tensor = torch.FloatTensor(desired_output).unsqueeze(0)
        actual_tensor = torch.FloatTensor(actual_output).unsqueeze(0)

        # نقصان کا حساب لگائیں
        tracking_error = desired_tensor - actual_tensor
        error_loss = torch.mean(tracking_error ** 2)

        # غلطی کی پیشن گوئی پر بھی تربیت کریں
        error_pred = self.neural_controller.error_predictor(
            torch.cat([state_tensor, actual_tensor], dim=1))
        prediction_loss = torch.nn.functional.mse_loss(error_pred, tracking_error)

        total_loss = error_loss + 0.1 * prediction_loss

        # اپ ڈیٹ کریں
        self.optimizer.zero_grad()
        total_loss.backward()
        self.optimizer.step()

        # کارکردگی کو ٹریک کریں
        self.performance_history.append(error_loss.item())

        return error_loss.item()
```

## سلسلہ وار فیصلہ سازی

پیچیدہ ہیومینائیڈ کاموں کو اکثر سلسلہ وار فیصلہ سازی سسٹمز کی ضرورت ہوتی ہے:

### ٹاسک اور موشن منصوبہ بندی (TAMP)

```python
class TaskAndMotionPlanner:
    """پیچیدہ ہیومینائیڈ کاموں کے لیے سلسلہ وار منصوبہ ساز"""
    def __init__(self):
        self.task_planner = SymbolicTaskPlanner()
        self.motion_planner = BiPedalMotionPlanner()
        self.high_level_reasoner = HighLevelReasoner()

    def plan_task(self, goal_description):
        """بالا سطحی ٹاسک ڈیکومپوزیشن منصوبہ بندی کریں"""
        # گول کی وضاحت کو پارس کریں
        goal = self.parse_goal(goal_description)

        # ذیلی ٹاسکس میں ڈیکومپوز کریں
        task_plan = self.task_planner.decompose_task(goal)

        # ہر ٹاسک کے لیے، موشن منصوبے تخلیق کریں
        complete_plan = []
        for task in task_plan:
            if task.type == "navigate":
                motion_plan = self.motion_planner.plan_navigate(task.destination)
            elif task.type == "manipulate":
                motion_plan = self.motion_planner.plan_manipulate(
                    task.object, task.destination)
            elif task.type == "communicate":
                motion_plan = self.motion_planner.plan_communicate(task.message)

            complete_plan.append({
                'task': task,
                'motion_plan': motion_plan
            })

        return complete_plan

    def parse_goal(self, goal_description):
        """نیچرل لینگویج گول کو سٹرکچرڈ فارمیٹ میں پارس کریں"""
        # یہ گولز کو پارس کرنے کے لیے NLP استعمال کرے گا
        # مثال: "کچن میں جاؤ اور مجھے ایک پانی کی بوتل لاؤ"
        # کو navigate → find_object → grasp → navigate → place میں پارس کیا جائے گا
        pass

class SymbolicTaskPlanner:
    """STRIPS جیسے فارمیلزم کا استعمال کرتے ہوئے علامتی ٹاسک پلینر"""
    def __init__(self):
        # آپریٹرز (ایکشنز) اور پریڈیکیٹس (اسٹیٹس) کی وضاحت کریں
        self.operators = self.define_operators()
        self.predicates = self.define_predicates()

    def define_operators(self):
        """ٹاسک منصوبہ بندی کے لیے دستیاب آپریٹرز کی وضاحت کریں"""
        return {
            'navigate': {
                'preconditions': ['at(X)', 'accessible(Y)'],
                'effects': ['at(Y)', '!at(X)'],
                'cost': 1.0
            },
            'grasp': {
                'preconditions': ['at(X)', 'reachable(X)', 'free_hand()'],
                'effects': ['holding(X)', '!free_hand()'],
                'cost': 0.5
            },
            'place': {
                'preconditions': ['holding(X)', 'at(Y)'],
                'effects': ['!holding(X)', 'placed(X, Y)', 'free_hand()'],
                'cost': 0.5
            }
        }

    def decompose_task(self, goal):
        """گول کو آپریٹرز کی ترتیب میں ڈیکومپوز کریں"""
        # فارورڈ/بیک ورڈ چیننگ سرچ کو لاگو کریں
        # یا کلاسیکل منصوبہ بندی الگورتھم استعمال کریں
        task_plan = []
        # سادہ نافذ کاری
        return task_plan

class BiPedalMotionPlanner:
    """بائی پیڈل روبوٹس کے لیے مخصوص موشن پلینر"""
    def __init__(self):
        self.footstep_planner = FootstepPlanner()
        self.balance_controller = BalanceController()
        self.manipulation_planner = ManipulationPlanner()

    def plan_navigate(self, destination):
        """بائی پیڈل روبوٹ کے لیے نیویگیشن موشن منصوبہ بندی کریں"""
        return self.footstep_planner.plan_path(destination)

    def plan_manipulate(self, obj, destination):
        """آبجیکٹ کے لیے مینوپولیشن موشن منصوبہ بندی کریں"""
        return self.manipulation_planner.plan_grasp_transport_place(obj, destination)

    def plan_communicate(self, message):
        """مواصلت کے لیے موشن منصوبہ بندی کریں (مثلاً اشارے)"""
        return [{'type': 'gesture', 'motion': 'wave', 'duration': 2.0}]
```

## سم ٹو ریل ٹرانسفر تکنیکیں

سیمولیشن سے حقیقی روبوٹس میں سیکھے ہوئے رویوں کو منتقل کرنے کے لیے خصوصی غور کی ضرورت ہوتی ہے:

### ڈومین رینڈمائزیشن

```python
class DomainRandomization:
    """سم ٹو ریل ٹرانسفر کے لیے ڈومین رینڈمائزیشن"""
    def __init__(self):
        self.randomization_params = {
            'visual': {
                'lighting': (0.5, 2.0),  # شدت کی حد
                'textures': ['concrete', 'wood', 'carpet', 'grass'],
                'colors': [(0.2, 0.2, 0.2), (0.8, 0.8, 0.8)],  # گہرے سے ہلکا
                'materials': ['matte', 'glossy', 'rough']
            },
            'dynamics': {
                'friction': (0.3, 1.0),
                'mass_variation': (0.8, 1.2),
                'inertia_scaling': (0.9, 1.1),
                'actuator_noise': (0.0, 0.05)
            },
            'sensor': {
                'camera_noise': (0.0, 0.02),
                'imu_drift': (0.0, 0.01),
                'delay_range': (0.01, 0.05)
            }
        }

    def randomize_domain(self, sim_env):
        """سیمولیشن ڈومین پیرامیٹرز کو رینڈمائز کریں"""
        # وژوئل رینڈمائزیشن
        lighting_mult = np.random.uniform(
            self.randomization_params['visual']['lighting'][0],
            self.randomization_params['visual']['lighting'][1]
        )
        sim_env.set_lighting_multiplier(lighting_mult)

        # ڈائنامکس رینڈمائزیشن
        friction = np.random.uniform(
            self.randomization_params['dynamics']['friction'][0],
            self.randomization_params['dynamics']['friction'][1]
        )
        sim_env.set_friction(friction)

        # سینسر نوائز شامل کریں
        camera_noise = np.random.uniform(
            self.randomization_params['sensor']['camera_noise'][0],
            self.randomization_params['sensor']['camera_noise'][1]
        )
        sim_env.add_camera_noise(camera_noise)

        return sim_env
```

### کریکولم لرننگ

```python
class CurriculumLearning:
    """ gradual skill acquisition کے لیے کریکولم لرننگ"""
    def __init__(self):
        self.curriculum_levels = [
            {
                'name': 'stationary_balance',
                'difficulty': 0.1,
                'tasks': ['maintain_balance'],
                'rewards': {'balance_time': 1.0, 'fall_penalty': -10.0}
            },
            {
                'name': 'simple_stepping',
                'difficulty': 0.3,
                'tasks': ['step_forward', 'step_backward'],
                'rewards': {'balance_time': 1.0, 'reach_target': 5.0, 'fall_penalty': -10.0}
            },
            {
                'name': 'straight_line_walking',
                'difficulty': 0.5,
                'tasks': ['walk_forward', 'walk_backward'],
                'rewards': {'forward_vel': 1.0, 'energy_efficiency': 0.5, 'balance_time': 0.5, 'fall_penalty': -10.0}
            },
            {
                'name': 'turning',
                'difficulty': 0.7,
                'tasks': ['turn_left', 'turn_right'],
                'rewards': {'heading_accuracy': 2.0, 'balance_time': 0.5, 'energy_efficiency': 0.3, 'fall_penalty': -10.0}
            },
            {
                'name': 'complex_maneuvers',
                'difficulty': 1.0,
                'tasks': ['sidestep', 'walk_over_small_obstacles'],
                'rewards': {'task_completion': 10.0, 'smoothness': 1.0, 'balance_time': 0.5, 'fall_penalty': -10.0}
            }
        ]

        self.current_level = 0
        self.level_progress_threshold = 0.8  # 80% کامیابی کی شرح سے آگے بڑھنے کے لیے

    def get_current_tasks(self):
        """موجودہ کریکولم لیول کے ٹاسکس حاصل کریں"""
        return self.curriculum_levels[self.current_level]['tasks']

    def evaluate_performance(self, episode_results):
        """موجودہ لیول پر ایجنٹ کارکردگی کا جائزہ لیں"""
        # ایپی سوڈ نتائج کی بنیاد پر کارکردگی میٹرکس کا حساب لگائیں
        success_rate = self.calculate_success_rate(episode_results)

        if success_rate >= self.level_progress_threshold and self.current_level < len(self.curriculum_levels) - 1:
            self.current_level += 1
            print(f"کریکولم لیول میں ترقی: {self.curriculum_levels[self.current_level]['name']}")

        return success_rate

    def calculate_success_rate(self, results):
        """ایپی سوڈ نتائج سے کامیابی کی شرح کا حساب لگائیں"""
        if not results:
            return 0.0

        successful_episodes = sum(1 for r in results if r.success)
        return successful_episodes / len(results)
```

## روبوٹکس کے لیے نیورل آرکیٹیکچر سرچ

نیورل آرکیٹیکچر سرچ (NAS) روبوٹکس کے کاموں کے لیے نیورل نیٹ ورکس کو بہتر بناسکتا ہے:

```python
class RobotNASCandidate:
    """روبوٹکس کے لیے ایک امیدوار نیورل آرکیٹیکچر کی نمائندگی کرتا ہے"""
    def __init__(self, layers_config):
        self.layers_config = layers_config  # لیئرز کی وضاحت کی فہرست
        self.fitness_score = 0.0
        self.computation_cost = 0.0  # FLOPs یا انفرینس ٹائم

    def build_network(self):
        """کنفیگریشن سے نیورل نیٹ ورک بنائیں"""
        layers = []
        for layer_config in self.layers_config:
            layer_type = layer_config['type']
            if layer_type == 'conv':
                layers.append(nn.Conv2d(layer_config['in_channels'],
                                      layer_config['out_channels'],
                                      layer_config['kernel_size']))
            elif layer_type == 'linear':
                layers.append(nn.Linear(layer_config['in_size'],
                                      layer_config['out_size']))
            elif layer_type == 'residual':
                layers.append(ResidualBlock(layer_config['channels']))
            # ایکٹیویشن فنکشنز شامل کریں
            layers.append(nn.ReLU())

        return nn.Sequential(*layers)

class RobotNeuralArchitectureSearch:
    """روبوٹکس ایپلی کیشنز کے لیے نیورل آرکیٹیکچر سرچ کو مخصوص کریں"""
    def __init__(self, search_space, population_size=50, generations=20):
        self.search_space = search_space
        self.population_size = population_size
        self.generations = generations
        self.population = []

        # روبوٹ مخصوص اہداف
        self.objectives = {
            'accuracy': 0.5,
            'latency': 0.3,
            'power_efficiency': 0.2
        }

    def initialize_population(self):
        """آرکیٹیکچرز کی بے ترتیب آبادی شروع کریں"""
        for _ in range(self.population_size):
            config = self.generate_random_architecture()
            candidate = RobotNASCandidate(config)
            self.population.append(candidate)

    def generate_random_architecture(self):
        """تلاش کی جگہ کے اندر ایک بے ترتیب آرکیٹیکچر تیار کریں"""
        layers = []

        # لیئرز کی بے ترتیب ترتیب تیار کریں
        num_layers = np.random.randint(3, 8)  # 3-8 لیئرز

        for _ in range(num_layers):
            layer_type = np.random.choice(['conv', 'linear', 'residual'])
            layer_config = self.sample_layer_configuration(layer_type)
            layers.append(layer_config)

        return layers

    def sample_layer_configuration(self, layer_type):
        """لیئر ٹائپ کے لیے کنفیگریشن نمونہ لیں"""
        if layer_type == 'conv':
            return {
                'type': 'conv',
                'in_channels': np.random.choice([16, 32, 64, 128]),
                'out_channels': np.random.choice([32, 64, 128, 256]),
                'kernel_size': np.random.choice([3, 5, 7])
            }
        elif layer_type == 'linear':
            return {
                'type': 'linear',
                'in_size': np.random.choice([64, 128, 256, 512]),
                'out_size': np.random.choice([32, 64, 128, 256])
            }
        elif layer_type == 'residual':
            return {
                'type': 'residual',
                'channels': np.random.choice([64, 128, 256])
            }

    def evaluate_candidate(self, candidate, eval_env):
        """ایک امیدوار آرکیٹیکچر کا جائزہ لیں"""
        try:
            network = candidate.build_network()

            # جائزہ ماحول پر درستگی کا جائزہ لیں
            accuracy = self.evaluate_accuracy(network, eval_env)

            # کمپیوٹیشنل لاگت کا تخمینہ لگائیں
            latency = self.estimate_latency(network)
            power = self.estimate_power_usage(network)

            # مجموعی فٹ نیس اسکور
            fitness = (self.objectives['accuracy'] * accuracy -
                      self.objectives['latency'] * latency -
                      self.objectives['power_efficiency'] * power)

            candidate.fitness_score = fitness
            candidate.computation_cost = latency

            return fitness

        except Exception as e:
            # غلط آرکیٹیکچرز کو سزا دیں
            candidate.fitness_score = -1.0
            return -1.0

    def evolve_population(self):
        """جینیٹک آپریٹرز کا استعمال کرتے ہوئے آبادی کو ترقی دیں"""
        # فٹ نیس کے حساب سے ترتیب دیں
        self.population.sort(key=lambda x: x.fitness_score, reverse=True)

        # سر فہرست کارکردگی کو برقرار رکھیں
        survivors = self.population[:int(0.2 * self.population_size)]

        # میوٹیشن اور کراس اوور کے ذریعے اولاد تیار کریں
        offspring = []
        for _ in range(self.population_size - len(survivors)):
            parent1 = np.random.choice(survivors)
            if np.random.rand() < 0.8:  # 80% وقت کراس اوور
                parent2 = np.random.choice(survivors)
                child_config = self.crossover(parent1.layers_config, parent2.layers_config)
            else:  # بصورت دیگر میوٹیشن
                child_config = self.mutate(parent1.layers_config)

            offspring.append(RobotNASCandidate(child_config))

        self.population = survivors + offspring

    def crossover(self, config1, config2):
        """دو آرکیٹیکچرز کو جوڑیں"""
        # سادہ کراس اوور: ہر ایک سے نصف لیں
        mid_point = len(config1) // 2
        child_config = config1[:mid_point] + config2[mid_point:]
        return child_config

    def mutate(self, config):
        """آرکیٹیکچر کو میوٹیٹ کریں"""
        mutated_config = config.copy()
        # ~20% لیئرز کو بے ترتیب طور پر تبدیل کریں
        for i in range(len(mutated_config)):
            if np.random.rand() < 0.2:
                mutated_config[i] = self.sample_layer_configuration(
                    mutated_config[i]['type'])
        return mutated_config

    def search(self, eval_env):
        """مکمل NAS عمل کو چلائیں"""
        self.initialize_population()

        for generation in range(self.generations):
            print(f"نسل {generation + 1}/{self.generations} کا جائزہ لیں")

            for candidate in self.population:
                self.evaluate_candidate(candidate, eval_env)

            # نسل سے بہترین کی اطلاع دیں
            best = max(self.population, key=lambda x: x.fitness_score)
            print(f"بہترین فٹ نیس: {best.fitness_score:.4f}")

            # اگلی نسل کے لیے ترقی دیں
            self.evolve_population()

        # بہترین آرکیٹیکچر واپس کریں
        best_final = max(self.population, key=lambda x: x.fitness_score)
        return best_final
```

## خلاصہ

اس باب میں ہم نے ہیومینائیڈ روبوٹ دماغوں کے لیے اعلیٰ درجے کی AI تکنیکوں پر بات کی:

- لوکوموشن اور کنٹرول کے لیے مضبوط سیکھنا
- ادراک ایکشن سسٹمز کے لیے Isaac ROS کا انضمام
- حقیقی وقت میں ایڈجسٹ کرنے والے ایڈاپٹیو کنٹرول سسٹمز
- پیچیدہ کاموں کے لیے سلسلہ وار فیصلہ سازی
- ڈومین رینڈمائزیشن سمیت سم ٹو ریل ٹرانسفر تکنیکیں
- تدریجی مہارتوں کے لیے کریکولم لرننگ
- روبوٹ نیورل نیٹ ورکس کو بہتر بنانے کے لیے نیورل آرکیٹیکچر سرچ

یہ تکنیکیں ہیومینائیڈ روبوٹس کو پیچیدہ رویے سیکھنے، تبدیل ہوتی حالت کے مطابق ایڈاپٹ کرنے، اور ایسے کام انجام دینے کے قابل بناتی ہیں جن کے لیے ادراک اور ایکشن کی صلاحیتوں کی ضرورت ہوتی ہے۔

## مشقیں

1. بنیادی ہیومینائیڈ کنٹرول کے لیے ایک سادہ DDPG ایجنٹ نافذ کریں
2. اپنے سیمولیشن ماحول کے لیے ڈومین رینڈمائزیشن اسکیم تخلیق کریں
3. مخصوص ہیومینائیڈ ٹاسک کے لیے سلسلہ وار ٹاسک پلینر ڈیزائن کریں

## اگلے اقدامات

ہیومینائیڈ روبوٹس کے لیے اعلیٰ درجے کی AI تکنیکوں کو پورا کرتے ہوئے، اگلا ماڈیول ویژن-لینگویج-ایکشن سسٹمز پر توجہ مرکز کرے گا جو ادراک، شعور اور ایکشن کو انسان-روبوٹ مواصلت کے لیے یکجا فریم ورکس میں ضم کرتے ہیں۔