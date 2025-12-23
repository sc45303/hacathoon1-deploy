---
sidebar_position: 5
---

# باب 4: URDF اور ہیومینائیڈ روبوٹ ماڈلز

## سیکھنے کے مقاصد

- روبوٹ ماڈلنگ کے لیے یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ (URDF) کو سمجھیں
- پیچیدہ ہیومینائیڈ روبوٹ سٹرکچرز کے لیے URDF فائلیں تخلیق کریں
- جوائنٹس، لنکس، جامد خصوصیات، اور وژول/کولیژن اجزاء کے بارے میں سیکھیں
- URDF ماڈلز کو ROS 2 سیمولیشن ماحول کے ساتھ ضم کریں
- ہیومینائیڈ روبوٹ ماڈلز کی توثیق اور ٹیسٹ کریں

## URDF کا تعارف

URDF (یونیفائیڈ روبوٹ ڈسکرپشن فارمیٹ) ایک XML فارمیٹ ہے جس کا استعمال ROS میں روبوٹ ماڈلز کی وضاحت کے لیے کیا جاتا ہے۔ یہ روبوٹ کی جسمانی ساخت کو بیان کرتا ہے جس میں لنکس، جوائنٹس، جامد خصوصیات، وژول نمائندگیاں، اور کولیژن ماڈلز شامل ہیں۔ ہیومینائیڈ روبوٹس کے لیے، URDF سیمولیشن، وژولائزیشن، اور موشن منصوبہ بندی کے لیے ضروری ہے۔

### ہیومینائیڈ روبوٹس کے لیے URDF کیوں اہم ہے

1. **سیمولیشن**: Gazebo اور دیگر سیمولیٹرز URDF کا استعمال فزکس ماڈلز تخلیق کرنے کے لیے کرتے ہیں
2. **وژولائزیشن**: RViz URDF کا استعمال 3D میں روبوٹس ڈسپلے کرنے کے لیے کرتا ہے
3. **موشن منصوبہ بندی**: منصوبہ بندی الگورتھم کو URDF ماڈلز کی ضرورت ہوتی ہے تاکہ روبوٹ کنیمیٹکس کو سمجھا جا سکے
4. **کنٹرول**: روبوٹ کنٹرولرز URDF کا استعمال کنیمیٹک حسابات کے لیے کرتے ہیں
5. **معیاریکردہ**: URDF ROS کمیونٹی میں روبوٹس کو ماڈل کرنے کا ایک معیاری طریقہ فراہم کرتا ہے

## URDF سٹرکچر کا جائزہ

URDF فائل میں یہ شامل ہوتا ہے:
- **لنکس**: روبوٹ کے سخت حصے (مثلاً ٹورسو، لیمب سیگمینٹس)
- **جوائنٹس**: لنکس کے درمیان کنکشنز (مثلاً ہنجز، پریزمیٹک جوائنٹس)
- **مواد**: وژول خصوصیات (رنگ، ٹیکسچرز)
- **Gazebos**: سیمولیشن مخصوص خصوصیات

### بنیادی URDF مثال

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <!-- روبوٹ کا بیس لنک -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- جوائنٹ کے ذریعے منسلک لنک -->
  <link name="upper_body">
    <visual>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.3 0.6"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- دونوں لنکس کو جوڑنے والا جوائنٹ -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="upper_body"/>
    <origin xyz="0 0 0.4" rpy="0 0 0"/>
  </joint>
</robot>
```

## URDF میں لنکس

لنکس روبوٹ میں سخت اجسام کی نمائندگی کرتے ہیں۔ ہر لنک میں یہ شامل ہوتا ہے:

1. **وژول خصوصیات**: ڈسپلے میں لنک کیسے نظر آتا ہے
2. **کولیژن خصوصیات**: فزکس سیمولیشن میں لنک کیسے تعامل کرتا ہے
3. **جامد خصوصیات**: فزکس سیمولیشن کے لیے جسمانی خصوصیات

### وژول اور کولیژن عناصر

```xml
<link name="arm_link">
  <!-- ڈسپلے کے لیے وژول خصوصیات -->
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_meshes/arm.dae"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1"/>
    </material>
  </visual>

  <!-- فزکس کے لیے کولیژن خصوصیات -->
  <collision>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry>
      <cylinder length="0.2" radius="0.05"/>
    </geometry>
  </collision>

  <!-- جامد خصوصیات -->
  <inertial>
    <mass value="0.5"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.002"/>
  </inertial>
</link>
```

## URDF میں جوائنٹس

جوائنٹس یہ بیان کرتے ہیں کہ لنکس کیسے جڑتے ہیں اور ایک دوسرے کے مقابلے میں کیسے حرکت کرتے ہیں۔ عام جوائنٹ اقسام:

1. **فکسڈ**: کوئی حرکت نہیں (ویلڈ جوائنٹ)
2. **ریوولوٹ**: واحد محور کے گرد گھومنے کی حرکت (جیسے ہنجز)
3. **کنٹینیوئس**: ریوولوٹ کی طرح لیکن لامحدود گھوماؤ
4. **پریزمیٹک**: لکیری سلائیڈنگ حرکت
5. **فلوٹنگ**: 6DOF حرکت
6. **پلینر**: ایک سطح میں حرکت

### جوائنٹ کی وضاحتیں

```xml
<!-- ریوولوٹ جوائنٹ مثال: ہپ پچ -->
<joint name="left_hip_pitch" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0 -0.1 -0.1" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- X-محور کے گرد گھومنا -->
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.0"/>
  <dynamics damping="0.1" friction="0.0"/>
</joint>

<!-- کنٹینیوئس جوائنٹ مثال: گردن کا گھوماؤ -->
<joint name="neck_yaw" type="continuous">
  <parent link="torso"/>
  <child link="head"/>
  <origin xyz="0 0 0.8" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>  <!-- Z-محور کے گرد گھومنا -->
  <dynamics damping="0.1"/>
</joint>

<!-- فکسڈ جوائنٹ مثال: سینسر ماؤنٹ -->
<joint name="imu_mount" type="fixed">
  <parent link="head"/>
  <child link="imu_link"/>
  <origin xyz="0.05 0 0.02" rpy="0 0 0"/>
</joint>
```

## ہیومینائیڈ روبوٹ ماڈل تخلیق کرنا

آئیے ٹانگوں، ٹورسو، بازوں، اور سر کے ساتھ ایک سادہ ہیومینائیڈ ماڈل تخلیق کریں:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid" xmlns:xacro="http://ros.org/wiki/xacro">
  <!-- عام خصوصیات شامل کریں -->
  <xacro:property name="M_PI" value="3.1415926535897931" />

  <!-- مادہ کی وضاحتیں -->
  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>
  <material name="black">
    <color rgba="0 0 0 1"/>
  </material>
  <material name="red">
    <color rgba="0.8 0.2 0.2 1"/>
  </material>
  <material name="blue">
    <color rgba="0.2 0.2 0.8 1"/>
  </material>

  <!-- بیس لنک: pelvis/hip -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.15 0.2 0.15"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.2 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- بائیں ٹانگ زنجیر -->
  <joint name="left_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="left_thigh"/>
    <origin xyz="0 -0.1 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="3.0"/>
  </joint>

  <link name="left_thigh">
    <visual>
      <geometry>
        <capsule length="0.35" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.35" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="left_knee" type="revolute">
    <parent link="left_thigh"/>
    <child link="left_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="100" velocity="3.0"/>
  </joint>

  <link name="left_shin">
    <visual>
      <geometry>
        <capsule length="0.35" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.35" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>

  <joint name="left_ankle" type="revolute">
    <parent link="left_shin"/>
    <child link="left_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="2.0"/>
  </joint>

  <link name="left_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.0015" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- دائیں ٹانگ (اسی طرح کی سٹرکچر) -->
  <joint name="right_hip_yaw" type="revolute">
    <parent link="base_link"/>
    <child link="right_thigh"/>
    <origin xyz="0 0.1 -0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="100" velocity="3.0"/>
  </joint>

  <link name="right_thigh">
    <visual>
      <geometry>
        <capsule length="0.35" radius="0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.35" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <joint name="right_knee" type="revolute">
    <parent link="right_thigh"/>
    <child link="right_shin"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="2.35" effort="100" velocity="3.0"/>
  </joint>

  <link name="right_shin">
    <visual>
      <geometry>
        <capsule length="0.35" radius="0.04"/>
      </geometry>
      <material name="blue"/>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.35" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.5"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.0015"/>
    </inertial>
  </link>

  <joint name="right_ankle" type="revolute">
    <parent link="right_shin"/>
    <child link="right_foot"/>
    <origin xyz="0 0 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="50" velocity="2.0"/>
  </joint>

  <link name="right_foot">
    <visual>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.8"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.0015" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- ٹورسو -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- سر -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="10" velocity="1.0"/>
  </joint>

  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
    </inertial>
  </link>

  <!-- بائیں بازو -->
  <joint name="left_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.05 -0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <link name="left_upper_arm">
    <visual>
      <geometry>
        <capsule length="0.25" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_elbow" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.35" upper="0" effort="50" velocity="2.0"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <capsule length="0.25" radius="0.035"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.25" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.7"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="left_wrist" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2.0"/>
  </joint>

  <link name="left_hand">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>

  <!-- دائیں بازو (اسی طرح کی سٹرکچر) -->
  <joint name="right_shoulder_pitch" type="revolute">
    <parent link="torso"/>
    <child link="right_upper_arm"/>
    <origin xyz="0.05 0.1 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0"/>
  </joint>

  <link name="right_upper_arm">
    <visual>
      <geometry>
        <capsule length="0.25" radius="0.04"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.25" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_elbow" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2.35" upper="0" effort="50" velocity="2.0"/>
  </joint>

  <link name="right_lower_arm">
    <visual>
      <geometry>
        <capsule length="0.25" radius="0.035"/>
      </geometry>
      <material name="red"/>
    </visual>
    <collision>
      <geometry>
        <capsule length="0.25" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.7"/>
      <inertia ixx="0.003" ixy="0.0" ixz="0.0" iyy="0.003" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>

  <joint name="right_wrist" type="revolute">
    <parent link="right_lower_arm"/>
    <child link="right_hand"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="2.0"/>
  </joint>

  <link name="right_hand">
    <visual>
      <geometry>
        <box size="0.08 0.08 0.1"/>
      </geometry>
      <material name="black"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.08 0.08 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.3"/>
      <inertia ixx="0.0005" ixy="0.0" ixz="0.0" iyy="0.0005" iyz="0.0" izz="0.0005"/>
    </inertial>
  </link>
</robot>
```

## پیچیدہ ماڈلز کے لیے Xacro

Xacro (XML میکروز) ایک میکرو زبان ہے جو URDF کو متغیرات، کنستس، اور میکروز جیسی خصوصیات کے ساتھ وسعت دیتا ہے تاکہ پیچیدہ روبوٹ ماڈلز کو آسان بنایا جا سکے:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="humanoid_with_xacro">

  <!-- خصوصیات -->
  <xacro:property name="M_PI" value="3.1415926535897931" />
  <xacro:property name="torso_height" value="0.4" />
  <xacro:property name="torso_width" value="0.2" />
  <xacro:property name="torso_depth" value="0.2" />

  <!-- ٹانگ کی وضاحت کے لیے میکرو -->
  <xacro:macro name="leg" params="side reflect">
    <joint name="${side}_hip_yaw" type="revolute">
      <parent link="base_link"/>
      <child link="${side}_thigh"/>
      <origin xyz="0 ${reflect * 0.1} -0.05" rpy="0 0 0"/>
      <axis xyz="0 0 1"/>
      <limit lower="-0.785" upper="0.785" effort="100" velocity="3.0"/>
    </joint>

    <link name="${side}_thigh">
      <visual>
        <geometry>
          <capsule length="0.35" radius="0.05"/>
        </geometry>
        <material name="blue"/>
      </visual>
      <collision>
        <geometry>
          <capsule length="0.35" radius="0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="2.0"/>
        <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>

    <joint name="${side}_knee" type="revolute">
      <parent link="${side}_thigh"/>
      <child link="${side}_shin"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0" upper="2.35" effort="100" velocity="3.0"/>
    </joint>

    <link name="${side}_shin">
      <visual>
        <geometry>
          <capsule length="0.35" radius="0.04"/>
        </geometry>
        <material name="blue"/>
      </visual>
      <collision>
        <geometry>
          <capsule length="0.35" radius="0.04"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.015" ixy="0.0" ixz="0.0" iyy="0.015" iyz="0.0" izz="0.0015"/>
      </inertial>
    </link>

    <joint name="${side}_ankle" type="revolute">
      <parent link="${side}_shin"/>
      <child link="${side}_foot"/>
      <origin xyz="0 0 -0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.5" upper="0.5" effort="50" velocity="2.0"/>
    </joint>

    <link name="${side}_foot">
      <visual>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.8"/>
        <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.0015" iyz="0.0" izz="0.002"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- بیس لنک -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.15 0.2 0.15"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="0.15 0.2 0.15"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" ixy="0.0" ixz="0.0" iyy="0.05" iyz="0.0" izz="0.05"/>
    </inertial>
  </link>

  <!-- ٹانگ میکرو کا استعمال دونوں ٹانگوں کی وضاحت کے لیے کریں -->
  <xacro:leg side="left" reflect="-1" />
  <xacro:leg side="right" reflect="1" />

  <!-- ٹورسو -->
  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>

  <link name="torso">
    <visual>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
      <material name="white"/>
    </visual>
    <collision>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="0.2" ixy="0.0" ixz="0.0" iyy="0.2" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

## Gazebo کے ساتھ URDF کے لیے

Gazebo سیمولیشن میں URDF استعمال کرنے کے لیے، ہمیں Gazebo مخصوص ٹیگز شامل کرنے کی ضرورت ہے:

```xml
<!-- Gazebo مادہ کی وضاحت -->
<gazebo reference="base_link">
  <material>Gazebo/White</material>
  <turnGravityOff>false</turnGravityOff>
</gazebo>

<!-- ros_control کے لیے Gazebo پلگ ان -->
<gazebo>
  <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
    <robotNamespace>/humanoid_robot</robotNamespace>
    <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
  </plugin>
</gazebo>

<!-- ros_control کے لیے جوائنٹ ٹرانسمیشن -->
<transmission name="left_hip_yaw_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_hip_yaw">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_hip_yaw_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>
```

## URDF ماڈلز کی توثیق

### URDF توثیق کے لیے ٹولز

1. **check_urdf**: URDF گرامر کی توثیق کے لیے کمانڈ لائن ٹول
   ```bash
   check_urdf /path/to/robot.urdf
   ```

2. **urdf_to_graphiz**: کنیمیٹک ٹری کا وژول گراف تیار کریں
   ```bash
   urdf_to_graphiz /path/to/robot.urdf
   ```

3. **RViz**: روبوٹ ماڈل کو وژولائز کریں

### عام URDF مسائل

1. **غلط ماس ویلیوز**: بہت ہلکے یا بہت بھاری لنکس
2. **غلط جامد ویلیوز**: غیر مثبت ڈیفنیٹ انرشن میٹرکسز
3. **غیر وضاحت شدہ مواد**: حوالہ دیے گئے مواد جو موجود نہیں ہیں
4. **جوائنٹ حد کے مسائل**: غیر درست یا غیر حقیقی جوائنٹ حدیں
5. **کنیمیٹک لوپس**: URDF صرف ٹری سٹرکچرز کی حمایت کرتا ہے، لوپس نہیں

## ROS 2 میں URDF لوڈ کرنا

ROS 2 میں اپنے URDF ماڈل کو استعمال کرنے کے لیے، آپ کو:

1. **ربوٹ اسٹیٹ پبلشر لانچ کرنا ہوگا**:

```xml
<!-- robot_state_publisher_launch.py -->
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # xacro کے ذریعے URDF حاصل کریں
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindPackageShare("your_robot_description"), "urdf", "robot.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    return LaunchDescription([
        node_robot_state_publisher,
    ])
```

2. **اسے ROS 2 نوڈ میں استعمال کرنا**:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # جوائنٹ اسٹیٹس کو سبسکرائب کریں
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # جوائنٹ کمانڈز کے لیے پبلشر
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/joint_commands',
            10
        )

        # روبوٹ ٹرانسفارمز کے لیے TF براڈکاسٹر
        self.tf_broadcaster = TransformBroadcaster(self)

        # کنٹرول لوپ کے لیے ٹائمر
        self.control_timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

        self.current_joint_states = JointState()
        self.get_logger().info('ہیومینائیڈ کنٹرولر شروع کیا گیا')

    def joint_state_callback(self, msg):
        """موجودہ جوائنٹ اسٹیٹس اپ ڈیٹ کریں"""
        self.current_joint_states = msg

    def control_loop(self):
        """مرکزی کنٹرول لوپ"""
        # مثال: بائیں بازو کو ایک سادہ پیٹرن میں حرکت دیں
        cmd_msg = JointState()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.name = ['left_shoulder_pitch', 'right_shoulder_pitch']

        # ایک سادہ اوسیلیٹنگ پیٹرن تیار کریں
        t = self.get_clock().now().nanoseconds / 1e9  # سیکنڈ میں وقت
        left_pos = 0.5 * math.sin(t)  # -0.5 اور 0.5 کے درمیان اوسیلیٹ
        right_pos = 0.5 * math.sin(t + math.pi)  # فیز سے باہر

        cmd_msg.position = [left_pos, right_pos]

        self.joint_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## اعلیٰ URDF تکنیکیں

### میش فائلیں استعمال کرنا

زیادہ تفصیلی روبوٹ ماڈلز کے لیے، بنیادی شکلوں کے بجائے میش فائلیں استعمال کریں:

```xml
<link name="head_link">
  <visual>
    <geometry>
      <!-- تفصیلی جیومیٹری کے لیے میش فائل استعمال کریں -->
      <mesh filename="package://humanoid_description/meshes/head.dae" scale="1 1 1"/>
    </geometry>
    <material name="skin_color">
      <color rgba="0.96 0.87 0.70 1.0"/>
    </material>
  </visual>
  <collision>
    <!-- بہتر کارکردگی کے لیے آسان کولیژن میش استعمال کریں -->
    <geometry>
      <mesh filename="package://humanoid_description/meshes/head_collision.stl" scale="1 1 1"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.005" ixy="0.0" ixz="0.0" iyy="0.005" iyz="0.0" izz="0.005"/>
  </inertial>
</link>
```

### Gazebo مخصوص اضافے

```xml
<!-- Gazebo کے لیے URDF میں سینسر کی وضاحت -->
<gazebo reference="head_camera_frame">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>800</width>
        <height>600</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>head_camera_frame</frame_name>
      <topic_name>image_raw</topic_name>
    </plugin>
  </sensor>
</gazebo>
```

## عام مسائل کا حل

### 1. روبوٹ زمین میں گر جاتا ہے
- جامد ویلیوز کی جانچ کریں (ماس اور انرشن میٹرکس)
- جوائنٹ حدیں اور ڈائنیمکس پیرامیٹرز کی تصدیق کریں
- مناسب کولیژن جیومیٹری کو یقینی بنائیں

### 2. سیمولیشن میں روبوٹ پھٹ جاتا ہے
- ماس ویلیوز کی جانچ کریں (بہت کم یا منفی نہ ہو)
- یقینی بنائیں کہ انرشن میٹرکس مثبت ڈیفنیٹ ہے
- جوائنٹ ڈائنیمکس کی جانچ کریں (ڈیمپنگ اور فریکشن)

### 3. RViz غلط ماڈل دکھاتا ہے
- URDF گرامر کی خامیوں کی جانچ کریں
- یقینی بنائیں کہ ربوٹ_اسٹیٹ_پبلشر چل رہا ہے
- یقینی بنائیں کہ جوائنٹ اسٹیٹس صحیح طریقے سے شائع کیے جا رہے ہیں

## خلاصہ

URDF ROS 2 میں ہیومینائیڈ روبوٹس کی نمائندگی کا ایک بنیادی جزو ہے۔ اس باب میں یہ باتیں شامل تھیں:

- لنکس، جوائنٹس، وژول، کولیژن، اور جامد خصوصیات کے ساتھ URDF فائلیں کا سٹرکچر
- ٹانگوں، ٹورسو، بازوؤں، اور سر کے ساتھ مکمل ہیومینائیڈ روبوٹ ماڈل کو تخلیق کرنا
- پیچیدہ روبوٹ کی وضاحت کو آسان بنانے کے لیے Xacro کا استعمال
- Gazebo سیمولیشن اور ROS 2 سسٹمز کے ساتھ انضمام
- توثیق کی تکنیکیں اور عام ٹربولشوٹنگ کے طریقے

آپ کے ہیومینائیڈ روبوٹ کو مناسب طریقے سے ماڈل کرنا سیمولیشن اور کنٹرول کی کامیابی کے لیے انتہائی اہم ہے۔ URDF ROS 2 ایکو سسٹم میں جسمانی روبوٹ ڈیزائن اور اس کی ڈیجیٹل نمائندگی کے درمیان پل کا کام کرتا ہے۔

## مشقیں

1. کم از کم 10 جوائنٹس کے ساتھ ایک سادہ ہیومینائیڈ روبوٹ کے لیے URDF فائل تخلیق کریں
2. RViz میں اپنے روبوٹ ماڈل کو وژولائز کریں
3. اپنے روبوٹ ماڈل میں ایک سینسر (جیسے کیمرہ یا IMU) شامل کریں

## اگلے اقدامات

ROS 2 کے بنیادی تصورات، رابطے کے نمونوں، AI انضمام، اور روبوٹ ماڈلنگ کی مکمل سمجھ کے ساتھ، آپ اب ہیومینائیڈ روبوٹکس میں مزید اعلیٰ موضوعات کا استعمال کرنے کے لیے تیار ہیں۔ اگلا ماڈیول Gazebo اور دیگر سیمولیشن ٹولز کا استعمال کرتے ہوئے فزکس سیمولیشن اور سینسر سسٹمز پر تبادلہ خیال کرے گا۔