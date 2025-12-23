---
sidebar_position: 4
---

# باب 3: ہائی فائیڈلٹی رینڈرنگ اور انسان-روبوٹ تعامل

## سیکھنے کے مقاصد

- سیمولیشن کے لیے Gazebo اور Unity کے درمیان فرق سمجھیں
- سیکھیں کہ Unity انسان-روبوٹ تعامل کے لیے ہائی فائیڈلٹی رینڈرنگ کیسے فراہم کرتا ہے
- Unity کی صلاحیتوں کی تلاش کریں ہیومینائیڈ روبوٹ سیمولیشن کو حقیقی طور پر تخلیق کرنے کے لیے
- Unity میں انسان-روبوٹ تعامل کے منظر نامے نافذ کریں
- مختلف استعمال کے معاملات کے لیے Gazebo اور Unity کا موازنہ کریں

## ہائی فائیڈلٹی رینڈرنگ کا تعارف

ہائی فائیڈلٹی رینڈرنگ کا مطلب حقیقی دنیا کی نقل کے قریب ماحول، اشیاء، اور روبوٹس کی حقیقی وژولائزیشن ہے۔ یہ اہم ہے:

1. **انسان-روبوٹ تعامل**: زیادہ حقیقی ویژول فیڈ بیک انسان-روبوٹ تعاملات کی قدرت کو بہتر بناتا ہے
2. **AI ماڈلز کی تربیت**: حقیقی رینڈرنگ AI ماڈلز کی مدد کرتی ہے جو سیمولیشن میں تربیت یافتہ ہیں تاکہ حقیقت میں بہتر منتقل ہو سکیں
3. **صارف کا تجربہ**: زیادہ حقیقی سیمولیشن صارفین کے تجربے کو بہتر بناتے ہیں جو مجازی روبوٹس کے ساتھ تعامل کر رہے ہیں
4. **توثیق**: حقیقی رینڈرنگ ادراک الگورتھم کی بہتر توثیق کی اجازت دیتا ہے

## Unity بمقابلہ Gazebo: ایک موازنہ کا تجزیہ

### Gazebo
- ** مضبوطیاں**: عمدہ فزکس سیمولیشن، ROS انضمام، روبوٹکس میں قائم
- **محدودیاں**: محدود ویژول رینڈرنگ کی صلاحیتیں، ہائی فائیڈلٹی وژولائزیشن کے لیے کم مناسب
- **بہترین کے لیے**: فزکس کے اعتبار سے درست سیمولیشن، سینسر سیمولیشن، نیویگیشن اور مینوپولیشن ٹیسٹنگ

### Unity
- ** مضبوطیاں**: جدید ترین رینڈرنگ، گیم انجن کی صلاحیتیں، حقیقی ویژول ایفیکٹس
- **محدودیاں**: روبوٹکس ایکو سسٹم میں کم قائم، اضافی انضمام کی ضرورت ہوتی ہے
- **بہترین کے لیے**: ہائی فائیڈلٹی وژولائزیشن، انسان-روبوٹ تعامل کے مطالعات، حقیقی منظر تخلیق

## Unity روبوٹکس انضمام

Unity نے روبوٹکس سیمولیشن کے لیے مخصوص ٹولز تیار کیے ہیں:

### Unity ML-Agents ٹول کٹ
- **مقصد**: Unity ماحول کے لیے مضبوط سیکھنے کا ڈھانچہ
- **استعمال کا مقصد**: ہیومینائیڈ لوکوموشن اور مینوپولیشن کی پالیسیوں کی تربیت
- **انضمام**: ROS-TCP-Connector کے ذریعے ROS 2 کے ساتھ کام کر سکتا ہے

### Unity ROS-TCP-Connector
- **مقصد**: Unity اور ROS 2 کے درمیان رابطہ کو فعال کرنا
- **کام**: Unity کے اندر سے ROS 2 ٹاپکس کو شائع کرتا ہے اور سبسکرائب کرتا ہے
- **استعمال کا مقصد**: پیچیدہ روبوٹ رویوں کی حقیقی وژولائزیشن

### Unity ادراک پیکج
- **مقصد**: کمپیوٹر وژن AI کے لیے لیبل شدہ مصنوعی ڈیٹا تخلیق کرنا
- **خصوصیات**: سیگمینٹیشن، باؤنڈنگ باکسز، ڈیپتھ میپس جمع سچائی کے ساتھ
- **استعمال کا مقصد**: فوٹو ریئل سٹک ڈیٹا کے ساتھ ادراک سسٹمز کی تربیت

## روبوٹکس کے لیے Unity کا سیٹ اپ

### Unity روبوٹکس ہب

Unity کو ROS 2 کے ساتھ ضم کرنے کے لیے، آپ عام طور پر یہ کریں گے:

1. **Unity 2021.3 LTS یا اس سے بعد میں انسٹال کریں**
2. **Unity روبوٹکس ہب انسٹال کریں**
3. **ROS-TCP-Connector پیکج شامل کریں**
4. **ROS 2 برج ٹولز انسٹال کریں**

### بنیادی Unity-ROS انضمام کی مثال

```csharp
using UnityEngine;
using System.Collections;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Std_msgs;

public class RobotController : MonoBehaviour
{
    ROSConnection ros;
    string robotTopicName = "unity_robot_command";

    // روبوٹ کی خصوصیات
    public float moveSpeed = 5.0f;
    public float rotateSpeed = 100.0f;

    void Start()
    {
        // ROS کنکشن کی سٹیٹک مثال حاصل کریں
        ros = ROSConnection.instance;

        // روبوٹ کمانڈ ٹاپک کو سبسکرائب کریں
        ros.Subscribe<StringMsg>(robotTopicName, CommandCallback);
    }

    void CommandCallback(StringMsg cmd)
    {
        Debug.Log("کمانڈ موصول ہوئی: " + cmd.data);

        // کمانڈ کو پروسیس کریں اور روبوٹ کو اپ ڈیٹ کریں
        ProcessCommand(cmd.data);
    }

    void ProcessCommand(string cmd)
    {
        // مثال: حرکت کمانڈز کو پارس کریں
        if (cmd == "move_forward")
        {
            transform.Translate(Vector3.forward * moveSpeed * Time.deltaTime);
        }
        else if (cmd == "turn_left")
        {
            transform.Rotate(Vector3.up, -rotateSpeed * Time.deltaTime);
        }
        else if (cmd == "turn_right")
        {
            transform.Rotate(Vector3.up, rotateSpeed * Time.deltaTime);
        }
    }

    // اپ ڈیٹ فریم کے ہر ایک پر کہا جاتا ہے
    void Update()
    {
        // روبوٹ کی حالت کو ROS کے پیچھے بھیجیں
        if (Time.time % 0.1f < Time.deltaTime) // 0.1 سیکنڈ کے بعد ہر بار
        {
            var robotState = new StringMsg
            {
                data = $"Position: {transform.position}, Rotation: {transform.rotation.eulerAngles}"
            };
            ros.Publish(robotTopicName + "_state", robotState);
        }
    }
}
```

## Unity میں حقیقی ہیومینائیڈ ماڈلز تخلیق کرنا

### حقیقی ہیومینائیڈز کے لیے کلیدی اجزاء

1. **رگنگ اور اینیمیشن**:
   - حقیقی جوائنٹ کنٹرینٹس کے ساتھ مناسب سکلیٹن
   - حرکات کے درمیان ہموار منتقلی کے لیے بلینڈ ٹریز
   - حقیقی پاؤں اور ہاتھ کی جگہ کے لیے انورس کنیمیٹکس

2. **مواد اور ٹیکسچرز**:
   - حقیقی سطحوں کے لیے PBR (فیزیکلی بیسڈ رینڈرنگ) مواد
   - نارمل میپس کے ساتھ زیادہ ریزولوشن ٹیکسچرز
   - مختلف مواد کے لیے مناسب لائٹنگ ماڈلز

3. **فزکس سیٹ اپ**:
   - حقیقی جسمانی خصوصیات (ماس، فرکشن، باؤنسی نیس)
   - درست فزکس ریسپانس کے لیے مناسب کولیژن شیپس
   - ایمرجنسی منظر ناموں کے لیے ریگڈول فزکس

### ہیومینائیڈ کریکٹر کنٹرولر تخلیق کرنا

```csharp
using UnityEngine;

[RequireComponent(typeof(CharacterController))]
public class UnityHumanoidController : MonoBehaviour
{
    CharacterController controller;
    Animator animator;

    // حرکت کے پیرامیٹرز
    public float walkSpeed = 2.0f;
    public float runSpeed = 4.0f;
    public float turnSpeed = 100.0f;
    public float gravity = -9.81f;

    // اسٹیٹ متغیرات
    Vector3 velocity;
    bool isGrounded;
    float speed;

    void Start()
    {
        controller = GetComponent<CharacterController>();
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        // چیک کریں کہ کریکٹر زمین پر ہے یا نہیں
        isGrounded = controller.isGrounded;
        if (isGrounded && velocity.y < 0)
        {
            velocity.y = -2f; // زمین پر رہنے کے لیے چھوٹا آفسیٹ
        }

        // حرکت کا ان پٹ ہینڈل کریں
        HandleMovement();

        // گریویٹی لاگو کریں
        velocity.y += gravity * Time.deltaTime;

        // کنٹرولر کو حرکت دیں
        Vector3 move = transform.right * speed + transform.up * velocity.y;
        controller.Move(move * Time.deltaTime);

        // اینیمیٹر کے پیرامیٹرز کو اپ ڈیٹ کریں
        UpdateAnimator();
    }

    void HandleMovement()
    {
        // ان پٹ حاصل کریں (ایک حقیقی سسٹم میں، یہ ROS سے آ سکتا ہے)
        float horizontal = Input.GetAxis("Horizontal");
        float vertical = Input.GetAxis("Vertical");

        // حرکت کی سمت کا حساب لگائیں
        Vector3 direction = new Vector3(horizontal, 0, vertical).normalized;

        // ان پٹ کی بنیاد پر رفتار کا حساب لگائیں
        if (direction.magnitude >= 0.1f)
        {
            float targetAngle = Mathf.Atan2(direction.x, direction.z) * Mathf.Rad2Deg + Camera.main.transform.eulerAngles.y;
            float angle = Mathf.SmoothDampAngle(transform.eulerAngles.y, targetAngle, ref turnSpeed, 0.1f);
            transform.rotation = Quaternion.Euler(0f, angle, 0f);

            Vector3 moveDir = Quaternion.Euler(0f, targetAngle, 0f) * Vector3.forward;
            controller.Move(moveDir.normalized * walkSpeed * Time.deltaTime);

            speed = walkSpeed;
        }
        else
        {
            speed = 0f;
        }
    }

    void UpdateAnimator()
    {
        if (animator != null)
        {
            animator.SetFloat("Speed", speed);
            animator.SetFloat("Direction", 0); // سادہ
            animator.SetBool("IsGrounded", isGrounded);
        }
    }
}
```

## Unity میں انسان-روبوٹ تعامل

### تعاملی ماحول تخلیق کرنا

```csharp
using UnityEngine;

public class InteractiveObject : MonoBehaviour
{
    public string objectName;
    public bool canBeGrabbed = true;

    void OnMouseOver()
    {
        // ہوور کیے جانے پر ظہور تبدیل کریں
        GetComponent<Renderer>().material.color = Color.yellow;
    }

    void OnMouseExit()
    {
        // ہوور نہ ہونے پر ظہور دوبارہ سیٹ کریں
        GetComponent<Renderer>().material.color = Color.white;
    }

    void OnMouseDown()
    {
        // تعامل ہینڈل کریں
        Debug.Log($"Object {objectName} کو کلک کیا گیا");
        HandleInteraction();
    }

    void HandleInteraction()
    {
        // ایک حقیقی سسٹم میں، یہ ROS کو ایک پیغام بھیجے گا
        // مثال کے طور پر، اس چیز کو اٹھانا
        Debug.Log($"{objectName} کے ساتھ تعامل کی کوشش کر رہا ہے");
    }
}

public class UnityHumanRobotInteraction : MonoBehaviour
{
    public Camera mainCamera;
    public LayerMask interactionLayer;
    public float interactionDistance = 5f;

    void Update()
    {
        // صارف کے تعامل کے لیے چیک کریں
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;

            if (Physics.Raycast(ray, out hit, interactionDistance, interactionLayer))
            {
                // ROS کو تعامل کمانڈ بھیجیں
                SendInteractionCommand(hit.collider.name, "interact");
            }
        }
    }

    void SendInteractionCommand(string objectName, string command)
    {
        // ایک حقیقی سسٹم میں، یہ ایک ROS پیغام بھیجے گا
        Debug.Log($"Interaction command: {command} with {objectName}");
    }
}
```

## ہیومینائیڈ روبوٹ سیمولیشن کے لیے Unity منظر

```csharp
using UnityEngine;

public class UnityRobotEnvironment : MonoBehaviour
{
    public GameObject robotPrefab;
    public Transform[] spawnPoints;
    public GameObject[] furniturePrefabs;
    public Light[] lightingSetup;

    [Header("Environment Settings")]
    public float gravity = -9.81f;
    public PhysicMaterial floorMaterial;

    void Start()
    {
        // فزکس کی ترتیبات کو شروع کریں
        Physics.gravity = new Vector3(0, gravity, 0);

        // فراہم کیا گیا تو فرش کا مادہ سیٹ کریں
        if (floorMaterial != null)
        {
            var floorColliders = FindObjectsOfType<Collider>();
            foreach (var col in floorColliders)
            {
                if (col.CompareTag("Floor"))
                {
                    col.material = floorMaterial;
                }
            }
        }

        // رینڈم اسپون پوائنٹ پر روبوٹ اسپون کریں
        if (spawnPoints.Length > 0)
        {
            Transform spawnPoint = spawnPoints[Random.Range(0, spawnPoints.Length)];
            Instantiate(robotPrefab, spawnPoint.position, spawnPoint.rotation);
        }

        // لائٹنگ سیٹ کریں
        SetupLighting();

        // تعاملی ماحول تخلیق کریں
        SetupInteractiveEnvironment();
    }

    void SetupLighting()
    {
        // حقیقی رینڈرنگ کو بہتر بنانے کے لیے لائٹنگ تشکیل دیں
        foreach (Light light in lightingSetup)
        {
            // حقیقی لائٹنگ کی خصوصیات کو ایڈجسٹ کریں
            if (light.type == LightType.Directional)
            {
                // سورج جیسی لائٹنگ تشکیل دیں
                RenderSettings.ambientIntensity = 0.5f;
                RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Trilight;
            }
        }

        // حقیقی لائٹنگ کی ترتیبات لاگو کریں
        RenderSettings.fog = true;
        RenderSettings.fogColor = Color.grey;
        RenderSettings.fogDensity = 0.01f;
    }

    void SetupInteractiveEnvironment()
    {
        // گھر جیسا ماحول تخلیق کریں
        CreateRoomLayout();
        AddInteractiveElements();
    }

    void CreateRoomLayout()
    {
        // فرنیچر اور رکاوٹیں تخلیق کریں
        // یہ عام طور پر Unity ایڈیٹر میں کیا جائے گا
        // لیکن ہم پروگرامی طور پر ایک بنیادی لے آؤٹ سیٹ کر سکتے ہیں
        foreach (Transform spawnPoint in spawnPoints)
        {
            if (Random.Range(0, 100) > 70) // 30% امکان فرنیچر رکھنے کا
            {
                if (furniturePrefabs.Length > 0)
                {
                    GameObject furniture = furniturePrefabs[Random.Range(0, furniturePrefabs.Length)];
                    Instantiate(furniture, spawnPoint.position + new Vector3(2, 0, 0), Quaternion.identity);
                }
            }
        }
    }

    void AddInteractiveElements()
    {
        // دروازے، سوئچز وغیرہ جیسے تعاملی عناصر شامل کریں
        // ان کے تعامل کے لیے خصوصی اسکرپٹس ہوں گے
    }
}
```

## Unity میں ہائی فائیڈلٹی رینڈرنگ کی خصوصیات

### پوسٹ پروسیسنگ ایفیکٹس

Unity جدید پوسٹ پروسیسنگ خصوصیات فراہم کرتا ہے جو ویژول کی حقیقت کو بہتر بناتا ہے:

```csharp
using UnityEngine;
using UnityEngine.Rendering.PostProcessing;

[RequireComponent(typeof(PostProcessVolume))]
public class RenderingController : MonoBehaviour
{
    public PostProcessVolume postProcessVolume;
    private DepthOfField depthOfField;
    private MotionBlur motionBlur;
    private AmbientOcclusion ambientOcclusion;

    public void SetRenderingQuality(int qualityLevel)
    {
        var profile = postProcessVolume.profile;

        switch (qualityLevel)
        {
            case 0: // کم معیار (ریل ٹائم سیمولیشن کے لیے)
                if (profile.TryGetSettings(out motionBlur))
                    motionBlur.active = false;
                if (profile.TryGetSettings(out ambientOcclusion))
                    ambientOcclusion.active = false;
                break;

            case 1: // میڈیم معیار
                if (profile.TryGetSettings(out motionBlur))
                {
                    motionBlur.active = true;
                    motionBlur.shutterAngle.value = 180f;
                }
                if (profile.TryGetSettings(out ambientOcclusion))
                {
                    ambientOcclusion.active = true;
                    ambientOcclusion.intensity.value = 1.0f;
                }
                break;

            case 2: // زیادہ معیار (وژولائزیشن کے لیے)
                if (profile.TryGetSettings(out motionBlur))
                {
                    motionBlur.active = true;
                    motionBlur.shutterAngle.value = 270f;
                }
                if (profile.TryGetSettings(out ambientOcclusion))
                {
                    ambientOcclusion.active = true;
                    ambientOcclusion.intensity.value = 2.0f;
                }
                // دیگر زیادہ معیار کے ایفیکٹس شامل کریں
                break;
        }
    }
}
```

### ڈائنا مک لائٹنگ اور سایہ

```csharp
using UnityEngine;

public class DynamicLightingController : MonoBehaviour
{
    public Light[] sceneLights;
    public AnimationCurve lightIntensityCurve;
    public float dayNightCycleDuration = 120f; // سیکنڈ

    private float cycleTime = 0f;

    void Update()
    {
        cycleTime += Time.deltaTime;
        float normalizedTime = (cycleTime % dayNightCycleDuration) / dayNightCycleDuration;

        // لائٹس میں دن-رات چکر لاگو کریں
        foreach (Light light in sceneLights)
        {
            float intensity = lightIntensityCurve.Evaluate(normalizedTime);
            light.intensity = intensity;

            // دن کے وقت کی بنیاد پر رنگ کا درجہ ایڈجسٹ کریں
            float colorTemperature = Mathf.Lerp(4000f, 6500f, intensity);
            light.color = GetColorForTemperature(colorTemperature);
        }
    }

    Color GetColorForTemperature(float temperatureK)
    {
        // سادہ رنگ کا درجہ کیلکولیشن
        temperatureK /= 100f;

        float r, g, b;

        if (temperatureK <= 66)
        {
            r = 255;
            g = temperatureK;
            g = 99.4708025861f * Mathf.Log(g) - 161.1195681661f;
        }
        else
        {
            r = temperatureK - 60;
            r = 329.698727446f * Mathf.Pow(r, -0.1332047592f);
            g = temperatureK - 60;
            g = 288.1221695283f * Mathf.Pow(g, -0.0755148492f);
        }

        if (temperatureK >= 66)
        {
            b = 255;
        }
        else if (temperatureK <= 19)
        {
            b = 0;
        }
        else
        {
            b = temperatureK - 10;
            b = 138.5177312231f * Mathf.Log(b) - 305.0447927307f;
        }

        return new Color(r / 255f, g / 255f, b / 255f);
    }
}
```

## ROS 2 کے ساتھ انضمام: Unity روبوٹکس پیکج

### پبلشر کی مثال

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;

public class UnityRobotPublisher : MonoBehaviour
{
    ROSConnection ros;
    string robotStateTopic = "unity_robot_state";
    string robotJointsTopic = "unity_joint_states";

    public Transform robotRoot;
    public Transform[] jointTransforms;
    public string[] jointNames;

    void Start()
    {
        ros = ROSConnection.instance;
    }

    void Update()
    {
        if (Time.time % 0.05f < Time.deltaTime) // ہر 50ms میں ایک بار بھیجیں
        {
            PublishRobotState();
            PublishJointStates();
        }
    }

    void PublishRobotState()
    {
        var robotState = new PointMsg
        {
            x = robotRoot.position.x,
            y = robotRoot.position.y,
            z = robotRoot.position.z
        };

        ros.Publish(robotStateTopic, robotState);
    }

    void PublishJointStates()
    {
        var jointState = new JointStateMsg
        {
            name = jointNames,
            position = new double[jointTransforms.Length],
            velocity = new double[jointTransforms.Length],
            effort = new double[jointTransforms.Length]
        };

        for (int i = 0; i < jointTransforms.Length; i++)
        {
            // جوائنٹ اینگلز حاصل کریں (یہ سادہ کیا گیا ہے)
            jointState.position[i] = jointTransforms[i].localEulerAngles.y;
            jointState.velocity[i] = 0.0; // ایک حقیقی سسٹم میں حساب لگایا جائے گا
            jointState.effort[i] = 0.0;   // ایک حقیقی سسٹم میں حساب لگایا جائے گا
        }

        // ٹائم اسٹیمپس سیٹ کریں
        jointState.header = new HeaderMsg
        {
            stamp = new TimeMsg { sec = (int)Time.time, nanosec = (uint)((Time.time % 1) * 1e9) },
            frame_id = "unity_robot"
        };

        ros.Publish(robotJointsTopic, jointState);
    }
}
```

### سبسکرائبر کی مثال

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Geometry_msgs;
using Unity.Robotics.ROSTCPConnector.MessageTypes.Sensor_msgs;

public class UnityRobotSubscriber : MonoBehaviour
{
    ROSConnection ros;
    string robotCmdTopic = "unity_robot_cmd";

    public Transform robotRoot;
    public Transform[] jointTransforms;
    public string[] jointNames;

    void Start()
    {
        ros = ROSConnection.instance;
        ros.Subscribe<TwistMsg>(robotCmdTopic + "/cmd_vel", CmdVelCallback);
        ros.Subscribe<JointStateMsg>(robotCmdTopic + "/joint_commands", JointCmdCallback);
    }

    void CmdVelCallback(TwistMsg cmd)
    {
        // رفتار کمانڈز کو پروسیس کریں
        Vector3 linearVelocity = new Vector3((float)cmd.linear.x, (float)cmd.linear.y, (float)cmd.linear.z);
        Vector3 angularVelocity = new Vector3((float)cmd.angular.x, (float)cmd.angular.y, (float)cmd.angular.z);

        // کمانڈز کو روبوٹ پر لاگو کریں
        robotRoot.Translate(linearVelocity * Time.deltaTime);
        robotRoot.Rotate(angularVelocity * Mathf.Rad2Deg * Time.deltaTime);
    }

    void JointCmdCallback(JointStateMsg jointCmd)
    {
        // جوائنٹ پوزیشن کمانڈز کو پروسیس کریں
        for (int i = 0; i < jointCmd.name.Length; i++)
        {
            string jointName = jointCmd.name[i];
            double position = jointCmd.position[i];

            // متعلقہ جوائنٹ ٹرانسفارم تلاش کریں
            for (int j = 0; j < jointNames.Length; j++)
            {
                if (jointNames[j] == jointName && j < jointTransforms.Length)
                {
                    // جوائنٹ پر پوزیشن کمانڈ لاگو کریں
                    // یہ سادہ کیا گیا ہے - حقیقت میں، آپ انورس کنیمیٹکس استعمال کریں گے
                    jointTransforms[j].localEulerAngles = new Vector3(
                        jointTransforms[j].localEulerAngles.x,
                        (float)position * Mathf.Rad2Deg,
                        jointTransforms[j].localEulerAngles.z
                    );
                    break;
                }
            }
        }
    }
}
```

## انسان-روبوٹ تعامل کے منظر نامے

### سوشل نیویگیشن

Unity سوشل نیویگیشن منظر نامے کی تخلیق کو فعال کرتا ہے:

```csharp
using UnityEngine;
using System.Collections.Generic;

public class SocialInteractionController : MonoBehaviour
{
    public List<GameObject> humanAvatars; // منظر میں ہیومینائیڈ کریکٹر
    public float personalSpaceRadius = 0.8f; // انسانوں کے لیے آرام دہ فاصلہ
    public float socialInteractionDistance = 2.0f; // تعامل کے لیے فاصلہ

    void Update()
    {
        // انسان-روبوٹ تعاملات کے لیے چیک کریں
        CheckSocialInteractions();
    }

    void CheckSocialInteractions()
    {
        Vector3 robotPos = transform.position;

        foreach (GameObject human in humanAvatars)
        {
            if (human == null) continue;

            Vector3 humanPos = human.transform.position;
            float distance = Vector3.Distance(robotPos, humanPos);

            if (distance < personalSpaceRadius)
            {
                // بہت قریب - دور ہٹ جائیں
                AvoidCollision(human);
            }
            else if (distance < socialInteractionDistance)
            {
                // تعامل کی حد میں - مسکرانا یا راستہ چھوڑنے پر غور کریں
                HandleSocialApproach(human);
            }
        }
    }

    void AvoidCollision(GameObject human)
    {
        // اجتنے کی سمت کا حساب لگائیں
        Vector3 direction = (transform.position - human.transform.position).normalized;
        transform.position += direction * Time.deltaTime * 0.5f;
    }

    void HandleSocialApproach(GameObject human)
    {
        // ایک حقیقی سسٹم میں، یہ ایک مسکرانے کی اینیمیشن کو متحرک کر سکتا ہے
        // یا راستہ چھوڑنے کا رویہ
        Debug.Log("روبوٹ تعامل کے لیے انسان کے قریب ہے");
    }
}
```

## حقیقی سینسر سیمولیشن

### Unity ادراک کے ساتھ کیمرہ سیمولیشن

```csharp
using UnityEngine;
#if UNITY_EDITOR
using Unity.Perception.GroundTruth;
#endif

public class UnityCameraSensor : MonoBehaviour
{
    public Camera sensorCamera;
    public string sensorName = "rgb_camera";

#if UNITY_EDITOR
    void Start()
    {
        if (sensorCamera != null)
        {
            // مصنوعی ڈیٹا تخلیق کے لیے ادراک کے اجزاء شامل کریں
            sensorCamera.gameObject.AddComponent<SegmentationLabel>();
            sensorCamera.gameObject.AddComponent<CameraSensor>();

            // ادراک کے کاموں کے لیے کیمرہ کو تشکیل دیں
            var cameraSensor = sensorCamera.GetComponent<CameraSensor>();
            cameraSensor.sensorName = sensorName;
        }
    }
#endif

    void Update()
    {
        // ایک حقیقی سسٹم میں، یہ ROS کو کیمرہ ڈیٹا شائع کرے گا
        // ROS-TCP-Connector کے ذریعے
    }
}
```

## خلاصہ

اس باب میں ہیومینائیڈ روبوٹکس کے لیے Unity کا استعمال کرتے ہوئے ہائی فائیڈلٹی رینڈرنگ کی تلاش کی گئی، مختلف استعمال کے معاملات کے لیے اس کا Gazebo کے ساتھ موازنہ کیا گیا:

- انسان-روبوٹ تعامل کے لیے حقیقی وژولائزیشن کے لیے Unity کی اعلیٰ رینڈرنگ کی صلاحیتیں
- Unity میں حقیقی ہیومینائیڈ ماڈلز تخلیق کرنے کی تکنیکیں
- Unity کی صلاحیتوں کا استعمال کرتے ہوئے انسان-روبوٹ تعامل کے منظر نامے
- روبوٹکس ایپلی کیشنز کے لیے Unity کا ROS 2 کے ساتھ انضمام
- حقیقی سیمولیشن کے لیے جدید رینڈرنگ کی خصوصیات

Unity ہائی فائیڈلٹی رینڈرنگ کے لیے طاقتور ٹولز فراہم کرتا ہے جو Gazebo کے فزکس-مرکز سیمولیشن کو ساتھ دیتا ہے، انسان-روبوٹ تعامل کے مطالعات کے لیے حقیقی وژولائزیشن کے ساتھ ہیومینائیڈ روبوٹ کی ترقی کے لیے مکمل حل پیش کرتا ہے۔

## مشقیں

1. Unity میں بنیادی اینیمیشنز کے ساتھ ایک سادہ ہیومینائیڈ کریکٹر تخلیق کریں
2. ROS-TCP-Connector کا استعمال کرتے ہوئے ایک بنیادی Unity-ROS کنکشن سیٹ کریں
3. ایک سادہ انسان-روبوٹ تعامل کا منظر نامہ نافذ کریں

## اگلے اقدامات

اگلے باب میں، ہم دونوں Gazebo اور Unity سیمولیشن کو ایک مربوط ورک فلو میں ضم کرنے کی تلاش کریں گے، ہیومینائیڈ روبوٹ کی ترقی اور ٹیسٹنگ کے مختلف پہلوؤں کے لیے ہر پلیٹ فارم کی مضبوطیوں کا فائدہ اٹھاتے ہوئے۔