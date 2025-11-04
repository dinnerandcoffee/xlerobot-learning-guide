# 3.6 URDF와 MJCF 로봇 모델 이해

로봇 시뮬레이션의 핵심은 **로봇 모델 파일**입니다. URDF와 MJCF 형식을 이해하고 XLeRobot 모델을 분석합니다.

## 목차
- [로봇 모델 파일 개요](#로봇-모델-파일-개요)
- [URDF 형식](#urdf-형식)
- [MJCF 형식](#mjcf-형식)
- [XLeRobot MJCF 분석](#xlerobot-mjcf-분석)
- [모델 수정하기](#모델-수정하기)
- [변환 도구](#변환-도구)

---

## 로봇 모델 파일 개요

### 로봇 모델이란?

로봇 시뮬레이션을 위해서는 로봇의 **기하학적 구조**와 **물리적 특성**을 정의한 파일이 필요합니다.

```
로봇 모델 파일 구성 요소:
┌─────────────────────────────────────┐
│  1️⃣ 링크 (Link)                     │
│     - 로봇의 각 부품 (팔, 다리 등)   │
│     - 질량, 관성                     │
│     - 3D 메시 (시각화)               │
│                                     │
│  2️⃣ 조인트 (Joint)                  │
│     - 링크 간 연결                   │
│     - 회전/슬라이드 타입              │
│     - 한계, 속도, 토크               │
│                                     │
│  3️⃣ 액추에이터 (Actuator)            │
│     - 관절 제어 방식                 │
│     - PD 제어기 파라미터             │
│                                     │
│  4️⃣ 센서 (Sensor)                   │
│     - 카메라, IMU, 힘 센서 등        │
│                                     │
└─────────────────────────────────────┘
```

### 주요 형식 비교

| 형식 | 전체 이름 | 개발사 | 주요 사용처 |
|------|-----------|--------|-------------|
| **URDF** | Unified Robot Description Format | ROS (Willow Garage) | ROS, Gazebo, Isaac Sim |
| **MJCF** | MuJoCo Model Format (XML) | DeepMind (구 Google) | MuJoCo 시뮬레이터 |
| **SDF** | Simulation Description Format | OSRF | Gazebo 시뮬레이터 |
| **USD** | Universal Scene Description | Pixar | Isaac Sim, 3D 렌더링 |

**XLeRobot에서 사용**:
- MuJoCo: **MJCF** (`xlerobot.xml`)
- ManiSkill: **URDF** (내부 변환)
- Isaac Sim: **USD** (Leisaac 제공)

---

## URDF 형식

### URDF 구조

URDF는 **XML 기반** 로봇 정의 형식으로, ROS 생태계의 표준입니다.

#### 기본 구조

```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  
  <!-- 링크 정의 -->
  <link name="base_link">
    <!-- 시각화 -->
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    
    <!-- 충돌 -->
    <collision>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
    </collision>
    
    <!-- 관성 -->
    <inertial>
      <mass value="10.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0"
               iyy="1.0" iyz="0.0"
               izz="1.0"/>
    </inertial>
  </link>
  
  <!-- 또 다른 링크 -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
      <material name="orange">
        <color rgba="1 0.5 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0"
               iyy="0.1" iyz="0.0"
               izz="0.01"/>
    </inertial>
  </link>
  
  <!-- 조인트 정의 -->
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>
  
</robot>
```

#### 주요 요소 설명

##### 1. `<link>`: 링크 (강체)

```xml
<link name="이름">
  <visual>     <!-- 시각화용 기하 -->
  <collision>  <!-- 충돌 감지용 기하 -->
  <inertial>   <!-- 질량, 관성 모멘트 -->
</link>
```

**visual vs collision**:
- `visual`: 화면에 표시되는 모양 (고해상도 메시)
- `collision`: 충돌 계산용 (단순 기하, 빠름)

##### 2. `<joint>`: 조인트

```xml
<joint name="이름" type="타입">
  <parent link="부모_링크"/>
  <child link="자식_링크"/>
  <origin xyz="x y z" rpy="roll pitch yaw"/>
  <axis xyz="x y z"/>  <!-- 회전 축 -->
  <limit lower="하한" upper="상한" effort="토크" velocity="속도"/>
</joint>
```

**조인트 타입**:
- `revolute`: 회전 (한계 있음)
- `continuous`: 연속 회전 (한계 없음)
- `prismatic`: 직선 이동 (슬라이드)
- `fixed`: 고정 (움직이지 않음)
- `floating`: 자유 공간 (6 DOF)
- `planar`: 평면 이동 (3 DOF: x, y, θ)

---

## MJCF 형식

### MJCF 구조

MJCF (MuJoCo XML Format)는 MuJoCo 전용 형식으로, **효율성**과 **정밀한 물리**에 최적화되어 있습니다.

#### 기본 구조

```xml
<mujoco model="simple_robot">
  
  <!-- 컴파일러 설정 -->
  <compiler angle="radian" meshdir="./meshes/"/>
  
  <!-- 옵션 (물리 설정) -->
  <option timestep="0.002" gravity="0 0 -9.81"/>
  
  <!-- 에셋 (메시, 재질) -->
  <asset>
    <mesh name="arm_mesh" file="arm.stl"/>
    <material name="orange" rgba="1 0.5 0 1"/>
  </asset>
  
  <!-- 기본값 -->
  <default>
    <joint damping="0.1" armature="0.01"/>
    <geom contype="1" conaffinity="1" friction="1 0.5 0.01"/>
  </default>
  
  <!-- 월드 바디 -->
  <worldbody>
    <light pos="0 0 3" dir="0 0 -1"/>
    <geom type="plane" size="0 0 1"/>  <!-- 바닥 -->
    
    <!-- 베이스 바디 -->
    <body name="base" pos="0 0 0.1">
      <joint type="free"/>  <!-- 자유 공간 이동 -->
      <geom type="box" size="0.25 0.25 0.05" rgba="0 0 1 1"/>
      <inertial pos="0 0 0" mass="10" diaginertia="1 1 1"/>
      
      <!-- 팔 바디 -->
      <body name="arm" pos="0 0 0.05">
        <joint name="arm_joint" type="hinge" axis="0 0 1" range="-1.57 1.57"/>
        <geom type="cylinder" size="0.05 0.15" rgba="1 0.5 0 1"/>
        <inertial pos="0 0 0.15" mass="2" diaginertia="0.1 0.1 0.01"/>
      </body>
    </body>
  </worldbody>
  
  <!-- 액추에이터 -->
  <actuator>
    <position name="arm_actuator" joint="arm_joint" kp="50"/>
  </actuator>
  
</mujoco>
```

#### URDF vs MJCF 차이점

| 항목 | URDF | MJCF |
|------|------|------|
| **구조** | link + joint 분리 | body 계층 (중첩) |
| **기하** | `<visual>`, `<collision>` | `<geom>` 하나로 통합 |
| **관성** | 6x6 텐서 | 대각 요소만 (`diaginertia`) |
| **액추에이터** | 없음 (별도 정의) | `<actuator>` 내장 |
| **기본값** | 없음 | `<default>` 클래스 |
| **효율성** | 중간 | 높음 (물리 최적화) |

---

## XLeRobot MJCF 분석

XLeRobot의 MuJoCo 모델 파일 `xlerobot.xml`을 상세히 분석합니다.

### 파일 구조 개요

```bash
~/XLeRobot/simulation/mujoco/
├── xlerobot.xml        # 로봇 모델 정의 (메인)
├── scene.xml           # 씬 설정 (바닥, 조명, xlerobot.xml 포함)
└── assets/
    ├── Base.stl
    ├── Upper_Arm.stl
    ├── Lower_Arm.stl
    └── ... (총 30+ STL 파일)
```

**scene.xml**이 **xlerobot.xml**을 포함:

```xml
<!-- scene.xml -->
<mujoco model="xlerobot_scene">
  <include file="./xlerobot.xml"/>  <!-- XLeRobot 모델 로드 -->
  
  <option timestep="0.002" gravity="0 0 -9.80665"/>
  
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1"/>
    <geom name="floor" type="plane"/>
  </worldbody>
</mujoco>
```

---

### xlerobot.xml 주요 섹션

#### 1. 컴파일러 설정

```xml
<compiler angle="radian" meshdir="./assets/"/>
```

- `angle="radian"`: 모든 각도를 라디안으로 해석
- `meshdir`: STL 메시 파일 경로

---

#### 2. 에셋 (메시, 재질)

```xml
<asset>
  <!-- 재질 정의 -->
  <material name="orange" rgba="1.0 0.331 0.0 1.0"/>
  <material name="black" rgba="0.1 0.1 0.1 1.0"/>
  
  <!-- SO-100 팔 메시 -->
  <mesh name="Base" file="Base.stl"/>
  <mesh name="Upper_Arm" file="Upper_Arm.stl"/>
  <mesh name="Lower_Arm" file="Lower_Arm.stl"/>
  
  <!-- 옴니휠 메시 (scale로 크기 조정) -->
  <mesh name="4-Omni-Directional-Wheel_Single_Body-v1" 
        file="simplify_4-Omni-Directional-Wheel_Single_Body-v1.stl" 
        scale="0.001 0.001 0.001"/>  <!-- mm → m -->
</asset>
```

**메시 파일 형식**:
- `.stl`: 삼각형 메시 (3D 모델 표준)
- `scale`: 크기 조정 (CAD 파일은 보통 mm 단위)

---

#### 3. 기본값 (default)

```xml
<default>
  <default class="so_arm100">
    <joint frictionloss="0.1" armature="0.1"/>
    <position kp="50" dampratio="1" forcerange="-35 35"/>
    
    <default class="Rotation">
      <joint axis="0 1 0" range="-2.2 2.2"/>
    </default>
    
    <default class="Pitch">
      <joint axis="1 0 0" range="-3.14158 0.2"/>
    </default>
    
    <default class="Elbow">
      <joint axis="1 0 0" range="0 3.14158"/>
    </default>
  </default>
</default>
```

**파라미터 설명**:
- `frictionloss`: 관절 마찰 (N·m)
- `armature`: 로터 관성 (kg·m²)
- `kp`: 위치 제어 게인
- `dampratio`: 감쇠 비율
- `forcerange`: 힘 한계 (N 또는 N·m)

---

#### 4. 텐던 (Tendon) - 옴니휠 제어

```xml
<tendon>
  <!-- X 방향 이동 -->
  <fixed name="slider_actuator_x">
    <joint joint="ST3215_Servo_Motor-v1-2_Hub---Servo" coef="0"/>
    <joint joint="ST3215_Servo_Motor-v1-1_Hub-2---Servo" coef="-0.5"/>
    <joint joint="ST3215_Servo_Motor-v1_Revolute-40" coef="0.5"/>
  </fixed>
  
  <!-- Y 방향 이동 -->
  <fixed name="slide">
    <joint joint="ST3215_Servo_Motor-v1-2_Hub---Servo" coef="0.0"/>
    <joint joint="ST3215_Servo_Motor-v1-1_Hub-2---Servo" coef="0.286"/>
    <joint joint="ST3215_Servo_Motor-v1_Revolute-40" coef="-0.286"/>
  </fixed>
  
  <!-- 회전 -->
  <fixed name="rotate">
    <joint joint="ST3215_Servo_Motor-v1-2_Hub---Servo" coef="0.33"/>
    <joint joint="ST3215_Servo_Motor-v1-1_Hub-2---Servo" coef="0.33"/>
    <joint joint="ST3215_Servo_Motor-v1_Revolute-40" coef="0.33"/>
  </fixed>
</tendon>
```

**텐던이란?**:
- 여러 관절을 묶어서 **하나의 제어 신호**로 구동
- 옴니휠: 3개 휠 → (x, y, θ) 이동
- `coef`: 각 휠의 기여도 (역기구학)

---

#### 5. 월드바디 - 섀시 (베이스)

```xml
<worldbody>
  <body name="chassis" pos="0 0 0.035">
    <!-- 자유 이동 조인트 (3DOF) -->
    <joint name="slide_joint_x" type="slide" axis="1 0 0"/>
    <joint name="slide_joint_y" type="slide" axis="0 1 0"/>
    <joint name="hinge_joint_z" type="hinge" axis="0 0 1"/>
    
    <!-- 충돌 박스 (시각화용) -->
    <geom type="box" size="0.2 0.2 0.38" rgba="0.2 0.2 0.6 0.2"/>
    
    <!-- 베이스 플레이트 -->
    <body name="base_plate_layer1-v5-1" pos="0.0 0.0 0.0">
      <geom name="base_plate_layer1-v5-1_geom" 
            type="mesh" 
            mesh="base_plate_layer1-v5-1" 
            rgba="0.1 0.1 0.1 1"/>
      <inertial mass="1.95" pos="0 0 -0.0035" 
                fullinertia="0.0056 0.0056 0.011 0 0 0"/>
      
      <!-- 옴니휠 3개 (중첩 바디) -->
      <body name="drive_motor_mount-v4-3" pos="-0.021 -0.09 0.002">
        <!-- ... 휠 1 -->
      </body>
      <body name="drive_motor_mount-v4-2" pos="0.088 0.027 0.002">
        <!-- ... 휠 2 -->
      </body>
      <body name="drive_motor_mount-v4-1" pos="-0.068 0.063 0.002">
        <!-- ... 휠 3 -->
      </body>
    </body>
  </body>
</worldbody>
```

**계층 구조**:
```
worldbody
  └─ chassis (자유 이동 3DOF)
      └─ base_plate_layer1
          ├─ drive_motor_mount-v4-3 (휠 1)
          ├─ drive_motor_mount-v4-2 (휠 2)
          ├─ drive_motor_mount-v4-3 (휠 3)
          ├─ 왼팔 (Rotation_Pitch → Upper_Arm → Lower_Arm → ...)
          └─ 오른팔 (Rotation_Pitch_R → ...)
```

---

#### 6. 팔 (SO-100) 구조

```xml
<!-- 왼팔 -->
<body name="Rotation_Pitch" pos="0.1352 -0.15 0.7865">
  <joint name="Rotation_L" axis="0 -1 0" range="-2.1 2.1"/>
  <geom type="mesh" mesh="Rotation_Pitch"/>
  <inertial mass="0.119" pos="0 0.059 0.031" 
            diaginertia="5.94e-05 5.90e-05 3.14e-05"/>
  
  <!-- 상완 (Upper Arm) -->
  <body name="Upper_Arm" pos="0 0.1025 0.0306">
    <joint name="Pitch_L" axis="-1 0 0" range="-0.1 3.45"/>
    <geom type="mesh" mesh="Upper_Arm"/>
    
    <!-- 하완 (Lower Arm) -->
    <body name="Lower_Arm" pos="0 0.113 0.028">
      <joint name="Elbow_L" axis="1 0 0" range="-0.2 3.14159"/>
      <geom type="mesh" mesh="Lower_Arm"/>
      
      <!-- 손목 (Wrist) -->
      <body name="Wrist_Pitch_Roll" pos="0 0.0052 0.1349">
        <joint name="Wrist_Pitch_L" axis="1 0 0" range="-1.8 1.8"/>
        
        <!-- 그리퍼 (Gripper) -->
        <body name="Fixed_Jaw" pos="0 -0.0601 0">
          <joint name="Wrist_Roll_L" axis="0 -1 0" range="-3.14 3.14"/>
          
          <!-- 움직이는 턱 (Moving Jaw) -->
          <body name="Moving_Jaw" pos="-0.0202 -0.0244 0">
            <joint name="Jaw_L" axis="0 0 1" range="-3.14 3.14"/>
            <geom type="mesh" mesh="Moving_Jaw_part1"/>
          </body>
        </body>
      </body>
    </body>
  </body>
</body>
```

**왼팔 관절 6개**:
1. `Rotation_L`: 어깨 회전 (Y축)
2. `Pitch_L`: 어깨 피치 (X축)
3. `Elbow_L`: 팔꿈치 (X축)
4. `Wrist_Pitch_L`: 손목 피치 (X축)
5. `Wrist_Roll_L`: 손목 롤 (Y축)
6. `Jaw_L`: 그리퍼 (Z축)

---

### 관절 매핑 테이블

| 인덱스 | 이름 | 타입 | 범위 (rad) | 설명 |
|--------|------|------|------------|------|
| 0 | `slide_joint_x` | slide | 무한 | 베이스 X 이동 |
| 1 | `slide_joint_y` | slide | 무한 | 베이스 Y 이동 |
| 2 | `hinge_joint_z` | hinge | 무한 | 베이스 Z 회전 |
| 3 | `Rotation_L` | hinge | -2.1 ~ 2.1 | 왼팔 어깨 회전 |
| 4 | `Pitch_L` | hinge | -0.1 ~ 3.45 | 왼팔 어깨 피치 |
| 5 | `Elbow_L` | hinge | -0.2 ~ 3.14 | 왼팔 팔꿈치 |
| 6 | `Wrist_Pitch_L` | hinge | -1.8 ~ 1.8 | 왼팔 손목 피치 |
| 7 | `Wrist_Roll_L` | hinge | -3.14 ~ 3.14 | 왼팔 손목 롤 |
| 8 | `Jaw_L` | hinge | -3.14 ~ 3.14 | 왼팔 그리퍼 |
| 9-14 | (오른팔 동일) | ... | ... | ... |
| 15-17 | (옴니휠 3개) | hinge | 무한 | 휠 회전 |

---

## 모델 수정하기

MJCF 파일을 수정하여 로봇을 커스터마이징할 수 있습니다.

### 예제 1: 관절 한계 변경

**문제**: 팔꿈치가 너무 많이 구부러짐

**해결**: `range` 수정

```xml
<!-- 원본 -->
<joint name="Elbow_L" axis="1 0 0" range="-0.2 3.14159"/>

<!-- 수정 (90도로 제한) -->
<joint name="Elbow_L" axis="1 0 0" range="0 1.57"/>
```

---

### 예제 2: 질량 조정

**문제**: 팔이 너무 무거워서 베이스가 기울어짐

**해결**: `mass` 감소

```xml
<!-- 원본 -->
<inertial mass="0.162409" pos="0 0.07 0.003" diaginertia="..."/>

<!-- 수정 (50% 감소) -->
<inertial mass="0.081" pos="0 0.07 0.003" diaginertia="..."/>
```

---

### 예제 3: 마찰 계수 변경

**문제**: 그리퍼가 물체를 잘 잡지 못함

**해결**: `friction` 증가

```xml
<!-- 그리퍼 geom 찾기 -->
<geom type="mesh" mesh="Moving_Jaw_part1" friction="1 0.5 0.01"/>

<!-- friction 값 증가 -->
<geom type="mesh" mesh="Moving_Jaw_part1" friction="2 1 0.1"/>
```

**friction 파라미터**:
- 첫 번째: 슬라이딩 마찰
- 두 번째: 회전 마찰 (torsional)
- 세 번째: 롤링 마찰

---

### 예제 4: 센서 추가

**목표**: 베이스에 IMU 센서 추가

```xml
<!-- xlerobot.xml의 </mujoco> 태그 직전에 추가 -->
<sensor>
  <!-- 가속도계 -->
  <accelerometer name="imu_accel" site="imu_site"/>
  
  <!-- 자이로스코프 -->
  <gyro name="imu_gyro" site="imu_site"/>
  
  <!-- 힘/토크 센서 (왼팔 손목) -->
  <force name="left_wrist_force" site="left_wrist_site"/>
  <torque name="left_wrist_torque" site="left_wrist_site"/>
</sensor>

<!-- 센서 위치 정의 (site) -->
<worldbody>
  <body name="chassis" pos="0 0 0.035">
    <site name="imu_site" pos="0 0 0.5" size="0.01"/>
    <!-- ... 나머지 코드 -->
  </body>
</worldbody>
```

**센서 읽기 (Python)**:
```python
import mujoco

model = mujoco.MjModel.from_xml_path("xlerobot.xml")
data = mujoco.MjData(model)

# 시뮬레이션 스텝
mujoco.mj_step(model, data)

# 센서 데이터 읽기
accel = data.sensordata[0:3]  # 가속도 (m/s²)
gyro = data.sensordata[3:6]   # 각속도 (rad/s)

print(f"Acceleration: {accel}")
print(f"Gyro: {gyro}")
```

---

## 변환 도구

### URDF ↔ MJCF 변환

#### 1. URDF → MJCF (공식 도구)

```bash
# MuJoCo 내장 컴파일러 사용
compile xlerobot.urdf xlerobot.xml
```

**Python으로 변환**:
```python
import mujoco

# URDF 파일 로드
model = mujoco.MjModel.from_xml_path("xlerobot.urdf")

# MJCF로 저장
mujoco.mj_saveLastXML("xlerobot_converted.xml", model)
```

---

#### 2. MJCF → URDF (비공식)

```bash
# mjcf2urdf 도구 설치
pip install mjcf2urdf

# 변환
mjcf2urdf xlerobot.xml -o xlerobot.urdf
```

⚠️ **주의**: 일부 MuJoCo 전용 기능(예: tendon)은 변환되지 않음

---

### 메시 파일 변환

#### STL → OBJ

```bash
# MeshLab 사용 (GUI)
meshlab

# 또는 Python (trimesh)
pip install trimesh
python -c "import trimesh; m = trimesh.load('arm.stl'); m.export('arm.obj')"
```

---

#### 메시 단순화 (Simplification)

**문제**: 고해상도 메시 → 느린 시뮬레이션

**해결**: 폴리곤 수 감소

```python
import trimesh

# 메시 로드
mesh = trimesh.load("Upper_Arm.stl")

print(f"Original faces: {len(mesh.faces)}")  # 예: 50000

# 단순화 (10% 유지)
simplified = mesh.simplify_quadric_decimation(5000)

print(f"Simplified faces: {len(simplified.faces)}")  # 5000

# 저장
simplified.export("Upper_Arm_simplified.stl")
```

---

## 모델 검증

### 1. MuJoCo 시각화

```bash
# MuJoCo 뷰어로 모델 확인
simulate xlerobot.xml
```

**키보드 단축키**:
- `Space`: 시뮬레이션 시작/정지
- `Ctrl+R`: 리셋
- `Ctrl+A`: 모든 관절 보기 (투명)

---

### 2. Python 검증 스크립트

```python
# validate_model.py
import mujoco
import numpy as np

# 모델 로드
model = mujoco.MjModel.from_xml_path("xlerobot.xml")
data = mujoco.MjData(model)

# 기본 정보 출력
print(f"Model name: {model.name}")
print(f"Number of bodies: {model.nbody}")
print(f"Number of joints: {model.njnt}")
print(f"Number of DOFs: {model.nv}")
print(f"Number of actuators: {model.nu}")

# 관절 범위 확인
for i in range(model.njnt):
    jnt_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
    jnt_type = model.jnt_type[i]
    jnt_range = model.jnt_range[i]
    
    print(f"{jnt_name}: type={jnt_type}, range={jnt_range}")

# 관성 확인 (음수 체크)
for i in range(model.nbody):
    body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
    mass = model.body_mass[i]
    inertia = model.body_inertia[i]
    
    if mass < 0 or (inertia < 0).any():
        print(f"⚠️ Warning: {body_name} has negative mass/inertia!")

print("\n✅ Model validation complete!")
```

**실행**:
```bash
python validate_model.py
```

---

## 고급 기법

### 1. 텐던을 이용한 커플링

```xml
<!-- 손가락 2개를 동시에 움직이기 -->
<tendon>
  <fixed name="gripper_coupling">
    <joint joint="finger1_joint" coef="1"/>
    <joint joint="finger2_joint" coef="-1"/>  <!-- 반대 방향 -->
  </fixed>
</tendon>

<actuator>
  <position name="gripper" tendon="gripper_coupling" kp="50"/>
</actuator>
```

---

### 2. 이퀄리티 제약 (Equality Constraint)

```xml
<!-- 두 관절을 항상 같은 각도로 유지 -->
<equality>
  <joint joint1="left_elbow" joint2="right_elbow" polycoef="0 1 0 0 0"/>
</equality>
```

---

### 3. 컨택트 파라미터 튜닝

```xml
<option>
  <!-- 충돌 감지 세밀도 -->
  <flag override="enable" contact="enable"/>
  
  <!-- Solver 설정 -->
  <option iterations="50" tolerance="1e-10"/>
  
  <!-- Contact 파라미터 -->
  <option impratio="10" cone="pyramidal"/>
</option>

<contact>
  <!-- 특정 geom 간 충돌 무시 -->
  <exclude body1="Upper_Arm" body2="Lower_Arm"/>
</contact>
```

---

## 요약

### 핵심 포인트

1. **URDF**: ROS 표준, XML 기반, link+joint 분리
2. **MJCF**: MuJoCo 전용, body 계층, 효율적
3. **XLeRobot MJCF**: 3 DOF 베이스 + 6 DOF 팔 x 2 = 15 DOF
4. **텐던**: 옴니휠 3개 → (x, y, θ) 변환
5. **수정**: 관절 한계, 질량, 마찰 등 조정 가능

### 주요 파일

- `xlerobot.xml`: 로봇 모델 정의
- `scene.xml`: 환경 (바닥, 조명) + xlerobot.xml
- `assets/*.stl`: 3D 메시 파일

### 다음 단계

- [4장 하드웨어 →](../04_hardware/README.md): 실제 로봇 조립
- [5장 소프트웨어 →](../05_software/README.md): 실제 로봇 제어 코드

---

[← 3.5 ManiSkill](05_maniskill.md) | [3장 목차](README.md) | [다음: 4장 하드웨어 →](../04_hardware/README.md)
