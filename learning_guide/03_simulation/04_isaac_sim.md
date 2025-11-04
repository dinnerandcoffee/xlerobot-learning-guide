# 3.4 Isaac Sim 환경 설정

NVIDIA Isaac Sim은 포토리얼리스틱 렌더링과 정밀한 센서 시뮬레이션을 제공하는 고급 로봇 시뮬레이터입니다.

## 목차
- [Isaac Sim 소개](#isaac-sim-소개)
- [시스템 요구사항](#시스템-요구사항)
- [Leisaac 환경 설정](#leisaac-환경-설정)
- [XLeRobot USD 로드](#xlerobot-usd-로드)
- [센서 설정](#센서-설정)
- [ROS2 통합](#ros2-통합)

---

## Isaac Sim 소개

### Isaac Sim이란?

NVIDIA Isaac Sim은 **Omniverse 플랫폼** 기반의 로봇 시뮬레이터로, RTX GPU를 활용한 사실적인 렌더링과 물리 시뮬레이션을 제공합니다.

```
┌─────────────────────────────────────┐
│       Isaac Sim 아키텍처             │
├─────────────────────────────────────┤
│                                     │
│  ┌─────────────────────────────┐   │
│  │   Omniverse Kit (GUI)       │   │
│  └─────────────────────────────┘   │
│              ↓                      │
│  ┌─────────────────────────────┐   │
│  │   PhysX 5 (물리 엔진)        │   │
│  └─────────────────────────────┘   │
│              ↓                      │
│  ┌─────────────────────────────┐   │
│  │   RTX Renderer (레이트레이싱) │   │
│  └─────────────────────────────┘   │
│              ↓                      │
│  ┌─────────────────────────────┐   │
│  │   ROS/ROS2 Bridge           │   │
│  └─────────────────────────────┘   │
│                                     │
└─────────────────────────────────────┘
```

### MuJoCo vs Isaac Sim 비교

| 기능 | MuJoCo | Isaac Sim |
|------|--------|-----------|
| **렌더링 품질** | 기본 | 포토리얼리스틱 (RTX) |
| **카메라 시뮬레이션** | 간단 | 정밀 (노이즈, 왜곡 등) |
| **LIDAR/Depth** | 제한적 | 완벽 지원 |
| **GPU 요구사항** | 불필요 | RTX 2060 이상 권장 |
| **설치 크기** | ~100MB | ~20GB |
| **학습 곡선** | 쉬움 | 가파름 |
| **ROS 통합** | 수동 | 네이티브 지원 |
| **사용 사례** | 제어 알고리즘 개발 | 비전 AI, 복잡한 환경 |

### XLeRobot에서의 활용

XLeRobot은 **Lightwheel AI**의 **Leisaac** 환경을 통해 Isaac Sim을 활용합니다:

- **USD 파일**: XLeRobot의 3D 모델 (Universal Scene Description)
- **Leisaac**: Isaac Sim용 사전 구성된 로봇 환경
- **강화학습**: GPU 병렬 시뮬레이션으로 빠른 학습

---

## 시스템 요구사항

### 최소 사양

| 구성 요소 | 최소 | 권장 |
|-----------|------|------|
| **OS** | Ubuntu 20.04 | Ubuntu 22.04 |
| **GPU** | NVIDIA RTX 2060 (6GB) | RTX 3080 (10GB+) |
| **VRAM** | 6GB | 10GB+ |
| **RAM** | 16GB | 32GB |
| **디스크** | 50GB SSD | 100GB NVMe |
| **CPU** | Intel i5 (4코어) | Intel i7 (8코어+) |
| **드라이버** | 525.60+ | 최신 |

### GPU 확인

```bash
# NVIDIA GPU 확인
nvidia-smi

# 예상 출력:
# +-----------------------------------------------------------------------------+
# | NVIDIA-SMI 535.129.03   Driver Version: 535.129.03   CUDA Version: 12.2   |
# |-------------------------------+----------------------+----------------------+
# | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
# | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
# |===============================+======================+======================|
# |   0  NVIDIA GeForce ...  Off  | 00000000:01:00.0  On |                  N/A |
# | 30%   45C    P8    20W / 250W |   1024MiB / 10240MiB |      5%      Default |
# +-------------------------------+----------------------+----------------------+
```

**요구 사항 확인**:
- ✅ **Driver Version**: 525 이상
- ✅ **GPU Memory**: 6GB 이상
- ✅ **CUDA Version**: 11.8 이상

### 호환되지 않는 GPU

❌ **AMD GPU**: Isaac Sim은 NVIDIA GPU 전용
❌ **Intel UHD/Iris**: 통합 그래픽 카드 불가
❌ **GTX 1000 시리즈 이하**: RTX 기능 없음 (느린 렌더링)

---

## Leisaac 환경 설정

XLeRobot은 **Lightwheel AI**의 **Leisaac** 프로젝트를 통해 Isaac Sim을 활용합니다.

### Leisaac이란?

```
Leisaac = Isaac Sim + XLeRobot 환경 + RL 도구
```

**주요 기능**:
- 사전 구성된 XLeRobot USD 파일
- GPU 병렬 시뮬레이션 (수백 개 환경)
- 강화학습 통합 (Stable Baselines3, RLlib 등)
- ROS2 통신 브리지

### 1단계: Omniverse 설치

#### 1.1 Omniverse Launcher 다운로드

```bash
# 공식 사이트에서 다운로드
# https://www.nvidia.com/en-us/omniverse/download/

# 또는 wget (Ubuntu)
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# 실행 권한 부여
chmod +x omniverse-launcher-linux.AppImage

# 런처 실행
./omniverse-launcher-linux.AppImage
```

#### 1.2 Isaac Sim 설치

1. **Launcher 실행** → **Exchange** 탭
2. **Isaac Sim** 검색
3. **Isaac Sim 2024.1** (또는 최신 버전) 선택
4. **Install** 클릭 (약 20GB, 30분~1시간 소요)

**설치 경로**:
```
~/.local/share/ov/pkg/isaac-sim-2024.1.0/
```

#### 1.3 Isaac Sim 실행 테스트

```bash
# Isaac Sim 실행 (Launcher에서 또는 터미널)
~/.local/share/ov/pkg/isaac-sim-2024.1.0/isaac-sim.sh

# GUI가 열리면 성공!
```

---

### 2단계: Leisaac 설치

```bash
# 홈 디렉토리로 이동
cd ~

# Leisaac 저장소 클론
git clone https://github.com/LightwheelAI/leisaac.git
cd leisaac

# Python 가상 환경 생성
python3 -m venv .venv
source .venv/bin/activate

# 의존성 설치
pip install -r requirements.txt

# Isaac Sim Python 바인딩 설치
pip install isaacsim
```

**예상 시간**: 10-15분

---

### 3단계: XLeRobot USD 파일 확인

```bash
# XLeRobot USD 파일 위치 (Leisaac 내)
ls ~/leisaac/assets/robots/xlerobot/

# 예상 출력:
# xlerobot.usd
# xlerobot_single_arm.usd
# xlerobot_dual_arm.usd
# materials/
# textures/
```

**USD 파일 종류**:
- `xlerobot.usd`: 전체 로봇 (베이스 + 양팔)
- `xlerobot_single_arm.usd`: 단일 팔 버전
- `xlerobot_dual_arm.usd`: 듀얼 팔만 (베이스 제외)

---

## XLeRobot USD 로드

### 방법 1: Isaac Sim GUI에서 로드

#### 1단계: Isaac Sim 실행

```bash
~/.local/share/ov/pkg/isaac-sim-2024.1.0/isaac-sim.sh
```

#### 2단계: USD 파일 열기

1. **File** → **Open**
2. 경로: `~/leisaac/assets/robots/xlerobot/xlerobot.usd`
3. **Open** 클릭

#### 3단계: 뷰포트 확인

```
┌─────────────────────────────────────┐
│  Isaac Sim - Viewport               │
├─────────────────────────────────────┤
│                                     │
│         [XLeRobot 3D 모델]          │
│                                     │
│  - 사실적인 재질 (금속, 플라스틱)    │
│  - 그림자, 반사 (RTX 레이트레이싱)   │
│  - 고해상도 텍스처                   │
│                                     │
└─────────────────────────────────────┘
```

#### 4단계: 카메라 조작

| 동작 | 조작 |
|------|------|
| **회전** | Alt + 좌클릭 드래그 |
| **팬** | Alt + 중클릭 드래그 |
| **줌** | Alt + 우클릭 드래그 또는 스크롤 |
| **초점** | F 키 (선택된 객체에 포커스) |

---

### 방법 2: Python 스크립트로 로드

```python
# load_xlerobot.py
from omni.isaac.kit import SimulationApp

# Isaac Sim 초기화 (헤드리스 또는 GUI)
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
import carb

# World 생성
world = World(stage_units_in_meters=1.0)

# XLeRobot USD 로드
xlerobot_usd_path = "/home/사용자명/leisaac/assets/robots/xlerobot/xlerobot.usd"
add_reference_to_stage(usd_path=xlerobot_usd_path, prim_path="/World/XLeRobot")

# 시뮬레이션 리셋
world.reset()

print("XLeRobot loaded successfully!")

# GUI 유지 (Ctrl+C로 종료)
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

**실행**:
```bash
python load_xlerobot.py
```

---

## 센서 설정

Isaac Sim의 강력한 기능은 **정밀한 센서 시뮬레이션**입니다.

### 1. RGB 카메라 추가

```python
from omni.isaac.sensor import Camera

# 카메라 생성 (로봇 헤드에 장착)
camera = Camera(
    prim_path="/World/XLeRobot/head_camera",
    position=np.array([0.0, 0.0, 1.2]),  # 베이스에서 1.2m 위
    frequency=30,  # 30 FPS
    resolution=(640, 480),
)

# 카메라 초기화
camera.initialize()

# 이미지 읽기
rgb_image = camera.get_rgba()
print(f"Image shape: {rgb_image.shape}")  # (480, 640, 4)
```

**카메라 파라미터**:
- `focal_length`: 초점 거리 (mm)
- `focus_distance`: 포커스 거리 (m)
- `f_stop`: 조리개 (F값)
- `horizontal_aperture`: 센서 폭 (mm)

---

### 2. Depth 카메라

```python
from omni.isaac.sensor import Camera

depth_camera = Camera(
    prim_path="/World/XLeRobot/depth_camera",
    position=np.array([0.0, 0.0, 1.2]),
    frequency=30,
    resolution=(640, 480),
)

depth_camera.initialize()

# Depth 이미지 읽기
depth_image = depth_camera.get_depth()
print(f"Depth range: {depth_image.min():.2f}m ~ {depth_image.max():.2f}m")
```

**활용**:
- 거리 측정
- 3D 재구성
- 장애물 회피

---

### 3. LIDAR 센서

```python
from omni.isaac.range_sensor import LidarRtx

# Rotating LIDAR (예: Velodyne VLP-16)
lidar = LidarRtx(
    prim_path="/World/XLeRobot/lidar",
    position=np.array([0.0, 0.0, 1.0]),
    config="Example_Rotary",  # 사전 정의된 설정
)

lidar.initialize()

# 포인트 클라우드 읽기
point_cloud = lidar.get_point_cloud()
print(f"Point count: {len(point_cloud)}")  # 예: 28000 points
```

---

### 4. IMU (Inertial Measurement Unit)

```python
from omni.isaac.sensor import IMUSensor

imu = IMUSensor(
    prim_path="/World/XLeRobot/base/imu",
    translation=np.array([0.0, 0.0, 0.1]),  # 베이스 중심
    frequency=100,  # 100 Hz
)

imu.initialize()

# IMU 데이터 읽기
imu_data = imu.get_current_frame()
print(f"Linear acceleration: {imu_data['lin_acc']}")
print(f"Angular velocity: {imu_data['ang_vel']}")
print(f"Orientation: {imu_data['orientation']}")
```

**IMU 데이터**:
- `lin_acc`: 선형 가속도 (m/s²)
- `ang_vel`: 각속도 (rad/s)
- `orientation`: 자세 (쿼터니언)

---

## ROS2 통합

Isaac Sim은 **ROS2 Bridge**를 통해 ROS2와 통신할 수 있습니다.

### ROS2 설치 (Ubuntu 22.04)

```bash
# ROS2 Humble 설치
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository universe

sudo apt install -y curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
  signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop

# 환경 변수 설정
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

### Isaac Sim에서 ROS2 Bridge 활성화

```python
# ros2_bridge_example.py
from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ROS2 Bridge 확장 활성화
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

from omni.isaac.core import World
import omni.graph.core as og

# World 생성
world = World()

# ROS2 Bridge 그래프 생성
keys = og.Controller.Keys
(graph, nodes, _, _) = og.Controller.edit(
    {"graph_path": "/World/ROS2Graph", "evaluator_name": "execution"},
    {
        keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("PublishJointState", "omni.isaac.ros2_bridge.ROS2PublishJointState"),
            ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        ],
        keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        ],
    },
)

# 관절 상태 퍼블리셔 설정
og.Controller.attribute("inputs:targetPrim", nodes[1]).set("/World/XLeRobot")
og.Controller.attribute("inputs:topicName", nodes[1]).set("/joint_states")

print("ROS2 Bridge activated!")
print("Subscribe to /joint_states topic:")
print("  ros2 topic echo /joint_states")

# 시뮬레이션 실행
world.reset()
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

**실행**:
```bash
# 터미널 1: Isaac Sim 실행
python ros2_bridge_example.py

# 터미널 2: ROS2 토픽 확인
ros2 topic list
# /joint_states
# /clock
# /rosout

# 터미널 3: 관절 상태 구독
ros2 topic echo /joint_states
```

---

## Leisaac 예제 실행

Leisaac 프로젝트는 사전 구성된 XLeRobot 환경을 제공합니다.

### 1. 단순 시뮬레이션

```bash
cd ~/leisaac

# 가상 환경 활성화
source .venv/bin/activate

# XLeRobot 시뮬레이션 실행
python examples/xlerobot_basic.py
```

**예상 동작**:
- Isaac Sim 창 열림
- XLeRobot 로드
- 중력 시뮬레이션 시작

---

### 2. 강화학습 환경

```bash
# RL 환경 예제
python examples/xlerobot_rl_env.py
```

**주요 기능**:
- Gymnasium 호환 환경
- GPU 병렬 시뮬레이션 (64개 환경)
- 보상 함수 커스터마이징

---

### 3. 키보드 텔레오퍼레이션

```bash
# 키보드 제어 예제
python examples/xlerobot_teleop_keyboard.py
```

**제어 키**:
- `W/A/S/D`: 베이스 이동
- `Q/E`: 회전
- `I/J/K/L`: 왼팔 제어
- `Arrow Keys`: 오른팔 제어

---

## 성능 최적화

### 1. 렌더링 품질 조정

```python
# 고품질 (느림, 데모용)
simulation_app = SimulationApp({
    "renderer": "RayTracedLighting",
    "samples_per_pixel_per_frame": 64,
})

# 저품질 (빠름, 학습용)
simulation_app = SimulationApp({
    "renderer": "RayTracedLighting",
    "samples_per_pixel_per_frame": 1,  # 1 샘플만
    "headless": True,  # GUI 없음
})
```

---

### 2. 물리 시뮬레이션 속도

```python
from omni.isaac.core import World

# 빠른 시뮬레이션 (정확도 희생)
world = World(physics_dt=1/60.0, rendering_dt=1/60.0)

# 정밀 시뮬레이션 (느림)
world = World(physics_dt=1/240.0, rendering_dt=1/60.0)
```

---

### 3. GPU 병렬화

```python
# 여러 환경 병렬 실행 (RL 학습)
from omni.isaac.core.utils.viewports import set_camera_view

num_envs = 64  # GPU VRAM에 따라 조정

for i in range(num_envs):
    add_reference_to_stage(
        usd_path=xlerobot_usd_path,
        prim_path=f"/World/env_{i}/XLeRobot"
    )
```

**VRAM 사용량**:
- 1 환경: ~2GB
- 64 환경: ~12GB (RTX 3080 10GB는 부족, RTX 3090 24GB 권장)

---

## 문제 해결

### 문제 1: Isaac Sim이 실행되지 않음

**원인**: NVIDIA 드라이버 오래됨

**해결**:
```bash
# 드라이버 버전 확인
nvidia-smi

# 드라이버 업데이트
sudo apt update
sudo apt install -y nvidia-driver-535  # 또는 최신 버전

# 재부팅
sudo reboot
```

---

### 문제 2: USD 파일 로드 오류

**원인**: 경로가 잘못되었거나 파일이 없음

**해결**:
```bash
# USD 파일 존재 확인
ls ~/leisaac/assets/robots/xlerobot/xlerobot.usd

# 없으면 Leisaac 재클론
cd ~
rm -rf leisaac
git clone https://github.com/LightwheelAI/leisaac.git
```

---

### 문제 3: ROS2 Bridge가 작동하지 않음

**원인**: ROS2 환경 변수 미설정

**해결**:
```bash
# ROS2 환경 변수 확인
echo $ROS_DOMAIN_ID  # 비어있으면 설정 필요

# 환경 변수 설정
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash

# Isaac Sim 재실행
```

---

### 문제 4: 느린 렌더링

**원인**: RTX 기능이 켜져 있음 (오래된 GPU)

**해결**:
```python
# 헤드리스 모드 사용 (GUI 없음)
simulation_app = SimulationApp({"headless": True})

# 또는 렌더링 품질 낮춤
simulation_app = SimulationApp({
    "renderer": "RayTracedLighting",
    "samples_per_pixel_per_frame": 1,
    "anti_aliasing": 0,
})
```

---

## 요약

### 핵심 포인트

1. **Isaac Sim**: NVIDIA RTX GPU 필수, 포토리얼리스틱 렌더링
2. **Leisaac**: XLeRobot용 사전 구성 환경 (Lightwheel AI 제공)
3. **USD 파일**: Universal Scene Description, XLeRobot 3D 모델
4. **센서**: RGB, Depth, LIDAR, IMU 정밀 시뮬레이션
5. **ROS2**: 네이티브 브리지로 실제 로봇 코드 재사용

### MuJoCo와의 차이

| 항목 | MuJoCo | Isaac Sim |
|------|--------|-----------|
| **학습 곡선** | 쉬움 | 어려움 |
| **설치 시간** | 5분 | 1-2시간 |
| **GPU** | 불필요 | RTX 필수 |
| **용도** | 제어 개발 | 비전 AI, 복잡한 환경 |

### 다음 단계

- [3.5 ManiSkill 환경 →](05_maniskill.md): 강화학습 전용 환경
- [3.6 URDF/MJCF 모델 →](06_robot_models.md): 로봇 모델 파일 이해

---

[← 3.3 키보드 제어](03_mujoco_control.md) | [3장 목차](README.md) | [다음: 3.5 ManiSkill →](05_maniskill.md)
