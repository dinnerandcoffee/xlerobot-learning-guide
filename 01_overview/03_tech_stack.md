# 1.3 기술 스택 및 의존성

XLeRobot 프로젝트에서 사용되는 기술 스택과 주요 라이브러리를 이해하면 프로젝트를 더 효과적으로 다룰 수 있습니다.

## 전체 기술 스택 개요

```
┌─────────────────────────────────────────────┐
│           Application Layer                  │
│  VR Control │ Web Control │ Vision Control  │
└─────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────┐
│         Control & Teleoperation              │
│  Keyboard │ Xbox │ Joycon │ VR Headset      │
└─────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────┐
│            Robot Control Layer               │
│     IK/FK │ Motion Planning │ Control       │
└─────────────────────────────────────────────┘
                      ↓
┌─────────────────────────────────────────────┐
│         Simulation & Hardware                │
│  MuJoCo │ Isaac Sim │ Physical Robot        │
└─────────────────────────────────────────────┘
```

---

## 🐍 Python 기반 기술

### 핵심 의존성

#### 1. **로봇 제어 & 시뮬레이션**

```python
# MuJoCo 시뮬레이션
mujoco==3.3.0
mujoco-python-viewer==0.1.4

# 로봇 제어
numpy>=1.20.0
scipy>=1.7.0

# 하드웨어 인터페이스
pyserial>=3.5
```

#### 2. **컴퓨터 비전**

```python
# YOLO 객체 감지
ultralytics>=8.0.0  # YOLOv8

# 이미지 처리
opencv-python>=4.5.0
opencv-contrib-python>=4.5.0
Pillow>=8.0.0

# 카메라 인터페이스
pyrealsense2>=2.50.0  # RealSense 카메라
```

#### 3. **입력 장치 제어**

```python
# 조이스틱 & 게임패드
pygame>=2.0.0  # Xbox 컨트롤러
inputs>=0.5    # 일반 게임패드

# Joycon
hidapi>=0.10.0  # HID 장치 접근
crc8>=0.1.0     # CRC 체크섬
```

#### 4. **웹 & 통신**

```python
# 웹 서버
fastapi>=0.100.0
uvicorn[standard]>=0.23.0
websockets>=11.0

# HTTP 클라이언트
requests>=2.28.0
httpx>=0.24.0

# WebRTC
aiortc>=1.5.0  # VR 통신용
```

#### 5. **데이터 & 유틸리티**

```python
# 설정 파일
pyyaml>=6.0
toml>=0.10.0

# 로깅 & 디버깅
loguru>=0.6.0
rich>=13.0.0

# 데이터 처리
pandas>=1.5.0
h5py>=3.7.0  # 데이터셋 저장
```

---

## 🌐 웹 기술 스택

### 프론트엔드 (`web_control/client/`)

```json
{
  "dependencies": {
    "vue": "^3.3.0",          // 또는 React
    "typescript": "^5.0.0",
    "vite": "^4.3.0",
    "tailwindcss": "^3.3.0",
    "axios": "^1.4.0"
  }
}
```

**주요 기술**:
- **프레임워크**: Vue.js 3 / React 18
- **언어**: TypeScript
- **빌드 도구**: Vite
- **스타일링**: Tailwind CSS
- **통신**: Axios + WebSocket

### 백엔드 (`web_control/server/`)

```python
fastapi==0.100.0
uvicorn[standard]==0.23.0
python-multipart==0.0.6
pydantic==2.0.0
```

---

## 🎮 시뮬레이션 플랫폼

### 1. MuJoCo (가장 추천)

```bash
# requirements.txt
mujoco==3.3.0
mujoco-python-viewer==0.1.4
glfw>=2.5.0
PyOpenGL>=3.1.0
```

**특징**:
- ✅ 빠른 시작
- ✅ 가벼움
- ✅ 정확한 물리 엔진
- ❌ 시각화 제한적

### 2. NVIDIA Isaac Sim

```bash
# 별도 설치 필요
# https://developer.nvidia.com/isaac-sim
```

**특징**:
- ✅ 사실적인 렌더링
- ✅ 고급 센서 시뮬레이션
- ✅ ROS/ROS2 통합
- ❌ 무거움 (GPU 필수)

### 3. ManiSkill

```bash
# requirements.txt
mani-skill2>=0.5.0
gymnasium>=0.28.0
sapien>=2.2.0
```

**특징**:
- ✅ RL 환경 제공
- ✅ 벤치마크 태스크
- ✅ GPU 가속
- ❌ 설정 복잡

---

## 🔧 하드웨어 인터페이스

### 로봇 팔 (SO-100/SO-101)

```python
# Dynamixel 모터 제어
dynamixel-sdk>=3.7.0
```

### 카메라

```python
# RealSense D435i
pyrealsense2>=2.50.0

# USB 카메라
opencv-python>=4.5.0
```

### 모바일 베이스

```python
# 시리얼 통신
pyserial>=3.5
```

---

## 📦 선택적 의존성

### VR 제어 (Quest 3)

```python
# XLeVR/requirements.txt
aiortc>=1.5.0          # WebRTC
aiohttp>=3.8.0
opencv-python>=4.5.0
numpy>=1.20.0
pyyaml>=6.0
```

### 문서 빌드

```python
# docs/requirements.txt
sphinx>=5.0.0
sphinx-rtd-theme>=1.2.0
myst-parser>=1.0.0
```

---

## 🖥️ 시스템 요구사항

### 최소 요구사항 (시뮬레이션)

- **OS**: Ubuntu 20.04+ / Linux
- **CPU**: 4 cores, 2.0 GHz
- **RAM**: 8 GB
- **GPU**: 통합 그래픽
- **저장공간**: 10 GB

### 권장 사양 (실제 로봇)

- **OS**: Ubuntu 22.04
- **CPU**: 8 cores, 3.0 GHz
- **RAM**: 16 GB
- **GPU**: NVIDIA GTX 1650 이상
- **저장공간**: 50 GB

### VR 사용 시

- **GPU**: NVIDIA RTX 3060 이상
- **RAM**: 32 GB
- **네트워크**: WiFi 6 또는 유선

---

## 🔗 주요 의존 프로젝트

### 1. LeRobot (Hugging Face)

```bash
git clone https://github.com/huggingface/lerobot
```

- RL 학습 프레임워크
- 데이터셋 관리
- 정책 학습

### 2. SO-ARM100

```bash
git clone https://github.com/TheRobotStudio/SO-ARM100
```

- 로봇 팔 설계
- Dynamixel 제어
- IK/FK 알고리즘

### 3. LeKiwi

```bash
git clone https://github.com/SIGRobotics-UIUC/LeKiwi
```

- 모바일 베이스 설계
- 옴니휠 제어

---

## 설치 가이드

### 전체 의존성 설치

```bash
# 저장소 클론
git clone https://github.com/Vector-Wangel/XLeRobot.git
cd XLeRobot

# MuJoCo 시뮬레이션
cd simulation/mujoco/
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 소프트웨어 제어
cd ../../software/
pip install -r requirements.txt

# 웹 제어 (선택)
cd ../web_control/server/
pip install -r requirements.txt

cd ../client/
npm install

# VR 제어 (선택)
cd ../../XLeVR/
pip install -r requirements.txt
```

### 개별 설치

```bash
# MuJoCo만
pip install mujoco mujoco-python-viewer glfw

# 비전만
pip install ultralytics opencv-python

# 웹 서버만
pip install fastapi uvicorn websockets
```

---

## 버전 호환성

| 패키지 | 최소 버전 | 권장 버전 | 참고 |
|--------|-----------|-----------|------|
| Python | 3.8 | 3.10 | 3.11도 호환 |
| MuJoCo | 3.0 | 3.3 | 최신 버전 권장 |
| OpenCV | 4.5 | 4.8 | contrib 포함 |
| NumPy | 1.20 | 1.24 | 2.0은 일부 호환 이슈 |
| FastAPI | 0.100 | 0.104 | |

---

## 플랫폼별 특이사항

### Ubuntu/Debian

```bash
# 시스템 패키지 설치
sudo apt update
sudo apt install -y \
    python3-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    libhidapi-dev
```

### macOS

```bash
# Homebrew로 설치
brew install libusb hidapi
```

### Windows (WSL2 권장)

```bash
# WSL2 Ubuntu에서 실행
# USB 장치는 usbipd 사용
```

---

## 요약

### 필수 기술
- **Python 3.10**: 메인 언어
- **MuJoCo**: 시뮬레이션
- **NumPy/OpenCV**: 수치 계산 & 비전
- **FastAPI**: 웹 서버

### 선택 기술
- **Vue.js/React**: 웹 UI
- **Isaac Sim**: 고급 시뮬레이션
- **WebRTC**: VR 통신

### 학습 우선순위
1. Python 기본
2. MuJoCo 시뮬레이션
3. OpenCV 비전
4. FastAPI 웹 개발

---

[← 1.2 프로젝트 구조](02_project_structure.md) | [다음: 1.4 개발 환경 설정 →](04_dev_environment.md)
