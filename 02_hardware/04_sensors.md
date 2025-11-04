# 2.4 카메라 및 센서 시스템

XLeRobot은 다양한 카메라와 센서로 환경을 인식하고 자율 작업을 수행합니다.

## 1. 헤드 카메라 시스템

### 옵션 1: Logitech C920 (기본형 $660)

- **해상도**: 1080p @ 30fps
- **시야각**: 78° 대각선
- **마이크**: 듀얼 스테레오
- **연결**: USB 2.0 타입 A
- **가격**: $66 (US), ¥309 (CN)
- **용도**: 기본 RGB 비전, 물체 인식

**장점:**
- ✓ 저렴한 가격
- ✓ 광범위한 호환성
- ✓ 우수한 화질

### 옵션 2: RealSense D415 (Pro형 +$220)

- **RGB**: 1920×1080 @ 30fps
- **Depth**: 1280×720 @ 90fps
- **깊이 범위**: 0.3~3m
- **시야각**: 65° × 40°
- **연결**: USB 3.0 Type-C
- **가격**: $272 (US), ¥1950 (CN)

**장점:**
- ✓ RGB + Depth 동시 제공
- ✓ 3D 물체 인식
- ✓ 거리 측정 (±2mm @1m)

**깊이 센서 원리:**
- 구조광 (Structured Light)
- IR 프로젝터 + IR 카메라
- 실내 환경에 최적화

---

## 2. 손목 카메라 (Hand Camera)

### 32x32 UVC 모듈

- **해상도**: 1080p @ 30fps
- **시야각**: 130° 광각
- **크기**: 32mm × 32mm
- **연결**: USB 2.0 Micro
- **가격**: $12.98/개 (2개 필요)
- **용도**: 정밀 파지 시각 피드백

**마운트:**
- 손목 카메라 마운트 STL 제공
- SO-100 손목에 M3 나사로 고정
- USB 케이블은 팔 내부로 라우팅

---

## 3. 헤드 짐벌 시스템

### 2축 팬/틸트 짐벌

- **Pan (좌우)**: STS3215 서보 (ID 16)
- **Tilt (상하)**: STS3215 서보 (ID 17)
- **가동 범위**:
  - Pan: -90° ~ +90°
  - Tilt: -45° ~ +45°

### 짐벌 마운트

**D435/D455용:**
- `Gimbal_mesh_all_d435.stl`
- `Gimbal_mesh_all_d455.stl`

**웹캠용:**
- 범용 카메라 마운트 STL
- 조절 가능한 각도 브래킷

**조립:**
1. 하부 Pan 서보를 베이스에 고정
2. Pan 서보에 Tilt 마운트 연결
3. Tilt 서보에 카메라 마운트 부착
4. 카메라를 마운트에 고정

---

## 4. 카메라 구성별 비교

| 구성 | 카메라 | 추가 비용 | 용도 |
|------|--------|----------|------|
| **기본형** | C920 × 1 | $0 | RGB 비전 |
| **스테레오** | C920 × 2 | +$30 | 입체시, 깊이 추정 |
| **Pro** | D415 × 1 | +$220 | 정확한 깊이, 3D |
| **Full** | D415 + Hand × 2 | +$246 | 모든 기능 |

---

## 5. 카메라 설정

### Python 코드 (OpenCV)

```python
from lerobot.cameras.opencv import OpenCVCameraConfig

camera = OpenCVCameraConfig(
    index_or_path=0,  # /dev/video0
    fps=30,
    width=640,
    height=480,
    color_mode=ColorMode.RGB
)
```

### RealSense 설정

```python
from lerobot.cameras.realsense import RealSenseCameraConfig

camera = RealSenseCameraConfig(
    serial_number="123456789",
    fps=30,
    width=640,
    height=480,
    enable_depth=True
)
```

---

## 6. 짐벌 제어

### 키보드 제어

```python
# Pan 좌측
action["head_pan.pos"] = -30.0  # -30도

# Tilt 상향
action["head_tilt.pos"] = 20.0  # +20도
```

### 자동 추적

```python
# YOLO 물체 중심으로 짐벌 이동
obj_x, obj_y = detect_object()
pan_error = (obj_x - 320) * 0.1  # 비례 제어
tilt_error = (240 - obj_y) * 0.1

action["head_pan.pos"] += pan_error
action["head_tilt.pos"] += tilt_error
```

---

## 7. 센서 융합

### RGB-D 융합

```python
import pyrealsense2 as rs

# RGB 이미지
color_frame = frames.get_color_frame()
rgb_image = np.asanyarray(color_frame.get_data())

# Depth 이미지
depth_frame = frames.get_depth_frame()
depth_image = np.asanyarray(depth_frame.get_data())

# 특정 픽셀의 거리
distance = depth_frame.get_distance(x, y)
print(f"Distance: {distance:.2f}m")
```

---

## 8. 문제 해결

### 카메라가 인식되지 않음

```bash
# 연결된 카메라 확인
ls /dev/video*

# 권한 설정
sudo usermod -a -G video $USER

# 재부팅
sudo reboot
```

### RealSense 드라이버 설치

```bash
# librealsense2 설치
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

# 확인
realsense-viewer
```

---

## 참고 자료

- [RealSense SDK](https://github.com/IntelRealSense/librealsense)
- [OpenCV Python](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [XLeRobot 카메라 설정](../../software/src/robots/xlerobot/config_xlerobot.py)
