# 4.1 소프트웨어 아키텍처

XLeRobot의 소프트웨어는 모듈화된 구조로 설계되어 다양한 제어 방식을 지원합니다.

## 1. 전체 아키텍처 개요

```
XLeRobot/
├── software/
│   ├── examples/          # 제어 예제 스크립트
│   │   ├── 0_so100_keyboard_joint_control.py
│   │   ├── 1_so100_keyboard_ee_control.py
│   │   ├── 2_dual_so100_keyboard_ee_control.py
│   │   ├── 3_so100_yolo_ee_control.py
│   │   ├── 4_xlerobot_teleop_keyboard.py
│   │   ├── 5_xlerobot_teleop_xbox.py
│   │   ├── 6_so100_joycon_ee_control.py
│   │   ├── 7_xlerobot_teleop_joycon.py
│   │   └── 8_xlerobot_teleop_vr.py
│   │
│   ├── src/
│   │   ├── robots/
│   │   │   ├── xlerobot/          # 3륜 옴니휠 버전
│   │   │   └── xlerobot_2wheels/  # 2륜 버전
│   │   └── model/                 # 학습 모델
│   │
│   └── joyconrobotics/    # Joycon 제어 라이브러리
│
├── simulation/            # 시뮬레이션 환경
└── XLeVR/                # VR 제어
```

---

## 2. 소프트웨어 레이어 구조

```
┌─────────────────────────────────────┐
│   사용자 인터페이스 레이어           │
│  (키보드, Xbox, Joycon, VR)         │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│   제어 레이어                        │
│  (조인트 제어, IK/FK, 모바일 베이스) │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│   하드웨어 추상화 레이어             │
│  (서보 모터, 카메라, 센서)           │
└──────────────┬──────────────────────┘
               ↓
┌─────────────────────────────────────┐
│   물리 하드웨어                      │
│  (STS3215 서보, RealSense, 옴니휠)  │
└─────────────────────────────────────┘
```

---

## 3. 주요 모듈 설명

### 3.1 예제 스크립트 (examples/)

| 파일 | 기능 | 제어 방식 |
|------|------|----------|
| `0_so100_keyboard_joint_control.py` | 조인트 공간 제어 | 키보드 |
| `1_so100_keyboard_ee_control.py` | 엔드 이펙터 제어 | 키보드 + IK |
| `2_dual_so100_keyboard_ee_control.py` | 듀얼암 협동 제어 | 키보드 + IK |
| `3_so100_yolo_ee_control.py` | YOLO 객체 추적 | 비전 + IK |
| `4_xlerobot_teleop_keyboard.py` | 전신 제어 (팔+베이스) | 키보드 |
| `5_xlerobot_teleop_xbox.py` | Xbox 컨트롤러 제어 | Xbox 패드 |
| `6_so100_joycon_ee_control.py` | Joycon 팔 제어 | Switch Joycon |
| `7_xlerobot_teleop_joycon.py` | Joycon 전신 제어 | Joycon 양손 |
| `8_xlerobot_teleop_vr.py` | VR 텔레오퍼레이션 | Quest 3 |

### 3.2 로봇 정의 (src/robots/)

**xlerobot/ (3륜 옴니휠 버전)**
```python
# 로봇 구성
- 17개 STS3215 서보 모터
  - 팔: 좌 1-6, 우 7-12
  - 옴니휠: 13-15 (120° 간격 배치)
  - 짐벌: 16-17 (pan/tilt)
```

**xlerobot_2wheels/ (2륜 차동 구동)**
```python
# 간소화된 구성
- 15개 서보 모터
  - 팔: 좌 1-6, 우 7-12
  - 바퀴: 13-14 (차동 구동)
  - 짐벌: 15 (pan만)
```

### 3.3 Joycon 라이브러리 (joyconrobotics/)

```
joyconrobotics/
├── __init__.py
├── joycon.py           # Joycon 연결 및 입력 처리
├── device.py           # 블루투스 장치 관리
├── event.py            # 이벤트 핸들러
├── gyro.py             # 자이로스코프 데이터
└── wrappers.py         # 편의 함수
```

**주요 기능:**
- 블루투스 자동 페어링
- 버튼/스틱 입력 처리
- 자이로/가속도계 읽기
- 진동 피드백

---

## 4. 제어 흐름 예시

### 키보드 조인트 제어 흐름

```python
# 1. 사용자 입력
키보드 입력 ('q', 'w', 'e', 'r', 't', 'y')
    ↓
# 2. 조인트 각도 변경
target_joint_angles[i] += step_size
    ↓
# 3. 캘리브레이션 적용
calibrated = apply_joint_calibration(name, raw_angle)
    ↓
# 4. 서보 모터로 전송
robot.set_joint_position(calibrated)
    ↓
# 5. 피드백 읽기
current_position = robot.get_joint_position()
```

### IK 기반 EE 제어 흐름

```python
# 1. 사용자 입력 (방향키)
dx, dy = get_keyboard_input()
    ↓
# 2. 목표 EE 위치 업데이트
target_x += dx
target_y += dy
    ↓
# 3. 역기구학 계산
joint2, joint3 = inverse_kinematics(target_x, target_y)
    ↓
# 4. 조인트 각도 설정
robot.set_joint_position([pan, joint2, joint3, ...])
    ↓
# 5. 순기구학으로 검증
actual_x, actual_y = forward_kinematics(joint2, joint3)
```

---

## 5. 주요 API 함수

### 5.1 조인트 제어

```python
# 조인트 위치 설정
robot.set_joint_position(joint_angles: list)

# 조인트 위치 읽기
current_angles = robot.get_joint_position()

# 캘리브레이션 적용
calibrated = apply_joint_calibration(joint_name, raw_position)
```

### 5.2 역기구학 (IK)

```python
def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    """
    2링크 로봇 팔의 역기구학 계산
    
    Args:
        x: 목표 x 좌표 (m)
        y: 목표 y 좌표 (m)
        l1: 상완 길이 (m)
        l2: 하완 길이 (m)
    
    Returns:
        joint2: 어깨 리프트 각도 (rad)
        joint3: 엘보 각도 (rad)
    """
    # 코사인 법칙으로 엘보 각도 계산
    r = math.sqrt(x**2 + y**2)
    cos_theta2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    theta2 = math.acos(cos_theta2)
    
    # 어깨 각도 계산
    alpha = math.atan2(y, x)
    beta = math.atan2(l2 * math.sin(theta2), 
                      l1 + l2 * math.cos(theta2))
    theta1 = alpha - beta
    
    return theta1, theta2
```

### 5.3 모바일 베이스 제어

```python
def set_mobile_base_velocity(vx, vy, omega):
    """
    옴니휠 속도 설정
    
    Args:
        vx: 전/후 속도 (m/s)
        vy: 좌/우 속도 (m/s)
        omega: 회전 속도 (rad/s)
    """
    # 옴니휠 기구학
    w1 = -vy + L * omega
    w2 = 0.866 * vx + 0.5 * vy + L * omega
    w3 = -0.866 * vx + 0.5 * vy + L * omega
    
    # 서보 13, 14, 15번에 전송
    robot.set_wheel_velocity([w1, w2, w3])
```

---

## 6. 설정 및 캘리브레이션

### 6.1 조인트 캘리브레이션

각 서보 모터의 오프셋과 스케일을 조정:

```python
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],      # 오프셋 +6°, 스케일 1.0
    ['shoulder_lift', 2.0, 0.97],    # 오프셋 +2°, 스케일 0.97
    ['elbow_flex', 0.0, 1.05],       # 오프셋 0°, 스케일 1.05
    ['wrist_flex', 0.0, 0.94],       # 오프셋 0°, 스케일 0.94
    ['wrist_roll', 0.0, 0.5],        # 오프셋 0°, 스케일 0.5
    ['gripper', 0.0, 1.0],           # 오프셋 0°, 스케일 1.0
]
```

### 6.2 P 제어 파라미터

부드러운 움직임을 위한 비례 제어:

```python
def p_control_step(current, target, kp=0.5, max_step=5.0):
    """
    비례 제어로 부드럽게 목표 위치로 이동
    
    Args:
        current: 현재 각도 (deg)
        target: 목표 각도 (deg)
        kp: 비례 게인
        max_step: 최대 스텝 (deg)
    
    Returns:
        next_position: 다음 목표 각도
    """
    error = target - current
    step = kp * error
    step = max(min(step, max_step), -max_step)
    return current + step
```

---

## 7. 데이터 흐름

### 7.1 센서 → 제어 → 액추에이터

```
[카메라]             [키보드/Xbox/Joycon]
   ↓                        ↓
[YOLO 객체 검출]       [입력 이벤트]
   ↓                        ↓
   └─────→ [제어 알고리즘] ←─┘
               ↓
          [IK/FK 계산]
               ↓
        [조인트 각도 명령]
               ↓
          [서보 모터]
               ↓
         [실제 움직임]
```

### 7.2 제어 루프 주기

```python
# 일반적인 제어 루프
CONTROL_FREQ = 50  # Hz
dt = 1.0 / CONTROL_FREQ

while True:
    start_time = time.time()
    
    # 1. 센서 읽기
    current_state = robot.get_state()
    
    # 2. 제어 계산
    target = compute_control(current_state, user_input)
    
    # 3. 명령 전송
    robot.set_target(target)
    
    # 4. 타이밍 유지
    elapsed = time.time() - start_time
    time.sleep(max(0, dt - elapsed))
```

---

## 8. 확장성 고려사항

### 8.1 새로운 제어 방식 추가

1. `examples/` 에 새 스크립트 생성
2. 입력 처리 함수 구현
3. 기존 IK/FK 함수 재사용
4. 테스트 및 캘리브레이션

### 8.2 새로운 센서 통합

1. 센서 드라이버 설치
2. `src/robots/` 에 센서 클래스 추가
3. 데이터 융합 알고리즘 구현
4. 예제 스크립트 작성

### 8.3 학습 모델 통합

```python
# 학습된 정책 로드
from src.model import LeRobotPolicy

policy = LeRobotPolicy.load("checkpoint.pth")

# 추론
while True:
    observation = get_observation()
    action = policy.predict(observation)
    robot.execute(action)
```

---

## 9. 디버깅 및 로깅

### 9.1 로깅 설정

```python
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 사용 예
logger.info("Robot initialized")
logger.warning("Joint limit exceeded")
logger.error("Servo communication failed")
```

### 9.2 디버그 출력

```python
# 조인트 상태 출력
print(f"Joint angles: {current_angles}")
print(f"Target EE: ({target_x:.3f}, {target_y:.3f})")
print(f"IK solution: ({joint2:.2f}°, {joint3:.2f}°)")
```

---

## 참고 자료

- [XLeRobot 소프트웨어 저장소](https://github.com/Vector-Wangel/XLeRobot/tree/main/software)
- [LeRobot 공식 문서](https://github.com/huggingface/lerobot)
- [STS3215 서보 프로토콜](http://feetechrc.com/STS3215.html)
- [Python Logging 가이드](https://docs.python.org/3/howto/logging.html)
