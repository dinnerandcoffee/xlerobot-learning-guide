# 4.2 키보드 제어 구현

키보드를 이용한 로봇 제어는 가장 간단하면서도 효과적인 텔레오퍼레이션 방법입니다.

## 1. 키보드 제어 개요

XLeRobot은 두 가지 키보드 제어 모드를 지원합니다:

| 모드 | 파일 | 특징 |
|------|------|------|
| **조인트 공간 제어** | `0_so100_keyboard_joint_control.py` | 각 관절을 개별적으로 제어 |
| **작업 공간 제어** | `1_so100_keyboard_ee_control.py` | 엔드 이펙터 위치를 직접 제어 (IK 사용) |

---

## 2. 조인트 공간 제어

### 2.1 키 맵핑

```
조인트 제어:
  q/a - Shoulder Pan  (좌/우 회전)
  w/s - Shoulder Lift (위/아래)
  e/d - Elbow Flex    (팔꿈치 굽히기/펴기)
  r/f - Wrist Flex    (손목 굽히기/펴기)
  t/g - Wrist Roll    (손목 회전)
  y/h - Gripper       (그리퍼 열기/닫기)

기타:
  0 - 제로 포지션으로 이동
  ESC - 종료
```

### 2.2 코드 구조

```python
#!/usr/bin/env python3
"""
조인트 공간 키보드 제어
0_so100_keyboard_joint_control.py
"""

import time
import logging

# 조인트 캘리브레이션 설정
JOINT_CALIBRATION = [
    ['shoulder_pan', 6.0, 1.0],      # 오프셋 +6°, 스케일 1.0
    ['shoulder_lift', 2.0, 0.97],    # 오프셋 +2°, 스케일 0.97
    ['elbow_flex', 0.0, 1.05],       # 오프셋 0°, 스케일 1.05
    ['wrist_flex', 0.0, 0.94],       # 오프셋 0°, 스케일 0.94
    ['wrist_roll', 0.0, 0.5],        # 오프셋 0°, 스케일 0.5
    ['gripper', 0.0, 1.0],           # 오프셋 0°, 스케일 1.0
]

def apply_joint_calibration(joint_name, raw_position):
    """
    캘리브레이션 계수 적용
    
    Returns:
        calibrated_position = (raw - offset) * scale
    """
    for joint_cal in JOINT_CALIBRATION:
        if joint_cal[0] == joint_name:
            offset = joint_cal[1]
            scale = joint_cal[2]
            return (raw_position - offset) * scale
    return raw_position
```

### 2.3 P 제어 구현

부드러운 움직임을 위해 비례 제어(P Control) 사용:

```python
def move_to_zero_position(robot, duration=3.0, kp=0.5):
    """
    P 제어로 부드럽게 제로 포지션으로 이동
    
    Args:
        robot: 로봇 인스턴스
        duration: 이동 시간 (초)
        kp: 비례 게인 (0~1)
    """
    control_freq = 50  # 50Hz
    total_steps = int(duration * control_freq)
    step_time = 1.0 / control_freq
    
    for step in range(total_steps):
        # 현재 위치 읽기
        current_positions = robot.get_joint_positions()
        
        # 목표: 모든 조인트 0°
        zero_positions = {
            'shoulder_pan': 0.0,
            'shoulder_lift': 0.0,
            'elbow_flex': 0.0,
            'wrist_flex': 0.0,
            'wrist_roll': 0.0,
            'gripper': 0.0
        }
        
        # P 제어 계산
        robot_action = {}
        for joint_name, target in zero_positions.items():
            current = current_positions[joint_name]
            error = target - current
            control_value = current + kp * error
            robot_action[joint_name] = control_value
        
        # 명령 전송
        robot.set_joint_positions(robot_action)
        time.sleep(step_time)
```

### 2.4 키보드 입력 처리

```python
import sys
import tty
import termios
import select

def get_key():
    """
    논블로킹 키보드 입력
    
    Returns:
        key: 눌린 키 (없으면 None)
    """
    tty.setcbreak(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        return key
    return None

# 메인 제어 루프
while True:
    key = get_key()
    
    if key == 'q':
        target_angles['shoulder_pan'] += step_size
    elif key == 'a':
        target_angles['shoulder_pan'] -= step_size
    elif key == 'w':
        target_angles['shoulder_lift'] += step_size
    # ... (다른 키들)
    elif key == '\x1b':  # ESC
        break
```

---

## 3. 작업 공간 제어 (IK)

### 3.1 키 맵핑

```
엔드 이펙터 제어:
  ↑/↓ - Y 방향 (전진/후진)
  ←/→ - X 방향 (좌/우)
  w/s - Z 방향 (위/아래)
  
조인트 직접 제어:
  q/a - Shoulder Pan
  r/f - Wrist Flex
  t/g - Wrist Roll
  y/h - Gripper

기타:
  0 - 제로 포지션
  ESC - 종료
```

### 3.2 역기구학 (IK) 함수

```python
import math

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    """
    2링크 로봇 팔의 역기구학 계산
    
    Args:
        x: 목표 X 좌표 (m)
        y: 목표 Y 좌표 (m)
        l1: 상완 길이 (m) - URDF 기준
        l2: 하완 길이 (m) - URDF 기준
    
    Returns:
        joint2: shoulder_lift 각도 (rad)
        joint3: elbow_flex 각도 (rad)
    """
    # URDF 오프셋 고려
    theta1_offset = math.atan2(0.028, 0.11257)  # joint2=0일 때 오프셋
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    
    # 목표 거리 계산
    r = math.sqrt(x**2 + y**2)
    r_max = l1 + l2
    
    # 워크스페이스 제한 확인
    if r > r_max:
        scale = r_max / r
        x *= scale
        y *= scale
        r = r_max
    
    r_min = abs(l1 - l2)
    if r < r_min and r > 0:
        scale = r_min / r
        x *= scale
        y *= scale
        r = r_min
    
    # 코사인 법칙으로 elbow 각도 계산
    cos_theta2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta2 = max(-1, min(1, cos_theta2))  # -1 ~ 1 범위로 제한
    theta2 = math.acos(cos_theta2)
    
    # shoulder 각도 계산
    alpha = math.atan2(y, x)
    beta = math.atan2(l2 * math.sin(theta2), 
                      l1 + l2 * math.cos(theta2))
    theta1 = alpha - beta
    
    # URDF 좌표계로 변환
    joint2 = theta1 - theta1_offset
    joint3 = theta2 - theta2_offset
    
    return joint2, joint3
```

### 3.3 순기구학 (FK) 함수

```python
def forward_kinematics(joint2, joint3, l1=0.1159, l2=0.1350):
    """
    순기구학: 조인트 각도 → EE 위치
    
    Args:
        joint2: shoulder_lift 각도 (rad)
        joint3: elbow_flex 각도 (rad)
    
    Returns:
        x, y: 엔드 이펙터 좌표 (m)
    """
    # URDF 오프셋 적용
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    
    theta1 = joint2 + theta1_offset
    theta2 = joint3 + theta2_offset
    
    # 상완 끝점
    x1 = l1 * math.cos(theta1)
    y1 = l1 * math.sin(theta1)
    
    # 하완 끝점 (엔드 이펙터)
    x = x1 + l2 * math.cos(theta1 + theta2)
    y = y1 + l2 * math.sin(theta1 + theta2)
    
    return x, y
```

### 3.4 EE 제어 루프

```python
# 초기 EE 위치 설정
target_x = 0.15  # 15cm 전방
target_y = 0.10  # 10cm 높이

# 제어 루프
while True:
    key = get_key()
    
    # 화살표 키로 EE 위치 변경
    if key == '\x1b[A':  # ↑
        target_y += ee_step_size
    elif key == '\x1b[B':  # ↓
        target_y -= ee_step_size
    elif key == '\x1b[C':  # →
        target_x += ee_step_size
    elif key == '\x1b[D':  # ←
        target_x -= ee_step_size
    
    # 역기구학 계산
    joint2, joint3 = inverse_kinematics(target_x, target_y)
    
    # 조인트 각도 설정
    target_angles['shoulder_lift'] = math.degrees(joint2)
    target_angles['elbow_flex'] = math.degrees(joint3)
    
    # P 제어로 부드럽게 이동
    current = robot.get_joint_positions()
    for joint_name, target in target_angles.items():
        error = target - current[joint_name]
        current[joint_name] += kp * error
    
    robot.set_joint_positions(current)
    
    # FK로 실제 위치 검증
    actual_x, actual_y = forward_kinematics(
        math.radians(current['shoulder_lift']),
        math.radians(current['elbow_flex'])
    )
    print(f"Target: ({target_x:.3f}, {target_y:.3f}), "
          f"Actual: ({actual_x:.3f}, {actual_y:.3f})")
```

---

## 4. 실행 방법

### 4.1 조인트 제어 실행

```bash
cd ~/XLeRobot/software
python examples/0_so100_keyboard_joint_control.py
```

**실행 화면:**
```
Robot initialized successfully
Using P control to slowly move robot to zero position...
Will move to zero position in 3.0 seconds

Keyboard Joint Control Started
Press keys to control joints:
  q/a - Shoulder Pan    e/d - Elbow Flex     t/g - Wrist Roll
  w/s - Shoulder Lift   r/f - Wrist Flex     y/h - Gripper
  0 - Zero Position     ESC - Exit

Current Angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

### 4.2 EE 제어 실행

```bash
python examples/1_so100_keyboard_ee_control.py
```

**실행 화면:**
```
Robot initialized successfully
Initial EE position: (0.150, 0.100)

Keyboard EE Control Started
Arrow Keys: ↑↓←→ (X-Y plane)
w/s: Z-axis
q/a: Shoulder Pan
r/f: Wrist Flex
t/g: Wrist Roll
y/h: Gripper

Target: (0.150, 0.100), Actual: (0.148, 0.102)
```

---

## 5. 캘리브레이션

### 5.1 왜 캘리브레이션이 필요한가?

- 서보 모터 제조 오차
- 조립 오차
- 기어 백래시
- 온도에 따른 변화

### 5.2 캘리브레이션 절차

**1단계: 제로 포지션 확인**
```bash
python examples/0_so100_keyboard_joint_control.py
# 키보드에서 '0' 입력
# 로봇 팔이 수직 자세가 되는지 확인
```

**2단계: 오프셋 측정**
- 실제 각도와 명령 각도 차이 측정
- 각도계 또는 육안으로 확인

**3단계: 캘리브레이션 값 수정**
```python
# 예: shoulder_lift가 실제로 2° 틀어짐
['shoulder_lift', 2.0, 0.97]  # 오프셋 2°
```

**4단계: 스케일 조정**
```python
# 예: elbow_flex가 명령보다 5% 더 움직임
['elbow_flex', 0.0, 1.05]  # 스케일 1.05 (보정)
```

### 5.3 자동 캘리브레이션 (고급)

```python
def auto_calibrate_joint(robot, joint_name, num_points=10):
    """
    자동 캘리브레이션
    
    여러 각도에서 명령 vs 실제 각도를 측정하여
    최적의 오프셋과 스케일 계산
    """
    commands = []
    readings = []
    
    for angle in np.linspace(-90, 90, num_points):
        robot.set_joint_position(joint_name, angle)
        time.sleep(0.5)
        actual = robot.get_joint_position(joint_name)
        commands.append(angle)
        readings.append(actual)
    
    # 선형 회귀로 오프셋, 스케일 추정
    # reading = scale * command + offset
    slope, intercept = np.polyfit(commands, readings, 1)
    
    offset = intercept
    scale = 1.0 / slope
    
    print(f"{joint_name}: offset={offset:.2f}, scale={scale:.3f}")
    return offset, scale
```

---

## 6. 문제 해결

### 문제 1: 키보드 입력 반응 없음

**원인:** 터미널 모드 설정 오류  
**해결:**
```python
import tty
import termios

# 키보드 raw 모드 활성화
old_settings = termios.tcgetattr(sys.stdin)
tty.setcbreak(sys.stdin.fileno())

# 프로그램 종료 시 복원
termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
```

### 문제 2: IK 계산 실패

**원인:** 목표 위치가 워크스페이스 밖  
**해결:**
```python
# 워크스페이스 경계 체크
r_max = l1 + l2  # 최대 도달 거리
r_min = abs(l1 - l2)  # 최소 거리

if r < r_min or r > r_max:
    print(f"Warning: Target out of workspace (r={r:.3f})")
    # 경계로 스케일링
```

### 문제 3: 로봇 움직임이 떨림

**원인:** P 게인(kp) 너무 높음  
**해결:**
```python
# kp 값 낮추기 (0.3~0.7 권장)
kp = 0.5  # 기본값
kp = 0.3  # 부드러운 움직임
```

---

## 7. 성능 최적화

### 7.1 제어 주기 조정

```python
# 높은 주파수 = 부드러운 움직임 (CPU 부하 증가)
CONTROL_FREQ = 100  # Hz

# 낮은 주파수 = 끊김 (CPU 부하 감소)
CONTROL_FREQ = 30   # Hz

# 권장: 50Hz
CONTROL_FREQ = 50   # Hz
```

### 7.2 스텝 크기 최적화

```python
# 조인트 제어 스텝 (deg)
JOINT_STEP = 2.0   # 빠르지만 거침
JOINT_STEP = 0.5   # 느리지만 정밀

# EE 제어 스텝 (m)
EE_STEP = 0.005    # 5mm (권장)
```

---

## 참고 자료

- [0_so100_keyboard_joint_control.py 전체 코드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/../software/examples/0_so100_keyboard_joint_control.py)
- [1_so100_keyboard_ee_control.py 전체 코드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/../software/examples/1_so100_keyboard_ee_control.py)
- [Python termios 문서](https://docs.python.org/3/library/termios.html)
- [역기구학 상세 설명 → 4.5장](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/05_kinematics.md)
