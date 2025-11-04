# 4.7 모바일 베이스 제어

XLeRobot의 3륜 옴니휠 이동 베이스 제어 방법을 설명합니다.

## 1. 옴니휠 구조

### 1.1 3륜 옴니휠 배치

```
      전방 (Front)
         ↑
    [Wheel 1]
       / \
      /   \
     /  R  \
    /       \
[W2]-------[W3]
 왼쪽       오른쪽

R: 중심에서 바퀴까지 거리 (0.10m)
각도: 120° 간격
```

### 1.2 하드웨어 사양

| 항목 | 값 |
|------|-----|
| 휠 개수 | 3개 (120° 배치) |
| 모터 | ST3215 × 3 (ID: 8, 9, 10) |
| 휠 반지름 (r) | 0.030 m (3cm) |
| 베이스 반지름 (R) | 0.100 m (10cm) |
| 최대 속도 | 0.5 m/s |
| 회전 속도 | 1.0 rad/s |

---

## 2. 운동학 (Kinematics)

### 2.1 순기구학 (FK)

**휠 속도 → 로봇 속도**

```
주어진: ω1, ω2, ω3 (각 휠의 각속도, rad/s)
구하기: vx, vy, ωz (로봇 속도)

[vx ]   [  -sin(30°)   -sin(150°)   -sin(270°) ]   [ω1]
[vy ] = [   cos(30°)    cos(150°)    cos(270°) ] × [ω2] × r
[ωz ]   [     1/R          1/R           1/R     ]   [ω3]

여기서:
  r = 휠 반지름 (0.03m)
  R = 베이스 반지름 (0.10m)
```

**Python 구현:**

```python
import numpy as np

def forward_kinematics_base(wheel1, wheel2, wheel3, r=0.03, R=0.10):
    """
    휠 각속도 → 로봇 속도
    
    Args:
        wheel1, wheel2, wheel3: 휠 각속도 (rad/s)
        r: 휠 반지름 (m)
        R: 베이스 반지름 (m)
    
    Returns:
        vx, vy, wz: 로봇 속도 (m/s, m/s, rad/s)
    """
    # 각도 (rad)
    theta1 = np.radians(90)   # 90°
    theta2 = np.radians(210)  # 210°
    theta3 = np.radians(330)  # 330°
    
    # FK 행렬
    J = np.array([
        [-np.sin(theta1), -np.sin(theta2), -np.sin(theta3)],
        [ np.cos(theta1),  np.cos(theta2),  np.cos(theta3)],
        [      1/R,             1/R,              1/R      ]
    ])
    
    # 휠 속도 벡터
    wheel_speeds = np.array([wheel1, wheel2, wheel3]) * r
    
    # 로봇 속도 계산
    robot_vel = J @ wheel_speeds
    
    return robot_vel[0], robot_vel[1], robot_vel[2]
```

### 2.2 역기구학 (IK)

**로봇 속도 → 휠 속도**

```
주어진: vx, vy, ωz
구하기: ω1, ω2, ω3

[ω1]       [  -sin(30°)    cos(30°)    R ]   [vx]
[ω2] = 1/r [  -sin(150°)   cos(150°)   R ] × [vy]
[ω3]       [  -sin(270°)   cos(270°)   R ]   [ωz]
```

**Python 구현:**

```python
def inverse_kinematics_base(vx, vy, wz, r=0.03, R=0.10):
    """
    로봇 속도 → 휠 각속도
    
    Args:
        vx, vy: 로봇 선속도 (m/s)
        wz: 로봇 각속도 (rad/s)
    
    Returns:
        wheel1, wheel2, wheel3: 휠 각속도 (rad/s)
    """
    # 각도
    theta1 = np.radians(90)
    theta2 = np.radians(210)
    theta3 = np.radians(330)
    
    # IK 행렬
    J_inv = np.array([
        [-np.sin(theta1),  np.cos(theta1), R],
        [-np.sin(theta2),  np.cos(theta2), R],
        [-np.sin(theta3),  np.cos(theta3), R]
    ])
    
    # 로봇 속도 벡터
    robot_vel = np.array([vx, vy, wz])
    
    # 휠 속도 계산
    wheel_speeds = (J_inv @ robot_vel) / r
    
    return wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]
```

---

## 3. 키보드 제어

### 3.1 예제 코드

**파일:** `software/examples/4_xlerobot_teleop_keyboard.py`

```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from robots.xlerobot import XLeRobot
import termios
import tty
import time

# 초기화
robot = XLeRobot()

# 속도 설정
LINEAR_SPEED = 0.2   # m/s
ANGULAR_SPEED = 0.5  # rad/s

# 키 매핑
KEYMAP = {
    'w': (LINEAR_SPEED, 0, 0),          # 전진
    's': (-LINEAR_SPEED, 0, 0),         # 후진
    'a': (0, LINEAR_SPEED, 0),          # 왼쪽
    'd': (0, -LINEAR_SPEED, 0),         # 오른쪽
    'q': (0, 0, ANGULAR_SPEED),         # 반시계 회전
    'e': (0, 0, -ANGULAR_SPEED),        # 시계 회전
}
```

### 3.2 제어 루프

```python
def get_key():
    """키보드 입력 (논블로킹)"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    robot = XLeRobot()
    
    print("=== Mobile Base Control ===")
    print("W/S: Forward/Backward")
    print("A/D: Left/Right")
    print("Q/E: Rotate CCW/CW")
    print("X: Stop, ESC: Exit\n")
    
    try:
        while True:
            key = get_key()
            
            if key in KEYMAP:
                vx, vy, wz = KEYMAP[key]
                robot.set_mobile_base_velocity(vx, vy, wz)
            
            elif key == 'x':  # 정지
                robot.set_mobile_base_velocity(0, 0, 0)
            
            elif key == '\x1b':  # ESC
                break
            
            time.sleep(0.02)  # 50Hz
    
    finally:
        robot.set_mobile_base_velocity(0, 0, 0)
        robot.cleanup()

if __name__ == "__main__":
    main()
```

---

## 4. XLeRobot 클래스

### 4.1 초기화

```python
class XLeRobot:
    """전신 로봇 (양팔 + 모바일 베이스)"""
    
    def __init__(self, port="/dev/ttyUSB0"):
        # 양팔 초기화
        self.left_arm = SO100Arm(
            port=port, 
            motor_ids=[1,2,3,4,5,6,7]
        )
        self.right_arm = SO100Arm(
            port=port, 
            motor_ids=[11,12,13,14,15,16,17]
        )
        
        # 모바일 베이스
        self.base = MobileBase(
            port=port,
            motor_ids=[8, 9, 10]  # 3개 휠
        )
        
        # 베이스 파라미터
        self.wheel_radius = 0.03   # m
        self.base_radius = 0.10    # m
```

### 4.2 속도 제어 함수

```python
def set_mobile_base_velocity(self, vx, vy, wz):
    """
    모바일 베이스 속도 설정
    
    Args:
        vx: X축 선속도 (m/s)
        vy: Y축 선속도 (m/s)
        wz: 회전 각속도 (rad/s)
    """
    # IK로 휠 속도 계산
    w1, w2, w3 = inverse_kinematics_base(
        vx, vy, wz,
        r=self.wheel_radius,
        R=self.base_radius
    )
    
    # 휠 각속도 제한
    max_wheel_speed = 10.0  # rad/s
    w1 = np.clip(w1, -max_wheel_speed, max_wheel_speed)
    w2 = np.clip(w2, -max_wheel_speed, max_wheel_speed)
    w3 = np.clip(w3, -max_wheel_speed, max_wheel_speed)
    
    # 모터 명령 전송
    self.base.set_wheel_speeds(w1, w2, w3)
```

---

## 5. 움직임 패턴

### 5.1 기본 이동

```python
# 1. 전진
robot.set_mobile_base_velocity(0.2, 0, 0)  # vx=0.2m/s

# 2. 후진
robot.set_mobile_base_velocity(-0.2, 0, 0)

# 3. 왼쪽 스트레이프
robot.set_mobile_base_velocity(0, 0.2, 0)  # vy=0.2m/s

# 4. 오른쪽 스트레이프
robot.set_mobile_base_velocity(0, -0.2, 0)

# 5. 제자리 회전 (CCW)
robot.set_mobile_base_velocity(0, 0, 0.5)  # wz=0.5rad/s

# 6. 제자리 회전 (CW)
robot.set_mobile_base_velocity(0, 0, -0.5)
```

### 5.2 복합 이동

```python
# 대각선 이동 (전진 + 왼쪽)
robot.set_mobile_base_velocity(0.2, 0.2, 0)

# 회전하며 전진
robot.set_mobile_base_velocity(0.2, 0, 0.3)

# 원 그리기
def move_circle(radius=0.5, speed=0.2):
    """원 궤적 이동"""
    wz = speed / radius  # 각속도
    robot.set_mobile_base_velocity(speed, 0, wz)
```

---

## 6. 전신 통합 제어

### 6.1 베이스 + 양팔 동시 제어

**파일:** `software/examples/4_xlerobot_teleop_keyboard.py`

```python
# 키 매핑 (베이스 + 양팔)
KEYMAP = {
    # 베이스
    'w': 'base_forward',
    's': 'base_backward',
    'a': 'base_left',
    'd': 'base_right',
    
    # 왼팔
    'i': 'left_arm_up',
    'k': 'left_arm_down',
    
    # 오른팔
    'o': 'right_arm_up',
    'l': 'right_arm_down',
}

def main():
    robot = XLeRobot()
    
    while True:
        key = get_key()
        
        # 베이스 제어
        if key == 'w':
            robot.set_mobile_base_velocity(0.2, 0, 0)
        elif key == 's':
            robot.set_mobile_base_velocity(-0.2, 0, 0)
        
        # 왼팔 제어
        elif key == 'i':
            robot.left_arm.move_ee_relative(0, 0.01)
        elif key == 'k':
            robot.left_arm.move_ee_relative(0, -0.01)
        
        # 오른팔 제어
        elif key == 'o':
            robot.right_arm.move_ee_relative(0, 0.01)
        elif key == 'l':
            robot.right_arm.move_ee_relative(0, -0.01)
```

### 6.2 협동 작업 예시

```python
def pick_and_place_mobile():
    """베이스 이동 + 물체 집기"""
    robot = XLeRobot()
    
    # 1. 물체 위치로 이동
    robot.set_mobile_base_velocity(0.2, 0, 0)
    time.sleep(2.0)  # 2초 전진
    robot.set_mobile_base_velocity(0, 0, 0)
    
    # 2. 왼팔로 물체 집기
    robot.left_arm.set_ee_position([0.15, 0.10])
    time.sleep(1.0)
    robot.left_arm.close_gripper()
    
    # 3. 목표 위치로 회전
    robot.set_mobile_base_velocity(0, 0, 0.5)
    time.sleep(3.14)  # 90° 회전 (π/2 rad)
    robot.set_mobile_base_velocity(0, 0, 0)
    
    # 4. 물체 놓기
    robot.left_arm.open_gripper()
```

---

## 7. 옴니휠 장점

### 7.1 전방향 이동 (Holonomic)

```
일반 차동 구동 (Differential Drive):
  - 전진/후진 + 회전만 가능
  - 옆으로 이동 불가 → 방향 전환 필요

옴니휠 (3-Wheel Omni):
  - 전/후/좌/우/회전 모두 독립적
  - 즉시 방향 전환 가능
  - 좁은 공간에서 유리
```

### 7.2 DOF 비교

| 시스템 | DOF | vx | vy | ωz | 특징 |
|--------|-----|----|----|----|----|
| 차동 구동 | 2 | ✓ | ✗ | ✓ | 옆으로 이동 불가 |
| 옴니 3륜 | 3 | ✓ | ✓ | ✓ | 전방향 이동 |
| 메카넘 4륜 | 3 | ✓ | ✓ | ✓ | 안정성 ↑ |

---

## 8. 실행 및 테스트

### 8.1 프로그램 실행

```bash
cd /home/clyde/XLeRobot/software/examples
python3 4_xlerobot_teleop_keyboard.py
```

**출력:**
```
=== XLeRobot Teleop ===
Base:  W/A/S/D (move) + Q/E (rotate)
Left:  I/K (Y-axis)
Right: O/L (Y-axis)

Status: [vx=0.00, vy=0.00, wz=0.00]
```

### 8.2 움직임 테스트

**1. 정사각형 이동**
```python
def test_square():
    """정사각형 궤적"""
    for _ in range(4):
        # 전진
        robot.set_mobile_base_velocity(0.2, 0, 0)
        time.sleep(1.0)
        
        # 정지
        robot.set_mobile_base_velocity(0, 0, 0)
        time.sleep(0.5)
        
        # 90° 회전
        robot.set_mobile_base_velocity(0, 0, 0.5)
        time.sleep(3.14)  # π/2 rad
        
        robot.set_mobile_base_velocity(0, 0, 0)
        time.sleep(0.5)
```

**2. 8자 이동**
```python
def test_figure_eight():
    """8자 궤적"""
    # 오른쪽 원
    robot.set_mobile_base_velocity(0.2, 0, 0.4)
    time.sleep(15.7)  # 2π 시간
    
    # 왼쪽 원
    robot.set_mobile_base_velocity(0.2, 0, -0.4)
    time.sleep(15.7)
```

---

## 9. 디버깅

### 9.1 속도 모니터링

```python
def monitor_velocity():
    """실시간 속도 출력"""
    while True:
        # 엔코더에서 휠 속도 읽기
        w1 = robot.base.get_wheel_speed(8)
        w2 = robot.base.get_wheel_speed(9)
        w3 = robot.base.get_wheel_speed(10)
        
        # FK로 로봇 속도 계산
        vx, vy, wz = forward_kinematics_base(w1, w2, w3)
        
        print(f"\rvx={vx:.2f} vy={vy:.2f} wz={wz:.2f}", end='')
        time.sleep(0.1)
```

### 9.2 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| 한 방향만 이동 | IK 행렬 오류 | 각도 확인 (90°, 210°, 330°) |
| 회전 이상 | R 파라미터 틀림 | 실측 베이스 반지름 확인 |
| 속도 불일치 | 휠 반지름 오류 | r=0.03m 확인 |
| 미끄러짐 | 바닥 마찰 부족 | 고무 패드 부착 |

---

## 10. 고급 기능

### 10.1 경로 추종

```python
def follow_path(waypoints):
    """경로점 추종"""
    for wp in waypoints:
        # 현재 위치
        x, y, theta = robot.get_odometry()
        
        # 목표까지 벡터
        dx = wp[0] - x
        dy = wp[1] - y
        
        # 로봇 좌표계로 변환
        vx = dx * np.cos(theta) + dy * np.sin(theta)
        vy = -dx * np.sin(theta) + dy * np.cos(theta)
        
        # P 제어
        kp_linear = 0.5
        kp_angular = 1.0
        
        robot.set_mobile_base_velocity(
            kp_linear * vx,
            kp_linear * vy,
            kp_angular * angle_diff
        )
```

### 10.2 오도메트리 (Odometry)

```python
class Odometry:
    """휠 엔코더 기반 위치 추정"""
    
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.last_time = time.time()
    
    def update(self, vx, vy, wz):
        """속도 적분으로 위치 업데이트"""
        dt = time.time() - self.last_time
        
        # 로봇 좌표계 속도 → 글로벌 좌표계
        vx_global = vx * np.cos(self.theta) - vy * np.sin(self.theta)
        vy_global = vx * np.sin(self.theta) + vy * np.cos(self.theta)
        
        # 적분
        self.x += vx_global * dt
        self.y += vy_global * dt
        self.theta += wz * dt
        
        self.last_time = time.time()
```

---

## 11. 참고 자료

- [Omni Wheel Kinematics](https://robohub.org/omnidirectional-wheels/)
- [소스 코드](../examples/4_xlerobot_teleop_keyboard.py)
- [MobileBase 클래스](../src/robots/mobile_base.py)
- [옴니휠 URDF](../../hardware/)
