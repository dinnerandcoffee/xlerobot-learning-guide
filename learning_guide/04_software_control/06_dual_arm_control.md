# 4.6 양팔 협동 제어

양손 로봇 팔의 독립 제어와 협동 작업 방법을 설명합니다.

## 1. 양팔 시스템 구성

### 1.1 하드웨어 구성

```
XLeRobot 양팔 시스템:
  ├─ 왼팔 (LEFT ARM)
  │   ├─ STS3215 ID: 1-6 (조인트)
  │   └─ STS3215 ID: 7 (그리퍼)
  │
  └─ 오른팔 (RIGHT ARM)
      ├─ STS3215 ID: 11-16 (조인트)
      └─ STS3215 ID: 17 (그리퍼)

통신: 단일 TTL 시리얼 버스 (daisy chain)
```

### 1.2 소프트웨어 아키텍처

```python
# 양팔 로봇 클래스 구조
robot = DualSO100Robot()
  ├─ left_arm: SO100Arm(port="/dev/ttyUSB0")
  │   ├─ motor_ids: [1, 2, 3, 4, 5, 6, 7]
  │   └─ calibration: JOINT_CALIBRATION[0:7]
  │
  └─ right_arm: SO100Arm(port="/dev/ttyUSB0")
      ├─ motor_ids: [11, 12, 13, 14, 15, 16, 17]
      └─ calibration: JOINT_CALIBRATION[7:14]
```

---

## 2. 독립 제어 모드

### 2.1 예제 코드 구조

**파일:** `software/examples/2_dual_so100_keyboard_ee_control.py`

```python
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src'))

from robots.dual_so100 import DualSO100Robot
import termios
import tty

# 초기화
robot = DualSO100Robot()

# 왼팔/오른팔 독립 제어
left_target = [0.15, 0.10]   # (x, y)
right_target = [0.15, -0.10]

robot.left_arm.set_ee_position(left_target)
robot.right_arm.set_ee_position(right_target)
```

### 2.2 키보드 매핑

```python
# 왼팔 제어 (WASD + QE)
LEFT_KEYMAP = {
    'w': (0, +MOVE_STEP),    # Y 증가
    's': (0, -MOVE_STEP),    # Y 감소
    'a': (-MOVE_STEP, 0),    # X 감소
    'd': (+MOVE_STEP, 0),    # X 증가
    'q': 'gripper_open',     # 그리퍼 열기
    'e': 'gripper_close',    # 그리퍼 닫기
}

# 오른팔 제어 (방향키 + NM)
RIGHT_KEYMAP = {
    'up':    (0, +MOVE_STEP),   # Y 증가
    'down':  (0, -MOVE_STEP),   # Y 감소
    'left':  (-MOVE_STEP, 0),   # X 감소
    'right': (+MOVE_STEP, 0),   # X 증가
    'n': 'gripper_open',        # 그리퍼 열기
    'm': 'gripper_close',       # 그리퍼 닫기
}

MOVE_STEP = 0.01  # 1cm 단위
```

---

## 3. DualSO100Robot 클래스

### 3.1 초기화

```python
class DualSO100Robot:
    def __init__(self, port="/dev/ttyUSB0"):
        """양팔 로봇 초기화"""
        # 왼팔 생성
        self.left_arm = SO100Arm(
            port=port,
            motor_ids=[1, 2, 3, 4, 5, 6, 7],
            name="LEFT"
        )
        
        # 오른팔 생성
        self.right_arm = SO100Arm(
            port=port,
            motor_ids=[11, 12, 13, 14, 15, 16, 17],
            name="RIGHT"
        )
        
        # 초기 위치
        self.left_target = [0.15, 0.10]
        self.right_target = [0.15, -0.10]
        
        # 홈 위치로 이동
        self.go_home()
    
    def go_home(self):
        """홈 포지션"""
        self.left_arm.set_ee_position(self.left_target)
        self.right_arm.set_ee_position(self.right_target)
```

### 3.2 독립 제어

```python
def update_left(self, dx, dy):
    """왼팔 위치 업데이트"""
    self.left_target[0] += dx
    self.left_target[1] += dy
    
    # 워크스페이스 제한
    self.left_target = self._clamp_workspace(self.left_target)
    
    # IK + 모터 제어
    self.left_arm.set_ee_position(self.left_target)

def update_right(self, dx, dy):
    """오른팔 위치 업데이트"""
    self.right_target[0] += dx
    self.right_target[1] += dy
    
    # 워크스페이스 제한
    self.right_target = self._clamp_workspace(self.right_target)
    
    # IK + 모터 제어
    self.right_arm.set_ee_position(self.right_target)

def _clamp_workspace(self, pos):
    """워크스페이스 제한"""
    x, y = pos
    x = max(0.05, min(0.25, x))  # 5~25cm
    y = max(-0.15, min(0.15, y))  # ±15cm
    return [x, y]
```

---

## 4. 메인 제어 루프

### 4.1 키보드 입력 처리

```python
def get_key():
    """논블로킹 키 입력"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
        
        # 방향키 처리
        if ch == '\x1b':  # ESC 시퀀스
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                if ch3 == 'A': return 'up'
                if ch3 == 'B': return 'down'
                if ch3 == 'C': return 'right'
                if ch3 == 'D': return 'left'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main():
    robot = DualSO100Robot()
    
    print("=== Dual Arm Control ===")
    print("Left Arm:  WASD (move) + Q/E (gripper)")
    print("Right Arm: Arrow keys (move) + N/M (gripper)")
    print("Press 'h' for home, 'x' to exit\n")
    
    try:
        while True:
            key = get_key()
            
            # 왼팔 제어
            if key in LEFT_KEYMAP:
                action = LEFT_KEYMAP[key]
                if isinstance(action, tuple):
                    dx, dy = action
                    robot.update_left(dx, dy)
                elif action == 'gripper_open':
                    robot.left_arm.open_gripper()
                elif action == 'gripper_close':
                    robot.left_arm.close_gripper()
            
            # 오른팔 제어
            elif key in RIGHT_KEYMAP:
                action = RIGHT_KEYMAP[key]
                if isinstance(action, tuple):
                    dx, dy = action
                    robot.update_right(dx, dy)
                elif action == 'gripper_open':
                    robot.right_arm.open_gripper()
                elif action == 'gripper_close':
                    robot.right_arm.close_gripper()
            
            # 공통 명령
            elif key == 'h':
                robot.go_home()
            elif key == 'x':
                break
            
            time.sleep(0.02)  # 50Hz
    
    finally:
        robot.cleanup()
```

---

## 5. 협동 제어 (Bimanual Coordination)

### 5.1 동기화 모드

```python
class CoordinatedControl:
    """양팔 협동 제어"""
    
    def __init__(self, robot):
        self.robot = robot
        self.mode = "independent"  # "independent" or "mirrored"
    
    def set_mirrored_mode(self, enable=True):
        """미러 모드: 왼팔 움직임을 오른팔이 대칭으로 따라감"""
        self.mode = "mirrored" if enable else "independent"
    
    def update(self, key):
        """키 입력 처리"""
        if self.mode == "independent":
            # 독립 제어 (위와 동일)
            self._update_independent(key)
        
        elif self.mode == "mirrored":
            # 왼팔 제어 → 오른팔 자동 미러링
            if key in LEFT_KEYMAP:
                action = LEFT_KEYMAP[key]
                if isinstance(action, tuple):
                    dx, dy = action
                    
                    # 왼팔 업데이트
                    self.robot.update_left(dx, dy)
                    
                    # 오른팔 미러링 (Y축 대칭)
                    self.robot.update_right(dx, -dy)
```

### 5.2 상대 위치 유지

```python
def move_together(self, dx, dy):
    """양팔 동시 이동 (상대 위치 유지)"""
    # 왼팔
    self.robot.left_target[0] += dx
    self.robot.left_target[1] += dy
    
    # 오른팔
    self.robot.right_target[0] += dx
    self.robot.right_target[1] += dy
    
    # 동시 명령 전송
    self.robot.left_arm.set_ee_position(self.robot.left_target)
    self.robot.right_arm.set_ee_position(self.robot.right_target)

def grasp_object_bimanual(self, object_width):
    """양손으로 물체 잡기"""
    center_x = 0.15
    center_y = 0.0
    
    # 양팔을 물체 너비만큼 벌림
    half_width = object_width / 2.0
    
    # 왼팔: 중심 + 오른쪽 오프셋
    self.robot.left_target = [center_x, center_y + half_width]
    
    # 오른팔: 중심 + 왼쪽 오프셋
    self.robot.right_target = [center_x, center_y - half_width]
    
    # 동시 이동
    self.robot.left_arm.set_ee_position(self.robot.left_target)
    self.robot.right_arm.set_ee_position(self.robot.right_target)
    
    time.sleep(1.0)  # 위치 도달 대기
    
    # 그리퍼 닫기
    self.robot.left_arm.close_gripper()
    self.robot.right_arm.close_gripper()
```

---

## 6. 충돌 회피

### 6.1 워크스페이스 분할

```
       Y축
        ↑
  L영역 | R영역
 -------+-------  → X축
   +Y   |  -Y
```

```python
def check_collision_risk(self):
    """충돌 위험 감지"""
    left_x, left_y = self.robot.left_target
    right_x, right_y = self.robot.right_target
    
    # 거리 계산
    distance = math.sqrt((left_x - right_x)**2 + 
                        (left_y - right_y)**2)
    
    SAFETY_MARGIN = 0.10  # 10cm 안전 거리
    
    if distance < SAFETY_MARGIN:
        print(f"⚠️  Warning: Arms too close ({distance*100:.1f}cm)")
        return True
    return False

def enforce_workspace_separation(self):
    """워크스페이스 분리 강제"""
    left_x, left_y = self.robot.left_target
    right_x, right_y = self.robot.right_target
    
    # 왼팔: Y > 0 영역만
    if left_y < 0:
        left_y = 0
        self.robot.left_target[1] = left_y
    
    # 오른팔: Y < 0 영역만
    if right_y > 0:
        right_y = 0
        self.robot.right_target[1] = right_y
```

---

## 7. 실행

### 7.1 프로그램 실행

```bash
cd /home/clyde/XLeRobot/software/examples
python3 2_dual_so100_keyboard_ee_control.py
```

**출력:**
```
=== Dual Arm EE Control ===
Left Arm:  WASD (XY) + Q/E (gripper)
Right Arm: Arrows (XY) + N/M (gripper)
Commands:  h (home), x (exit)

Left:  [0.150, 0.100]
Right: [0.150, -0.100]
```

### 7.2 사용 예시

**1. 독립 제어**
```
1. W 키: 왼팔 Y+ 이동 → Left: [0.150, 0.110]
2. ↑ 키: 오른팔 Y+ 이동 → Right: [0.150, -0.090]
3. Q 키: 왼쪽 그리퍼 열기
4. M 키: 오른쪽 그리퍼 닫기
```

**2. 양손 잡기**
```python
# 코드 추가
coord = CoordinatedControl(robot)
coord.grasp_object_bimanual(object_width=0.08)  # 8cm 물체
```

---

## 8. 고급 협동 기능

### 8.1 리더-팔로워 모드

```python
class LeaderFollowerControl:
    """한 팔이 리더, 다른 팔이 팔로워"""
    
    def __init__(self, robot, leader='left'):
        self.robot = robot
        self.leader = leader
        self.offset = [0, 0]  # 팔로워 오프셋
    
    def update_leader(self, dx, dy):
        """리더 팔 이동 → 팔로워 자동 추종"""
        if self.leader == 'left':
            # 왼팔 이동
            self.robot.update_left(dx, dy)
            
            # 오른팔 추종 (오프셋 적용)
            right_x = self.robot.left_target[0] + self.offset[0]
            right_y = self.robot.left_target[1] + self.offset[1]
            self.robot.right_target = [right_x, right_y]
            self.robot.right_arm.set_ee_position(self.robot.right_target)
```

### 8.2 상대 모션 제어

```python
def move_relative(self, left_dx, left_dy, right_dx, right_dy):
    """각 팔 독립적 상대 이동"""
    # 왼팔
    self.robot.left_target[0] += left_dx
    self.robot.left_target[1] += left_dy
    
    # 오른팔
    self.robot.right_target[0] += right_dx
    self.robot.right_target[1] += right_dy
    
    # 워크스페이스 제한
    self.robot.left_target = self.robot._clamp_workspace(
        self.robot.left_target
    )
    self.robot.right_target = self.robot._clamp_workspace(
        self.robot.right_target
    )
    
    # 동시 명령
    self.robot.left_arm.set_ee_position(self.robot.left_target)
    self.robot.right_arm.set_ee_position(self.robot.right_target)
```

---

## 9. 디버깅

### 9.1 실시간 모니터링

```python
def print_status(self):
    """양팔 상태 출력"""
    left_pos = self.robot.left_arm.get_current_position()
    right_pos = self.robot.right_arm.get_current_position()
    
    print(f"\r Left: [{left_pos[0]:.3f}, {left_pos[1]:.3f}] | "
          f"Right: [{right_pos[0]:.3f}, {right_pos[1]:.3f}]", 
          end='')
```

### 9.2 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| 한쪽 팔만 움직임 | 모터 ID 충돌 | `motor_ids` 확인 (1-7 vs 11-17) |
| 미러 모드 이상 | Y축 부호 오류 | `update_right(dx, -dy)` 확인 |
| 충돌 발생 | 안전 거리 미설정 | `check_collision_risk()` 활성화 |
| 동기화 지연 | 순차 명령 전송 | 동시 명령으로 변경 |

---

## 10. 참고 자료

- [Dual Arm Manipulation (MIT)](http://manipulation.csail.mit.edu/bimanual.html)
- [소스 코드](../examples/2_dual_so100_keyboard_ee_control.py)
- [SO100Arm 클래스](../src/robots/so100.py)
