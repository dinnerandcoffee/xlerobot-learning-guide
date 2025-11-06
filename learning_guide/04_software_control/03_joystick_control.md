# 4.3 조이스틱 제어 (Xbox)

Xbox 컨트롤러를 이용한 정밀하고 직관적인 로봇 제어 방법을 설명합니다.

## 1. Xbox 컨트롤러 개요

### 1.1 왜 Xbox 컨트롤러인가?

| 장점 | 설명 |
|------|------|
| **아날로그 입력** | 스틱으로 연속적인 속도 제어 가능 |
| **양손 제어** | 왼손/오른손 팔을 동시에 독립 제어 |
| **트리거** | 압력 감지로 부드러운 그리퍼 제어 |
| **에르고노믹** | 장시간 사용 시 피로도 낮음 |
| **호환성** | Linux/Windows/Mac 공식 지원 |

### 1.2 지원 컨트롤러

- **Xbox One 컨트롤러** (USB/무선)
- **Xbox Series X|S 컨트롤러** (USB/Bluetooth)
- **Xbox 360 컨트롤러** (USB)

---

## 2. 버튼 맵핑

### 2.1 왼팔 제어 (Left Arm)

```
┌─────────────────────────────┐
│   [LB]           [RB]       │
│    ↑               ↑        │
│  Pitch         (오른팔)     │
│ Wrist Roll                  │
│                             │
│  [왼쪽 스틱]    [오른쪽 스틱]│
│     ↑↓              ↑↓      │
│     X방향         (오른팔)  │
│   ←→ Y방향                  │
│                             │
│   [LT]          [RT]        │
│  왼 그리퍼      오른 그리퍼 │
└─────────────────────────────┘

왼쪽 스틱:
  - 기본: X-Y 평면 이동 (EE 제어)
  - 눌림(L3): Shoulder Pan 회전

LB 버튼 + 왼쪽 스틱:
  - ↑↓: Pitch 제어
  - ←→: Wrist Roll 제어

LT 트리거:
  - 압력: 그리퍼 열림 정도 (0~100%)
```

### 2.2 오른팔 제어 (Right Arm)

```
오른쪽 스틱:
  - 기본: X-Y 평면 이동 (EE 제어)
  - 눌림(R3): Shoulder Pan 회전

RB 버튼 + 오른쪽 스틱:
  - ↑↓: Pitch 제어
  - ←→: Wrist Roll 제어

RT 트리거:
  - 압력: 그리퍼 열림 정도 (0~100%)
```

### 2.3 모바일 베이스 제어

```
D-Pad (십자 버튼):
  ↑: 후진
  ↓: 전진
  ←: 좌회전
  →: 우회전
```

### 2.4 헤드 짐벌 제어

```
Face Buttons:
  X: 헤드 팬(Pan) 왼쪽
  B: 헤드 팬(Pan) 오른쪽
  A: 헤드 틸트(Tilt) 아래
  Y: 헤드 틸트(Tilt) 위
```

### 2.5 특수 기능

```
Back 버튼: 전체 리셋 (제로 포지션)
Start 버튼: (예약됨)
```

---

## 3. 코드 구조

### 3.1 Pygame 초기화

```python
import pygame

# Pygame 및 조이스틱 초기화
pygame.init()
pygame.joystick.init()

# 컨트롤러 연결 확인
if pygame.joystick.get_count() == 0:
    raise RuntimeError("No Xbox controller detected!")

joystick = pygame.joystick.Joystick(0)
joystick.init()

print(f"Controller: {joystick.get_name()}")
print(f"Axes: {joystick.get_numaxes()}")
print(f"Buttons: {joystick.get_numbuttons()}")
```

### 3.2 키맵 정의

```python
# 왼팔 키맵 (의미 있는 동작 → 컨트롤러 입력)
LEFT_KEYMAP = {
    # 왼쪽 스틱으로 X-Y 제어
    'x+': 'left_stick_up', 
    'x-': 'left_stick_down',
    'y+': 'left_stick_right', 
    'y-': 'left_stick_left',
    
    # 왼쪽 스틱 눌림 + 이동 = Shoulder Pan
    'shoulder_pan+': 'left_stick_pressed_right', 
    'shoulder_pan-': 'left_stick_pressed_left',
    
    # LB + 스틱 = Pitch/Wrist Roll
    'pitch+': 'lb_up', 
    'pitch-': 'lb_down',
    'wrist_roll+': 'lb_right', 
    'wrist_roll-': 'lb_left',
    
    # 트리거 = 그리퍼
    'gripper+': 'left_trigger',
    
    # 헤드 모터 (A/B/X/Y)
    "head_motor_1+": 'x', 
    "head_motor_1-": 'b',
    "head_motor_2+": 'a', 
    "head_motor_2-": 'y',
}

# 오른팔 키맵 (대칭)
RIGHT_KEYMAP = {
    'x+': 'right_stick_up', 
    'x-': 'right_stick_down',
    'y+': 'right_stick_right', 
    'y-': 'right_stick_left',
    'shoulder_pan+': 'right_stick_pressed_right', 
    'shoulder_pan-': 'right_stick_pressed_left',
    'pitch+': 'rb_up', 
    'pitch-': 'rb_down',
    'wrist_roll+': 'rb_right', 
    'wrist_roll-': 'rb_left',
    'gripper+': 'right_trigger',
}

# 베이스 키맵
BASE_KEYMAP = {
    'forward': 'dpad_down',      # ↓
    'backward': 'dpad_up',       # ↑
    'rotate_left': 'dpad_left',  # ←
    'rotate_right': 'dpad_right',# →
}

# 리셋 키
RESET_KEY = 'back'
```

### 3.3 입력 읽기 함수

```python
def get_xbox_input(joystick):
    """
    Xbox 컨트롤러 입력 읽기
    
    Returns:
        dict: 버튼/스틱/트리거 상태
    """
    pygame.event.pump()  # 이벤트 업데이트
    
    # 스틱 입력 (범위: -1.0 ~ +1.0)
    left_x = joystick.get_axis(0)   # 좌우
    left_y = -joystick.get_axis(1)  # 상하 (반전)
    right_x = joystick.get_axis(2)
    right_y = -joystick.get_axis(3)
    
    # 트리거 입력 (범위: 0.0 ~ 1.0)
    # Xbox 컨트롤러는 -1~1 범위로 보고하므로 0~1로 변환
    left_trigger = (joystick.get_axis(4) + 1.0) / 2.0
    right_trigger = (joystick.get_axis(5) + 1.0) / 2.0
    
    # 버튼 입력 (0: 안 눌림, 1: 눌림)
    buttons = {}
    for i in range(joystick.get_numbuttons()):
        buttons[i] = joystick.get_button(i)
    
    # D-Pad 입력 (Hat)
    hat_x, hat_y = joystick.get_hat(0)
    
    return {
        'left_stick': (left_x, left_y),
        'right_stick': (right_x, right_y),
        'left_trigger': left_trigger,
        'right_trigger': right_trigger,
        'buttons': buttons,
        'dpad': (hat_x, hat_y),
        'left_stick_pressed': buttons.get(8, 0),  # L3
        'right_stick_pressed': buttons.get(9, 0), # R3
        'lb': buttons.get(4, 0),
        'rb': buttons.get(5, 0),
        'back': buttons.get(6, 0),
    }
```

### 3.4 EE 제어 (IK 기반)

```python
class ArmController:
    def __init__(self, kinematics, joint_map):
        self.ik = kinematics
        self.joint_map = joint_map
        self.target_x = 0.15  # 초기 X 위치
        self.target_y = 0.10  # 초기 Y 위치
        self.shoulder_pan = 0.0
        self.pitch = 0.0
        self.wrist_roll = 0.0
        self.gripper = 0.0
    
    def update(self, stick_x, stick_y, stick_pressed, 
               shoulder_input, lb_pressed, lb_stick, 
               trigger_value):
        """
        스틱 입력으로 팔 제어
        
        Args:
            stick_x, stick_y: 메인 스틱 (-1~1)
            stick_pressed: L3/R3 눌림 여부
            shoulder_input: 스틱 X 입력 (shoulder_pan용)
            lb_pressed: LB/RB 버튼
            lb_stick: LB+스틱 입력 (pitch/wrist_roll)
            trigger_value: 트리거 압력 (0~1)
        """
        dt = 0.02  # 제어 주기 50Hz
        
        # 모드 1: 일반 스틱 = X-Y EE 제어
        if not stick_pressed and not lb_pressed:
            self.target_x += stick_y * 0.002  # 2mm/step
            self.target_y += stick_x * 0.002
        
        # 모드 2: 스틱 눌림 = Shoulder Pan
        elif stick_pressed:
            self.shoulder_pan += shoulder_input * 1.0  # 1°/step
        
        # 모드 3: LB/RB 눌림 = Pitch/Wrist Roll
        elif lb_pressed:
            lb_x, lb_y = lb_stick
            self.pitch += lb_y * 1.0
            self.wrist_roll += lb_x * 1.0
        
        # 트리거 = 그리퍼 (0~60°)
        self.gripper = trigger_value * 60.0
        
        # IK 계산
        joint2, joint3 = self.ik.inverse_kinematics(
            self.target_x, self.target_y
        )
        
        # 조인트 각도 반환
        return {
            self.joint_map['shoulder_pan']: self.shoulder_pan,
            self.joint_map['shoulder_lift']: math.degrees(joint2),
            self.joint_map['elbow_flex']: math.degrees(joint3),
            self.joint_map['wrist_flex']: self.pitch,
            self.joint_map['wrist_roll']: self.wrist_roll,
            self.joint_map['gripper']: self.gripper,
        }
```

### 3.5 베이스 제어

```python
class BaseController:
    def __init__(self):
        self.linear_speed = 0.2   # m/s
        self.angular_speed = 1.0  # rad/s
    
    def update(self, dpad_x, dpad_y):
        """
        D-Pad 입력으로 베이스 제어
        
        Args:
            dpad_x: 좌우 (-1, 0, 1)
            dpad_y: 상하 (-1, 0, 1)
        
        Returns:
            vx, vy, omega: 속도 명령
        """
        vx = 0.0
        vy = 0.0
        omega = 0.0
        
        # 전후진
        if dpad_y == -1:  # ↓ = 전진
            vx = self.linear_speed
        elif dpad_y == 1:  # ↑ = 후진
            vx = -self.linear_speed
        
        # 회전
        if dpad_x == -1:  # ← = 좌회전
            omega = self.angular_speed
        elif dpad_x == 1:  # → = 우회전
            omega = -self.angular_speed
        
        return vx, vy, omega
```

---

## 4. 메인 제어 루프

```python
from lerobot.robots.xlerobot import XLerobotConfig, XLerobot
from lerobot.model.SO101Robot import SO101Kinematics

# 로봇 초기화
config = XLerobotConfig()
robot = XLerobot(config)

# IK 엔진
left_ik = SO101Kinematics()
right_ik = SO101Kinematics()

# 컨트롤러 초기화
left_arm = ArmController(left_ik, LEFT_JOINT_MAP)
right_arm = ArmController(right_ik, RIGHT_JOINT_MAP)
base = BaseController()

# 제어 루프
CONTROL_FREQ = 50  # Hz
dt = 1.0 / CONTROL_FREQ

try:
    while True:
        start_time = time.time()
        
        # Xbox 입력 읽기
        xbox = get_xbox_input(joystick)
        
        # 리셋 체크
        if xbox['back']:
            robot.reset()
            left_arm.reset()
            right_arm.reset()
            print("Reset to zero position")
            time.sleep(0.5)
            continue
        
        # 왼팔 제어
        left_joints = left_arm.update(
            stick_x=xbox['left_stick'][0],
            stick_y=xbox['left_stick'][1],
            stick_pressed=xbox['left_stick_pressed'],
            shoulder_input=xbox['left_stick'][0],
            lb_pressed=xbox['lb'],
            lb_stick=xbox['left_stick'],
            trigger_value=xbox['left_trigger']
        )
        
        # 오른팔 제어
        right_joints = right_arm.update(
            stick_x=xbox['right_stick'][0],
            stick_y=xbox['right_stick'][1],
            stick_pressed=xbox['right_stick_pressed'],
            shoulder_input=xbox['right_stick'][0],
            lb_pressed=xbox['rb'],
            lb_stick=xbox['right_stick'],
            trigger_value=xbox['right_trigger']
        )
        
        # 베이스 제어
        vx, vy, omega = base.update(
            dpad_x=xbox['dpad'][0],
            dpad_y=xbox['dpad'][1]
        )
        
        # 로봇에 명령 전송
        robot.set_arm_joints(left_joints, 'left')
        robot.set_arm_joints(right_joints, 'right')
        robot.set_base_velocity(vx, vy, omega)
        
        # 타이밍 유지
        elapsed = time.time() - start_time
        time.sleep(max(0, dt - elapsed))

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    robot.stop()
    pygame.quit()
```

---

## 5. 실행 방법

### 5.1 설치

```bash
# Pygame 설치
pip install pygame

# 권한 설정 (Linux)
sudo usermod -aG input $USER
# 로그아웃 후 다시 로그인
```

### 5.2 컨트롤러 연결

**USB 연결:**
```bash
# 연결 확인
lsusb | grep Xbox
# 출력 예: Bus 001 Device 005: ID 045e:02ea Microsoft Corp. Xbox One Controller
```

**무선 연결 (Bluetooth):**
```bash
# Bluetooth 페어링
bluetoothctl
> scan on
> pair XX:XX:XX:XX:XX:XX  # Xbox 컨트롤러 MAC
> connect XX:XX:XX:XX:XX:XX
> trust XX:XX:XX:XX:XX:XX
```

### 5.3 실행

```bash
cd ~/XLeRobot/software
PYTHONPATH=src python examples/5_xlerobot_teleop_xbox.py
```

**실행 화면:**
```
Controller: Xbox Wireless Controller
Axes: 6
Buttons: 11
Robot initialized
Press Back button to reset

Left Arm:  X=0.150 Y=0.100 Pan=0.0°
Right Arm: X=0.150 Y=0.100 Pan=0.0°
Base: vx=0.00 vy=0.00 ω=0.00
```

---

## 6. 고급 기능

### 6.1 데드존 설정

스틱 중립 위치 근처의 노이즈 제거:

```python
def apply_deadzone(value, threshold=0.1):
    """
    데드존 적용
    
    Args:
        value: 스틱 입력 (-1~1)
        threshold: 데드존 크기
    
    Returns:
        조정된 값
    """
    if abs(value) < threshold:
        return 0.0
    
    # 선형 스케일링
    sign = 1 if value > 0 else -1
    scaled = (abs(value) - threshold) / (1.0 - threshold)
    return sign * scaled

# 사용 예
stick_x = apply_deadzone(joystick.get_axis(0), 0.15)
```

### 6.2 입력 곡선 (Exponential)

작은 입력에 더 높은 정밀도:

```python
def expo_curve(value, expo=2.0):
    """
    지수 곡선 적용
    
    Args:
        value: 입력 (-1~1)
        expo: 지수 (1.0=선형, 2.0=제곱)
    """
    sign = 1 if value > 0 else -1
    return sign * (abs(value) ** expo)

# 사용 예
stick_x = expo_curve(stick_x, 2.0)  # 부드러운 제어
```

### 6.3 속도 프로파일

가속/감속 구간 추가:

```python
class SmoothController:
    def __init__(self, max_accel=0.5):
        self.current_vel = 0.0
        self.max_accel = max_accel  # m/s²
    
    def update(self, target_vel, dt):
        """
        부드러운 가속/감속
        """
        error = target_vel - self.current_vel
        max_change = self.max_accel * dt
        
        if abs(error) > max_change:
            self.current_vel += max_change * (1 if error > 0 else -1)
        else:
            self.current_vel = target_vel
        
        return self.current_vel
```

---

## 7. 문제 해결

### 문제 1: 컨트롤러 인식 안 됨

**Linux:**
```bash
# xboxdrv 설치
sudo apt install xboxdrv

# 수동 드라이버 로드
sudo xboxdrv --detach-kernel-driver
```

**권한 문제:**
```bash
# udev 규칙 추가
sudo nano /etc/udev/rules.d/99-xbox.rules

# 내용:
SUBSYSTEM=="usb", ATTRS{idVendor}=="045e", MODE="0666"

# 재시작
sudo udevadm control --reload-rules
```

### 문제 2: 입력 지연

**원인:** Pygame 이벤트 큐 오버플로우  
**해결:**
```python
# 매 루프마다 이벤트 처리
for event in pygame.event.get():
    pass  # 큐 비우기
```

### 문제 3: 버튼 번호 불일치

컨트롤러 종류마다 버튼 번호가 다름:

```python
# 버튼 테스트 스크립트
for i in range(joystick.get_numbuttons()):
    if joystick.get_button(i):
        print(f"Button {i} pressed")
```

---

## 8. 성능 비교

| 제어 방식 | 정밀도 | 속도 | 학습 곡선 | 가격 |
|-----------|--------|------|-----------|------|
| 키보드 | ★★☆☆☆ | ★★☆☆☆ | ★★★★★ | $0 |
| Xbox | ★★★★☆ | ★★★★☆ | ★★★☆☆ | $60 |
| Joycon | ★★★☆☆ | ★★★☆☆ | ★★☆☆☆ | $80 |
| VR | ★★★★★ | ★★★★★ | ★★☆☆☆ | $500 |

---

## 참고 자료

- [5_xlerobot_teleop_xbox.py 전체 코드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/../software/examples/5_xlerobot_teleop_xbox.py)
- [Pygame 조이스틱 문서](https://www.pygame.org/docs/ref/joystick.html)
- [Xbox 컨트롤러 사양](https://support.xbox.com/help/hardware-network/controller/xbox-wireless-controller)
- [xboxdrv 문서](https://xboxdrv.gitlab.io/)
