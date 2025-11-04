# 4.4 Switch Joycon 제어

Nintendo Switch Joycon을 이용한 휴대성 높은 로봇 제어 방법입니다.

## 1. Joycon 개요

### 1.1 Joycon의 장점

| 특징 | 설명 |
|------|------|
| **휴대성** | 작고 가벼워 이동 중 제어 가능 |
| **블루투스** | 무선 연결, 케이블 불필요 |
| **자이로스코프** | 손목 기울기로 직관적 제어 |
| **양손 독립** | 왼/오른 Joycon 분리 사용 |
| **배터리** | 내장 충전식 배터리 (~20시간) |

### 1.2 단점 및 한계

- 버튼 수 제한 (Xbox보다 적음)
- 블루투스 연결 안정성 (간섭 주의)
- 배터리 관리 필요
- Linux 페어링 복잡

---

## 2. 하드웨어 준비

### 2.1 필요한 장비

- **Nintendo Switch Joycon** (L+R 세트)
- **Bluetooth 동글** (내장 블루투스도 가능)
- **충전 그립** (선택, 배터리 충전용)

### 2.2 블루투스 페어링 (Linux)

**1단계: Bluetooth 활성화**
```bash
# Bluetooth 서비스 시작
sudo systemctl start bluetooth
sudo systemctl enable bluetooth

# bluetoothctl 실행
bluetoothctl
```

**2단계: Joycon 페어링 모드**
```
왼쪽 Joycon:
  - SR + SL 버튼 동시에 3초 이상 누름
  - LED가 좌우로 움직이면 페어링 모드

오른쪽 Joycon:
  - SR + SL 버튼 동시에 3초 이상 누름
```

**3단계: 페어링 실행**
```bash
[bluetooth]# scan on
# Joycon이 감지되면:
# Joy-Con (L) 또는 Joy-Con (R)

[bluetooth]# pair XX:XX:XX:XX:XX:XX
[bluetooth]# connect XX:XX:XX:XX:XX:XX
[bluetooth]# trust XX:XX:XX:XX:XX:XX

# 완료 후
[bluetooth]# scan off
[bluetooth]# exit
```

**4단계: 연결 확인**
```bash
# 연결된 장치 확인
bluetoothctl devices
```

---

## 3. joyconrobotics 라이브러리

### 3.1 설치

XLeRobot 프로젝트에 내장된 Joycon 라이브러리:

```bash
cd ~/XLeRobot/software
# 이미 joyconrobotics/ 디렉토리에 포함됨

# 의존성 설치
pip install hid
pip install hidapi
```

### 3.2 라이브러리 구조

```
joyconrobotics/
├── __init__.py
├── joycon.py           # 메인 Joycon 클래스
├── device.py           # HID 장치 관리
├── event.py            # 이벤트 핸들러
├── gyro.py             # 자이로/가속도 센서
├── wrappers.py         # 편의 함수
└── constants.py        # 버튼 코드 정의
```

### 3.3 기본 사용법

```python
from joyconrobotics import JoyConLeft, JoyConRight

# Joycon 연결
joycon_left = JoyConLeft()
joycon_right = JoyConRight()

# 버튼 상태 읽기
while True:
    # 왼쪽 Joycon
    if joycon_left.get_button_up():
        print("Up button pressed")
    
    # 오른쪽 Joycon
    if joycon_right.get_button_a():
        print("A button pressed")
    
    # 스틱 입력 (-100 ~ 100)
    stick_x, stick_y = joycon_left.get_stick_left()
    print(f"Stick: ({stick_x}, {stick_y})")
    
    # 자이로 데이터
    gyro_x, gyro_y, gyro_z = joycon_left.get_gyro_data()
```

---

## 4. 버튼 맵핑

### 4.1 왼쪽 Joycon

```
      [−]  [Capture]
       ↓      ↓
    ┌──────────┐
[L] │          │
[ZL]│   [↑]   │
    │ [←][→]  │
    │   [↓]   │
    │          │
    │  (스틱)  │
    └──────────┘
      [SR][SL]

방향 버튼:
  ↑: EE Y+ (전진)
  ↓: EE Y- (후진)
  ←: EE X- (왼쪽)
  →: EE X+ (오른쪽)

스틱:
  - Shoulder Pan 제어

L 버튼:
  - 누른 상태에서 스틱 = Pitch 제어

ZL 버튼:
  - 그리퍼 열기

특수:
  - 버튼: (미사용)
  Capture: 리셋
```

### 4.2 오른쪽 Joycon

```
[+]  [Home]
 ↓      ↓
┌──────────┐
│          │ [R]
│   [X]    │ [ZR]
│ [Y] [A]  │
│   [B]    │
│          │
│  (스틱)  │
└──────────┘
 [SR][SL]

Face 버튼:
  A: 그리퍼 닫기
  B: Wrist Flex +
  X: Wrist Flex -
  Y: Wrist Roll

스틱:
  - 오른팔 Shoulder Pan

R 버튼:
  - 누른 상태에서 스틱 = Pitch

ZR 버튼:
  - 그리퍼 열기

Home: 리셋
```

---

## 5. 코드 구현

### 5.1 Joycon 입력 읽기

```python
from joyconrobotics import JoyConLeft, JoyConRight
import time

class JoyconController:
    def __init__(self):
        # Joycon 연결
        try:
            self.left = JoyConLeft()
            self.right = JoyConRight()
            print("Joycons connected successfully")
        except Exception as e:
            raise RuntimeError(f"Failed to connect Joycons: {e}")
        
        # LED 색상 설정 (초록색)
        self.left.set_player_lamp(0b0001)
        self.right.set_player_lamp(0b0001)
    
    def get_left_input(self):
        """왼쪽 Joycon 입력"""
        return {
            'dpad_up': self.left.get_button_up(),
            'dpad_down': self.left.get_button_down(),
            'dpad_left': self.left.get_button_left(),
            'dpad_right': self.left.get_button_right(),
            'l': self.left.get_button_l(),
            'zl': self.left.get_button_zl(),
            'minus': self.left.get_button_minus(),
            'capture': self.left.get_button_capture(),
            'stick': self.left.get_stick_left(),  # (x, y)
        }
    
    def get_right_input(self):
        """오른쪽 Joycon 입력"""
        return {
            'a': self.right.get_button_a(),
            'b': self.right.get_button_b(),
            'x': self.right.get_button_x(),
            'y': self.right.get_button_y(),
            'r': self.right.get_button_r(),
            'zr': self.right.get_button_zr(),
            'plus': self.right.get_button_plus(),
            'home': self.right.get_button_home(),
            'stick': self.right.get_stick_right(),  # (x, y)
        }
```

### 5.2 팔 제어 로직

```python
class JoyconArmController:
    def __init__(self, ik_engine):
        self.ik = ik_engine
        self.target_x = 0.15
        self.target_y = 0.10
        self.shoulder_pan = 0.0
        self.pitch = 0.0
        self.wrist_roll = 0.0
        self.gripper = 0.0
    
    def update(self, joy_input):
        """
        Joycon 입력으로 팔 제어
        
        Args:
            joy_input: get_left_input() 또는 get_right_input() 결과
        """
        # D-Pad로 EE X-Y 제어
        if joy_input['dpad_up']:
            self.target_y += 0.005  # 5mm
        if joy_input['dpad_down']:
            self.target_y -= 0.005
        if joy_input['dpad_left']:
            self.target_x -= 0.005
        if joy_input['dpad_right']:
            self.target_x += 0.005
        
        # 스틱으로 Shoulder Pan
        stick_x, stick_y = joy_input['stick']
        self.shoulder_pan += stick_x / 100.0 * 2.0  # 2°/step
        
        # L/R 버튼 + 스틱 = Pitch
        if joy_input.get('l') or joy_input.get('r'):
            self.pitch += stick_y / 100.0 * 2.0
        
        # Face 버튼 (오른쪽 Joycon만)
        if 'b' in joy_input and joy_input['b']:
            self.pitch += 1.0
        if 'x' in joy_input and joy_input['x']:
            self.pitch -= 1.0
        if 'y' in joy_input and joy_input['y']:
            self.wrist_roll += 2.0
        
        # 그리퍼
        if joy_input.get('zl') or joy_input.get('a'):
            self.gripper = 60.0  # 열림
        else:
            self.gripper = 0.0   # 닫힘
        
        # IK 계산
        joint2, joint3 = self.ik.inverse_kinematics(
            self.target_x, self.target_y
        )
        
        return {
            'shoulder_pan': self.shoulder_pan,
            'shoulder_lift': math.degrees(joint2),
            'elbow_flex': math.degrees(joint3),
            'wrist_flex': self.pitch,
            'wrist_roll': self.wrist_roll,
            'gripper': self.gripper,
        }
```

### 5.3 메인 루프

```python
from lerobot.model.SO101Robot import SO101Kinematics

# 초기화
joycon = JoyconController()
left_arm = JoyconArmController(SO101Kinematics())
right_arm = JoyconArmController(SO101Kinematics())

# 제어 루프
CONTROL_FREQ = 50  # Hz
dt = 1.0 / CONTROL_FREQ

try:
    while True:
        start = time.time()
        
        # 입력 읽기
        left_input = joycon.get_left_input()
        right_input = joycon.get_right_input()
        
        # 리셋 체크
        if left_input['capture'] or right_input['home']:
            print("Reset!")
            left_arm.reset()
            right_arm.reset()
            time.sleep(0.5)
            continue
        
        # 팔 제어
        left_joints = left_arm.update(left_input)
        right_joints = right_arm.update(right_input)
        
        # 로봇에 전송
        robot.set_arm_joints(left_joints, 'left')
        robot.set_arm_joints(right_joints, 'right')
        
        # 타이밍
        elapsed = time.time() - start
        time.sleep(max(0, dt - elapsed))

except KeyboardInterrupt:
    print("\nStopping...")
finally:
    joycon.left.disconnect()
    joycon.right.disconnect()
```

---

## 6. 자이로 센서 활용

### 6.1 자이로 데이터 읽기

```python
# 자이로 데이터 (각속도, deg/s)
gyro_x, gyro_y, gyro_z = joycon_left.get_gyro_data()

# 가속도 데이터 (m/s²)
accel_x, accel_y, accel_z = joycon_left.get_accel_data()
```

### 6.2 손목 기울기로 제어

```python
class GyroArmController(JoyconArmController):
    def update_with_gyro(self, joy_input):
        """자이로로 손목 제어"""
        # 기본 제어
        joints = self.update(joy_input)
        
        # 자이로 데이터
        gyro_x, gyro_y, gyro_z = joy_input.get('gyro', (0, 0, 0))
        
        # 손목 기울기 → Wrist Roll
        # gyro_z > 0 = 시계방향 회전
        if abs(gyro_z) > 50:  # 임계값
            self.wrist_roll += gyro_z / 1000.0
        
        joints['wrist_roll'] = self.wrist_roll
        return joints
```

---

## 7. 실행

### 7.1 실행 스크립트

```bash
cd ~/XLeRobot/software
python examples/6_so100_joycon_ee_control.py
```

**출력:**
```
Connecting to Joycons...
Joy-Con (L) connected: XX:XX:XX:XX:XX:XX
Joy-Con (R) connected: YY:YY:YY:YY:YY:YY
Joycons connected successfully

Press Capture or Home to reset
Press Ctrl+C to exit

Left:  X=0.150 Y=0.100 Pan=0.0°
Right: X=0.150 Y=0.100 Pan=0.0°
```

---

## 8. 문제 해결

### 문제 1: Joycon 연결 안 됨

**원인:** Bluetooth 드라이버 이슈  
**해결:**
```bash
# Bluetooth 재시작
sudo systemctl restart bluetooth

# HID 권한 확인
sudo chmod 666 /dev/hidraw*
```

### 문제 2: 입력 지연

**원인:** Bluetooth 간섭  
**해결:**
- Wi-Fi 2.4GHz 대신 5GHz 사용
- USB Bluetooth 동글 위치 변경 (금속 물체에서 멀리)

### 문제 3: 배터리 부족

**체크:**
```python
# 배터리 레벨 확인
battery = joycon_left.get_battery_level()
print(f"Battery: {battery}%")
```

---

## 9. 전체 로봇 제어

### 9.1 베이스 제어 추가

```python
# 왼쪽 Joycon L 버튼으로 베이스 활성화
if left_input['l']:
    # 스틱으로 베이스 제어
    vx = stick_y / 100.0 * 0.2  # m/s
    vy = stick_x / 100.0 * 0.2
    robot.set_base_velocity(vx, vy, 0)
```

### 9.2 듀얼 제어 모드

```python
# Minus 버튼: 모드 전환
if left_input['minus']:
    mode = 'base' if mode == 'arm' else 'arm'
    print(f"Mode: {mode}")
```

---

## 참고 자료

- [6_so100_joycon_ee_control.py](../../software/examples/6_so100_joycon_ee_control.py)
- [7_xlerobot_teleop_joycon.py](../../software/examples/7_xlerobot_teleop_joycon.py)
- [joyconrobotics 라이브러리](../../software/joyconrobotics/)
- [Nintendo Switch Joycon 사양](https://en.wikipedia.org/wiki/Joy-Con)
