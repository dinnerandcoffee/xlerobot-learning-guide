# 3.3 MuJoCo 키보드 제어

XLeRobot의 MuJoCo 시뮬레이션에서 키보드로 로봇을 직접 제어하는 방법을 학습합니다.

## 목차
- [제어 개요](#제어-개요)
- [키보드 매핑](#키보드-매핑)
- [제어 시작하기](#제어-시작하기)
- [XLeRobotController 분석](#xlerobot controller-분석)
- [고급 제어 기법](#고급-제어-기법)
- [실습 과제](#실습-과제)

---

## 제어 개요

### 제어 구조

XLeRobot은 **3가지 주요 부분**으로 구성됩니다:

```
┌─────────────────────────────────────┐
│         XLeRobot 제어 구조           │
├─────────────────────────────────────┤
│                                     │
│  1️⃣ 베이스 (Chassis)                │
│     - 3자유도 전방향 이동            │
│     - Home/End/Delete/PgDn/Ins/PgUp │
│                                     │
│  2️⃣ 왼팔 (Left Arm - SO-100)        │
│     - 6자유도 관절 (실제론 3개만)    │
│     - Q/A, W/S, E/D                 │
│                                     │
│  3️⃣ 오른팔 (Right Arm - SO-100)     │
│     - 6자유도 관절 (실제론 3개만)    │
│     - U/J, I/K, O/L                 │
│                                     │
└─────────────────────────────────────┘
```

### 제어 방식

- **증분 제어 (Incremental Control)**: 키를 누르면 조금씩 움직임
- **실시간 피드백**: 화면에 현재 상태 표시
- **부드러운 움직임**: 60Hz로 업데이트

---

## 키보드 매핑

### 📐 베이스(섀시) 제어 - 전방향 이동

| 키 | 동작 | 방향 | 설명 |
|----|------|------|------|
| **`Home`** | 전진 | +X | 로봇 앞으로 이동 |
| **`End`** | 후진 | -X | 로봇 뒤로 이동 |
| **`Delete`** | 좌이동 | +Y | 로봇 왼쪽으로 이동 |
| **`Page Down`** | 우이동 | -Y | 로봇 오른쪽으로 이동 |
| **`Insert`** | 좌회전 | +θ (CCW) | 제자리에서 반시계 회전 |
| **`Page Up`** | 우회전 | -θ (CW) | 제자리에서 시계 회전 |

#### 전방향 이동 원리

```
        전진 (Home)
           ↑
           │
좌이동 ←───┼───→ 우이동
(Delete)   │   (Page Down)
           │
           ↓
        후진 (End)

회전: Insert(↶) / Page Up(↷)
```

**옴니 휠 (Omni-directional Wheel)**:
- 3개의 메카넘 휠로 전방향 이동 가능
- 각도 제약 없이 자유롭게 이동
- 좁은 공간에서도 기동성 우수

---

### 🦾 왼팔 제어 (Q/W/E - A/S/D)

| 키 쌍 | 관절 | 증가 (+) | 감소 (-) | 스텝 크기 |
|-------|------|----------|----------|-----------|
| **Q / A** | Joint 1 | Q | A | 0.005 rad |
| **W / S** | Joint 2 | W | S | 0.005 rad |
| **E / D** | Joint 3 | E | D | 0.005 rad |

#### 관절 번호 매핑

```python
# qCmd 배열 인덱스
qCmd[3]  → 왼팔 관절 1 (어깨 회전)
qCmd[4]  → 왼팔 관절 2 (어깨 상하)
qCmd[5]  → 왼팔 관절 3 (팔꿈치)
qCmd[6:9] → 왼팔 관절 4-6 (현재 미사용, 항상 0)
```

#### 왼팔 키 레이아웃 (QWERTY 키보드)

```
  Q   W   E     ← 관절 증가 (+)
  A   S   D     ← 관절 감소 (-)
  ↑   ↑   ↑
  1   2   3     ← 관절 번호
```

---

### 🦾 오른팔 제어 (U/I/O - J/K/L)

| 키 쌍 | 관절 | 증가 (+) | 감소 (-) | 스텝 크기 |
|-------|------|----------|----------|-----------|
| **U / J** | Joint 1 | U | J | 0.005 rad |
| **I / K** | Joint 2 | I | K | 0.005 rad |
| **O / L** | Joint 3 | O | L | 0.005 rad |

#### 관절 번호 매핑

```python
# qCmd 배열 인덱스
qCmd[9]   → 오른팔 관절 1 (어깨 회전)
qCmd[10]  → 오른팔 관절 2 (어깨 상하)
qCmd[11]  → 오른팔 관절 3 (팔꿈치)
qCmd[12:15] → 오른팔 관절 4-6 (현재 미사용, 항상 0)
```

#### 오른팔 키 레이아웃

```
  U   I   O     ← 관절 증가 (+)
  J   K   L     ← 관절 감소 (-)
  ↑   ↑   ↑
  1   2   3     ← 관절 번호
```

---

## 제어 시작하기

### 1단계: 프로그램 실행

```bash
# MuJoCo 디렉토리로 이동
cd ~/XLeRobot/simulation/mujoco/

# 가상 환경 활성화
source .venv/bin/activate

# 시뮬레이션 실행
python xlerobot_mujoco.py
```

**예상 출력**:
```
Starting XLeRobot keyboard Controller...
```

3D 뷰어 창이 열립니다.

---

### 2단계: 베이스 제어 실습

> ⚠️ **중요**: 3D 뷰어 창을 마우스로 클릭해서 활성화!

#### 실습 1: 사각형 그리기

```
목표: 로봇을 정사각형 경로로 이동
```

**절차**:
1. **`Home`** 키를 5초간 연속 누름 (전진)
2. **`Delete`** 키를 5초간 누름 (좌이동)
3. **`End`** 키를 5초간 누름 (후진)
4. **`Page Down`** 키를 5초간 누름 (우이동)
5. 시작 위치로 복귀 확인

**화면 하단 피드백**:
```
command: Chassis Vel: [0.50, 0.00, 0.00]   ← Home 키 눌렀을 때
feedback: Chassis Vel: [0.48, 0.01, 0.00]  ← 실제 속도
```

#### 실습 2: 제자리 회전

```
목표: 360도 회전
```

**절차**:
1. **`Insert`** 키를 10초간 누름 (반시계 회전)
2. 로봇이 한 바퀴 돌았는지 확인
3. **`Page Up`** 키로 반대 방향 회전

**관찰 포인트**:
- 베이스는 회전하지만 팔은 고정
- 화면에서 `Chassis Vel: [0.00, 0.00, 0.52]` 확인

---

### 3단계: 왼팔 제어 실습

#### 실습 3: 관절 1 테스트 (Q/A)

```
목표: 어깨 회전 관절 이해
```

**절차**:
1. 3D 창을 왼쪽 팔이 잘 보이도록 회전 (마우스 드래그)
2. **`Q`** 키를 5회 짧게 누름 (탭 탭 탭...)
3. 왼팔이 앞쪽으로 회전하는지 관찰
4. **`A`** 키로 원래 위치로 복귀

**화면 피드백**:
```
Left Arm: [0.03, 0.00, 0.00]   ← 관절 1만 변화
```

#### 실습 4: 관절 2 테스트 (W/S)

```
목표: 어깨 상하 관절 이해
```

**절차**:
1. **`W`** 키를 누름 → 팔이 위로 올라감
2. **`S`** 키를 누름 → 팔이 아래로 내려감
3. 각 관절의 움직임을 천천히 관찰

#### 실습 5: 관절 3 테스트 (E/D)

```
목표: 팔꿈치 관절 이해
```

**절차**:
1. **`E`** 키를 누름 → 팔꿈치가 굽혀짐
2. **`D`** 키를 누름 → 팔꿈치가 펴짐
3. 팔이 접혔다 펴졌다 하는 동작 확인

---

### 4단계: 오른팔 제어 실습

#### 실습 6: 양팔 동시 제어

```
목표: 양팔을 동시에 움직여 대칭 동작 만들기
```

**절차**:
1. **`Q`** + **`U`** 동시 누름 (양쪽 관절 1 증가)
2. 양팔이 대칭으로 움직이는지 확인
3. **`A`** + **`J`** 로 복귀

**화면 피드백**:
```
Left Arm:  [0.03, 0.00, 0.00]
Right Arm: [0.03, 0.00, 0.00]   ← 대칭!
```

---

## XLeRobotController 분석

### 클래스 구조

```python
class XLeRobotController:
    def __init__(self, mjcf_path):
        """초기화: 모델 로드, 뷰어 생성, 변수 초기화"""
        
    def update_feedback(self):
        """센서에서 피드백 읽기 (현재 속도, 위치)"""
        
    def update_keyboards(self):
        """키보드 상태 읽기 (GLFW 이벤트)"""
        
    def update_reference(self):
        """키보드 입력 → 제어 명령 변환"""
        
    def update_control(self):
        """제어 명령 → MuJoCo actuator 전달"""
        
    def render_ui(self):
        """화면 렌더링 및 UI 오버레이"""
        
    def run(self):
        """메인 루프 (60Hz)"""
        
    def cleanup(self):
        """종료 처리"""
```

---

### 핵심 코드 분석

#### 1. 키보드 상태 딕셔너리

```python
# __init__ 메서드 내부
self.key_states = {
    "home": False,      # 전진
    "end": False,       # 후진
    "delete": False,    # 좌이동
    "page_down": False, # 우이동
    "insert": False,    # 좌회전
    "page_up": False,   # 우회전
    "q": False, "a": False,  # 왼팔 관절 1
    "w": False, "s": False,  # 왼팔 관절 2
    "e": False, "d": False,  # 왼팔 관절 3
    "u": False, "j": False,  # 오른팔 관절 1
    "i": False, "k": False,  # 오른팔 관절 2
    "o": False, "l": False,  # 오른팔 관절 3
}
```

**동작 원리**:
- GLFW 창에서 키 이벤트 감지
- `True`: 키가 눌림, `False`: 키가 떼어짐
- 매 프레임마다 상태 확인

---

#### 2. 키보드 입력 읽기

```python
def update_keyboards(self):
    glfw.poll_events()  # GLFW 이벤트 처리
    
    # 각 키의 상태 확인
    self.key_states["home"] = glfw.get_key(
        self.viewer.window, glfw.KEY_HOME
    ) == glfw.PRESS
    
    self.key_states["q"] = glfw.get_key(
        self.viewer.window, glfw.KEY_Q
    ) == glfw.PRESS
    
    # ... 나머지 키들도 동일
```

**GLFW 키 코드**:
- `glfw.KEY_HOME`, `glfw.KEY_END`, `glfw.KEY_DELETE` 등
- `glfw.PRESS`: 키가 눌림
- `glfw.RELEASE`: 키가 떼어짐

---

#### 3. 베이스 제어 명령 계산

```python
def update_reference(self):
    # 현재 로봇 방향 (yaw angle)
    yaw = self.qFb[2]
    
    # 회전 변환 행렬
    rotmz = np.array([
        [np.cos(yaw), np.sin(yaw), 0],
        [-np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1],
    ])
    
    # 목표 속도 계산
    self.chassis_ref_vel = np.zeros(3)
    if self.key_states["home"]:
        self.chassis_ref_vel[0] = self.abs_vel[0]  # +x
    elif self.key_states["end"]:
        self.chassis_ref_vel[0] = -self.abs_vel[0]  # -x
    
    if self.key_states["delete"]:
        self.chassis_ref_vel[1] = self.abs_vel[1]  # +y
    elif self.key_states["page_down"]:
        self.chassis_ref_vel[1] = -self.abs_vel[1]  # -y
    
    if self.key_states["insert"]:
        self.chassis_ref_vel[2] = self.abs_vel[2]  # +θ (CCW)
    elif self.key_states["page_up"]:
        self.chassis_ref_vel[2] = -self.abs_vel[2]  # -θ (CW)
    
    # PD 제어로 부드러운 움직임
    k_p = 10
    k_p_rot = 100
    self.qdCmd[0] = ... # 복잡한 계산 (생략)
```

**제어 로직**:
1. **키 상태 확인**: `if self.key_states["home"]:`
2. **목표 속도 설정**: `chassis_ref_vel[0] = 0.5 m/s`
3. **PD 제어**: 목표 속도와 실제 속도 차이를 줄임
4. **옴니휠 변환**: 3D 속도 → 3개 휠 속도

---

#### 4. 팔 제어 명령 계산

```python
def update_reference(self):
    # ... 베이스 제어 이후
    
    arm_step = 0.005  # 라디안 단위 증분
    
    # 왼팔 관절 1 (Q/A)
    if self.key_states["q"]:
        self.qCmd[3] += arm_step   # +0.005 rad
    elif self.key_states["a"]:
        self.qCmd[3] -= arm_step   # -0.005 rad
    
    # 왼팔 관절 2 (W/S)
    if self.key_states["w"]:
        self.qCmd[4] += arm_step
    elif self.key_states["s"]:
        self.qCmd[4] -= arm_step
    
    # 왼팔 관절 3 (E/D)
    if self.key_states["e"]:
        self.qCmd[5] += arm_step
    elif self.key_states["d"]:
        self.qCmd[5] -= arm_step
    
    # 오른팔도 동일 패턴 (U/J, I/K, O/L)
    # ... (코드 생략)
    
    # 미사용 관절은 0으로 고정
    self.qCmd[6:9] = 0.0    # 왼팔 관절 4-6
    self.qCmd[12:15] = 0.0  # 오른팔 관절 4-6
```

**제어 방식**:
- **위치 제어**: 현재 관절 각도에 `arm_step` 더하거나 빼기
- **증분 제어**: 절대 위치가 아닌 상대 증분
- **클리핑 없음**: 관절 한계는 MuJoCo가 자동 처리

---

#### 5. MuJoCo actuator에 명령 전달

```python
def update_control(self):
    # 베이스 속도 명령에 게인 적용
    self.qdCmd[0:3] = self.kp * self.qdCmd[0:3]
    
    # MuJoCo data.ctrl에 기록
    self.data.ctrl[:3] = self.qdCmd[:3]    # 베이스 (x, y, θ)
    self.data.ctrl[3:] = self.qCmd[3:]     # 팔 관절들
```

**`data.ctrl` 배열 구조**:
```python
# 인덱스: 의미
0-2:   베이스 속도 (vx, vy, ωz)
3-8:   왼팔 6관절 위치
9-14:  오른팔 6관절 위치
15-17: 옴니휠 3개 속도 (자동 계산됨)
```

---

#### 6. 메인 루프 (60Hz)

```python
def run(self):
    print("Starting XLeRobot keyboard Controller...")
    
    while self.viewer.is_alive:  # 창이 열려있는 동안
        self.update_feedback()    # 1. 센서 읽기
        self.update_keyboards()   # 2. 키보드 읽기
        self.update_reference()   # 3. 제어 명령 계산
        self.update_control()     # 4. actuator에 전달
        
        mujoco.mj_step(self.model, self.data)  # 5. 물리 시뮬레이션
        self.render_ui()          # 6. 화면 렌더링
        
        time.sleep(0.002)  # 7. 2ms 대기 (→ ~500Hz)
    
    self.cleanup()
```

**실행 흐름**:
```
┌──────────────────────────┐
│ 1. update_feedback()     │  센서 데이터 읽기
├──────────────────────────┤
│ 2. update_keyboards()    │  Q 키 눌림 감지
├──────────────────────────┤
│ 3. update_reference()    │  qCmd[3] += 0.005
├──────────────────────────┤
│ 4. update_control()      │  data.ctrl[3] = qCmd[3]
├──────────────────────────┤
│ 5. mj_step()             │  물리 계산 (충돌, 중력)
├──────────────────────────┤
│ 6. render_ui()           │  화면 업데이트 (60Hz)
├──────────────────────────┤
│ 7. sleep(0.002)          │  2ms 대기
└──────────────────────────┘
         ↓
    루프 반복 (500Hz)
```

**주파수 차이**:
- **제어 루프**: ~500Hz (sleep 0.002초)
- **렌더링**: 60Hz (render_interval로 제한)
- **물리 시뮬레이션**: 500Hz (매 루프마다)

---

## 고급 제어 기법

### 1. 부드러운 움직임 (Smooth Control)

현재 코드는 키를 누르면 즉시 속도 변화 → 뚝뚝 끊김

**개선안: 가속도 제한**

```python
# update_reference() 메서드 수정
max_accel = 0.1  # m/s^2

# 목표 속도와 현재 속도 차이 계산
vel_error = self.chassis_ref_vel[0] - chassis_vel[0]

# 가속도 제한 적용
if abs(vel_error) > max_accel * dt:
    vel_change = max_accel * dt * np.sign(vel_error)
else:
    vel_change = vel_error

self.qdCmd[0] += vel_change
```

---

### 2. 관절 한계 확인

현재는 MuJoCo가 자동으로 관절 한계를 처리하지만, 직접 체크도 가능:

```python
# xlerobot.xml에서 관절 한계 확인
# <joint name="left_arm_joint1" range="-3.14 3.14"/>

# update_reference()에서 클리핑
joint_limits = {
    3: (-3.14, 3.14),  # 왼팔 관절 1
    4: (-1.57, 1.57),  # 왼팔 관절 2
    # ... 나머지 관절
}

for joint_idx, (min_val, max_val) in joint_limits.items():
    self.qCmd[joint_idx] = np.clip(
        self.qCmd[joint_idx], min_val, max_val
    )
```

---

### 3. 화면에 키 상태 표시

```python
def render_ui(self):
    # 기존 코드에 추가
    active_keys = [k for k, v in self.key_states.items() if v]
    
    self.viewer._overlay[mujoco.mjtGridPos.mjGRID_TOPRIGHT] = [
        f"Active Keys: {', '.join(active_keys) if active_keys else 'None'}",
        "",
    ]
```

**예상 화면**:
```
Active Keys: q, home     ← 동시 입력 확인
```

---

### 4. 키보드 단축키 추가

```python
# update_keyboards()에서 특수 키 추가
self.key_states["r"] = glfw.get_key(
    self.viewer.window, glfw.KEY_R
) == glfw.PRESS

# update_reference()에서 리셋 기능
if self.key_states["r"]:
    # 모든 관절을 초기 위치로
    self.qCmd[:] = self.initial_qCmd.copy()
    print("Position reset!")
```

---

## 실습 과제

### 과제 1: 기본 동작 마스터 ⭐

**목표**: 모든 제어 키를 사용해보기

**체크리스트**:
- [ ] 베이스 전진/후진 (Home/End)
- [ ] 베이스 좌우 이동 (Delete/PgDown)
- [ ] 베이스 회전 (Insert/PgUp)
- [ ] 왼팔 3관절 움직임 (Q/A, W/S, E/D)
- [ ] 오른팔 3관절 움직임 (U/J, I/K, O/L)

**성공 기준**: 각 키의 동작을 정확히 설명할 수 있음

---

### 과제 2: 경로 추종 ⭐⭐

**목표**: 베이스를 사각형 경로로 이동

**절차**:
1. 시작 위치 스크린샷
2. 정사각형 (한 변 = 5초 이동) 그리기
3. 종료 위치가 시작 위치와 일치하는지 확인

**힌트**: 각 변마다 같은 시간 동안 키를 누르세요.

---

### 과제 3: 양팔 동기화 ⭐⭐⭐

**목표**: 양팔을 대칭으로 움직이기

**절차**:
1. 양팔을 초기 자세로 (모든 관절 0)
2. Q + U 동시 누름 (양쪽 관절 1)
3. W + I 동시 누름 (양쪽 관절 2)
4. E + O 동시 누름 (양쪽 관절 3)
5. 화면에서 `Left Arm`과 `Right Arm` 값 비교

**성공 기준**: 
```
Left Arm:  [0.03, 0.02, 0.05]
Right Arm: [0.03, 0.02, 0.05]  ← 완전히 동일!
```

---

### 과제 4: 코드 수정 - 스텝 크기 조정 ⭐⭐⭐⭐

**목표**: `arm_step`을 수정해서 더 빠른 움직임 만들기

**절차**:
1. `xlerobot_mujoco.py` 파일 열기
2. 156번째 줄 찾기:
   ```python
   arm_step = 0.005  # 현재 값
   ```
3. `0.01`로 변경 (2배 빠르게)
4. 저장 후 재실행
5. Q 키 눌렀을 때 움직임 속도 비교

**질문**:
- 너무 큰 값 (예: 0.1)을 사용하면 어떻게 되나요?
- 어떤 값이 가장 자연스러운가요?

---

### 과제 5: 새 키 추가 - 리셋 기능 ⭐⭐⭐⭐⭐

**목표**: `R` 키로 모든 관절을 초기 위치로 되돌리기

**절차**:

1. **`__init__` 메서드에 초기 위치 저장**:
   ```python
   # 약 40번째 줄 근처에 추가
   self.initial_qCmd = self.qCmd.copy()
   ```

2. **`update_keyboards`에 R 키 추가**:
   ```python
   # 약 95번째 줄 근처에 추가
   self.key_states["r"] = glfw.get_key(
       self.viewer.window, glfw.KEY_R
   ) == glfw.PRESS
   ```

3. **`update_reference`에 리셋 로직 추가**:
   ```python
   # 약 190번째 줄 (왼팔 제어 이후) 추가
   # Reset functionality
   if self.key_states["r"]:
       self.qCmd[:] = self.initial_qCmd.copy()
       print("Reset to initial position!")
   ```

4. **테스트**:
   - Q/W/E로 왼팔 움직이기
   - `R` 키 누르기
   - 팔이 초기 위치로 돌아가는지 확인

**성공 기준**: R 키를 누르면 콘솔에 "Reset to initial position!" 출력되고 로봇이 초기 자세로 복귀

---

## 요약

### 핵심 개념

1. **3개 제어 그룹**: 베이스(6키) + 왼팔(6키) + 오른팔(6키)
2. **증분 제어**: 키를 누를 때마다 `+0.005 rad` 또는 `-0.005 rad`
3. **실시간 피드백**: 화면 하단에 현재 상태 표시
4. **60Hz 렌더링**: 부드러운 시각화

### 제어 흐름

```
키보드 입력 → key_states 업데이트 
            → 제어 명령 계산 (qCmd, qdCmd)
            → MuJoCo actuator에 전달
            → 물리 시뮬레이션
            → 화면 렌더링
```

### 다음 단계

- [3.4 Isaac Sim 설정 →](04_isaac_sim.md): 사실적인 렌더링
- [3.5 ManiSkill 환경 →](05_maniskill.md): 강화학습 훈련
- [3.6 URDF/MJCF 모델 →](06_robot_models.md): 로봇 모델 이해

---

[← 3.2 MuJoCo 설정](02_mujoco_setup.md) | [3장 목차](README.md) | [다음: 3.4 Isaac Sim →](04_isaac_sim.md)
