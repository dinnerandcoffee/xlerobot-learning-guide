# 4.5 로봇 팔 운동학 (IK/FK)

로봇 팔의 순기구학(FK)과 역기구학(IK)을 상세히 설명합니다.

## 1. 운동학 기초

### 1.1 순기구학 (Forward Kinematics, FK)

조인트 각도 → 엔드 이펙터 위치

```
입력: [θ1, θ2, θ3, θ4, θ5, θ6]
출력: (x, y, z, roll, pitch, yaw)
```

### 1.2 역기구학 (Inverse Kinematics, IK)

엔드 이펙터 위치 → 조인트 각도

```
입력: (x, y, z)
출력: [θ1, θ2, θ3, θ4, θ5, θ6]
```

---

## 2. SO-101 팔 구조

### 2.1 6-DOF 조인트 체인

```
Base (고정)
  ↓ Joint 1: Shoulder Pan (±180°)
[상완 베이스]
  ↓ Joint 2: Shoulder Lift (-5.7° ~ +197.5°)
[상완 링크 l1 = 0.1159m]
  ↓ Joint 3: Elbow Flex (-11.5° ~ +180°)
[하완 링크 l2 = 0.1350m]
  ↓ Joint 4: Wrist Flex (±123°)
  ↓ Joint 5: Wrist Roll (±180°)
  ↓ Joint 6: Gripper (0° ~ 60°)
[엔드 이펙터]
```

### 2.2 주요 파라미터

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| l1 | 0.1159 m | 상완 길이 (URDF) |
| l2 | 0.1350 m | 하완 길이 (URDF) |
| θ1_offset | atan2(0.028, 0.11257) | Joint2=0일 때 오프셋 |
| θ2_offset | atan2(0.0052, 0.1349) + θ1_offset | Joint3=0일 때 오프셋 |

---

## 3. 2D 평면 IK (Joint 2, 3)

### 3.1 문제 정의

XY 평면에서 2링크 로봇 팔의 IK:

```
목표: (x, y) 위치에 엔드 이펙터 배치
조인트: θ2 (shoulder_lift), θ3 (elbow_flex)
```

### 3.2 수학적 유도

**1단계: 목표까지 거리**

```
r = √(x² + y²)
```

**2단계: 코사인 법칙으로 θ3 계산**

```
r² = l1² + l2² - 2·l1·l2·cos(π - θ3)
cos(θ3) = (r² - l1² - l2²) / (2·l1·l2)
θ3 = arccos(cos(θ3))
```

**3단계: θ2 계산**

```
α = atan2(y, x)        # 목표 각도
β = atan2(l2·sin(θ3), l1 + l2·cos(θ3))  # 삼각형 내각
θ2 = α - β
```

### 3.3 Python 구현

```python
import math

def inverse_kinematics(x, y, l1=0.1159, l2=0.1350):
    """
    2링크 로봇 팔 역기구학
    
    Args:
        x: 목표 X 좌표 (m)
        y: 목표 Y 좌표 (m)
        l1: 상완 길이 (m)
        l2: 하완 길이 (m)
    
    Returns:
        joint2: shoulder_lift 각도 (rad)
        joint3: elbow_flex 각도 (rad)
    """
    # URDF 오프셋
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    
    # 목표 거리
    r = math.sqrt(x**2 + y**2)
    r_max = l1 + l2
    r_min = abs(l1 - l2)
    
    # 워크스페이스 제한
    if r > r_max:
        scale = r_max / r
        x *= scale
        y *= scale
        r = r_max
    
    if r < r_min and r > 0:
        scale = r_min / r
        x *= scale
        y *= scale
        r = r_min
    
    # θ3 계산 (코사인 법칙)
    cos_theta3 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
    cos_theta3 = max(-1, min(1, cos_theta3))  # 범위 제한
    theta3 = math.acos(cos_theta3)
    
    # θ2 계산
    alpha = math.atan2(y, x)
    beta = math.atan2(l2 * math.sin(theta3), 
                      l1 + l2 * math.cos(theta3))
    theta2 = alpha - beta
    
    # URDF 좌표계로 변환
    joint2 = theta2 - theta1_offset
    joint3 = theta3 - theta2_offset
    
    return joint2, joint3
```

---

## 4. 순기구학 (FK)

### 4.1 수학적 유도

```
주어진: θ2, θ3
구하기: (x, y)

θ1 = θ2 + θ1_offset
θ2_actual = θ3 + θ2_offset

# 상완 끝점
x1 = l1 · cos(θ1)
y1 = l1 · sin(θ1)

# 엔드 이펙터
x = x1 + l2 · cos(θ1 + θ2_actual)
y = y1 + l2 · sin(θ1 + θ2_actual)
```

### 4.2 Python 구현

```python
def forward_kinematics(joint2, joint3, l1=0.1159, l2=0.1350):
    """
    순기구학: 조인트 각도 → EE 위치
    
    Args:
        joint2: shoulder_lift (rad)
        joint3: elbow_flex (rad)
    
    Returns:
        x, y: 엔드 이펙터 좌표 (m)
    """
    # URDF 오프셋
    theta1_offset = math.atan2(0.028, 0.11257)
    theta2_offset = math.atan2(0.0052, 0.1349) + theta1_offset
    
    # 실제 각도
    theta1 = joint2 + theta1_offset
    theta2 = joint3 + theta2_offset
    
    # 상완 끝점
    x1 = l1 * math.cos(theta1)
    y1 = l1 * math.sin(theta1)
    
    # 엔드 이펙터
    x = x1 + l2 * math.cos(theta1 + theta2)
    y = y1 + l2 * math.sin(theta1 + theta2)
    
    return x, y
```

---

## 5. 워크스페이스 분석

### 5.1 도달 범위

```
최대 거리: r_max = l1 + l2 = 0.2509 m
최소 거리: r_min = |l1 - l2| = 0.0191 m

워크스페이스: 도넛 형태 (annulus)
  - 외부 반지름: 25.09 cm
  - 내부 반지름: 1.91 cm
```

### 5.2 특이점 (Singularity)

**1. 완전히 펼쳤을 때 (θ3 = 0)**
```
r = r_max
문제: IK 해가 무한개 (팔이 일직선)
```

**2. 완전히 접었을 때 (θ3 = π)**
```
r = r_min
문제: 미세한 움직임도 큰 각도 변화
```

### 5.3 워크스페이스 시각화

```python
import numpy as np
import matplotlib.pyplot as plt

def plot_workspace(l1=0.1159, l2=0.1350):
    """워크스페이스 플롯"""
    fig, ax = plt.subplots(figsize=(8, 8))
    
    # 도달 가능 영역
    theta = np.linspace(0, 2*np.pi, 100)
    r_max = l1 + l2
    r_min = abs(l1 - l2)
    
    # 외부 원
    x_outer = r_max * np.cos(theta)
    y_outer = r_max * np.sin(theta)
    ax.plot(x_outer, y_outer, 'b-', label='Max reach')
    
    # 내부 원
    x_inner = r_min * np.cos(theta)
    y_inner = r_min * np.sin(theta)
    ax.plot(x_inner, y_inner, 'r-', label='Min reach')
    
    # 샘플 포즈
    for joint2 in np.linspace(-90, 90, 10):
        for joint3 in np.linspace(0, 180, 10):
            x, y = forward_kinematics(
                math.radians(joint2), 
                math.radians(joint3)
            )
            ax.plot(x, y, 'g.', markersize=2)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('SO-101 Workspace')
    ax.axis('equal')
    ax.grid(True)
    ax.legend()
    plt.show()
```

---

## 6. IK 검증

### 6.1 FK-IK 왕복 테스트

```python
def test_ik_accuracy():
    """IK 정확도 테스트"""
    test_points = [
        (0.15, 0.10),
        (0.20, 0.05),
        (0.10, 0.15),
        (0.05, 0.20),
    ]
    
    for x_target, y_target in test_points:
        # IK 계산
        joint2, joint3 = inverse_kinematics(x_target, y_target)
        
        # FK로 검증
        x_actual, y_actual = forward_kinematics(joint2, joint3)
        
        # 오차 계산
        error = math.sqrt((x_target - x_actual)**2 + 
                         (y_target - y_actual)**2)
        
        print(f"Target: ({x_target:.3f}, {y_target:.3f})")
        print(f"Actual: ({x_actual:.3f}, {y_actual:.3f})")
        print(f"Error: {error*1000:.2f} mm")
        print(f"Joints: {math.degrees(joint2):.1f}°, "
              f"{math.degrees(joint3):.1f}°\n")
```

**예상 출력:**
```
Target: (0.150, 0.100)
Actual: (0.150, 0.100)
Error: 0.00 mm
Joints: 23.4°, 67.8°

Target: (0.200, 0.050)
Actual: (0.200, 0.050)
Error: 0.01 mm
Joints: 14.2°, 12.5°
```

---

## 7. 3D 전신 IK (고급)

### 7.1 전체 6-DOF 분해

```
Step 1: Shoulder Pan (θ1)
  - XY 평면 회전
  - θ1 = atan2(y_target, x_target)

Step 2: 2D IK (θ2, θ3)
  - 위에서 설명한 평면 IK
  - r = √(x² + y²)

Step 3: Wrist Orientation (θ4, θ5)
  - θ4 = pitch (사용자 입력)
  - θ5 = roll (사용자 입력)

Step 4: Gripper (θ6)
  - θ6 = gripper_angle
```

### 7.2 전신 IK 구현

```python
def full_ik(x, y, z, pitch=0, roll=0, gripper=0):
    """
    6-DOF 전신 IK
    
    Args:
        x, y, z: 목표 위치 (m)
        pitch: Wrist pitch (rad)
        roll: Wrist roll (rad)
        gripper: 그리퍼 각도 (deg)
    
    Returns:
        [θ1, θ2, θ3, θ4, θ5, θ6]: 모든 조인트 각도
    """
    # Step 1: Shoulder Pan
    theta1 = math.atan2(y, x)
    
    # Step 2: 2D 평면 IK
    r_xy = math.sqrt(x**2 + y**2)
    theta2, theta3 = inverse_kinematics(r_xy, z)
    
    # Step 3: Wrist
    theta4 = pitch
    theta5 = roll
    
    # Step 4: Gripper
    theta6 = gripper
    
    return [theta1, theta2, theta3, theta4, theta5, theta6]
```

---

## 8. 야코비안 (Jacobian) - 고급

### 8.1 속도 관계

```
ẋ = J(θ) · θ̇

ẋ: 엔드 이펙터 속도 (m/s)
θ̇: 조인트 속도 (rad/s)
J: 야코비안 행렬 (6×6)
```

### 8.2 야코비안 계산

```python
def compute_jacobian(joint_angles):
    """
    수치 야코비안 계산
    
    Returns:
        J: 6×6 야코비안 행렬
    """
    J = np.zeros((6, 6))
    epsilon = 1e-6
    
    # 현재 EE 위치
    x0, y0, z0 = forward_kinematics_3d(joint_angles)
    
    for i in range(6):
        # 조인트 i를 epsilon만큼 변경
        theta_plus = joint_angles.copy()
        theta_plus[i] += epsilon
        
        # FK 계산
        x, y, z = forward_kinematics_3d(theta_plus)
        
        # 편미분 근사
        J[0, i] = (x - x0) / epsilon
        J[1, i] = (y - y0) / epsilon
        J[2, i] = (z - z0) / epsilon
    
    return J
```

---

## 9. 최적화 기반 IK

### 9.1 수치 최적화

```python
from scipy.optimize import minimize

def numerical_ik(x_target, y_target, z_target, 
                 initial_guess=None):
    """
    최적화 기반 IK
    
    Args:
        x_target, y_target, z_target: 목표 위치
        initial_guess: 초기 조인트 각도
    
    Returns:
        최적 조인트 각도
    """
    if initial_guess is None:
        initial_guess = [0, 0, 0, 0, 0, 0]
    
    def objective(theta):
        """목표와 실제 위치 차이"""
        x, y, z = forward_kinematics_3d(theta)
        error = (x - x_target)**2 + \
                (y - y_target)**2 + \
                (z - z_target)**2
        return error
    
    # 조인트 제한
    bounds = [
        (-math.pi, math.pi),      # θ1
        (-0.1, 3.45),             # θ2
        (-0.2, math.pi),          # θ3
        (-2.15, 2.15),            # θ4
        (-math.pi, math.pi),      # θ5
        (0, math.radians(60)),    # θ6
    ]
    
    result = minimize(objective, initial_guess, 
                     bounds=bounds, method='SLSQP')
    
    return result.x
```

---

## 10. 참고 자료

- [DH 파라미터 설명](https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters)
- [IK 알고리즘 비교](https://robotics.stackexchange.com/questions/164/inverse-kinematics)
- [SO-101 URDF 파일](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/../hardware)
- [Python robotics 라이브러리](https://github.com/AtsushiSakai/PythonRobotics)
