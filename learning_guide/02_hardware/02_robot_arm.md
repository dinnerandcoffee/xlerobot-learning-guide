# 2.2 로봇 팔 구조 (SO-100/SO-101)

XLeRobot의 핵심인 SO-100/SO-101 로봇 팔은 6자유도(6-DOF)를 가진 시리얼 매니퓰레이터입니다.

## 1. SO-100 vs SO-101 비교

| 항목 | SO-100 | SO-101 |
|------|--------|--------|
| 출시일 | 2024년 초 | 2025년 4월 |
| 조립 시간 | ~2시간 (2개) | ~67분 (2개) |
| 전선 관리 | 복잡 | 개선 |
| DOF | 6 | 6 |
| 펌웨어 호환 | ✓ | ✓ |

**권장:** 신규 제작 시 SO-101 선택 (조립 시간 40% 단축)

---

## 2. 6자유도 관절 구조

```
베이스
  ↓
① Shoulder Pan (어깨 회전) - 수평 360°
  ↓  
② Shoulder Lift (어깨 피치) - 상하 203°
  ↓
③ Elbow Flex (팔꿈치) - 굽힘 192°
  ↓
④ Wrist Flex (손목 피치) - 360°
  ↓
⑤ Wrist Roll (손목 롤) - 360°
  ↓
⑥ Gripper (그리퍼) - 개폐 70°
```

### 관절 스펙

| Joint | 가동 범위 | 속도 | 토크 | 역할 |
|-------|----------|------|------|------|
| 1. Pan | -180°~+180° | 114°/s | 30kg·cm | 수평 회전 |
| 2. Lift | -5.7°~+197.5° | 114°/s | 30kg·cm | IK 주요 |
| 3. Elbow | -11.5°~+180° | 114°/s | 30kg·cm | IK 주요 |
| 4. Wrist | -180°~+180° | 114°/s | 30kg·cm | 방향 조정 |
| 5. Roll | -180°~+180° | 114°/s | 30kg·cm | 회전 |
| 6. Grip | 0°~+70° | 114°/s | 30kg·cm | 파지 |

---

## 3. 기구학 (Kinematics)

### 링크 길이

```python
l1 = 0.1159 m  # 상완 (Joint2~Joint3)
l2 = 0.1350 m  # 하완 (Joint3~End)
```

**작업 공간:**
- 최대 도달: 25.09cm
- 최소 도달: 1.91cm  
- 권장 영역: x=10~20cm, y=5~15cm

### 역기구학 예시

```python
from SO101Robot import SO101Kinematics

kin = SO101Kinematics()
# 목표: (15cm, 10cm)
j2, j3 = kin.inverse_kinematics(0.15, 0.10)
print(f"Shoulder: {j2:.1f}°, Elbow: {j3:.1f}°")
```

---

## 4. 서보 모터

### STS3215 스펙

- **모델**: Feetech STS3215
- **전압**: 12V
- **토크**: 30 kg·cm
- **통신**: TTL 시리얼 (5264 커넥터)
- **분해능**: 4096 (0.088°/step)

### 서보 ID 설정

**왼쪽 팔:** ID 1~6  
**오른쪽 팔:** ID 7~12

---

## 5. 그리퍼 시스템

### 하드 그리퍼 (기본)

- 재질: PLA/PETG
- 파지력: ~5N (500gf)
- 파지 범위: 10~80mm, ~200g

### 소프트 그리퍼 (TPU95A)

- 재질: TPU95A
- 파지력: ~3N (섬세함)
- 장점: 불규칙 형상, 깨지기 쉬운 물체

**프린팅 설정:**
- 노즐: 220-230°C
- 베드: 60°C
- 속도: 20-30mm/s
- 인필: 20-30%

---

## 6. 제어 모드

### 관절 공간 제어

```python
target = {
    "shoulder_pan": 30.0,
    "shoulder_lift": 45.0,
    "elbow_flex": -60.0,
    "gripper": 70.0
}
```

### 작업 공간 제어

```python
x, y = 0.16, 0.11  # 목표 위치
j2, j3 = kin.inverse_kinematics(x, y)
```

---

## 참고 자료

- [SO-101 GitHub](https://github.com/TheRobotStudio/SO-ARM100)
- [XLeRobot SO101Robot.py](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/../software/src/model/SO101Robot.py)
- [Feetech STS3215 데이터시트](https://www.feetechrc.com/)
