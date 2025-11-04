# 1.2 프로젝트 구조 이해하기

XLeRobot 프로젝트의 디렉토리 구조를 이해하면 필요한 파일을 빠르게 찾고 프로젝트를 효율적으로 탐색할 수 있습니다.

## 전체 디렉토리 구조

```
XLeRobot/
├── docs/                    # 문서화
├── hardware/                # 하드웨어 설계 파일
├── simulation/              # 시뮬레이션 환경
├── software/                # 제어 소프트웨어
├── web_control/             # 웹 인터페이스
├── XLeVR/                   # VR 제어 시스템
├── others/                  # 기타 리소스
├── README.md                # 프로젝트 소개
├── README_CN.md             # 중국어 README
├── LICENSE                  # 라이선스
└── CONTRIBUTING.md          # 기여 가이드
```

---

## 📁 주요 디렉토리 상세 설명

### 1. `docs/` - 문서화

프로젝트의 공식 문서가 포함된 디렉토리입니다.

```
docs/
├── en/                      # 영어 문서
│   └── source/
│       ├── conf.py          # Sphinx 설정
│       ├── index.md         # 문서 메인 페이지
│       ├── hardware/        # 하드웨어 문서
│       ├── demos/           # 데모 문서
│       └── relatedworks/    # 관련 작업
├── zh/                      # 중국어 문서
├── metadata/                # 메타데이터
│   └── robot.json          # 로봇 사양
└── requirements.txt         # 문서 빌드 의존성
```

**주요 파일**:
- `en/source/index.md`: 영어 문서 시작점
- `metadata/robot.json`: 로봇 하드웨어 사양 정의

---

### 2. `hardware/` - 하드웨어 설계

로봇의 물리적 구성 요소와 설계 파일입니다.

```
hardware/
├── readme.md                # 하드웨어 개요
├── XLeRobot_0_3_0.3mf      # 3D 프린팅 파일 (3MF)
├── step/                    # STEP 파일 (CAD)
│   ├── Drive_Motor_Mount_upgraded.step
│   ├── soft_gripper_finger.step
│   ├── RGBD_Gimbal/        # RGBD 카메라 김벌
│   └── Archiv/             # 이전 버전
└── ongoing_upgrades/        # 진행 중인 업그레이드
    └── XLeRobot 035 wheels.step
```

**중요 파일**:
- `.3mf`: 바로 3D 프린팅 가능한 파일
- `.step`: CAD 소프트웨어에서 편집 가능

---

### 3. `simulation/` - 시뮬레이션 환경

실제 하드웨어 없이 로봇을 테스트할 수 있는 시뮬레이션 환경입니다.

```
simulation/
├── mujoco/                  # MuJoCo 시뮬레이션
│   ├── xlerobot_mujoco.py  # 메인 컨트롤러
│   ├── xlerobot.xml        # 로봇 모델
│   ├── scene.xml           # 시뮬레이션 씬
│   ├── requirements.txt
│   └── assets/             # 3D 모델 및 텍스처
├── Isaac_sim/              # NVIDIA Isaac Sim
│   └── readme.md
└── Maniskill/              # ManiSkill RL 환경
    ├── run_xlerobot_sim.py
    ├── agents/             # RL 에이전트
    ├── assets/             # 리소스
    └── envs/               # 환경 정의
```

**사용 순서**:
1. **MuJoCo**: 가장 빠르고 간단 (초보자 추천)
2. **ManiSkill**: RL 학습 환경
3. **Isaac Sim**: 고급 물리 시뮬레이션

---

### 4. `software/` - 제어 소프트웨어

실제 로봇 제어 코드와 예제입니다.

```
software/
├── readme.md
├── test_yolo.py            # YOLO 테스트
├── examples/               # 제어 예제
│   ├── 0_so100_keyboard_joint_control.py
│   ├── 1_so100_keyboard_ee_control.py
│   ├── 2_dual_so100_keyboard_ee_control.py
│   ├── 3_so100_yolo_ee_control.py
│   ├── 4_xlerobot_teleop_keyboard.py
│   ├── 5_xlerobot_teleop_xbox.py
│   ├── 6_so100_joycon_ee_control.py
│   ├── 7_xlerobot_teleop_joycon.py
│   └── 8_xlerobot_teleop_vr.py
├── joyconrobotics/         # Joycon 라이브러리
│   ├── joycon.py
│   └── ...
└── src/                    # 소스 코드
    ├── robots/             # 로봇 정의
    └── model/              # 모델 파일
```

**예제 파일 설명**:
- `0_*`: 단일 팔 조인트 제어
- `1_*`: 단일 팔 End-Effector 제어
- `2_*`: 듀얼 팔 제어
- `3_*`: YOLO 비전 통합
- `4_*`: 모바일 베이스 제어
- `5-8_*`: 다양한 입력 장치

---

### 5. `web_control/` - 웹 제어 인터페이스

브라우저를 통한 원격 제어 시스템입니다.

```
web_control/
├── README.md
├── client/                 # 프론트엔드
│   ├── src/
│   ├── package.json
│   ├── vite.config.ts
│   └── public/
└── server/                 # 백엔드
    ├── main.py            # FastAPI 서버
    ├── requirements.txt
    ├── api/               # API 엔드포인트
    └── core/              # 핵심 로직
```

**기술 스택**:
- 프론트엔드: Vue.js/React + TypeScript
- 백엔드: Python FastAPI
- 통신: WebSocket

---

### 6. `XLeVR/` - VR 제어 시스템

Meta Quest 3를 이용한 VR 텔레오퍼레이션입니다.

```
XLeVR/
├── README.md
├── config.yaml             # VR 설정
├── vr_monitor.py          # 모니터링
├── requirements.txt
├── web-ui/                # VR 웹 인터페이스
│   ├── index.html
│   ├── vr_app.js
│   ├── interface.js
│   └── styles.css
└── xlevr/                 # VR 제어 라이브러리
    ├── config.py
    ├── utils.py
    └── inputs/
```

**주요 기능**:
- Quest 3 VR 헤드셋 지원
- WebRTC 실시간 비디오
- 데이터셋 레코딩

---

### 7. `others/` - 기타 리소스

추가 리소스와 참고 자료입니다.

```
others/
└── README.md              # 기타 정보
```

---

## 파일 명명 규칙

### Python 파일
- `{번호}_{로봇}_{제어방식}_{기능}.py`
- 예: `3_so100_yolo_ee_control.py`
  - `3`: 예제 순서
  - `so100`: 로봇 타입
  - `yolo`: 비전 사용
  - `ee_control`: End-Effector 제어

### 설정 파일
- `requirements.txt`: Python 의존성
- `package.json`: Node.js 의존성
- `config.yaml`: YAML 설정

---

## 디렉토리 탐색 팁

### 1. 시뮬레이션부터 시작
```bash
cd simulation/mujoco/
python xlerobot_mujoco.py
```

### 2. 예제 코드 실행
```bash
cd software/examples/
python 0_so100_keyboard_joint_control.py
```

### 3. 문서 확인
```bash
cd docs/en/source/
# 온라인: https://xlerobot.readthedocs.io
```

---

## 요약

| 디렉토리 | 용도 | 우선순위 |
|---------|------|----------|
| `docs/` | 공식 문서 | ⭐⭐⭐ |
| `simulation/` | 시뮬레이션 환경 | ⭐⭐⭐⭐⭐ |
| `software/` | 제어 코드 | ⭐⭐⭐⭐ |
| `hardware/` | 하드웨어 설계 | ⭐⭐⭐ |
| `web_control/` | 웹 인터페이스 | ⭐⭐ |
| `XLeVR/` | VR 제어 | ⭐⭐ |

**초보자 추천 경로**: `simulation/mujoco/` → `software/examples/` → `docs/`

---

[← 1.1 XLeRobot이란?](01_what_is_xlerobot.md) | [다음: 1.3 기술 스택 →](03_tech_stack.md)
