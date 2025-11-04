# 3.2 MuJoCo 시뮬레이션 시작하기

MuJoCo는 XLeRobot 학습의 첫 단계로, 가장 빠르고 쉽게 시작할 수 있는 시뮬레이터입니다.

## 목차
- [시스템 요구사항](#시스템-요구사항)
- [설치 과정](#설치-과정)
- [첫 실행](#첫-실행)
- [화면 구성](#화면-구성)
- [문제 해결](#문제-해결)

---

## 시스템 요구사항

### 최소 사양
- **OS**: Ubuntu 20.04+ / Windows 10+ / macOS 10.15+
- **Python**: 3.8 이상
- **RAM**: 4GB
- **디스크**: 500MB
- **GPU**: 불필요 (CPU만으로 충분)

### 권장 사양
- **OS**: Ubuntu 22.04
- **Python**: 3.10+
- **RAM**: 8GB
- **CPU**: 4코어 이상
- **디스플레이**: 1920x1080

### 검증 방법

```bash
# Python 버전 확인
python3 --version
# Python 3.8.10 이상이어야 함

# pip 확인
pip3 --version

# 디스크 공간 확인
df -h ~/XLeRobot/simulation/mujoco/
```

---

## 설치 과정

### 1단계: 저장소 클론 (이미 완료된 경우 건너뛰기)

```bash
cd ~
git clone https://github.com/huggingface/xlerobot.git XLeRobot
cd XLeRobot/simulation/mujoco/
```

---

### 2단계: Python 가상 환경 생성

가상 환경을 사용하는 이유:
- ✅ 프로젝트 간 패키지 충돌 방지
- ✅ 깨끗한 개발 환경
- ✅ 의존성 관리 용이

#### Ubuntu/macOS

```bash
# mujoco 디렉토리로 이동
cd ~/XLeRobot/simulation/mujoco/

# 가상 환경 생성
python3 -m venv .venv

# 가상 환경 활성화
source .venv/bin/activate

# 활성화 확인 (프롬프트에 (.venv) 표시됨)
which python
# /home/사용자명/XLeRobot/simulation/mujoco/.venv/bin/python
```

#### Windows (PowerShell)

```powershell
# mujoco 디렉토리로 이동
cd ~\XLeRobot\simulation\mujoco\

# 가상 환경 생성
python -m venv .venv

# 가상 환경 활성화
.\.venv\Scripts\Activate.ps1

# PowerShell 실행 정책 오류 시
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

---

### 3단계: 의존성 설치

#### requirements.txt 내용 확인

```txt
mujoco==3.3.0
mujoco-python-viewer==0.1.4
glfw
numpy
```

#### 패키지 설치

```bash
# 가상 환경 활성화 상태에서
pip install -r requirements.txt

# 또는 개별 설치
pip install mujoco==3.3.0
pip install mujoco-python-viewer==0.1.4
pip install glfw
pip install numpy
```

#### 설치 시간
- 일반적으로 1-2분
- 인터넷 속도에 따라 달라짐

#### 설치 검증

```bash
# Python에서 import 테스트
python -c "import mujoco; print(mujoco.__version__)"
# 3.3.0 출력되어야 함

python -c "import mujoco_viewer"
# 에러 없으면 성공

python -c "import glfw; print(glfw.init())"
# True 출력되어야 함
```

---

## 첫 실행

### 1단계: 스크립트 실행

```bash
# 가상 환경 활성화 확인
source .venv/bin/activate  # Linux/Mac
# .\.venv\Scripts\Activate.ps1  # Windows

# MuJoCo 시뮬레이션 실행
python xlerobot_mujoco.py
```

### 2단계: 정상 실행 확인

**예상 출력**:
```
MuJoCo version: 3.3.0
Loading model from: xlerobot.xml
Model loaded successfully
Initializing viewer...
Viewer initialized
Starting simulation loop (60 FPS)...
```

**3D 뷰어 창**이 열리고 로봇이 표시되어야 합니다.

---

## 화면 구성

### 3D 뷰어 창

```
┌─────────────────────────────────────────┐
│  MuJoCo Viewer                    [_][□][X]│
├─────────────────────────────────────────┤
│                                         │
│           [로봇 3D 모델]                │
│              ┌─┐                        │
│            ┌─┤●├─┐  ← 로봇 헤드          │
│          ┌─┴─┴─┴─┴─┐                   │
│     왼팔 │  □  □  │ 오른팔              │
│        └───┘  └───┘                    │
│           ┌───┐                        │
│           │   │ ← 베이스(바퀴)          │
│           └───┘                        │
│                                         │
│  [바닥 그리드]                           │
└─────────────────────────────────────────┘
```

### 뷰어 컨트롤 (마우스)

| 동작 | 기능 |
|------|------|
| **좌클릭 + 드래그** | 카메라 회전 (orbit) |
| **우클릭 + 드래그** | 카메라 이동 (pan) |
| **스크롤 휠** | 줌 인/아웃 |
| **더블클릭** | 카메라 리셋 |

### 키보드 제어 (뷰어 창 활성화 필수!)

> ⚠️ **중요**: 터미널이 아닌 **3D 뷰어 창**을 클릭해야 키보드 입력이 작동합니다!

#### 베이스(섀시) 제어

| 키 | 동작 |
|----|------|
| `Home` | 전진 (+x) |
| `End` | 후진 (-x) |
| `Delete` | 좌이동 (+y) |
| `Page Down` | 우이동 (-y) |
| `Insert` | 좌회전 (+θ) |
| `Page Up` | 우회전 (-θ) |

#### 왼팔 제어 (6DOF)

| 키 | 관절 | 동작 |
|----|------|------|
| `Q` | 관절 1 | 증가 |
| `A` | 관절 1 | 감소 |
| `W` | 관절 2 | 증가 |
| `S` | 관절 2 | 감소 |
| `E` | 관절 3 | 증가 |
| `D` | 관절 3 | 감소 |

#### 오른팔 제어 (6DOF)

| 키 | 관절 | 동작 |
|----|------|------|
| `U` | 관절 1 | 증가 |
| `J` | 관절 1 | 감소 |
| `I` | 관절 2 | 증가 |
| `K` | 관절 2 | 감소 |
| `O` | 관절 3 | 증가 |
| `L` | 관절 3 | 감소 |

### 제어 팁

1. **한 번에 한 키씩**: 여러 키를 동시에 누르면 예측하기 어려움
2. **천천히 테스트**: 각 관절의 움직임을 확인
3. **창 포커스 확인**: 키가 안 먹히면 3D 창을 다시 클릭
4. **리셋 방법**: 프로그램을 재시작하면 초기 자세로 돌아감

---

## 실행 흐름 이해

### 프로그램 구조

```python
# xlerobot_mujoco.py 간략 구조

1. MuJoCo 모델 로드 (xlerobot.xml)
   ├─ MJCF 파싱
   └─ 물리 시뮬레이션 초기화

2. 뷰어 초기화
   ├─ GLFW 윈도우 생성
   └─ 렌더링 컨텍스트 설정

3. 제어기 생성 (XLeRobotController)
   ├─ 키보드 상태 딕셔너리
   └─ 관절 제어 로직

4. 메인 루프 (60Hz)
   ├─ 키보드 입력 읽기
   ├─ 제어 신호 계산
   ├─ 물리 시뮬레이션 스텝
   ├─ 렌더링
   └─ 루프 반복

5. 종료 처리
   ├─ 뷰어 닫기
   └─ 리소스 해제
```

### 렌더링 루프 (60 FPS)

```
시작
  ↓
┌─────────────────────┐
│ 키보드 이벤트 체크    │ ← 3D 창에서 키 입력
├─────────────────────┤
│ 제어 신호 계산       │   Q 누름 → 왼팔 관절1 +0.01 rad
├─────────────────────┤
│ MuJoCo 시뮬레이션    │   물리 엔진 계산 (충돌, 중력 등)
│ mj_step()           │
├─────────────────────┤
│ 화면 렌더링          │   업데이트된 로봇 자세 표시
│ viewer.render()     │
├─────────────────────┤
│ 60Hz 타이밍 대기     │   ~16.67ms 대기
└─────────────────────┘
  ↓
  루프 (창 닫을 때까지)
```

---

## 문제 해결

### 문제 1: `ModuleNotFoundError: No module named 'mujoco'`

**원인**: 가상 환경이 활성화되지 않았거나 패키지 미설치

**해결**:
```bash
# 1. 가상 환경 활성화 확인
source .venv/bin/activate

# 2. 패키지 재설치
pip install mujoco==3.3.0
```

---

### 문제 2: `GLFW error: X11: Failed to open display`

**원인**: WSL 또는 SSH 환경에서 디스플레이 미설정

**해결 (WSL2)**:
```bash
# Windows에서 VcXsrv 또는 X410 설치 후
export DISPLAY=:0
python xlerobot_mujoco.py
```

**해결 (SSH)**:
```bash
# X11 포워딩 활성화
ssh -X 사용자명@서버주소
```

---

### 문제 3: `Error loading model from xlerobot.xml`

**원인**: 현재 디렉토리가 잘못됨 (xlerobot.xml 파일 없음)

**해결**:
```bash
# 올바른 디렉토리로 이동
cd ~/XLeRobot/simulation/mujoco/

# 파일 존재 확인
ls xlerobot.xml
# xlerobot.xml이 보여야 함

# 실행
python xlerobot_mujoco.py
```

---

### 문제 4: 키보드가 작동하지 않음

**원인**: 터미널에 입력 중 (3D 창 포커스 필요)

**해결**:
1. **3D 뷰어 창을 마우스로 클릭**
2. 창 테두리가 활성화되었는지 확인
3. 키보드 입력 재시도

**테스트 방법**:
```
1. 3D 창 클릭
2. Q 키 누름
3. 왼팔이 움직이는지 확인
```

---

### 문제 5: 프레임레이트가 느림 (< 30 FPS)

**원인**: CPU 성능 부족 또는 과도한 백그라운드 프로세스

**해결**:
```bash
# 1. 다른 프로그램 종료 (브라우저, IDE 등)

# 2. 프로세스 확인
htop  # Ctrl+C로 종료

# 3. 뷰어 옵션 조정 (코드 수정)
# xlerobot_mujoco.py에서:
viewer._render_every_frame = False  # 렌더링 빈도 감소
```

---

### 문제 6: `ImportError: libGL.so.1: cannot open shared object file`

**원인**: OpenGL 라이브러리 누락 (Ubuntu)

**해결**:
```bash
sudo apt update
sudo apt install -y libgl1-mesa-glx libglu1-mesa
```

---

### 문제 7: macOS에서 `zsh: killed python xlerobot_mujoco.py`

**원인**: Rosetta 2 번역 문제 (M1/M2 Mac)

**해결**:
```bash
# ARM 네이티브 Python 사용
which python3
# /opt/homebrew/bin/python3 (ARM) 확인

# Intel Python을 사용 중이면:
arch -arm64 brew install python@3.10
```

---

## 다음 단계 체크리스트

실행에 성공했다면:
- [ ] 3D 창에 로봇이 표시됨
- [ ] 마우스로 카메라 회전 가능
- [ ] `Q` 키로 왼팔 움직임 확인
- [ ] `Home` 키로 베이스 전진 확인
- [ ] 60 FPS로 부드럽게 실행

모두 체크되었다면 [3.3 MuJoCo 키보드 제어 →](03_mujoco_control.md)로 이동!

---

## 추가 자료

### MuJoCo 공식 문서
- [MuJoCo 공식 사이트](https://mujoco.org/)
- [Python 바인딩 가이드](https://mujoco.readthedocs.io/en/stable/python.html)
- [MJCF 모델 형식](https://mujoco.readthedocs.io/en/stable/XMLreference.html)

### XLeRobot MuJoCo 관련
- `simulation/mujoco/README.md`: 원본 설명서
- `simulation/mujoco/xlerobot.xml`: 로봇 모델 정의
- `simulation/mujoco/scene.xml`: 환경 씬 설정

### 커뮤니티
- [MuJoCo GitHub Discussions](https://github.com/google-deepmind/mujoco/discussions)
- [XLeRobot Discord](https://discord.gg/xlerobot) (가상)

---

[← 3.1 시뮬레이션 개요](01_simulation_overview.md) | [3장 목차](README.md) | [다음: 3.3 키보드 제어 →](03_mujoco_control.md)
