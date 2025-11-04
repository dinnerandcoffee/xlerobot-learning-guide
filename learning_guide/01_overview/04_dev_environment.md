# 1.4 개발 환경 설정

XLeRobot 개발을 시작하기 위한 환경 설정 가이드입니다.

## 시스템 준비

### 운영체제 확인

XLeRobot은 Linux 환경에서 가장 잘 작동합니다.

**권장 환경**:
- Ubuntu 22.04 LTS (가장 추천)
- Ubuntu 20.04 LTS
- Debian 11+
- WSL2 (Windows)

```bash
# Ubuntu 버전 확인
lsb_release -a

# 커널 버전 확인
uname -r
```

---

## 1단계: 시스템 패키지 설치

### Ubuntu/Debian

```bash
# 시스템 업데이트
sudo apt update
sudo apt upgrade -y

# 기본 개발 도구
sudo apt install -y \
    build-essential \
    git \
    wget \
    curl \
    vim

# Python 개발 도구
sudo apt install -y \
    python3 \
    python3-pip \
    python3-dev \
    python3-venv

# 하드웨어 인터페이스
sudo apt install -y \
    libusb-1.0-0-dev \
    libudev-dev \
    libhidapi-dev \
    libhidapi-libusb0

# OpenCV 의존성
sudo apt install -y \
    libopencv-dev \
    python3-opencv \
    libgl1-mesa-glx \
    libglib2.0-0

# 그래픽 라이브러리
sudo apt install -y \
    libglfw3 \
    libglfw3-dev \
    libosmesa6-dev \
    patchelf
```

---

## 2단계: Python 환경 설정

### Python 버전 확인

```bash
# Python 버전 확인 (3.8 이상 필요)
python3 --version

# pip 버전 확인
pip3 --version

# pip 업그레이드
pip3 install --upgrade pip
```

### 가상환경 생성 (권장)

```bash
# 프로젝트 디렉토리로 이동
cd ~/XLeRobot

# 가상환경 생성
python3 -m venv .venv

# 가상환경 활성화
source .venv/bin/activate

# 비활성화 (필요시)
# deactivate
```

**가상환경 사용 이유**:
- ✅ 프로젝트별 의존성 격리
- ✅ 시스템 Python과 충돌 방지
- ✅ 깔끔한 개발 환경

---

## 3단계: 프로젝트 클론 및 설정

### Git 저장소 클론

```bash
# 홈 디렉토리로 이동
cd ~

# 저장소 클론
git clone https://github.com/Vector-Wangel/XLeRobot.git

# 프로젝트 디렉토리 진입
cd XLeRobot

# 파일 확인
ls -la
```

### 의존성 설치

#### 시뮬레이션 환경 (MuJoCo)

```bash
# MuJoCo 디렉토리로 이동
cd simulation/mujoco/

# 가상환경 생성 및 활성화
python3 -m venv .venv
source .venv/bin/activate

# 의존성 설치
pip install -r requirements.txt

# 설치 확인
python -c "import mujoco; print(mujoco.__version__)"
```

#### 소프트웨어 제어

```bash
# 소프트웨어 디렉토리로 이동
cd ~/XLeRobot/software/

# 의존성 설치
pip install -r requirements.txt

# YOLO 설치 (비전 작업용)
pip install ultralytics
```

---

## 4단계: 개발 도구 설치

### VS Code (권장)

```bash
# VS Code 설치 (Ubuntu)
wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > packages.microsoft.gpg
sudo install -D -o root -g root -m 644 packages.microsoft.gpg /etc/apt/keyrings/packages.microsoft.gpg
sudo sh -c 'echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/keyrings/packages.microsoft.gpg] https://packages.microsoft.com/repos/code stable main" > /etc/apt/sources.list.d/vscode.list'
sudo apt update
sudo apt install code

# VS Code 실행
code .
```

**추천 VS Code 확장**:
- Python (Microsoft)
- Pylance
- Jupyter
- GitLens
- YAML

### 또는 PyCharm

```bash
# Snap으로 설치
sudo snap install pycharm-community --classic
```

---

## 5단계: 테스트 실행

### MuJoCo 시뮬레이션 테스트

```bash
# MuJoCo 디렉토리로 이동
cd ~/XLeRobot/simulation/mujoco/

# 가상환경 활성화
source .venv/bin/activate

# 시뮬레이션 실행
python xlerobot_mujoco.py
```

**기대 결과**:
- 3D 뷰어 창이 열림
- XLeRobot이 표시됨
- 키보드 제어 가능

**키 테스트**:
- `Home`: 전진
- `Q`/`A`: 왼팔 관절 1
- `U`/`J`: 오른팔 관절 1

### 간단한 Python 테스트

```bash
# Python 인터프리터 실행
python3

# 테스트 코드
>>> import numpy as np
>>> import cv2
>>> import mujoco
>>> print("모든 패키지 정상 작동!")
>>> exit()
```

---

## 6단계: 웹 제어 설정 (선택)

### Node.js 설치

```bash
# NodeSource 저장소 추가
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -

# Node.js 설치
sudo apt install -y nodejs

# 버전 확인
node --version
npm --version
```

### 웹 클라이언트 설치

```bash
# 클라이언트 디렉토리로 이동
cd ~/XLeRobot/web_control/client/

# 의존성 설치
npm install

# 개발 서버 실행
npm run dev
```

### 웹 서버 설치

```bash
# 서버 디렉토리로 이동
cd ~/XLeRobot/web_control/server/

# Python 의존성 설치
pip install -r requirements.txt

# 서버 실행
python main.py
```

---

## 7단계: VR 설정 (선택)

### VR 요구사항
- Meta Quest 3
- WiFi 6 또는 유선 연결
- NVIDIA GPU (RTX 3060 이상)

```bash
# VR 디렉토리로 이동
cd ~/XLeRobot/XLeVR/

# 의존성 설치
pip install -r requirements.txt

# 설정 파일 확인
cat config.yaml
```

---

## 트러블슈팅

### 문제 1: `mujoco` 설치 실패

```bash
# OpenGL 라이브러리 설치
sudo apt install -y libgl1-mesa-glx libglew-dev

# 다시 시도
pip install mujoco
```

### 문제 2: 권한 오류

```bash
# USB 장치 권한 설정
sudo usermod -aG dialout $USER
sudo usermod -aG plugdev $USER

# 로그아웃 후 다시 로그인
```

### 문제 3: Python 버전 문제

```bash
# pyenv 설치 (Python 버전 관리)
curl https://pyenv.run | bash

# .bashrc에 추가
echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
echo 'eval "$(pyenv init -)"' >> ~/.bashrc

# 새로운 터미널 열기
# Python 3.10 설치
pyenv install 3.10.12
pyenv global 3.10.12
```

### 문제 4: GLFW 오류

```bash
# GLFW3 재설치
sudo apt remove libglfw3
sudo apt install libglfw3 libglfw3-dev
```

---

## 환경변수 설정

### `.bashrc` 설정 추가

```bash
# .bashrc 편집
nano ~/.bashrc

# 다음 추가
export XLEROBOT_HOME=~/XLeRobot
export PYTHONPATH=$XLEROBOT_HOME/software:$PYTHONPATH

# 저장 후 적용
source ~/.bashrc
```

---

## 개발 워크플로우 설정

### Git 설정

```bash
# Git 사용자 정보 설정
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 브랜치 확인
git branch

# 원격 저장소 확인
git remote -v
```

### 프로젝트 구조 확인

```bash
# 디렉토리 트리 설치
sudo apt install tree

# 프로젝트 구조 확인
tree -L 2 ~/XLeRobot
```

---

## 빠른 시작 체크리스트

환경 설정이 완료되었는지 확인하세요:

- [ ] Ubuntu 20.04+ 설치됨
- [ ] Python 3.8+ 설치됨
- [ ] Git 설치 및 저장소 클론 완료
- [ ] 시스템 패키지 설치 완료
- [ ] Python 가상환경 생성 및 활성화
- [ ] MuJoCo 의존성 설치 완료
- [ ] VS Code 또는 IDE 설치
- [ ] `xlerobot_mujoco.py` 실행 성공
- [ ] (선택) Node.js 및 웹 도구 설치
- [ ] (선택) VR 도구 설치

---

## 다음 단계

환경 설정이 완료되었다면:

1. **시뮬레이션 학습** → [3장. 시뮬레이션 환경](../03_simulation/README.md)
2. **하드웨어 이해** → [2장. 하드웨어 구성](../02_hardware/README.md)
3. **제어 코드 탐색** → [4장. 소프트웨어 제어](../04_software_control/README.md)

---

## 요약

### 필수 설치
```bash
# 한 번에 설치 (Ubuntu)
sudo apt update && sudo apt install -y \
    build-essential git python3 python3-pip python3-venv \
    libusb-1.0-0-dev libhidapi-dev libglfw3 libopencv-dev

# 프로젝트 클론
git clone https://github.com/Vector-Wangel/XLeRobot.git
cd XLeRobot/simulation/mujoco/

# 환경 설정
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 테스트
python xlerobot_mujoco.py
```

축하합니다! 이제 XLeRobot 개발 환경이 준비되었습니다! 🎉

---

[← 1.3 기술 스택](03_tech_stack.md) | [다음: 2장. 하드웨어 구성 →](../02_hardware/README.md)
