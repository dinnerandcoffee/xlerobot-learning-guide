# A. 트러블슈팅 가이드

XLeRobot 사용 중 발생할 수 있는 문제와 해결 방법입니다.

## 📋 목차

1. [설치 관련 문제](#설치-관련-문제)
2. [시뮬레이션 문제](#시뮬레이션-문제)
3. [하드웨어 문제](#하드웨어-문제)
4. [네트워크 문제](#네트워크-문제)
5. [성능 문제](#성능-문제)

---

## 설치 관련 문제

### Python 패키지 설치 실패

**문제**: `pip install` 중 오류 발생

```bash
# 해결 1: pip 업그레이드
pip install --upgrade pip

# 해결 2: 시스템 패키지 설치
sudo apt install python3-dev build-essential

# 해결 3: 가상환경 재생성
rm -rf .venv
python3 -m venv .venv
source .venv/bin/activate
```

### MuJoCo 설치 오류

**문제**: `import mujoco` 실패

```bash
# OpenGL 라이브러리 설치
sudo apt install -y libgl1-mesa-glx libglew-dev libosmesa6-dev

# GLFW 설치
sudo apt install -y libglfw3 libglfw3-dev

# 재설치
pip uninstall mujoco
pip install mujoco==3.3.0
```

---

## 시뮬레이션 문제

### 시뮬레이션 창이 열리지 않음

**문제**: `xlerobot_mujoco.py` 실행 시 창이 안 열림

```bash
# X11 포워딩 확인 (SSH 사용 시)
echo $DISPLAY

# WSL2의 경우 X 서버 실행
# Windows에서 VcXsrv 설치 및 실행

# DISPLAY 환경변수 설정
export DISPLAY=:0
```

### 시뮬레이션이 느림

**해결책**:
1. 렌더링 품질 낮추기
2. 시뮬레이션 스텝 간격 조정
3. GPU 드라이버 업데이트

---

## 하드웨어 문제

### 로봇 팔이 응답하지 않음

**체크리스트**:
- [ ] USB 케이블 연결 확인
- [ ] 전원 공급 확인
- [ ] 모터 ID 설정 확인
- [ ] 시리얼 포트 권한 확인

```bash
# 포트 권한 설정
sudo usermod -aG dialout $USER
sudo chmod 666 /dev/ttyUSB0

# 재부팅 후 재시도
```

### 카메라가 인식되지 않음

```bash
# 카메라 장치 확인
ls -l /dev/video*

# RealSense 확인
rs-enumerate-devices

# 권한 설정
sudo usermod -aG video $USER
```

---

## 네트워크 문제

### 웹 인터페이스 접속 불가

```bash
# 방화벽 확인
sudo ufw status

# 포트 열기
sudo ufw allow 8000

# 서버 주소 확인
hostname -I
```

### VR 연결 끊김

**해결책**:
1. WiFi 6 또는 유선 사용
2. 라우터와 가까운 위치
3. 방화벽 설정 확인
4. WebRTC STUN/TURN 서버 설정

---

## 성능 문제

### CPU 사용률이 높음

```bash
# 프로세스 확인
top -p $(pgrep -f python)

# 렌더링 프레임레이트 조정
# xlerobot_mujoco.py에서 fps 변수 수정
```

### 메모리 부족

```bash
# 메모리 사용량 확인
free -h

# 스왑 증가
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

---

## 자주 발생하는 오류 메시지

### `ModuleNotFoundError: No module named 'mujoco'`

```bash
# 가상환경 활성화 확인
source .venv/bin/activate
pip list | grep mujoco
```

### `Permission denied: '/dev/ttyUSB0'`

```bash
sudo chmod 666 /dev/ttyUSB0
# 또는
sudo usermod -aG dialout $USER
# 로그아웃 후 다시 로그인
```

### `GLFW: Failed to initialize`

```bash
# GLFW 재설치
sudo apt remove libglfw3
sudo apt install libglfw3 libglfw3-dev

# 환경변수 설정
export MUJOCO_GL=glfw
```

---

## 도움 받기

문제가 해결되지 않으면:

1. **GitHub Issues**: https://github.com/Vector-Wangel/XLeRobot/issues
2. **Discord**: https://discord.gg/bjZveEUh6F
3. **문서**: https://xlerobot.readthedocs.io

---

[← 부록 목차](README.md) | [다음: B. FAQ →](faq.md)
