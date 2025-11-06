# 1.1 XLeRobot이란?

## 프로젝트 소개

**XLeRobot**은 아이폰보다 저렴한 가격으로 만들 수 있는 듀얼암 모바일 가정용 로봇 플랫폼입니다.

### 핵심 특징

- 💰 **저비용**: $660부터 시작 (기본 구성)
- ⏰ **빠른 조립**: 4시간 이내 조립 가능
- 🦾 **듀얼암**: 두 개의 SO-100/SO-101 로봇 팔
- 🚗 **모바일**: 옴니휠 기반 전방향 이동
- 🎮 **다양한 제어**: 키보드, 조이스틱, VR 지원
- 🤖 **Embodied AI**: 가정용 작업 자동화를 위한 플랫폼

## 프로젝트 목표

XLeRobot은 다음을 목표로 합니다:

1. **접근성**: 누구나 저렴하게 로봇을 만들 수 있도록
2. **교육**: Embodied AI와 로봇공학 학습 플랫폼
3. **연구**: 저비용 로봇 연구 플랫폼 제공
4. **실용성**: 실제 가정에서 사용 가능한 로봇

## 비용 구성

| 구성 | 미국 | 유럽 | 중국 | 인도 |
|------|------|------|------|------|
| **기본** (노트북 사용, 단일 RGB 카메라) | ~$660 | ~€680 | ~¥3999 | ~₹87000 |
| + 스테레오 듀얼 RGB 카메라 | +$30 | +€30 | +¥199 | +₹6550 |
| + 라즈베리파이 | +$79 | +€79 | +¥399 | +₹7999 |
| + RealSense RGBD 카메라 | +$220 | +€230 | +¥1499 | +₹35726 |

> 💡 **참고**: 가격은 3D 프린팅, 도구, 배송비, 세금을 제외한 부품 원가입니다.

## 주요 구성 요소

### 하드웨어
- **로봇 팔**: SO-100/SO-101 듀얼암
- **모바일 베이스**: 옴니휠 기반 샤시
- **카메라**: RGB/RGBD 헤드 카메라, 핸드 카메라
- **컨트롤러**: 라즈베리파이 또는 노트북
- **배터리**: 리튬 배터리 팩

### 소프트웨어
- **시뮬레이션**: MuJoCo, Isaac Sim, ManiSkill
- **제어**: 키보드, Xbox, Switch Joycon, VR
- **비전**: YOLO 기반 객체 감지
- **통신**: 블루투스, WiFi, WebRTC

## 기반 프로젝트

XLeRobot은 다음 프로젝트들을 기반으로 만들어졌습니다:

- [**LeRobot**](https://github.com/huggingface/lerobot): Hugging Face의 로봇 학습 프레임워크
- [**SO-100/SO-101**](https://github.com/TheRobotStudio/SO-ARM100): 오픈소스 로봇 팔
- [**LeKiwi**](https://github.com/SIGRobotics-UIUC/LeKiwi): 모바일 매니퓰레이션 플랫폼
- [**Bambot**](https://github.com/timqian/bambot): 저비용 가정용 로봇

## 프로젝트 제작자

**Gaotian/Vector Wang**
- Rice University CS 대학원생
- RobotPi Lab 소속
- 전공: Robust Object Manipulation
- 연구: 불확실성 하에서의 강건한 조작

### 주요 기여자
- **Zhuoyi Lu**: RL sim2real, 실제 로봇 텔레오퍼레이션
- **Yiyang Huang**: RL & VLA 구현
- **YCP**: 웹 UI 원격 제어
- **Lixing Zhang**: 하드웨어 설계 개선
- **Nicole Yue**: 문서 웹사이트 구축
- **Yuesong Wang**: MuJoCo 시뮬레이션

## 활용 사례

XLeRobot은 다음과 같은 용도로 사용될 수 있습니다:

1. 🏠 **가정용 작업**: 물건 정리, 청소 보조
2. 🎓 **교육**: 로봇공학 교육 플랫폼
3. 🔬 **연구**: Embodied AI 연구
4. 🛠️ **개발**: 로봇 알고리즘 테스트
5. 🎮 **취미**: DIY 로봇 프로젝트

## 커뮤니티 및 지원

- **GitHub**: https://github.com/Vector-Wangel/XLeRobot
- **문서**: https://xlerobot.readthedocs.io
- **Discord**: https://discord.gg/bjZveEUh6F
- **Twitter**: @VectorWang2

## 라이선스

XLeRobot은 **Apache 2.0 라이선스**로 제공됩니다.

---

## 요약

- XLeRobot은 $660부터 시작하는 저비용 듀얼암 모바일 로봇
- Embodied AI와 가정용 작업 자동화를 위한 플랫폼
- 오픈소스 프로젝트 기반으로 구축
- 교육, 연구, 개발, 취미용으로 활용 가능

---

[← 1장 목차](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/README.md) | [다음: 1.2 프로젝트 구조 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/02_project_structure.md)
