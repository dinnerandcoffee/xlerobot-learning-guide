# XLeRobot 한국어 학습 가이드 📚

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Korean](https://img.shields.io/badge/lang-한국어-red.svg)](README.md)

XLeRobot 프로젝트를 체계적으로 학습하기 위한 완전한 한국어 가이드입니다.

## 📖 소개

이 가이드는 [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) 프로젝트를 처음 접하는 분들부터 고급 기능을 개발하고자 하는 분들까지 체계적으로 학습할 수 있도록 구성되었습니다.

**XLeRobot**은 아이폰보다 저렴한 가격($660부터)으로 만들 수 있는 듀얼암 모바일 가정용 로봇입니다.

## 🎯 대상 독자

- 🤖 로봇공학에 관심있는 개발자
- 🎓 Embodied AI를 연구하는 학생/연구자
- 🛠️ 메이커 및 DIY 로봇 제작자
- 🧪 저비용 로봇 플랫폼이 필요한 실험자

## 📚 목차

### 제1부: 시작하기

#### [1장. 프로젝트 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/README.md)
- [1.1 XLeRobot이란?](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/01_what_is_xlerobot.md)
- [1.2 프로젝트 구조 이해하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/02_project_structure.md)
- [1.3 기술 스택 및 의존성](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/03_tech_stack.md)
- [1.4 개발 환경 설정](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/04_dev_environment.md)

#### [2장. 하드웨어 구성](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/README.md)
- [2.1 부품 목록 (BOM)](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/01_bill_of_materials.md)
- [2.2 SO-100/SO-101 로봇 팔 이해하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/02_robot_arm.md)
- [2.3 모바일 베이스 및 옴니휠](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/03_mobile_base.md)
- [2.4 카메라 및 센서 시스템](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/04_sensors.md)
- [2.5 3D 프린팅 가이드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/05_3d_printing.md)
- [2.6 하드웨어 조립 가이드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/06_assembly.md)

### 제2부: 시뮬레이션

#### [3장. 시뮬레이션 환경](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/README.md)
- [3.1 시뮬레이션 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/01_simulation_overview.md)
- [3.2 MuJoCo 시뮬레이션 시작하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/02_mujoco_setup.md)
- [3.3 MuJoCo 키보드 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/03_mujoco_control.md)
- [3.4 Isaac Sim 설정](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/04_isaac_sim.md)
- [3.5 ManiSkill 환경](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/05_maniskill.md)
- [3.6 URDF/MJCF 모델 이해](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/06_robot_models.md)

### 제3부: 소프트웨어 제어

#### [4장. 기본 소프트웨어 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/README.md)
- [4.1 소프트웨어 아키텍처](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/01_architecture.md)
- [4.2 키보드 제어 구현](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/02_keyboard_control.md)
- [4.3 조이스틱 제어 (Xbox)](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/03_joystick_control.md)
- [4.4 Switch Joycon 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/04_joycon_control.md)
- [4.5 로봇 팔 운동학 (IK/FK)](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/05_kinematics.md)
- [4.6 듀얼암 협동 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/06_dual_arm_control.md)
- [4.7 모바일 베이스 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/07_mobile_base.md)

### 제4부: 고급 기능

#### [5장. 컴퓨터 비전 통합](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/README.md)
- [5.1 YOLO 객체 감지 기초](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/01_yolo_basics.md)
- [5.2 실시간 객체 추적](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/02_object_tracking.md)
- [5.3 세그멘테이션 활용](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/03_segmentation.md)
- [5.4 비전 기반 로봇 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/04_vision_control.md)
- [5.5 핸드 카메라 활용](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/05_hand_camera.md)

#### [6장. 웹 제어 인터페이스](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/README.md)
- [6.1 웹 제어 시스템 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/01_overview.md)
- [6.2 서버 API 이해하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/02_server_api.md)
- [6.3 클라이언트 구성](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/03_client_setup.md)
- [6.4 원격 제어 구현](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/04_remote_control.md)
- [6.5 실시간 비디오 스트리밍](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/05_video_streaming.md)

#### [7장. VR 제어 시스템](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/README.md)
- [7.1 VR 텔레오퍼레이션 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/01_vr_overview.md)
- [7.2 Quest3 VR 설정](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/02_quest3_setup.md)
- [7.3 WebRTC 통신](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/03_webrtc.md)
- [7.4 VR 인터페이스 사용하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/04_vr_interface.md)
- [7.5 데이터셋 레코딩](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/05_dataset_recording.md)

### 제5부: 실습 및 응용

#### [8장. 실습 프로젝트](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/README.md)
- [8.1 프로젝트 1: 첫 시뮬레이션 실행](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/01_first_simulation.md)
- [8.2 프로젝트 2: 커스텀 제어 스크립트](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/02_custom_control.md)
- [8.3 프로젝트 3: 객체 픽앤플레이스](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/03_pick_and_place.md)
- [8.4 프로젝트 4: 자율 내비게이션](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/04_navigation.md)
- [8.5 프로젝트 5: 가정용 작업 자동화](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/05_household_tasks.md)
- [8.6 다음 단계 및 고급 주제](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/06_next_steps.md)

### 부록

- [A. 트러블슈팅 가이드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/troubleshooting.md)
- [B. FAQ](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/faq.md)
- [C. 용어집](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/glossary.md)
- [D. 참고 자료](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/references.md)

## 🚀 빠른 시작

1. **저장소 클론**
   ```bash
   git clone https://github.com/dinnerandcoffee/xlerobot-learning-guide.git
   cd xlerobot-learning-guide
   ```

2. **학습 시작**
   - [전체 목차 보기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/00_table_of_contents.md)
   - [1장부터 시작하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/README.md)

3. **WikiDocs 온라인 버전** (준비 중)
   - 더 편한 온라인 읽기 환경

## 📖 학습 가이드

### 초급 학습 경로 (1-2주)
1장 → 2장 → 3장(MuJoCo만) → 4장(기본 제어) → 8.1

### 중급 학습 경로 (3-4주)
전체 1-4장 → 5장 → 8.1-8.3

### 고급 학습 경로 (6-8주)
전체 1-7장 → 8.1-8.6 → 커스텀 프로젝트

## 🤝 기여하기

이 학습 가이드는 커뮤니티와 함께 만들어갑니다!

- 오류를 발견하셨나요? → [Issue 생성](../../issues)
- 내용을 개선하고 싶으신가요? → [Pull Request](../../pulls)
- 질문이 있으신가요? → [Discussions](../../discussions)

### 기여 방법

1. 이 저장소를 Fork 합니다
2. 새 브랜치를 생성합니다 (`git checkout -b feature/amazing-content`)
3. 변경사항을 커밋합니다 (`git commit -m 'Add amazing content'`)
4. 브랜치에 Push 합니다 (`git push origin feature/amazing-content`)
5. Pull Request를 생성합니다

## 📝 작성 현황

- ✅ 완전 작성: 전체 8개 장 (48개 섹션) + 부록 4개
- ✅ 총 분량: 약 30,000 줄
- ✅ 완성도: 100% (WikiDocs 게시 준비 완료)

## 🔗 관련 링크

- **원본 프로젝트**: [XLeRobot GitHub](https://github.com/Vector-Wangel/XLeRobot)
- **공식 문서**: [XLeRobot Docs](https://xlerobot.readthedocs.io)
- **Discord**: [XLeRobot Community](https://discord.gg/bjZveEUh6F)

## 📄 라이선스

이 프로젝트는 MIT 라이선스로 배포됩니다. 자세한 내용은 [LICENSE](LICENSE) 파일을 참조하세요.

## ✨ 감사의 말

- [XLeRobot](https://github.com/Vector-Wangel/XLeRobot) 프로젝트 제작자 및 기여자들
- 한국 로봇공학 커뮤니티

---

**Happy Learning! 🚀**

*마지막 업데이트: 2025-11-05*
