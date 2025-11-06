# 🏆 GitHub 로봇 저장소 TOP 2위! XLeRobot 완전 학습 가이드

## 책 소개

**GitHub 로봇 관련 저장소 TOP 100 중 2위**를 차지한 **XLeRobot**은 아이폰보다 저렴한 가격($660부터)으로 만들 수 있는 듀얼암 모바일 가정용 로봇입니다. 이 책은 XLeRobot을 처음 접하는 분들부터 고급 기능을 개발하고자 하는 분들까지 체계적으로 학습할 수 있도록 구성되었습니다.

## 이 책의 대상 독자

- 🤖 로봇공학에 관심있는 개발자
- 🎓 Embodied AI를 연구하는 학생/연구자
- 🛠️ 메이커 및 DIY 로봇 제작자
- 🧪 저비용 로봇 플랫폼이 필요한 실험자

## 이 책에서 배울 수 있는 것

1. XLeRobot의 하드웨어 및 소프트웨어 전체 구조
2. 시뮬레이션 환경에서의 로봇 제어
3. 키보드, 조이스틱, VR을 이용한 텔레오퍼레이션
4. YOLO 기반 컴퓨터 비전 통합
5. 웹/VR 인터페이스를 통한 원격 제어
6. 실제 로봇 하드웨어 제작 및 조립

---

## 📑 전체 목차

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

---

### 제2부: 시뮬레이션

#### [3장. 시뮬레이션 환경](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/README.md)
- [3.1 시뮬레이션 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/01_simulation_overview.md)
- [3.2 MuJoCo 시뮬레이션 시작하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/02_mujoco_setup.md)
- [3.3 MuJoCo 키보드 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/03_mujoco_control.md)
- [3.4 Isaac Sim 설정](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/04_isaac_sim_setup.md)
- [3.5 ManiSkill 환경](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/05_maniskill_env.md)
- [3.6 URDF/MJCF 모델 이해](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/06_robot_models.md)

---

### 제3부: 소프트웨어 제어

#### [4장. 기본 소프트웨어 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/README.md)
- [4.1 소프트웨어 아키텍처](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/01_architecture.md)
- [4.2 키보드 제어 구현](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/02_keyboard_control.md)
- [4.3 조이스틱 제어 (Xbox)](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/03_joystick_control.md)
- [4.4 Switch Joycon 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/04_joycon_control.md)
- [4.5 로봇 팔 운동학 (IK/FK)](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/05_kinematics.md)
- [4.6 듀얼암 협동 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/06_dual_arm_control.md)
- [4.7 모바일 베이스 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/07_mobile_base.md)

---

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

---

### 제5부: 실습 및 응용

#### [8장. 실습 프로젝트](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/README.md)
- [8.1 프로젝트 1: 첫 시뮬레이션 실행](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/01_first_simulation.md)
- [8.2 프로젝트 2: 커스텀 제어 스크립트](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/02_custom_control.md)
- [8.3 프로젝트 3: 객체 픽앤플레이스](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/03_pick_and_place.md)
- [8.4 프로젝트 4: 자율 내비게이션](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/04_navigation.md)
- [8.5 프로젝트 5: 가정용 작업 자동화](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/05_household_tasks.md)
- [8.6 다음 단계 및 고급 주제](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/06_next_steps.md)

---

## 부록

### [A. 트러블슈팅 가이드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/troubleshooting.md)
### [B. FAQ](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/faq.md)
### [C. 용어집](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/glossary.md)
### [D. 참고 자료](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/references.md)

---

## 학습 가이드

### 초급 학습 경로 (1-2주)
1장 → 2장 → 3장(MuJoCo만) → 4장(기본 제어) → 8.1

### 중급 학습 경로 (3-4주)
전체 1-4장 → 5장 → 8.1-8.3

### 고급 학습 경로 (6-8주)
전체 1-7장 → 8.1-8.6 → 커스텀 프로젝트

---

## 기여 및 피드백

이 학습 가이드는 오픈소스 프로젝트입니다. 개선 사항이나 오류를 발견하시면 GitHub에서 이슈를 열거나 PR을 제출해주세요.

- GitHub: https://github.com/Vector-Wangel/XLeRobot
- 문서 기여: [CONTRIBUTING.md](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/../CONTRIBUTING.md)

---

**Happy Learning! 🚀**
