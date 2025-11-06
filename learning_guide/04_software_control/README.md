# 4장. 소프트웨어 제어

XLeRobot을 다양한 방법으로 제어하는 소프트웨어를 학습합니다.

## 📚 이 장에서 배울 내용

- 소프트웨어 아키텍처 이해
- 키보드, 조이스틱, Joycon을 이용한 제어
- 로봇 팔 운동학 (IK/FK)
- 듀얼암 협동 제어
- 모바일 베이스 제어

## 📖 하위 섹션

1. [소프트웨어 아키텍처](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/01_architecture.md)
2. [키보드 제어 구현](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/02_keyboard_control.md)
3. [조이스틱 제어 (Xbox)](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/03_joystick_control.md)
4. [Switch Joycon 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/04_joycon_control.md)
5. [로봇 팔 운동학 (IK/FK)](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/05_kinematics.md)
6. [듀얼암 협동 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/06_dual_arm_control.md)
7. [모바일 베이스 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/07_mobile_base.md)

## 🎯 학습 목표

이 장을 완료하면:
- ✅ XLeRobot의 소프트웨어 구조를 이해할 수 있습니다
- ✅ 다양한 입력 장치로 로봇을 제어할 수 있습니다
- ✅ IK/FK 알고리즘을 이해하고 활용할 수 있습니다
- ✅ 커스텀 제어 스크립트를 작성할 수 있습니다

## 🎮 제어 방식 비교

| 방식 | 장점 | 단점 | 추천 용도 |
|------|------|------|-----------|
| **키보드** | 간단, 설정 불필요 | 정밀도 낮음 | 테스트, 학습 |
| **Xbox** | 정밀, 익숙함 | 추가 구매 필요 | 텔레오퍼레이션 |
| **Joycon** | 휴대성, 블루투스 | 배터리 관리 | 이동 제어 |
| **VR** | 직관적, 몰입 | 고가, 설정 복잡 | 데이터 수집 |

---

[← 3장. 시뮬레이션 환경](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/README.md) | [다음: 4.1 소프트웨어 아키텍처 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/01_architecture.md)
