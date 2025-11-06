# 5장. 컴퓨터 비전 통합

YOLO를 활용한 객체 감지 및 비전 기반 로봇 제어를 학습합니다.

## 📚 이 장에서 배울 내용

- YOLO 객체 감지 기초
- 실시간 객체 추적
- 세그멘테이션 활용
- 비전 기반 로봇 제어
- 핸드 카메라 활용

## 📖 하위 섹션

1. [YOLO 객체 감지 기초](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/01_yolo_basics.md)
2. [실시간 객체 추적](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/02_object_tracking.md)
3. [세그멘테이션 활용](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/03_segmentation.md)
4. [비전 기반 로봇 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/04_vision_control.md)
5. [핸드 카메라 활용](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/05_hand_camera.md)

## 🎯 학습 목표

이 장을 완료하면:
- ✅ YOLO를 사용한 객체 감지를 구현할 수 있습니다
- ✅ 실시간으로 객체를 추적할 수 있습니다
- ✅ 비전 정보를 로봇 제어에 통합할 수 있습니다
- ✅ 핸드 카메라로 정밀한 조작을 수행할 수 있습니다

## 🔍 비전 시스템 구성

```
카메라 입력 → YOLO 감지 → 객체 위치 추정 → 로봇 제어
    ↓            ↓              ↓              ↓
  RGB/RGBD    바운딩박스    3D 좌표 계산    End-Effector 이동
```

---

[← 4장. 소프트웨어 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/README.md) | [다음: 5.1 YOLO 기초 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/01_yolo_basics.md)
