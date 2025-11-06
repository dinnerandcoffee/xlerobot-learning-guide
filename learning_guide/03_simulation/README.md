# 3장. 시뮬레이션 환경

실제 하드웨어 없이 XLeRobot을 제어하고 학습할 수 있는 시뮬레이션 환경을 배웁니다.

## 📚 이 장에서 배울 내용

- 시뮬레이션의 중요성과 장점
- MuJoCo 시뮬레이터 설정 및 사용
- Isaac Sim 환경 구축
- ManiSkill RL 환경
- 로봇 모델 (URDF/MJCF) 이해

## 📖 하위 섹션

1. [시뮬레이션 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/01_simulation_overview.md)
2. [MuJoCo 시뮬레이션 시작하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/02_mujoco_setup.md)
3. [MuJoCo 키보드 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/03_mujoco_control.md)
4. [Isaac Sim 설정](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/04_isaac_sim.md)
5. [ManiSkill 환경](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/05_maniskill.md)
6. [URDF/MJCF 모델 이해](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/06_robot_models.md)

## 🎯 학습 목표

이 장을 완료하면:
- ✅ 시뮬레이션 환경을 설정하고 실행할 수 있습니다
- ✅ 가상 로봇을 키보드로 제어할 수 있습니다
- ✅ 다양한 시뮬레이터의 차이를 이해할 수 있습니다
- ✅ 로봇 모델 파일을 읽고 수정할 수 있습니다

## 💡 왜 시뮬레이션인가?

시뮬레이션의 장점:
- 🚀 **빠른 시작**: 하드웨어 없이 바로 학습 시작
- 💰 **비용 절감**: 실제 로봇 구매 전 테스트
- 🔁 **반복 실험**: 무한 테스트 가능
- 🛡️ **안전**: 로봇 손상 걱정 없음
- 📊 **데이터 생성**: RL 학습용 데이터 수집

## 🎮 시뮬레이터 비교

| 특징 | MuJoCo | Isaac Sim | ManiSkill |
|------|--------|-----------|-----------|
| **난이도** | ⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| **속도** | 빠름 | 중간 | 빠름 |
| **그래픽** | 기본 | 우수 | 좋음 |
| **RL 지원** | 기본 | 우수 | 전문 |
| **GPU 필요** | ❌ | ✅ | 선택 |
| **추천 대상** | 초보자 | 고급 사용자 | RL 연구자 |

**권장 학습 순서**: MuJoCo → ManiSkill → Isaac Sim

---

[← 2장. 하드웨어 구성](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/README.md) | [다음: 3.1 시뮬레이션 개요 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/01_simulation_overview.md)
