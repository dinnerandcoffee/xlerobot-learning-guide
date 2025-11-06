# B. FAQ (자주 묻는 질문)

XLeRobot에 대해 자주 묻는 질문과 답변입니다.

## 일반 질문

### Q1. XLeRobot을 만드는 데 얼마나 걸리나요?

**A**: 
- **부품 구매**: 1-2주 (배송 시간)
- **3D 프린팅**: 2-3일
- **조립**: 3-4시간 (처음 조립 시)
- **소프트웨어 설정**: 1-2일

총 약 **2-3주** 정도 소요됩니다.

### Q2. 프로그래밍 경험이 없어도 만들 수 있나요?

**A**: 기본적인 Python 지식이 필요합니다. 완전 초보자라면:
1. Python 기초 학습 (1-2주)
2. Ubuntu/Linux 기본 사용법 (3-5일)
3. XLeRobot 학습 시작

온라인 무료 강의로 충분히 배울 수 있습니다.

### Q3. 실제 로봇 없이 학습할 수 있나요?

**A**: 네! 시뮬레이션만으로도 충분히 학습 가능합니다.
- MuJoCo 시뮬레이션 사용
- 모든 제어 알고리즘 테스트 가능
- 나중에 하드웨어 구매 가능

---

## 비용 관련

### Q4. 정확히 얼마가 드나요?

**A**: 구성에 따라 다릅니다.

| 구성 | 가격 (USD) |
|------|------------|
| 최소 (기본) | ~$660 |
| 권장 (RGBD 카메라) | ~$880 |
| 풀옵션 (라즈베리파이 + RGBD) | ~$1,160 |

*3D 프린팅, 도구, 배송비, 세금 제외

### Q5. 3D 프린팅 비용은 얼마인가요?

**A**: 
- **직접 프린팅**: $30-50 (필라멘트 비용)
- **출력 서비스**: $100-200
- **소요 시간**: 2-3일

### Q6. 가장 저렴하게 만들 수 있나요?

**A**: 비용 절감 팁:
- 중고 부품 활용
- 단일 RGB 카메라 사용
- 기존 노트북 활용
- 커뮤니티 그룹 구매

최소 ~$500까지 가능합니다.

---

## 기술적 질문

### Q7. 어떤 컴퓨터가 필요한가요?

**A**: 
- **시뮬레이션**: 중급 노트북 (8GB RAM)
- **실제 로봇**: 라즈베리파이 4 또는 노트북
- **VR/고급**: 데스크탑 (NVIDIA GPU)

### Q8. Windows에서 작동하나요?

**A**: 
- **권장**: Ubuntu Linux
- **가능**: WSL2 (Windows Subsystem for Linux)
- **비권장**: 네이티브 Windows

WSL2 설치 후 Ubuntu처럼 사용 가능합니다.

### Q9. ROS가 필요한가요?

**A**: 
- **기본 기능**: ROS 불필요
- **고급 기능**: 선택 사항
- **RL 학습**: 불필요

XLeRobot은 ROS 없이도 작동합니다.

### Q10. 어떤 카메라를 사용해야 하나요?

**A**: 
- **입문**: USB 웹캠
- **권장**: Intel RealSense D435i (RGBD)
- **예산형**: Raspberry Pi Camera Module

---

## 하드웨어 관련

### Q11. SO-100과 SO-101의 차이는?

**A**: 
- **SO-100**: 오리지널, 가격 저렴
- **SO-101**: 개선 버전, 토크 향상
- **호환성**: 둘 다 XLeRobot과 호환

코드는 양쪽 모두 작동합니다.

### Q12. 3D 프린터가 없으면 어떻게 하나요?

**A**: 
1. **출력 서비스**: Shapeways, Treatstock
2. **로컬 메이커스페이스**: 공용 3D 프린터 이용
3. **대행 업체**: Wowrobo 등에서 키트 구매

### Q13. 배터리 수명은 얼마나 되나요?

**A**: 
- **일반 사용**: 1-2시간
- **무거운 작업**: 30-60분
- **권장**: 18650 배터리 팩 (5000mAh 이상)

---

## 소프트웨어 관련

### Q14. RL 알고리즘을 학습시킬 수 있나요?

**A**: 네! 
- ManiSkill 환경 제공
- Isaac Sim 연동 가능
- PPO, SAC 등 알고리즘 지원

### Q15. YOLO 외 다른 비전 모델 사용 가능한가요?

**A**: 가능합니다.
- SAM (Segment Anything)
- DINO (객체 감지)
- 커스텀 모델 통합 가능

### Q16. 여러 대의 로봇을 제어할 수 있나요?

**A**: 
- **시뮬레이션**: 여러 로봇 가능
- **실제**: 네트워크 통신으로 가능
- **예제**: 웹 제어 인터페이스 활용

---

## 학습 관련

### Q17. 얼마나 배워야 사용할 수 있나요?

**A**: 
- **기본 제어**: 1주일
- **시뮬레이션**: 2-3주
- **고급 기능**: 1-2개월
- **전문가**: 3-6개월

### Q18. 추천 학습 순서는?

**A**: 
1. 1-2장: 개요 및 하드웨어 (2일)
2. 3장: 시뮬레이션 (1주)
3. 4-5장: 제어 및 비전 (2주)
4. 6-7장: 웹/VR (선택)
5. 8장: 실습 프로젝트 (계속)

### Q19. 온라인 커뮤니티가 있나요?

**A**: 
- **Discord**: https://discord.gg/bjZveEUh6F
- **GitHub Discussions**: GitHub 저장소
- **Twitter**: @VectorWang2

---

## 문제 해결

### Q20. 시뮬레이션이 실행되지 않아요

**A**: [트러블슈팅 가이드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/troubleshooting.md) 참조
- OpenGL 라이브러리 확인
- GLFW 설치 확인
- 가상환경 활성화 확인

### Q21. 로봇이 움직이지 않아요

**A**: 
1. 전원 확인
2. USB 연결 확인
3. 모터 ID 설정 확인
4. 시리얼 포트 권한 확인

상세한 해결책은 [트러블슈팅](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/troubleshooting.md)에서 확인하세요.

---

## 기여 관련

### Q22. 프로젝트에 기여하고 싶어요

**A**: 
1. [CONTRIBUTING.md](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/../CONTRIBUTING.md) 읽기
2. GitHub에서 이슈 확인
3. PR 제출
4. Discord에서 논의

### Q23. 제 프로젝트를 공유하고 싶어요

**A**: 
- **GitHub**: Discussion 섹션
- **Discord**: #showcase 채널
- **Twitter**: #XLeRobot 해시태그

---

## 더 궁금한 점이 있나요?

- **GitHub Issues**: 버그 리포트 및 기능 요청
- **Discord**: 실시간 질문
- **Documentation**: https://xlerobot.readthedocs.io

---

[← A. 트러블슈팅](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/troubleshooting.md) | [다음: C. 용어집 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/glossary.md)
