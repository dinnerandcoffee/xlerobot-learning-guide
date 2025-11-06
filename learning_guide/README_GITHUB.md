# XLeRobot 학습 가이드

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Korean](https://img.shields.io/badge/lang-한국어-red.svg)](README_GITHUB.md)

XLeRobot 프로젝트를 체계적으로 학습하기 위한 완전 가이드입니다.

## 📚 소개

이 학습 가이드는 **XLeRobot** ($660부터 시작하는 저비용 듀얼암 모바일 로봇)을 처음 접하는 분들부터 고급 기능을 개발하고자 하는 분들까지 체계적으로 학습할 수 있도록 구성되었습니다.

### XLeRobot이란?

- 💰 **저비용**: $660부터 시작
- ⏰ **빠른 조립**: 4시간 이내
- 🦾 **듀얼암**: SO-100/SO-101 로봇 팔
- 🚗 **모바일**: 옴니휠 기반 전방향 이동
- 🤖 **Embodied AI**: 가정용 작업 자동화

## 📖 목차

### 제1부: 시작하기

#### [1장. 프로젝트 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/README.md)
- [1.1 XLeRobot이란?](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/01_what_is_xlerobot.md)
- [1.2 프로젝트 구조 이해하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/02_project_structure.md)
- [1.3 기술 스택 및 의존성](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/03_tech_stack.md)
- [1.4 개발 환경 설정](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/01_overview/04_dev_environment.md)

#### [2장. 하드웨어 구성](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_hardware/README.md)
- 부품 목록, 3D 프린팅, 조립 가이드

### 제2부: 시뮬레이션

#### [3장. 시뮬레이션 환경](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/03_simulation/README.md)
- MuJoCo, Isaac Sim, ManiSkill 환경

### 제3부: 소프트웨어 제어

#### [4장. 기본 소프트웨어 제어](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/04_software_control/README.md)
- 키보드, Xbox, Joycon, VR 제어

### 제4부: 고급 기능

#### [5장. 컴퓨터 비전 통합](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/README.md)
- YOLO 객체 감지 및 추적

#### [6장. 웹 제어 인터페이스](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/README.md)
- FastAPI 서버 및 웹 클라이언트

#### [7장. VR 제어 시스템](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/README.md)
- Quest 3 VR 텔레오퍼레이션

### 제5부: 실습

#### [8장. 실습 프로젝트](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/08_practice_projects/README.md)
- 픽앤플레이스, 자율 내비게이션 등

### 부록

- [A. 트러블슈팅 가이드](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/troubleshooting.md)
- [B. FAQ](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/faq.md)
- [C. 용어집](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/glossary.md)
- [D. 참고 자료](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/appendix/references.md)

## 🚀 빠른 시작

```bash
# 저장소 클론
git clone https://github.com/dinnerandcoffee/xlerobot-learning-guide.git
cd xlerobot-learning-guide

# 전체 목차 확인
cat 00_table_of_contents.md

# 1장부터 시작
cat 01_overview/01_what_is_xlerobot.md
```

## 🎯 학습 경로

### 초급 (1-2주)
- 1장: 프로젝트 개요
- 3장: MuJoCo 시뮬레이션
- 4장: 기본 제어
- 8.1: 첫 시뮬레이션

### 중급 (3-4주)
- 2장: 하드웨어
- 5장: 컴퓨터 비전
- 8.2-8.3: 커스텀 제어 및 픽앤플레이스

### 고급 (6-8주)
- 6-7장: 웹/VR 제어
- 8.4-8.6: 고급 프로젝트

## 📚 WikiDocs 출판

이 가이드는 WikiDocs.net에 출판할 수 있도록 구성되어 있습니다.

출판 방법은 [WIKIDOCS_GUIDE.md](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/WIKIDOCS_GUIDE.md)를 참고하세요.

## 🔗 관련 링크

- **XLeRobot 원본**: https://github.com/Vector-Wangel/XLeRobot
- **공식 문서**: https://xlerobot.readthedocs.io
- **Discord**: https://discord.gg/bjZveEUh6F

## 🤝 기여

개선 사항이나 오류를 발견하시면:

1. Issue 열기
2. Fork 및 수정
3. Pull Request 제출

## 📝 라이선스

이 학습 가이드는 [Apache 2.0 License](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/LICENSE)로 제공됩니다.

## ✨ 감사의 말

- **Vector Wang** - XLeRobot 프로젝트 제작자
- **XLeRobot 커뮤니티** - 피드백 및 개선 사항
- **기여자 여러분**

---

**Happy Learning! 🤖**

XLeRobot으로 로봇공학의 세계를 탐험하세요!
