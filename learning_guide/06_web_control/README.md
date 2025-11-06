# 6장. 웹 제어 인터페이스

브라우저를 통한 원격 로봇 제어 시스템을 학습합니다.

## 📚 이 장에서 배울 내용

- 웹 제어 시스템 아키텍처
- FastAPI 서버 API
- Vue.js/React 클라이언트
- 원격 제어 구현
- 실시간 비디오 스트리밍

## 📖 하위 섹션

1. [웹 제어 시스템 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/01_overview.md)
2. [서버 API 이해하기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/02_server_api.md)
3. [클라이언트 구성](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/03_client_setup.md)
4. [원격 제어 구현](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/04_remote_control.md)
5. [실시간 비디오 스트리밍](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/05_video_streaming.md)

## 🎯 학습 목표

이 장을 완료하면:
- ✅ 웹 기반 제어 시스템을 구축할 수 있습니다
- ✅ FastAPI로 로봇 제어 API를 만들 수 있습니다
- ✅ 브라우저에서 로봇을 제어할 수 있습니다
- ✅ 실시간 비디오를 스트리밍할 수 있습니다

## 🌐 시스템 아키텍처

```
브라우저 (클라이언트)
    ↕ WebSocket
FastAPI 서버
    ↕ 시리얼/블루투스
XLeRobot (하드웨어)
```

---

[← 5장. 컴퓨터 비전](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/README.md) | [다음: 6.1 웹 제어 개요 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/01_overview.md)
