# WikiDocs 출판 가이드

이 문서는 `learning_guide` 폴더의 내용을 WikiDocs.net에 출판하는 방법을 안내합니다.

## 📚 준비된 콘텐츠

총 **20개의 마크다운 파일**이 준비되었습니다:

### 메인 페이지
- `README.md` - 학습 가이드 소개
- `00_table_of_contents.md` - 전체 목차 및 책 소개

### 1장: 프로젝트 개요 (5개 파일)
- `01_overview/README.md`
- `01_overview/01_what_is_xlerobot.md`
- `01_overview/02_project_structure.md`
- `01_overview/03_tech_stack.md`
- `01_overview/04_dev_environment.md`

### 2-8장: README 파일 (7개 파일)
각 장의 개요 페이지가 준비되어 있습니다:
- `02_hardware/README.md`
- `03_simulation/README.md`
- `04_software_control/README.md`
- `05_computer_vision/README.md`
- `06_web_control/README.md`
- `07_vr_control/README.md`
- `08_practice_projects/README.md`

### 부록 (5개 파일)
- `appendix/README.md`
- `appendix/troubleshooting.md`
- `appendix/faq.md`
- `appendix/glossary.md`
- `appendix/references.md`

---

## 🚀 WikiDocs 출판 단계

### 1단계: WikiDocs 계정 생성

1. https://wikidocs.net 접속
2. 회원가입 또는 로그인
3. "새 책 만들기" 클릭

### 2단계: 책 기본 정보 설정

- **책 제목**: XLeRobot 완전 학습 가이드
- **부제목**: 아이폰보다 저렴한 가정용 로봇 만들기
- **설명**: XLeRobot을 처음 접하는 분들부터 고급 기능을 개발하고자 하는 분들까지 체계적으로 학습할 수 있는 완전 가이드
- **태그**: 로봇공학, Python, 머신러닝, AI, DIY
- **공개 여부**: 공개 또는 비공개 선택

### 3단계: 목차 구성

WikiDocs에서 다음과 같이 목차를 구성하세요:

```
XLeRobot 학습 가이드
├─ 소개
│  └─ 전체 목차 (00_table_of_contents.md)
│
├─ 1부: 시작하기
│  ├─ 1장. 프로젝트 개요
│  │  ├─ 1.1 XLeRobot이란?
│  │  ├─ 1.2 프로젝트 구조
│  │  ├─ 1.3 기술 스택
│  │  └─ 1.4 개발 환경 설정
│  │
│  └─ 2장. 하드웨어 구성
│     └─ 2.0 개요
│
├─ 2부: 시뮬레이션
│  └─ 3장. 시뮬레이션 환경
│     └─ 3.0 개요
│
├─ 3부: 소프트웨어 제어
│  └─ 4장. 기본 소프트웨어 제어
│     └─ 4.0 개요
│
├─ 4부: 고급 기능
│  ├─ 5장. 컴퓨터 비전
│  │  └─ 5.0 개요
│  ├─ 6장. 웹 제어
│  │  └─ 6.0 개요
│  └─ 7장. VR 제어
│     └─ 7.0 개요
│
├─ 5부: 실습
│  └─ 8장. 실습 프로젝트
│     └─ 8.0 개요
│
└─ 부록
   ├─ A. 트러블슈팅
   ├─ B. FAQ
   ├─ C. 용어집
   └─ D. 참고 자료
```

### 4단계: 콘텐츠 업로드

각 파일을 WikiDocs 편집기에 복사하여 붙여넣으세요:

1. 새 페이지 추가
2. 제목 입력
3. 마크다운 내용 붙여넣기
4. 저장

**팁**: 
- WikiDocs는 마크다운을 지원합니다
- 이미지는 별도로 업로드해야 합니다
- 내부 링크는 WikiDocs 페이지 링크로 수정해야 합니다

### 5단계: 이미지 처리

현재 문서에는 텍스트만 포함되어 있습니다. 이미지가 필요한 경우:

1. XLeRobot GitHub에서 이미지 다운로드
2. WikiDocs 이미지 업로드 기능 사용
3. 마크다운에서 이미지 경로 업데이트

### 6단계: 링크 수정

내부 링크를 WikiDocs 링크로 수정:

**변경 전**:
```markdown
[다음: 1.2 프로젝트 구조 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/02_project_structure.md)
```

**변경 후**:
```markdown
[다음: 1.2 프로젝트 구조 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide//위키독스페이지번호)
```

### 7단계: 출판

1. 모든 페이지 작성 완료
2. 목차 순서 조정
3. 미리보기 확인
4. 공개 설정
5. 출판!

---

## 📝 추가 작성이 필요한 콘텐츠

현재 **개요 페이지**만 작성되었습니다. 각 장의 세부 섹션은 아직 작성되지 않았습니다:

### 작성 필요 파일 (예시)

**2장: 하드웨어**
- `02_hardware/01_bill_of_materials.md`
- `02_hardware/02_robot_arm.md`
- `02_hardware/03_mobile_base.md`
- `02_hardware/04_sensors.md`
- `02_hardware/05_3d_printing.md`
- `02_hardware/06_assembly.md`

**3장: 시뮬레이션**
- `03_simulation/01_simulation_overview.md`
- `03_simulation/02_mujoco_setup.md`
- ... (나머지 섹션)

### 작성 전략

1. **단계별 작성**: 한 장씩 완성하면서 출판
2. **실습 중심**: 실제 코드와 예제 포함
3. **스크린샷**: 시뮬레이션, 제어 화면 캡처
4. **동영상 링크**: YouTube 튜토리얼 연결

---

## 🎯 출판 체크리스트

### 출판 전
- [ ] 모든 페이지 작성 완료
- [ ] 목차 구조 확인
- [ ] 내부 링크 테스트
- [ ] 이미지 업로드 및 링크
- [ ] 코드 블록 포매팅 확인
- [ ] 오탈자 검토

### 출판 후
- [ ] 책 URL 공유 (GitHub README에 추가)
- [ ] Discord에 공지
- [ ] Twitter/X에 홍보
- [ ] 피드백 수집
- [ ] 지속적 업데이트

---

## 📊 작성 우선순위

초기 출판을 위한 최소 콘텐츠:

### Phase 1: 핵심 내용 (1-2주)
1. ✅ 0-1장 (완료)
2. ⏳ 3장: 시뮬레이션 (MuJoCo 중심)
3. ⏳ 4장: 소프트웨어 제어 (기본 제어)
4. ✅ 부록 (완료)

### Phase 2: 확장 (2-4주)
5. 2장: 하드웨어 (상세)
6. 5장: 컴퓨터 비전
7. 8장: 실습 프로젝트

### Phase 3: 고급 (1-2개월)
8. 6장: 웹 제어
9. 7장: VR 제어
10. 추가 실습 예제

---

## 💡 팁

### WikiDocs 마크다운 특징
- 표준 마크다운 지원
- 코드 하이라이팅 지원
- 수식 지원 (KaTeX)
- 테이블 지원
- 체크박스 지원

### 좋은 문서 작성법
1. **명확한 구조**: 목차가 잘 보이도록
2. **실습 중심**: 따라 할 수 있는 예제
3. **시각 자료**: 이미지, 다이어그램
4. **코드 예제**: 실행 가능한 코드
5. **문제 해결**: 흔한 오류와 해결책

---

## 🔄 다음 단계

1. **나머지 섹션 작성**: 2-8장의 세부 내용
2. **코드 예제 추가**: 실행 가능한 예제
3. **이미지 준비**: 스크린샷, 다이어그램
4. **동영상 제작**: 튜토리얼 영상 (선택)
5. **커뮤니티 피드백**: 초안 공유 및 개선

---

## 📞 도움이 필요하신가요?

- **GitHub Issues**: 내용 제안 및 오류 보고
- **Discord**: 실시간 토론
- **Email**: 직접 문의

---

**축하합니다!** 🎉

XLeRobot 학습 가이드의 기본 구조가 완성되었습니다. 이제 세부 내용을 채워나가며 훌륭한 학습 자료를 만들어보세요!

---

**작성 일자**: 2025-11-05  
**버전**: 1.0  
**작성자**: XLeRobot 커뮤니티
