# 6.1 웹 제어 시스템 개요

브라우저를 통한 원격 로봇 제어 시스템의 전체 구조를 이해합니다.

## 1. 시스템 아키텍처

### 1.1 전체 구조

```
┌─────────────────┐
│   브라우저      │ (클라이언트)
│   - HTML/CSS    │
│   - JavaScript  │
│   - Vue.js      │
└────────┬────────┘
         │ HTTP/WebSocket
         │ (포트 8000)
┌────────▼────────┐
│  FastAPI 서버   │ (백엔드)
│   - REST API    │
│   - WebSocket   │
│   - 비디오 스트림│
└────────┬────────┘
         │ Serial/Bluetooth
         │ (/dev/ttyUSB0)
┌────────▼────────┐
│   XLeRobot      │ (하드웨어)
│   - SO100 팔    │
│   - 이동 베이스 │
│   - 카메라      │
└─────────────────┘
```

### 1.2 통신 프로토콜

**HTTP (REST API)**
- 요청/응답 방식
- 로봇 상태 조회
- 단발성 명령 전송

**WebSocket**
- 양방향 실시간 통신
- 조이스틱 제어
- 연속 명령 스트림

---

## 2. 구성 요소

### 2.1 클라이언트 (브라우저)

**기술 스택**
```
프론트엔드 프레임워크: Vue.js 3 / React
UI 라이브러리: Tailwind CSS
HTTP 클라이언트: Axios
WebSocket: native WebSocket API
```

**주요 기능**
- 로봇 제어 UI (버튼, 슬라이더)
- 비디오 피드 표시
- 상태 모니터링
- 조이스틱 입력

### 2.2 서버 (FastAPI)

**기술 스택**
```python
# requirements.txt
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
python-multipart>=0.0.6
opencv-python>=4.8.0
pyserial>=3.5
websockets>=12.0
```

**주요 기능**
- REST API 엔드포인트
- WebSocket 핸들러
- 시리얼 통신 (로봇)
- 비디오 스트리밍

### 2.3 하드웨어 (XLeRobot)

**통신 인터페이스**
- USB 시리얼 (/dev/ttyUSB0)
- Bluetooth (선택)
- Wi-Fi (ESP32 사용 시)

---

## 3. 디렉토리 구조

### 3.1 전체 프로젝트

```
web_control/
├── server/                # 백엔드 (Python)
│   ├── main.py           # FastAPI 앱
│   ├── requirements.txt
│   ├── api/
│   │   ├── __init__.py
│   │   ├── robot.py      # 로봇 제어 API
│   │   ├── camera.py     # 카메라 API
│   │   └── websocket.py  # WebSocket 핸들러
│   └── core/
│       ├── __init__.py
│       ├── robot_control.py  # 로봇 통신
│       └── video_stream.py   # 비디오 스트리밍
│
└── client/               # 프론트엔드 (Vue.js)
    ├── package.json
    ├── vite.config.ts
    ├── index.html
    └── src/
        ├── main.ts
        ├── App.vue
        ├── components/
        │   ├── RobotControl.vue
        │   ├── VideoFeed.vue
        │   └── StatusPanel.vue
        └── api/
            └── robot.ts      # API 클라이언트
```

---

## 4. 프로젝트 설정

### 4.1 서버 설정

```bash
# 디렉토리 생성
mkdir -p web_control/server/api web_control/server/core
cd web_control/server

# 가상환경 (선택)
python3 -m venv venv
source venv/bin/activate

# 의존성 설치
pip install fastapi uvicorn[standard] python-multipart \
            opencv-python pyserial websockets
```

**requirements.txt**
```txt
fastapi==0.104.1
uvicorn[standard]==0.24.0
python-multipart==0.0.6
opencv-python==4.8.1
pyserial==3.5
websockets==12.0
pydantic==2.5.0
```

### 4.2 클라이언트 설정

```bash
cd web_control/client

# Node.js 프로젝트 초기화
npm create vite@latest . -- --template vue-ts

# 의존성 설치
npm install
npm install axios
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

**package.json**
```json
{
  "name": "xlerobot-client",
  "version": "1.0.0",
  "scripts": {
    "dev": "vite",
    "build": "vite build",
    "preview": "vite preview"
  },
  "dependencies": {
    "vue": "^3.3.8",
    "axios": "^1.6.2"
  },
  "devDependencies": {
    "@vitejs/plugin-vue": "^4.5.0",
    "typescript": "^5.3.2",
    "vite": "^5.0.2",
    "tailwindcss": "^3.3.5"
  }
}
```

---

## 5. 기본 서버 (FastAPI)

### 5.1 최소 서버

```python
# server/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

app = FastAPI(title="XLeRobot Web Control")

# CORS 설정 (브라우저에서 접근 허용)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 개발용 (프로덕션에서는 특정 도메인만)
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.get("/")
async def root():
    """루트 엔드포인트"""
    return {"message": "XLeRobot Web Control API"}

@app.get("/health")
async def health():
    """헬스 체크"""
    return {"status": "ok"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

**실행**
```bash
cd server
python main.py

# 또는
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

**테스트**
```bash
# 브라우저에서
http://localhost:8000

# cURL
curl http://localhost:8000/health
```

### 5.2 API 문서

FastAPI는 자동으로 API 문서를 생성합니다:
- **Swagger UI**: http://localhost:8000/docs
- **ReDoc**: http://localhost:8000/redoc

---

## 6. 기본 클라이언트 (Vue.js)

### 6.1 최소 클라이언트

```vue
<!-- client/src/App.vue -->
<template>
  <div class="min-h-screen bg-gray-900 text-white p-8">
    <h1 class="text-3xl font-bold mb-4">XLeRobot Web Control</h1>
    
    <!-- 상태 표시 -->
    <div class="bg-gray-800 p-4 rounded mb-4">
      <p>Server Status: <span :class="statusClass">{{ status }}</span></p>
    </div>
    
    <!-- 테스트 버튼 -->
    <button 
      @click="testConnection"
      class="bg-blue-600 hover:bg-blue-700 px-4 py-2 rounded"
    >
      Test Connection
    </button>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted } from 'vue'
import axios from 'axios'

const API_URL = 'http://localhost:8000'

const status = ref('Disconnected')
const statusClass = ref('text-red-500')

const testConnection = async () => {
  try {
    const response = await axios.get(`${API_URL}/health`)
    
    if (response.data.status === 'ok') {
      status.value = 'Connected'
      statusClass.value = 'text-green-500'
    }
  } catch (error) {
    status.value = 'Error'
    statusClass.value = 'text-red-500'
    console.error(error)
  }
}

onMounted(() => {
  testConnection()
})
</script>
```

**실행**
```bash
cd client
npm run dev

# 브라우저에서
http://localhost:5173
```

---

## 7. HTTP vs WebSocket

### 7.1 HTTP (REST API)

**장점**
- 간단한 구현
- 캐싱 가능
- 상태 조회에 적합

**단점**
- 단방향 (클라이언트 → 서버)
- 폴링 필요 (실시간성 떨어짐)
- 오버헤드 (헤더 반복)

**사용 예시**
```javascript
// 로봇 상태 조회
const status = await axios.get('/api/robot/status')

// 그리퍼 열기
await axios.post('/api/robot/gripper', { action: 'open' })
```

### 7.2 WebSocket

**장점**
- 양방향 통신
- 실시간 (낮은 지연)
- 적은 오버헤드

**단점**
- 복잡한 구현
- 연결 유지 필요
- 프록시 문제 가능

**사용 예시**
```javascript
// WebSocket 연결
const ws = new WebSocket('ws://localhost:8000/ws')

ws.onopen = () => {
  // 조이스틱 입력 전송
  ws.send(JSON.stringify({ type: 'joystick', x: 0.5, y: 0.3 }))
}

ws.onmessage = (event) => {
  const data = JSON.parse(event.data)
  console.log('Robot data:', data)
}
```

### 7.3 선택 가이드

| 기능 | 프로토콜 | 이유 |
|------|---------|------|
| 로봇 상태 조회 | HTTP | 단발성 |
| 그리퍼 열기/닫기 | HTTP | 단순 명령 |
| 조이스틱 제어 | WebSocket | 연속 입력 |
| 실시간 피드백 | WebSocket | 낮은 지연 |
| 비디오 스트림 | HTTP (MJPEG) | 표준 지원 |

---

## 8. 보안 고려사항

### 8.1 인증

```python
# 간단한 API 키 인증
from fastapi import Header, HTTPException

API_KEY = "your-secret-key"

async def verify_api_key(x_api_key: str = Header(...)):
    if x_api_key != API_KEY:
        raise HTTPException(status_code=401, detail="Invalid API Key")
    return x_api_key
```

### 8.2 CORS

```python
# 특정 도메인만 허용 (프로덕션)
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:5173",
        "https://your-domain.com"
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)
```

### 8.3 HTTPS/WSS

```bash
# SSL 인증서 (Let's Encrypt)
sudo apt install certbot

# 인증서 발급
sudo certbot certonly --standalone -d your-domain.com

# Uvicorn HTTPS
uvicorn main:app \
  --ssl-keyfile /etc/letsencrypt/live/your-domain.com/privkey.pem \
  --ssl-certfile /etc/letsencrypt/live/your-domain.com/fullchain.pem
```

---

## 9. 개발 워크플로

### 9.1 병렬 개발

**터미널 1: 서버**
```bash
cd server
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

**터미널 2: 클라이언트**
```bash
cd client
npm run dev
```

**터미널 3: 로봇 (선택)**
```bash
# 로봇이 없으면 Mock 사용
python server/core/robot_mock.py
```

### 9.2 디버깅

**서버 로그**
```python
import logging

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

@app.get("/test")
async def test():
    logger.debug("Test endpoint called")
    return {"status": "ok"}
```

**클라이언트 디버깅**
```javascript
// Chrome DevTools
// F12 → Console/Network 탭

console.log('API Response:', response.data)
```

---

## 10. 참고 자료

- [FastAPI Documentation](https://fastapi.tiangolo.com/)
- [Vue.js Guide](https://vuejs.org/guide/)
- [WebSocket API (MDN)](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket)
- [Tailwind CSS](https://tailwindcss.com/)

---

[← 6장 목차](README.md) | [다음: 6.2 서버 API 이해하기 →](02_server_api.md)
