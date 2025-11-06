# 6.3 클라이언트 구성

Vue.js를 사용한 웹 제어 인터페이스 구축 방법을 학습합니다.

## 1. 프로젝트 초기화

### 1.1 Vite + Vue.js

```bash
# 프로젝트 생성
npm create vite@latest xlerobot-client -- --template vue-ts
cd xlerobot-client

# 의존성 설치
npm install

# 추가 라이브러리
npm install axios                    # HTTP 클라이언트
npm install -D tailwindcss postcss autoprefixer
npx tailwindcss init -p
```

### 1.2 Tailwind CSS 설정

```javascript
// tailwind.config.js
/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{vue,js,ts,jsx,tsx}",
  ],
  theme: {
    extend: {},
  },
  plugins: [],
}
```

```css
/* src/style.css */
@tailwind base;
@tailwind components;
@tailwind utilities;
```

### 1.3 디렉토리 구조

```
client/
├── index.html
├── package.json
├── vite.config.ts
├── tailwind.config.js
└── src/
    ├── main.ts
    ├── App.vue
    ├── style.css
    ├── api/
    │   └── robot.ts         # API 클라이언트
    ├── components/
    │   ├── RobotControl.vue # 로봇 제어 UI
    │   ├── VideoFeed.vue    # 비디오 피드
    │   ├── StatusPanel.vue  # 상태 패널
    │   └── Joystick.vue     # 조이스틱
    └── types/
        └── robot.ts         # 타입 정의
```

---

## 2. API 클라이언트

### 2.1 타입 정의

```typescript
// src/types/robot.ts
export interface JointPosition {
  joint_id: number
  angle: number
}

export interface EEPosition {
  x: number
  y: number
  z?: number
}

export interface RobotStatus {
  connected: boolean
  joint_count: number
  gripper_state: string
}

export interface GripperCommand {
  action: 'open' | 'close'
}
```

### 2.2 Axios 클라이언트

```typescript
// src/api/robot.ts
import axios from 'axios'
import type { JointPosition, EEPosition, RobotStatus, GripperCommand } from '../types/robot'

const API_URL = import.meta.env.VITE_API_URL || 'http://localhost:8000'

const api = axios.create({
  baseURL: API_URL,
  timeout: 5000,
})

// 연결 관리
export const connectRobot = async (port: string = '/dev/ttyUSB0') => {
  const response = await api.post('/api/robot/connect', null, { params: { port } })
  return response.data
}

export const disconnectRobot = async () => {
  const response = await api.post('/api/robot/disconnect')
  return response.data
}

export const getRobotStatus = async (): Promise<RobotStatus> => {
  const response = await api.get<RobotStatus>('/api/robot/status')
  return response.data
}

// 조인트 제어
export const setJointAngle = async (joint: JointPosition) => {
  const response = await api.post('/api/robot/joint', joint)
  return response.data
}

export const getJointAngle = async (joint_id: number) => {
  const response = await api.get(`/api/robot/joint/${joint_id}`)
  return response.data
}

// 엔드 이펙터
export const setEEPosition = async (pos: EEPosition) => {
  const response = await api.post('/api/robot/ee', pos)
  return response.data
}

export const getEEPosition = async (): Promise<EEPosition> => {
  const response = await api.get<EEPosition>('/api/robot/ee')
  return response.data
}

// 그리퍼
export const controlGripper = async (cmd: GripperCommand) => {
  const response = await api.post('/api/robot/gripper', cmd)
  return response.data
}

// 프리셋
export const goHome = async () => {
  const response = await api.post('/api/robot/home')
  return response.data
}

export const goSleep = async () => {
  const response = await api.post('/api/robot/sleep')
  return response.data
}

export default api
```

### 2.3 환경 변수

```bash
# .env
VITE_API_URL=http://localhost:8000
```

---

## 3. 컴포넌트

### 3.1 상태 패널

```vue
<!-- src/components/StatusPanel.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4 mb-4">
    <h2 class="text-xl font-bold mb-3">Robot Status</h2>
    
    <div class="space-y-2">
      <!-- 연결 상태 -->
      <div class="flex justify-between">
        <span>Connection:</span>
        <span :class="statusColor">{{ status.connected ? 'Connected' : 'Disconnected' }}</span>
      </div>
      
      <!-- EE 위치 -->
      <div class="flex justify-between">
        <span>EE Position:</span>
        <span class="font-mono">
          ({{ eePosition.x.toFixed(3) }}, {{ eePosition.y.toFixed(3) }})
        </span>
      </div>
      
      <!-- 그리퍼 -->
      <div class="flex justify-between">
        <span>Gripper:</span>
        <span>{{ status.gripper_state }}</span>
      </div>
    </div>
    
    <!-- 연결 버튼 -->
    <div class="mt-4 flex gap-2">
      <button 
        @click="handleConnect"
        :disabled="status.connected"
        class="flex-1 bg-green-600 hover:bg-green-700 disabled:bg-gray-600 
               px-4 py-2 rounded transition"
      >
        Connect
      </button>
      
      <button 
        @click="handleDisconnect"
        :disabled="!status.connected"
        class="flex-1 bg-red-600 hover:bg-red-700 disabled:bg-gray-600 
               px-4 py-2 rounded transition"
      >
        Disconnect
      </button>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import { connectRobot, disconnectRobot, getRobotStatus, getEEPosition } from '../api/robot'
import type { RobotStatus, EEPosition } from '../types/robot'

const status = ref<RobotStatus>({
  connected: false,
  joint_count: 6,
  gripper_state: 'unknown'
})

const eePosition = ref<EEPosition>({ x: 0, y: 0 })

const statusColor = computed(() => {
  return status.value.connected ? 'text-green-500' : 'text-red-500'
})

let intervalId: number | null = null

const updateStatus = async () => {
  try {
    const [robotStatus, eePos] = await Promise.all([
      getRobotStatus(),
      getEEPosition()
    ])
    
    status.value = robotStatus
    eePosition.value = eePos
  } catch (error) {
    // 연결 안됨
    status.value.connected = false
  }
}

const handleConnect = async () => {
  try {
    await connectRobot()
    await updateStatus()
  } catch (error) {
    console.error('Connect failed:', error)
  }
}

const handleDisconnect = async () => {
  try {
    await disconnectRobot()
    status.value.connected = false
  } catch (error) {
    console.error('Disconnect failed:', error)
  }
}

onMounted(() => {
  updateStatus()
  
  // 1초마다 상태 업데이트
  intervalId = window.setInterval(updateStatus, 1000)
})

onUnmounted(() => {
  if (intervalId !== null) {
    clearInterval(intervalId)
  }
})
</script>
```

### 3.2 로봇 제어 패널

```vue
<!-- src/components/RobotControl.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4">
    <h2 class="text-xl font-bold mb-3">Robot Control</h2>
    
    <!-- 조인트 제어 -->
    <div class="mb-4">
      <h3 class="text-lg mb-2">Joint Control</h3>
      <div 
        v-for="joint in 6" 
        :key="joint"
        class="flex items-center gap-2 mb-2"
      >
        <span class="w-16">J{{ joint }}:</span>
        <input 
          type="range" 
          min="-180" 
          max="180"
          v-model.number="jointAngles[joint - 1]"
          @change="setJoint(joint - 1)"
          class="flex-1"
        >
        <span class="w-16 text-right font-mono">{{ jointAngles[joint - 1] }}°</span>
      </div>
    </div>
    
    <!-- EE 제어 -->
    <div class="mb-4">
      <h3 class="text-lg mb-2">End Effector</h3>
      <div class="grid grid-cols-2 gap-2">
        <div>
          <label class="block mb-1">X:</label>
          <input 
            type="number" 
            step="0.01"
            v-model.number="eeTarget.x"
            class="w-full bg-gray-700 px-3 py-2 rounded"
          >
        </div>
        <div>
          <label class="block mb-1">Y:</label>
          <input 
            type="number" 
            step="0.01"
            v-model.number="eeTarget.y"
            class="w-full bg-gray-700 px-3 py-2 rounded"
          >
        </div>
      </div>
      <button 
        @click="moveEE"
        class="w-full mt-2 bg-blue-600 hover:bg-blue-700 px-4 py-2 rounded"
      >
        Move EE
      </button>
    </div>
    
    <!-- 그리퍼 -->
    <div class="mb-4">
      <h3 class="text-lg mb-2">Gripper</h3>
      <div class="flex gap-2">
        <button 
          @click="openGripper"
          class="flex-1 bg-purple-600 hover:bg-purple-700 px-4 py-2 rounded"
        >
          Open
        </button>
        <button 
          @click="closeGripper"
          class="flex-1 bg-purple-600 hover:bg-purple-700 px-4 py-2 rounded"
        >
          Close
        </button>
      </div>
    </div>
    
    <!-- 프리셋 -->
    <div>
      <h3 class="text-lg mb-2">Presets</h3>
      <div class="flex gap-2">
        <button 
          @click="home"
          class="flex-1 bg-orange-600 hover:bg-orange-700 px-4 py-2 rounded"
        >
          Home
        </button>
        <button 
          @click="sleep"
          class="flex-1 bg-orange-600 hover:bg-orange-700 px-4 py-2 rounded"
        >
          Sleep
        </button>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref } from 'vue'
import { 
  setJointAngle, 
  setEEPosition, 
  controlGripper, 
  goHome, 
  goSleep 
} from '../api/robot'

const jointAngles = ref([0, 0, 0, 0, 0, 0])
const eeTarget = ref({ x: 0.15, y: 0.0 })

const setJoint = async (index: number) => {
  try {
    await setJointAngle({
      joint_id: index,
      angle: jointAngles.value[index]
    })
  } catch (error) {
    console.error('Set joint failed:', error)
  }
}

const moveEE = async () => {
  try {
    await setEEPosition(eeTarget.value)
  } catch (error) {
    console.error('Move EE failed:', error)
  }
}

const openGripper = async () => {
  try {
    await controlGripper({ action: 'open' })
  } catch (error) {
    console.error('Open gripper failed:', error)
  }
}

const closeGripper = async () => {
  try {
    await controlGripper({ action: 'close' })
  } catch (error) {
    console.error('Close gripper failed:', error)
  }
}

const home = async () => {
  try {
    await goHome()
  } catch (error) {
    console.error('Home failed:', error)
  }
}

const sleep = async () => {
  try {
    await goSleep()
  } catch (error) {
    console.error('Sleep failed:', error)
  }
}
</script>
```

### 3.3 비디오 피드

```vue
<!-- src/components/VideoFeed.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4">
    <h2 class="text-xl font-bold mb-3">Camera Feed</h2>
    
    <div class="relative bg-black rounded overflow-hidden aspect-video">
      <img 
        v-if="isStreaming"
        :src="streamUrl"
        alt="Camera Feed"
        class="w-full h-full object-contain"
        @error="handleError"
      >
      
      <div 
        v-else
        class="absolute inset-0 flex items-center justify-center text-gray-500"
      >
        No video feed
      </div>
    </div>
    
    <!-- 제어 -->
    <div class="mt-3 flex gap-2">
      <button 
        @click="toggleStream"
        class="flex-1 px-4 py-2 rounded transition"
        :class="isStreaming ? 'bg-red-600 hover:bg-red-700' : 'bg-green-600 hover:bg-green-700'"
      >
        {{ isStreaming ? 'Stop' : 'Start' }}
      </button>
      
      <button 
        @click="takeSnapshot"
        class="px-4 py-2 bg-blue-600 hover:bg-blue-700 rounded"
      >
        📸 Snapshot
      </button>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue'

const API_URL = import.meta.env.VITE_API_URL || 'http://localhost:8000'

const isStreaming = ref(false)

const streamUrl = computed(() => {
  return `${API_URL}/api/camera/stream?t=${Date.now()}`
})

const toggleStream = () => {
  isStreaming.value = !isStreaming.value
}

const takeSnapshot = () => {
  const url = `${API_URL}/api/camera/snapshot`
  window.open(url, '_blank')
}

const handleError = () => {
  console.error('Video stream error')
  isStreaming.value = false
}
</script>
```

---

## 4. 메인 앱

### 4.1 App.vue

```vue
<!-- src/App.vue -->
<template>
  <div class="min-h-screen bg-gray-900 text-white p-8">
    <header class="mb-8">
      <h1 class="text-4xl font-bold">XLeRobot Web Control</h1>
      <p class="text-gray-400">Remote robot control via web browser</p>
    </header>
    
    <div class="grid grid-cols-1 lg:grid-cols-3 gap-6">
      <!-- 좌측: 비디오 + 상태 -->
      <div class="lg:col-span-2 space-y-6">
        <VideoFeed />
        <StatusPanel />
      </div>
      
      <!-- 우측: 제어 -->
      <div>
        <RobotControl />
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import StatusPanel from './components/StatusPanel.vue'
import RobotControl from './components/RobotControl.vue'
import VideoFeed from './components/VideoFeed.vue'
</script>
```

### 4.2 main.ts

```typescript
// src/main.ts
import { createApp } from 'vue'
import './style.css'
import App from './App.vue'

createApp(App).mount('#app')
```

---

## 5. 빌드 & 배포

### 5.1 개발 서버

```bash
# 개발 모드
npm run dev

# 브라우저에서
http://localhost:5173
```

### 5.2 프로덕션 빌드

```bash
# 빌드
npm run build

# dist/ 폴더 생성됨
# 정적 파일 서버로 배포
```

### 5.3 Nginx 배포

```nginx
# /etc/nginx/sites-available/xlerobot
server {
    listen 80;
    server_name your-domain.com;
    
    # 클라이언트
    location / {
        root /var/www/xlerobot-client/dist;
        try_files $uri $uri/ /index.html;
    }
    
    # API 프록시
    location /api/ {
        proxy_pass http://localhost:8000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
        proxy_set_header Host $host;
    }
    
    # WebSocket
    location /ws {
        proxy_pass http://localhost:8000;
        proxy_http_version 1.1;
        proxy_set_header Upgrade $http_upgrade;
        proxy_set_header Connection "upgrade";
    }
}
```

---

## 6. React 대안

### 6.1 React + TypeScript

```bash
# 프로젝트 생성
npm create vite@latest xlerobot-client -- --template react-ts

# 의존성
npm install axios
npm install -D tailwindcss
```

### 6.2 React 컴포넌트

```tsx
// src/components/RobotControl.tsx
import { useState } from 'react'
import { setJointAngle } from '../api/robot'

export default function RobotControl() {
  const [jointAngles, setJointAngles] = useState([0, 0, 0, 0, 0, 0])
  
  const handleJointChange = async (index: number, angle: number) => {
    const newAngles = [...jointAngles]
    newAngles[index] = angle
    setJointAngles(newAngles)
    
    await setJointAngle({ joint_id: index, angle })
  }
  
  return (
    <div className="bg-gray-800 rounded-lg p-4">
      <h2 className="text-xl font-bold mb-3">Robot Control</h2>
      
      {jointAngles.map((angle, index) => (
        <div key={index} className="flex items-center gap-2 mb-2">
          <span className="w-16">J{index + 1}:</span>
          <input 
            type="range"
            min="-180"
            max="180"
            value={angle}
            onChange={(e) => handleJointChange(index, Number(e.target.value))}
            className="flex-1"
          />
          <span className="w-16 text-right font-mono">{angle}°</span>
        </div>
      ))}
    </div>
  )
}
```

---

## 7. 참고 자료

- [Vue.js Documentation](https://vuejs.org/)
- [Vite Guide](https://vitejs.dev/guide/)
- [Axios Documentation](https://axios-http.com/)
- [Tailwind CSS](https://tailwindcss.com/docs)

---

[← 6.2 서버 API](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/02_server_api.md) | [다음: 6.4 원격 제어 구현 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/04_remote_control.md)
