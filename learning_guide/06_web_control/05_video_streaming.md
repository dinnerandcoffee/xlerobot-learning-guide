# 6.5 실시간 비디오 스트리밍

웹 브라우저를 통한 실시간 카메라 피드 스트리밍을 구현합니다.

## 1. MJPEG 스트리밍

### 1.1 서버 구현 (FastAPI)

```python
# server/api/camera.py
from fastapi import APIRouter
from fastapi.responses import StreamingResponse
import cv2
import threading
import time

router = APIRouter(prefix="/api/camera", tags=["camera"])

class MJPEGStreamer:
    """MJPEG 스트리밍 클래스"""
    
    def __init__(self, camera_id=0, width=640, height=480, fps=30):
        self.camera_id = camera_id
        self.width = width
        self.height = height
        self.fps = fps
        
        self.cap = None
        self.frame = None
        self.is_streaming = False
        
        self.lock = threading.Lock()
        
    def start(self):
        """스트리밍 시작"""
        if self.is_streaming:
            return
            
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)
        
        if not self.cap.isOpened():
            raise Exception(f"Cannot open camera {self.camera_id}")
        
        self.is_streaming = True
        
        # 별도 스레드에서 프레임 읽기
        threading.Thread(target=self._capture_frames, daemon=True).start()
    
    def stop(self):
        """스트리밍 중지"""
        self.is_streaming = False
        
        if self.cap:
            self.cap.release()
            self.cap = None
    
    def _capture_frames(self):
        """프레임 캡처 스레드"""
        while self.is_streaming:
            ret, frame = self.cap.read()
            
            if ret:
                with self.lock:
                    self.frame = frame
            
            time.sleep(1.0 / self.fps)
    
    def get_frame(self):
        """현재 프레임 반환"""
        with self.lock:
            if self.frame is not None:
                return self.frame.copy()
        return None
    
    def get_jpeg_frame(self, quality=80):
        """JPEG 인코딩된 프레임"""
        frame = self.get_frame()
        
        if frame is None:
            return None
        
        # JPEG 압축
        ret, jpeg = cv2.imencode(
            '.jpg', 
            frame, 
            [cv2.IMWRITE_JPEG_QUALITY, quality]
        )
        
        if ret:
            return jpeg.tobytes()
        
        return None

# 전역 스트리머
streamer = MJPEGStreamer()

def generate_mjpeg():
    """MJPEG 스트림 생성기"""
    while True:
        frame = streamer.get_jpeg_frame()
        
        if frame is None:
            # 빈 프레임 전송
            continue
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@router.get("/stream")
async def video_stream():
    """MJPEG 비디오 스트림"""
    return StreamingResponse(
        generate_mjpeg(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@router.post("/start")
async def start_stream():
    """스트리밍 시작"""
    try:
        streamer.start()
        return {"status": "started"}
    except Exception as e:
        return {"error": str(e)}

@router.post("/stop")
async def stop_stream():
    """스트리밍 중지"""
    streamer.stop()
    return {"status": "stopped"}

@router.get("/snapshot")
async def snapshot():
    """스냅샷 캡처"""
    frame = streamer.get_jpeg_frame()
    
    if frame is None:
        return {"error": "No frame available"}
    
    return StreamingResponse(
        iter([frame]),
        media_type="image/jpeg"
    )
```

### 1.2 클라이언트 구현

```vue
<!-- src/components/MJPEGViewer.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4">
    <h2 class="text-xl font-bold mb-3">MJPEG Stream</h2>
    
    <!-- 비디오 컨테이너 -->
    <div class="relative bg-black rounded overflow-hidden aspect-video mb-4">
      <img 
        v-if="isStreaming"
        :src="streamUrl"
        alt="Video Stream"
        class="w-full h-full object-contain"
        @load="handleLoad"
        @error="handleError"
        @click="toggleFullscreen"
      >
      
      <!-- 오버레이 -->
      <div 
        v-if="!isStreaming"
        class="absolute inset-0 flex items-center justify-center text-gray-500"
      >
        <div class="text-center">
          <div class="text-4xl mb-2">📹</div>
          <div>Video stream offline</div>
        </div>
      </div>
      
      <!-- 상태 표시 -->
      <div class="absolute top-2 left-2 flex gap-2">
        <div 
          :class="connectionStatus === 'connected' ? 'bg-green-500' : 'bg-red-500'"
          class="px-2 py-1 rounded text-xs text-white"
        >
          {{ connectionStatus }}
        </div>
        
        <div class="bg-black bg-opacity-50 px-2 py-1 rounded text-xs text-white">
          {{ resolution }}
        </div>
      </div>
      
      <!-- 로딩 -->
      <div 
        v-if="isLoading"
        class="absolute inset-0 flex items-center justify-center bg-black bg-opacity-50"
      >
        <div class="animate-spin w-8 h-8 border-2 border-white border-t-transparent rounded-full"></div>
      </div>
    </div>
    
    <!-- 제어 버튼 -->
    <div class="grid grid-cols-3 gap-2 mb-4">
      <button 
        @click="startStream"
        :disabled="isStreaming"
        class="bg-green-600 hover:bg-green-700 disabled:bg-gray-600 px-4 py-2 rounded"
      >
        Start
      </button>
      
      <button 
        @click="stopStream"
        :disabled="!isStreaming"
        class="bg-red-600 hover:bg-red-700 disabled:bg-gray-600 px-4 py-2 rounded"
      >
        Stop
      </button>
      
      <button 
        @click="takeSnapshot"
        class="bg-blue-600 hover:bg-blue-700 px-4 py-2 rounded"
      >
        📸 Snap
      </button>
    </div>
    
    <!-- 설정 -->
    <div class="space-y-2">
      <div class="flex items-center gap-2">
        <label class="w-20">Quality:</label>
        <input 
          type="range" 
          min="10" 
          max="100" 
          v-model="quality"
          class="flex-1"
        >
        <span class="w-12 text-right">{{ quality }}%</span>
      </div>
      
      <div class="flex items-center gap-2">
        <label class="w-20">FPS:</label>
        <select v-model="fps" class="flex-1 bg-gray-700 rounded px-2 py-1">
          <option value="15">15 FPS</option>
          <option value="30">30 FPS</option>
          <option value="60">60 FPS</option>
        </select>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'
import axios from 'axios'

const API_URL = import.meta.env.VITE_API_URL || 'http://localhost:8000'

const isStreaming = ref(false)
const isLoading = ref(false)
const connectionStatus = ref<'connected' | 'disconnected' | 'error'>('disconnected')
const resolution = ref('640x480')
const quality = ref(80)
const fps = ref(30)

const streamUrl = computed(() => {
  return `${API_URL}/api/camera/stream?quality=${quality.value}&fps=${fps.value}&t=${Date.now()}`
})

const startStream = async () => {
  try {
    isLoading.value = true
    
    const response = await axios.post(`${API_URL}/api/camera/start`)
    
    if (response.data.status === 'started') {
      isStreaming.value = true
      connectionStatus.value = 'connected'
    }
  } catch (error) {
    console.error('Failed to start stream:', error)
    connectionStatus.value = 'error'
  } finally {
    isLoading.value = false
  }
}

const stopStream = async () => {
  try {
    await axios.post(`${API_URL}/api/camera/stop`)
    
    isStreaming.value = false
    connectionStatus.value = 'disconnected'
  } catch (error) {
    console.error('Failed to stop stream:', error)
  }
}

const takeSnapshot = () => {
  const url = `${API_URL}/api/camera/snapshot`
  window.open(url, '_blank')
}

const handleLoad = () => {
  connectionStatus.value = 'connected'
  isLoading.value = false
}

const handleError = () => {
  console.error('Stream error')
  connectionStatus.value = 'error'
  isLoading.value = false
}

const toggleFullscreen = (event: Event) => {
  const img = event.target as HTMLImageElement
  
  if (document.fullscreenElement) {
    document.exitFullscreen()
  } else {
    img.requestFullscreen()
  }
}

onMounted(() => {
  // 자동 시작
  startStream()
})

onUnmounted(() => {
  stopStream()
})
</script>
```

---

## 2. WebRTC 스트리밍

### 2.1 WebRTC 서버 (aiortc)

```python
# server/api/webrtc.py
from fastapi import APIRouter, WebSocket
from aiortc import RTCPeerConnection, RTCSessionDescription, VideoStreamTrack
from aiortc.contrib.media import MediaPlayer
import asyncio
import json

router = APIRouter(prefix="/api/webrtc", tags=["webrtc"])

class CameraTrack(VideoStreamTrack):
    """카메라 비디오 트랙"""
    
    def __init__(self, camera_id=0):
        super().__init__()
        self.cap = cv2.VideoCapture(camera_id)
        
    async def recv(self):
        """프레임 전송"""
        ret, frame = self.cap.read()
        
        if not ret:
            return None
        
        # BGR → RGB 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # aiortc VideoFrame으로 변환
        from aiortc import VideoFrame
        video_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        video_frame.pts = self.time
        video_frame.time_base = fractions.Fraction(1, 1000)
        
        return video_frame

# WebRTC 피어 연결 관리
peer_connections = set()

@router.websocket("/ws")
async def webrtc_websocket(websocket: WebSocket):
    """WebRTC 시그널링"""
    await websocket.accept()
    
    pc = RTCPeerConnection()
    peer_connections.add(pc)
    
    try:
        # 카메라 트랙 추가
        camera_track = CameraTrack(camera_id=0)
        pc.addTrack(camera_track)
        
        while True:
            message = await websocket.receive_text()
            data = json.loads(message)
            
            if data["type"] == "offer":
                # 오퍼 처리
                offer = RTCSessionDescription(
                    sdp=data["sdp"],
                    type=data["type"]
                )
                
                await pc.setRemoteDescription(offer)
                
                # 응답 생성
                answer = await pc.createAnswer()
                await pc.setLocalDescription(answer)
                
                await websocket.send_text(json.dumps({
                    "type": "answer",
                    "sdp": pc.localDescription.sdp
                }))
                
            elif data["type"] == "ice-candidate":
                # ICE 후보 추가
                candidate = data["candidate"]
                await pc.addIceCandidate(candidate)
                
    except Exception as e:
        print(f"WebRTC error: {e}")
    finally:
        peer_connections.discard(pc)
        await pc.close()
```

### 2.2 WebRTC 클라이언트

```vue
<!-- src/components/WebRTCViewer.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4">
    <h2 class="text-xl font-bold mb-3">WebRTC Stream</h2>
    
    <!-- 비디오 요소 -->
    <div class="relative bg-black rounded overflow-hidden aspect-video mb-4">
      <video 
        ref="videoElement"
        autoplay
        muted
        class="w-full h-full object-contain"
      ></video>
      
      <!-- 연결 상태 -->
      <div class="absolute top-2 left-2">
        <div 
          :class="getStatusColor(connectionState)"
          class="px-2 py-1 rounded text-xs text-white"
        >
          {{ connectionState }}
        </div>
      </div>
    </div>
    
    <!-- 제어 -->
    <div class="flex gap-2">
      <button 
        @click="connect"
        :disabled="connectionState === 'connected'"
        class="flex-1 bg-green-600 hover:bg-green-700 disabled:bg-gray-600 px-4 py-2 rounded"
      >
        Connect
      </button>
      
      <button 
        @click="disconnect"
        :disabled="connectionState === 'disconnected'"
        class="flex-1 bg-red-600 hover:bg-red-700 disabled:bg-gray-600 px-4 py-2 rounded"
      >
        Disconnect
      </button>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'

const WS_URL = import.meta.env.VITE_WS_URL || 'ws://localhost:8000/api/webrtc/ws'

const videoElement = ref<HTMLVideoElement>()
const connectionState = ref<'disconnected' | 'connecting' | 'connected' | 'failed'>('disconnected')

let peerConnection: RTCPeerConnection | null = null
let websocket: WebSocket | null = null

const getStatusColor = (state: string) => {
  switch (state) {
    case 'connected': return 'bg-green-500'
    case 'connecting': return 'bg-yellow-500'
    case 'failed': return 'bg-red-500'
    default: return 'bg-gray-500'
  }
}

const connect = async () => {
  if (connectionState.value === 'connected') return
  
  connectionState.value = 'connecting'
  
  // WebSocket 연결
  websocket = new WebSocket(WS_URL)
  
  websocket.onopen = () => {
    console.log('WebSocket connected')
    setupPeerConnection()
  }
  
  websocket.onmessage = async (event) => {
    const data = JSON.parse(event.data)
    
    if (data.type === 'answer') {
      const answer = new RTCSessionDescription({
        type: 'answer',
        sdp: data.sdp
      })
      
      await peerConnection?.setRemoteDescription(answer)
      connectionState.value = 'connected'
    }
  }
  
  websocket.onerror = () => {
    connectionState.value = 'failed'
  }
  
  websocket.onclose = () => {
    connectionState.value = 'disconnected'
  }
}

const setupPeerConnection = async () => {
  peerConnection = new RTCPeerConnection({
    iceServers: [
      { urls: 'stun:stun.l.google.com:19302' }
    ]
  })
  
  // 원격 스트림 처리
  peerConnection.ontrack = (event) => {
    if (videoElement.value) {
      videoElement.value.srcObject = event.streams[0]
    }
  }
  
  // ICE 후보 처리
  peerConnection.onicecandidate = (event) => {
    if (event.candidate && websocket) {
      websocket.send(JSON.stringify({
        type: 'ice-candidate',
        candidate: event.candidate
      }))
    }
  }
  
  // 오퍼 생성
  const offer = await peerConnection.createOffer()
  await peerConnection.setLocalDescription(offer)
  
  // 오퍼 전송
  if (websocket) {
    websocket.send(JSON.stringify({
      type: 'offer',
      sdp: offer.sdp
    }))
  }
}

const disconnect = () => {
  if (peerConnection) {
    peerConnection.close()
    peerConnection = null
  }
  
  if (websocket) {
    websocket.close()
    websocket = null
  }
  
  if (videoElement.value) {
    videoElement.value.srcObject = null
  }
  
  connectionState.value = 'disconnected'
}

onUnmounted(() => {
  disconnect()
})
</script>
```

---

## 3. 스트림 선택 & 설정

### 3.1 스트림 매니저

```vue
<!-- src/components/StreamManager.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4">
    <h2 class="text-xl font-bold mb-3">Stream Manager</h2>
    
    <!-- 스트림 타입 선택 -->
    <div class="mb-4">
      <h3 class="font-bold mb-2">Stream Type</h3>
      <div class="flex gap-2">
        <button 
          @click="setStreamType('mjpeg')"
          :class="streamType === 'mjpeg' ? 'bg-blue-600' : 'bg-gray-600'"
          class="px-3 py-1 rounded"
        >
          MJPEG
        </button>
        <button 
          @click="setStreamType('webrtc')"
          :class="streamType === 'webrtc' ? 'bg-blue-600' : 'bg-gray-600'"
          class="px-3 py-1 rounded"
        >
          WebRTC
        </button>
      </div>
    </div>
    
    <!-- 해상도 설정 -->
    <div class="mb-4">
      <h3 class="font-bold mb-2">Resolution</h3>
      <select v-model="resolution" class="w-full bg-gray-700 rounded px-3 py-2">
        <option value="320x240">320x240 (QVGA)</option>
        <option value="640x480">640x480 (VGA)</option>
        <option value="1280x720">1280x720 (HD)</option>
        <option value="1920x1080">1920x1080 (FHD)</option>
      </select>
    </div>
    
    <!-- 품질 설정 -->
    <div class="mb-4">
      <h3 class="font-bold mb-2">Quality</h3>
      <div class="space-y-2">
        <div class="flex items-center gap-2">
          <label class="w-16">Bitrate:</label>
          <input 
            type="range" 
            min="100" 
            max="5000" 
            v-model="bitrate"
            class="flex-1"
          >
          <span class="w-20 text-right">{{ bitrate }} kbps</span>
        </div>
        
        <div class="flex items-center gap-2">
          <label class="w-16">FPS:</label>
          <input 
            type="range" 
            min="5" 
            max="60" 
            v-model="fps"
            class="flex-1"
          >
          <span class="w-20 text-right">{{ fps }} fps</span>
        </div>
      </div>
    </div>
    
    <!-- 통계 -->
    <div class="mb-4">
      <h3 class="font-bold mb-2">Statistics</h3>
      <div class="bg-gray-900 p-2 rounded text-sm space-y-1">
        <div>Frames: {{ stats.frames }}</div>
        <div>Dropped: {{ stats.dropped }}</div>
        <div>Latency: {{ stats.latency }}ms</div>
        <div>Bandwidth: {{ stats.bandwidth }} kbps</div>
      </div>
    </div>
    
    <!-- 현재 스트림 -->
    <div>
      <component 
        :is="currentStreamComponent"
        v-bind="streamProps"
        @stats="updateStats"
      />
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed } from 'vue'
import MJPEGViewer from './MJPEGViewer.vue'
import WebRTCViewer from './WebRTCViewer.vue'

const streamType = ref<'mjpeg' | 'webrtc'>('mjpeg')
const resolution = ref('640x480')
const bitrate = ref(1000)
const fps = ref(30)

const stats = ref({
  frames: 0,
  dropped: 0,
  latency: 0,
  bandwidth: 0
})

const currentStreamComponent = computed(() => {
  return streamType.value === 'mjpeg' ? MJPEGViewer : WebRTCViewer
})

const streamProps = computed(() => {
  const [width, height] = resolution.value.split('x').map(Number)
  
  return {
    width,
    height,
    bitrate: bitrate.value,
    fps: fps.value
  }
})

const setStreamType = (type: 'mjpeg' | 'webrtc') => {
  streamType.value = type
}

const updateStats = (newStats: any) => {
  stats.value = { ...stats.value, ...newStats }
}
</script>
```

---

## 4. 다중 카메라 지원

### 4.1 카메라 매니저

```python
# server/core/camera_manager.py
import cv2
import threading
from typing import Dict, Optional

class MultiCameraManager:
    """다중 카메라 관리자"""
    
    def __init__(self):
        self.cameras: Dict[str, cv2.VideoCapture] = {}
        self.frames: Dict[str, any] = {}
        self.threads: Dict[str, threading.Thread] = {}
        self.running: Dict[str, bool] = {}
        self.lock = threading.Lock()
    
    def add_camera(self, name: str, source: int | str, width=640, height=480):
        """카메라 추가"""
        if name in self.cameras:
            self.remove_camera(name)
        
        cap = cv2.VideoCapture(source)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        
        if not cap.isOpened():
            raise Exception(f"Cannot open camera {source}")
        
        self.cameras[name] = cap
        self.running[name] = True
        
        # 캡처 스레드 시작
        thread = threading.Thread(
            target=self._capture_loop, 
            args=(name,), 
            daemon=True
        )
        self.threads[name] = thread
        thread.start()
    
    def remove_camera(self, name: str):
        """카메라 제거"""
        if name in self.running:
            self.running[name] = False
        
        if name in self.threads:
            self.threads[name].join(timeout=1.0)
            del self.threads[name]
        
        if name in self.cameras:
            self.cameras[name].release()
            del self.cameras[name]
        
        with self.lock:
            if name in self.frames:
                del self.frames[name]
    
    def _capture_loop(self, name: str):
        """카메라 캡처 루프"""
        cap = self.cameras[name]
        
        while self.running.get(name, False):
            ret, frame = cap.read()
            
            if ret:
                with self.lock:
                    self.frames[name] = frame
            
            time.sleep(1/30)  # 30 FPS
    
    def get_frame(self, name: str) -> Optional[any]:
        """프레임 가져오기"""
        with self.lock:
            return self.frames.get(name)
    
    def get_cameras(self) -> list:
        """카메라 목록"""
        return list(self.cameras.keys())
    
    def cleanup(self):
        """정리"""
        for name in list(self.cameras.keys()):
            self.remove_camera(name)

# 전역 매니저
camera_manager = MultiCameraManager()

# API 엔드포인트 추가
@router.get("/cameras")
async def list_cameras():
    """카메라 목록"""
    return {"cameras": camera_manager.get_cameras()}

@router.post("/cameras/{name}")
async def add_camera(name: str, source: int = 0):
    """카메라 추가"""
    try:
        camera_manager.add_camera(name, source)
        return {"status": "added", "name": name}
    except Exception as e:
        return {"error": str(e)}

@router.delete("/cameras/{name}")
async def remove_camera(name: str):
    """카메라 제거"""
    camera_manager.remove_camera(name)
    return {"status": "removed", "name": name}

@router.get("/stream/{name}")
async def camera_stream(name: str):
    """특정 카메라 스트림"""
    def generate():
        while True:
            frame = camera_manager.get_frame(name)
            
            if frame is None:
                continue
            
            ret, jpeg = cv2.imencode('.jpg', frame)
            if ret:
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + 
                       jpeg.tobytes() + b'\r\n')
    
    return StreamingResponse(generate(), media_type="multipart/x-mixed-replace; boundary=frame")
```

---

## 5. 성능 최적화

### 5.1 적응형 품질

```python
# server/core/adaptive_streaming.py
import time
from collections import deque

class AdaptiveStreaming:
    """적응형 스트리밍"""
    
    def __init__(self, target_fps=30):
        self.target_fps = target_fps
        self.frame_times = deque(maxlen=30)
        self.current_quality = 80
        self.current_resolution = (640, 480)
        
    def update_performance(self, frame_time: float):
        """성능 업데이트"""
        self.frame_times.append(frame_time)
        
        if len(self.frame_times) >= 10:
            avg_time = sum(self.frame_times) / len(self.frame_times)
            actual_fps = 1.0 / avg_time if avg_time > 0 else 0
            
            # FPS가 목표보다 낮으면 품질 조정
            if actual_fps < self.target_fps * 0.8:
                self._reduce_quality()
            elif actual_fps > self.target_fps * 1.2:
                self._increase_quality()
    
    def _reduce_quality(self):
        """품질 감소"""
        if self.current_quality > 30:
            self.current_quality = max(30, self.current_quality - 10)
        elif self.current_resolution[0] > 320:
            # 해상도 감소
            w, h = self.current_resolution
            self.current_resolution = (w // 2, h // 2)
            self.current_quality = 80
    
    def _increase_quality(self):
        """품질 증가"""
        if self.current_quality < 90:
            self.current_quality = min(90, self.current_quality + 10)
        elif self.current_resolution[0] < 1280:
            # 해상도 증가
            w, h = self.current_resolution
            self.current_resolution = (w * 2, h * 2)
            self.current_quality = 60
    
    def get_encode_params(self):
        """인코딩 파라미터"""
        return {
            'quality': self.current_quality,
            'resolution': self.current_resolution
        }
```

### 5.2 프레임 드롭핑

```python
# server/core/frame_dropper.py
import time

class FrameDropper:
    """프레임 드롭핑"""
    
    def __init__(self, target_fps=30):
        self.target_fps = target_fps
        self.frame_interval = 1.0 / target_fps
        self.last_frame_time = 0
        
        self.total_frames = 0
        self.dropped_frames = 0
    
    def should_process_frame(self) -> bool:
        """프레임 처리 여부"""
        current_time = time.time()
        
        self.total_frames += 1
        
        if current_time - self.last_frame_time >= self.frame_interval:
            self.last_frame_time = current_time
            return True
        else:
            self.dropped_frames += 1
            return False
    
    def get_stats(self):
        """통계"""
        drop_rate = self.dropped_frames / self.total_frames if self.total_frames > 0 else 0
        
        return {
            'total_frames': self.total_frames,
            'dropped_frames': self.dropped_frames,
            'drop_rate': drop_rate,
            'actual_fps': 1.0 / self.frame_interval if self.frame_interval > 0 else 0
        }
```

---

## 6. 참고 자료

- [OpenCV Video Capture](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)
- [WebRTC API](https://developer.mozilla.org/en-US/docs/Web/API/WebRTC_API)
- [aiortc (Python WebRTC)](https://github.com/aiortc/aiortc)
- [MJPEG Streaming](https://en.wikipedia.org/wiki/Motion_JPEG)

---

[← 6.4 원격 제어 구현](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/04_remote_control.md) | [목차로 돌아가기](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/06_web_control/README.md)
