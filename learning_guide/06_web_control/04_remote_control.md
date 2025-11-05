# 6.4 원격 제어 구현

키보드, 조이스틱을 통한 실시간 로봇 제어 시스템을 구현합니다.

## 1. 키보드 제어

### 1.1 키 이벤트 핸들링

```vue
<!-- src/components/KeyboardControl.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4" tabindex="0" @keydown="handleKeyDown" @keyup="handleKeyUp">
    <h2 class="text-xl font-bold mb-3">Keyboard Control</h2>
    
    <!-- 키 가이드 -->
    <div class="grid grid-cols-2 gap-4 mb-4">
      <div>
        <h3 class="font-bold mb-2">Movement</h3>
        <div class="text-sm space-y-1">
          <div>W/S: Forward/Backward</div>
          <div>A/D: Left/Right</div>
          <div>Q/E: Rotate Left/Right</div>
        </div>
      </div>
      
      <div>
        <h3 class="font-bold mb-2">Arm Control</h3>
        <div class="text-sm space-y-1">
          <div>↑/↓: Y axis</div>
          <div>←/→: X axis</div>
          <div>Space: Open Gripper</div>
          <div>Enter: Close Gripper</div>
        </div>
      </div>
    </div>
    
    <!-- 활성화 상태 -->
    <div class="flex items-center gap-2 mb-2">
      <div 
        :class="isActive ? 'bg-green-500' : 'bg-red-500'"
        class="w-3 h-3 rounded-full"
      ></div>
      <span>{{ isActive ? 'Active' : 'Click to activate' }}</span>
    </div>
    
    <!-- 현재 입력 -->
    <div v-if="currentKeys.size > 0" class="text-sm text-blue-400">
      Active keys: {{ Array.from(currentKeys).join(', ') }}
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import { setEEPosition, controlGripper } from '../api/robot'

const isActive = ref(false)
const currentKeys = ref(new Set<string>())

// 키 상태
const keyState = ref({
  w: false, s: false,  // forward/backward
  a: false, d: false,  // left/right
  q: false, e: false,  // rotate
  ArrowUp: false, ArrowDown: false,    // Y axis
  ArrowLeft: false, ArrowRight: false, // X axis
})

// 현재 EE 위치 (추정)
const eePosition = ref({ x: 0.15, y: 0.0 })

const handleKeyDown = (event: KeyboardEvent) => {
  if (!isActive.value) return
  
  const key = event.key.toLowerCase()
  
  if (key in keyState.value || event.key in keyState.value) {
    currentKeys.value.add(key === ' ' ? 'Space' : event.key)
    
    // 키 상태 업데이트
    if (key in keyState.value) {
      keyState.value[key as keyof typeof keyState.value] = true
    } else if (event.key in keyState.value) {
      keyState.value[event.key as keyof typeof keyState.value] = true
    }
    
    // 즉시 실행 명령
    handleImmediateCommands(event.key)
    
    event.preventDefault()
  }
}

const handleKeyUp = (event: KeyboardEvent) => {
  if (!isActive.value) return
  
  const key = event.key.toLowerCase()
  
  currentKeys.value.delete(key === ' ' ? 'Space' : event.key)
  
  // 키 상태 업데이트
  if (key in keyState.value) {
    keyState.value[key as keyof typeof keyState.value] = false
  } else if (event.key in keyState.value) {
    keyState.value[event.key as keyof typeof keyState.value] = false
  }
}

const handleImmediateCommands = async (key: string) => {
  switch (key) {
    case ' ': // Space
      await controlGripper({ action: 'open' })
      break
    case 'Enter':
      await controlGripper({ action: 'close' })
      break
  }
}

// 연속 제어 (60Hz)
let controlInterval: number | null = null

const continuousControl = async () => {
  const step = 0.01 // 1cm step
  let dx = 0, dy = 0
  
  // X/Y 축 제어
  if (keyState.value.ArrowLeft) dx -= step
  if (keyState.value.ArrowRight) dx += step
  if (keyState.value.ArrowUp) dy += step
  if (keyState.value.ArrowDown) dy -= step
  
  // 이동 베이스 제어 (WebSocket으로 전송)
  if (keyState.value.w || keyState.value.s || keyState.value.a || keyState.value.d) {
    const moveCmd = {
      type: 'move',
      forward: keyState.value.w ? 1 : (keyState.value.s ? -1 : 0),
      strafe: keyState.value.d ? 1 : (keyState.value.a ? -1 : 0),
      rotate: keyState.value.e ? 1 : (keyState.value.q ? -1 : 0)
    }
    
    // WebSocket 전송 (후에 구현)
    console.log('Move command:', moveCmd)
  }
  
  // EE 제어
  if (dx !== 0 || dy !== 0) {
    eePosition.value.x += dx
    eePosition.value.y += dy
    
    try {
      await setEEPosition(eePosition.value)
    } catch (error) {
      console.error('EE control failed:', error)
    }
  }
}

const activate = () => {
  isActive.value = true
}

const deactivate = () => {
  isActive.value = false
  currentKeys.value.clear()
}

onMounted(() => {
  // 60Hz 제어 루프
  controlInterval = window.setInterval(continuousControl, 16) // ~60fps
  
  // 포커스 이벤트
  window.addEventListener('focus', activate)
  window.addEventListener('blur', deactivate)
})

onUnmounted(() => {
  if (controlInterval !== null) {
    clearInterval(controlInterval)
  }
  
  window.removeEventListener('focus', activate)
  window.removeEventListener('blur', deactivate)
})
</script>
```

### 1.2 키 매핑 설정

```typescript
// src/composables/useKeyboardControl.ts
import { ref, onMounted, onUnmounted } from 'vue'

export interface KeyMapping {
  [key: string]: () => void
}

export function useKeyboardControl(keyMapping: KeyMapping) {
  const isActive = ref(false)
  const pressedKeys = ref(new Set<string>())
  
  const handleKeyDown = (event: KeyboardEvent) => {
    if (!isActive.value) return
    
    const key = event.key.toLowerCase()
    
    if (key in keyMapping && !pressedKeys.value.has(key)) {
      pressedKeys.value.add(key)
      keyMapping[key]()
      event.preventDefault()
    }
  }
  
  const handleKeyUp = (event: KeyboardEvent) => {
    const key = event.key.toLowerCase()
    pressedKeys.value.delete(key)
  }
  
  const activate = () => {
    isActive.value = true
  }
  
  const deactivate = () => {
    isActive.value = false
    pressedKeys.value.clear()
  }
  
  onMounted(() => {
    window.addEventListener('keydown', handleKeyDown)
    window.addEventListener('keyup', handleKeyUp)
  })
  
  onUnmounted(() => {
    window.removeEventListener('keydown', handleKeyDown)
    window.removeEventListener('keyup', handleKeyUp)
  })
  
  return {
    isActive,
    pressedKeys,
    activate,
    deactivate
  }
}
```

---

## 2. 가상 조이스틱

### 2.1 조이스틱 컴포넌트

```vue
<!-- src/components/VirtualJoystick.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4">
    <h2 class="text-xl font-bold mb-3">Virtual Joystick</h2>
    
    <div class="flex justify-center gap-8">
      <!-- 이동 조이스틱 -->
      <div class="text-center">
        <h3 class="mb-2">Movement</h3>
        <div 
          ref="moveStickContainer"
          @mousedown="startDrag($event, 'move')"
          @touchstart="startDrag($event, 'move')"
          class="relative w-32 h-32 bg-gray-700 rounded-full cursor-pointer select-none"
        >
          <!-- 조이스틱 핸들 -->
          <div 
            :style="moveStickStyle"
            class="absolute w-8 h-8 bg-blue-500 rounded-full transition-all duration-75"
          ></div>
          
          <!-- 중심점 -->
          <div class="absolute top-1/2 left-1/2 w-1 h-1 bg-white rounded-full transform -translate-x-1/2 -translate-y-1/2"></div>
        </div>
        
        <div class="mt-2 text-sm">
          X: {{ moveValues.x.toFixed(2) }}<br>
          Y: {{ moveValues.y.toFixed(2) }}
        </div>
      </div>
      
      <!-- 회전 조이스틱 -->
      <div class="text-center">
        <h3 class="mb-2">Rotation</h3>
        <div 
          ref="rotateStickContainer"
          @mousedown="startDrag($event, 'rotate')"
          @touchstart="startDrag($event, 'rotate')"
          class="relative w-32 h-32 bg-gray-700 rounded-full cursor-pointer select-none"
        >
          <!-- 조이스틱 핸들 -->
          <div 
            :style="rotateStickStyle"
            class="absolute w-8 h-8 bg-red-500 rounded-full transition-all duration-75"
          ></div>
          
          <!-- 중심점 -->
          <div class="absolute top-1/2 left-1/2 w-1 h-1 bg-white rounded-full transform -translate-x-1/2 -translate-y-1/2"></div>
        </div>
        
        <div class="mt-2 text-sm">
          Rotate: {{ rotateValues.x.toFixed(2) }}
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, onMounted, onUnmounted } from 'vue'

interface JoystickValues {
  x: number
  y: number
}

const moveStickContainer = ref<HTMLElement>()
const rotateStickContainer = ref<HTMLElement>()

const moveValues = ref<JoystickValues>({ x: 0, y: 0 })
const rotateValues = ref<JoystickValues>({ x: 0, y: 0 })

const isDragging = ref<'move' | 'rotate' | null>(null)

// 조이스틱 핸들 위치 계산
const moveStickStyle = computed(() => {
  const radius = 64 - 16 // 컨테이너 반지름 - 핸들 반지름
  const x = moveValues.value.x * radius + 64 - 16
  const y = -moveValues.value.y * radius + 64 - 16
  
  return {
    left: `${x}px`,
    top: `${y}px`
  }
})

const rotateStickStyle = computed(() => {
  const radius = 64 - 16
  const x = rotateValues.value.x * radius + 64 - 16
  const y = -rotateValues.value.y * radius + 64 - 16
  
  return {
    left: `${x}px`,
    top: `${y}px`
  }
})

const startDrag = (event: MouseEvent | TouchEvent, type: 'move' | 'rotate') => {
  isDragging.value = type
  updatePosition(event, type)
  
  // 마우스 이벤트
  if (event instanceof MouseEvent) {
    document.addEventListener('mousemove', handleMouseMove)
    document.addEventListener('mouseup', stopDrag)
  }
  // 터치 이벤트
  else {
    document.addEventListener('touchmove', handleTouchMove, { passive: false })
    document.addEventListener('touchend', stopDrag)
  }
}

const handleMouseMove = (event: MouseEvent) => {
  if (isDragging.value) {
    updatePosition(event, isDragging.value)
  }
}

const handleTouchMove = (event: TouchEvent) => {
  if (isDragging.value) {
    event.preventDefault()
    updatePosition(event, isDragging.value)
  }
}

const stopDrag = () => {
  isDragging.value = null
  
  // 중심으로 복귀
  moveValues.value = { x: 0, y: 0 }
  rotateValues.value = { x: 0, y: 0 }
  
  // 이벤트 리스너 제거
  document.removeEventListener('mousemove', handleMouseMove)
  document.removeEventListener('mouseup', stopDrag)
  document.removeEventListener('touchmove', handleTouchMove)
  document.removeEventListener('touchend', stopDrag)
}

const updatePosition = (event: MouseEvent | TouchEvent, type: 'move' | 'rotate') => {
  const container = type === 'move' ? moveStickContainer.value : rotateStickContainer.value
  
  if (!container) return
  
  const rect = container.getBoundingClientRect()
  const centerX = rect.left + rect.width / 2
  const centerY = rect.top + rect.height / 2
  
  let clientX: number, clientY: number
  
  if (event instanceof MouseEvent) {
    clientX = event.clientX
    clientY = event.clientY
  } else {
    clientX = event.touches[0].clientX
    clientY = event.touches[0].clientY
  }
  
  // 상대 위치 계산
  const deltaX = clientX - centerX
  const deltaY = centerY - clientY // Y축 반전
  
  // 정규화 (-1 ~ 1)
  const radius = rect.width / 2
  let x = deltaX / radius
  let y = deltaY / radius
  
  // 원형 범위 제한
  const distance = Math.sqrt(x * x + y * y)
  if (distance > 1) {
    x = x / distance
    y = y / distance
  }
  
  // 값 업데이트
  if (type === 'move') {
    moveValues.value = { x, y }
  } else {
    rotateValues.value = { x, y }
  }
}

// WebSocket으로 값 전송 (60Hz)
let sendInterval: number | null = null

const sendJoystickData = () => {
  // WebSocket 전송 로직
  const data = {
    type: 'joystick',
    move: moveValues.value,
    rotate: rotateValues.value,
    timestamp: Date.now()
  }
  
  console.log('Joystick data:', data)
  // websocket.send(JSON.stringify(data))
}

onMounted(() => {
  sendInterval = window.setInterval(sendJoystickData, 16) // ~60fps
})

onUnmounted(() => {
  if (sendInterval !== null) {
    clearInterval(sendInterval)
  }
})
</script>
```

---

## 3. WebSocket 실시간 제어

### 3.1 WebSocket 클라이언트

```typescript
// src/composables/useWebSocket.ts
import { ref, onMounted, onUnmounted } from 'vue'

export function useWebSocket(url: string) {
  const socket = ref<WebSocket | null>(null)
  const isConnected = ref(false)
  const lastMessage = ref<any>(null)
  
  const connect = () => {
    socket.value = new WebSocket(url)
    
    socket.value.onopen = () => {
      isConnected.value = true
      console.log('WebSocket connected')
    }
    
    socket.value.onmessage = (event) => {
      try {
        lastMessage.value = JSON.parse(event.data)
      } catch (error) {
        console.error('Failed to parse message:', error)
      }
    }
    
    socket.value.onclose = () => {
      isConnected.value = false
      console.log('WebSocket disconnected')
      
      // 자동 재연결 (5초 후)
      setTimeout(connect, 5000)
    }
    
    socket.value.onerror = (error) => {
      console.error('WebSocket error:', error)
    }
  }
  
  const send = (data: any) => {
    if (socket.value && isConnected.value) {
      socket.value.send(JSON.stringify(data))
    }
  }
  
  const close = () => {
    if (socket.value) {
      socket.value.close()
    }
  }
  
  onMounted(() => {
    connect()
  })
  
  onUnmounted(() => {
    close()
  })
  
  return {
    isConnected,
    lastMessage,
    send,
    close
  }
}
```

### 3.2 통합 제어 컴포넌트

```vue
<!-- src/components/RealtimeControl.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4">
    <h2 class="text-xl font-bold mb-3">Realtime Control</h2>
    
    <!-- 연결 상태 -->
    <div class="flex items-center gap-2 mb-4">
      <div 
        :class="wsIsConnected ? 'bg-green-500' : 'bg-red-500'"
        class="w-3 h-3 rounded-full"
      ></div>
      <span>WebSocket {{ wsIsConnected ? 'Connected' : 'Disconnected' }}</span>
    </div>
    
    <!-- 제어 모드 -->
    <div class="mb-4">
      <h3 class="font-bold mb-2">Control Mode</h3>
      <div class="flex gap-2">
        <button 
          @click="setControlMode('keyboard')"
          :class="controlMode === 'keyboard' ? 'bg-blue-600' : 'bg-gray-600'"
          class="px-3 py-1 rounded"
        >
          Keyboard
        </button>
        <button 
          @click="setControlMode('joystick')"
          :class="controlMode === 'joystick' ? 'bg-blue-600' : 'bg-gray-600'"
          class="px-3 py-1 rounded"
        >
          Joystick
        </button>
      </div>
    </div>
    
    <!-- 실시간 데이터 -->
    <div class="mb-4">
      <h3 class="font-bold mb-2">Live Data</h3>
      <div class="bg-gray-900 p-2 rounded text-xs font-mono">
        <div>Sent: {{ sentCount }} msgs</div>
        <div>Received: {{ receivedCount }} msgs</div>
        <div>Latency: {{ latency }}ms</div>
      </div>
    </div>
    
    <!-- 명령 히스토리 -->
    <div>
      <h3 class="font-bold mb-2">Command History</h3>
      <div class="bg-gray-900 p-2 rounded text-xs max-h-32 overflow-y-auto">
        <div 
          v-for="(cmd, index) in commandHistory.slice(-10)" 
          :key="index"
          class="mb-1"
        >
          <span class="text-gray-500">{{ formatTime(cmd.timestamp) }}</span>
          <span class="text-blue-400">{{ cmd.type }}</span>
          <span>{{ JSON.stringify(cmd.data) }}</span>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, watch } from 'vue'
import { useWebSocket } from '../composables/useWebSocket'

const API_WS_URL = import.meta.env.VITE_API_WS_URL || 'ws://localhost:8000/ws'

const { isConnected: wsIsConnected, lastMessage, send } = useWebSocket(API_WS_URL)

const controlMode = ref<'keyboard' | 'joystick'>('joystick')
const sentCount = ref(0)
const receivedCount = ref(0)
const latency = ref(0)

interface Command {
  type: string
  data: any
  timestamp: number
}

const commandHistory = ref<Command[]>([])

const setControlMode = (mode: 'keyboard' | 'joystick') => {
  controlMode.value = mode
  
  send({
    type: 'control_mode',
    mode: mode
  })
}

const sendCommand = (type: string, data: any) => {
  const timestamp = Date.now()
  
  const command = {
    type,
    data,
    timestamp
  }
  
  send(command)
  
  commandHistory.value.push(command)
  sentCount.value++
}

const formatTime = (timestamp: number) => {
  return new Date(timestamp).toLocaleTimeString()
}

// 응답 처리
watch(lastMessage, (message) => {
  if (message) {
    receivedCount.value++
    
    if (message.type === 'pong' && message.timestamp) {
      latency.value = Date.now() - message.timestamp
    }
  }
})

// 핑 전송 (지연 시간 측정)
setInterval(() => {
  if (wsIsConnected.value) {
    send({
      type: 'ping',
      timestamp: Date.now()
    })
  }
}, 2000)

// 외부에서 사용할 수 있도록 expose
defineExpose({
  sendCommand
})
</script>
```

---

## 4. 게임패드 지원

### 4.1 게임패드 API

```typescript
// src/composables/useGamepad.ts
import { ref, onMounted, onUnmounted } from 'vue'

export function useGamepad() {
  const gamepads = ref<Gamepad[]>([])
  const isConnected = ref(false)
  
  let animationId: number | null = null
  
  const updateGamepads = () => {
    const gamepadList = navigator.getGamepads()
    const connectedGamepads = Array.from(gamepadList).filter(
      (gamepad): gamepad is Gamepad => gamepad !== null
    )
    
    gamepads.value = connectedGamepads
    isConnected.value = connectedGamepads.length > 0
  }
  
  const getGamepadInput = (gamepadIndex = 0) => {
    const gamepad = gamepads.value[gamepadIndex]
    
    if (!gamepad) return null
    
    return {
      // 좌측 스틱 (이동)
      leftStick: {
        x: gamepad.axes[0] || 0,
        y: gamepad.axes[1] || 0
      },
      
      // 우측 스틱 (회전)
      rightStick: {
        x: gamepad.axes[2] || 0,
        y: gamepad.axes[3] || 0
      },
      
      // 버튼
      buttons: {
        a: gamepad.buttons[0]?.pressed || false,
        b: gamepad.buttons[1]?.pressed || false,
        x: gamepad.buttons[2]?.pressed || false,
        y: gamepad.buttons[3]?.pressed || false,
        lb: gamepad.buttons[4]?.pressed || false,
        rb: gamepad.buttons[5]?.pressed || false,
        lt: gamepad.buttons[6]?.value || 0,
        rt: gamepad.buttons[7]?.value || 0,
        select: gamepad.buttons[8]?.pressed || false,
        start: gamepad.buttons[9]?.pressed || false,
      }
    }
  }
  
  const pollGamepads = () => {
    updateGamepads()
    animationId = requestAnimationFrame(pollGamepads)
  }
  
  onMounted(() => {
    // 이벤트 리스너
    window.addEventListener('gamepadconnected', updateGamepads)
    window.addEventListener('gamepaddisconnected', updateGamepads)
    
    // 폴링 시작
    pollGamepads()
  })
  
  onUnmounted(() => {
    window.removeEventListener('gamepadconnected', updateGamepads)
    window.removeEventListener('gamepaddisconnected', updateGamepads)
    
    if (animationId !== null) {
      cancelAnimationFrame(animationId)
    }
  })
  
  return {
    gamepads,
    isConnected,
    getGamepadInput
  }
}
```

### 4.2 게임패드 제어 컴포넌트

```vue
<!-- src/components/GamepadControl.vue -->
<template>
  <div class="bg-gray-800 rounded-lg p-4">
    <h2 class="text-xl font-bold mb-3">Gamepad Control</h2>
    
    <div v-if="!isConnected" class="text-center text-gray-500 py-8">
      <div class="text-4xl mb-2">🎮</div>
      <div>No gamepad connected</div>
      <div class="text-sm">Connect a gamepad and press any button</div>
    </div>
    
    <div v-else>
      <!-- 게임패드 정보 -->
      <div class="mb-4">
        <div class="text-sm text-gray-400">{{ gamepads[0]?.id }}</div>
      </div>
      
      <!-- 스틱 값 -->
      <div class="grid grid-cols-2 gap-4 mb-4">
        <div>
          <h3 class="font-bold mb-1">Left Stick (Move)</h3>
          <div class="text-sm">
            X: {{ input?.leftStick.x.toFixed(2) || '0.00' }}<br>
            Y: {{ input?.leftStick.y.toFixed(2) || '0.00' }}
          </div>
        </div>
        
        <div>
          <h3 class="font-bold mb-1">Right Stick (Look)</h3>
          <div class="text-sm">
            X: {{ input?.rightStick.x.toFixed(2) || '0.00' }}<br>
            Y: {{ input?.rightStick.y.toFixed(2) || '0.00' }}
          </div>
        </div>
      </div>
      
      <!-- 버튼 상태 -->
      <div class="mb-4">
        <h3 class="font-bold mb-2">Buttons</h3>
        <div class="grid grid-cols-4 gap-2 text-sm">
          <div :class="input?.buttons.a ? 'text-green-400' : 'text-gray-600'">A</div>
          <div :class="input?.buttons.b ? 'text-red-400' : 'text-gray-600'">B</div>
          <div :class="input?.buttons.x ? 'text-blue-400' : 'text-gray-600'">X</div>
          <div :class="input?.buttons.y ? 'text-yellow-400' : 'text-gray-600'">Y</div>
        </div>
      </div>
      
      <!-- 트리거 -->
      <div>
        <h3 class="font-bold mb-2">Triggers</h3>
        <div class="flex gap-4">
          <div>LT: {{ (input?.buttons.lt || 0).toFixed(2) }}</div>
          <div>RT: {{ (input?.buttons.rt || 0).toFixed(2) }}</div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, onMounted, onUnmounted } from 'vue'
import { useGamepad } from '../composables/useGamepad'

const { gamepads, isConnected, getGamepadInput } = useGamepad()

const input = ref<ReturnType<typeof getGamepadInput>>(null)

let controlInterval: number | null = null

const processGamepadInput = () => {
  if (!isConnected.value) return
  
  const currentInput = getGamepadInput(0)
  input.value = currentInput
  
  if (!currentInput) return
  
  // 데드존 적용
  const deadzone = 0.1
  const leftStick = {
    x: Math.abs(currentInput.leftStick.x) > deadzone ? currentInput.leftStick.x : 0,
    y: Math.abs(currentInput.leftStick.y) > deadzone ? currentInput.leftStick.y : 0
  }
  
  const rightStick = {
    x: Math.abs(currentInput.rightStick.x) > deadzone ? currentInput.rightStick.x : 0,
    y: Math.abs(currentInput.rightStick.y) > deadzone ? currentInput.rightStick.y : 0
  }
  
  // 로봇 제어 명령 생성
  const command = {
    type: 'gamepad',
    move: {
      x: leftStick.x,
      y: -leftStick.y  // Y축 반전
    },
    look: {
      x: rightStick.x,
      y: -rightStick.y
    },
    buttons: currentInput.buttons
  }
  
  // WebSocket으로 전송
  console.log('Gamepad command:', command)
}

onMounted(() => {
  // 60Hz로 게임패드 입력 처리
  controlInterval = window.setInterval(processGamepadInput, 16)
})

onUnmounted(() => {
  if (controlInterval !== null) {
    clearInterval(controlInterval)
  }
})
</script>
```

---

## 5. 명령 큐 & 버퍼링

### 5.1 명령 스케줄러

```typescript
// src/utils/CommandScheduler.ts
export interface Command {
  id: string
  type: string
  data: any
  priority: number
  timestamp: number
  retry: number
}

export class CommandScheduler {
  private queue: Command[] = []
  private isProcessing = false
  private maxRetries = 3
  
  addCommand(type: string, data: any, priority = 0): string {
    const command: Command = {
      id: `cmd_${Date.now()}_${Math.random().toString(36).substr(2, 9)}`,
      type,
      data,
      priority,
      timestamp: Date.now(),
      retry: 0
    }
    
    this.queue.push(command)
    this.queue.sort((a, b) => b.priority - a.priority) // 높은 우선순위 먼저
    
    this.processQueue()
    
    return command.id
  }
  
  private async processQueue() {
    if (this.isProcessing || this.queue.length === 0) return
    
    this.isProcessing = true
    
    while (this.queue.length > 0) {
      const command = this.queue.shift()!
      
      try {
        await this.executeCommand(command)
      } catch (error) {
        console.error(`Command ${command.id} failed:`, error)
        
        if (command.retry < this.maxRetries) {
          command.retry++
          this.queue.unshift(command) // 앞에 다시 추가
        }
      }
    }
    
    this.isProcessing = false
  }
  
  private async executeCommand(command: Command): Promise<void> {
    // 명령 실행 로직
    return new Promise((resolve, reject) => {
      // WebSocket 전송 또는 API 호출
      setTimeout(() => {
        // 시뮬레이션
        if (Math.random() > 0.9) {
          reject(new Error('Random failure'))
        } else {
          resolve()
        }
      }, 10)
    })
  }
  
  clearQueue() {
    this.queue = []
  }
  
  getQueueLength(): number {
    return this.queue.length
  }
}
```

---

## 6. 참고 자료

- [Gamepad API (MDN)](https://developer.mozilla.org/en-US/docs/Web/API/Gamepad_API)
- [KeyboardEvent (MDN)](https://developer.mozilla.org/en-US/docs/Web/API/KeyboardEvent)
- [WebSocket API](https://developer.mozilla.org/en-US/docs/Web/API/WebSocket)
- [Vue.js Composables](https://vuejs.org/guide/reusability/composables.html)

---

[← 6.3 클라이언트 구성](03_client_setup.md) | [다음: 6.5 실시간 비디오 스트리밍 →](05_video_streaming.md)
