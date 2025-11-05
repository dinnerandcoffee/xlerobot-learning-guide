# 7.1 VR 텔레오퍼레이션 개요

VR(Virtual Reality) 헤드셋을 활용한 로봇 원격 제어 시스템의 전체 구조를 이해합니다.

## 1. VR 텔레오퍼레이션이란?

### 1.1 개념

VR 텔레오퍼레이션은 사용자가 VR 환경에서 직관적으로 로봇을 제어하는 방식입니다.

```
    VR 헤드셋        네트워크          로봇
┌─────────────┐     ┌─────┐      ┌─────────┐
│ 사용자 동작 │ →→→ │WiFi │ →→→  │XLeRobot │
│ - 손 위치   │     │/5G  │      │- 팔 움직임│
│ - 시선 추적 │     └─────┘      │- 그리퍼 │
│ - 제스처    │                  │- 이동    │
└─────────────┘                  └─────────┘
       ↑                               ↓
   시각 피드백   ←←←←← 카메라 스트림 ←←←←←
```

### 1.2 장점

**🎯 직관성**
- 자연스러운 3D 공간 제어
- 실제 손동작과 1:1 매핑
- 학습 곡선 최소화

**🔄 실시간성**
- 낮은 지연시간 (<100ms)
- 부드러운 동작 제어
- 즉시 피드백

**📊 데이터 품질**
- 정확한 궤적 기록
- 다양한 관점에서 데이터 수집
- 고품질 데모 데이터셋

**🌐 원격성**
- 안전한 거리에서 제어
- 위험한 환경 작업 가능
- 다중 로봇 동시 제어

---

## 2. 시스템 아키텍처

### 2.1 전체 구조

```
┌────────────────────────────────────────────────────────────┐
│                      VR Client                             │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │   Quest 3    │  │  Hand Track  │  │   WebXR App      │  │
│  │ - 6DOF 추적  │  │ - 손가락 추적│  │ - 3D 시각화      │  │
│  │ - 스테레오   │  │ - 제스처 인식│  │ - UI 인터페이스  │  │
│  └──────────────┘  └──────────────┘  └──────────────────┘  │
└─────────────────────┬──────────────────────────────────────┘
                      │ WebSocket/WebRTC
                      ▼
┌────────────────────────────────────────────────────────────┐
│                   VR Server                                │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │  WebRTC Hub  │  │  Data Logger │  │  Robot Control   │  │
│  │ - 신호 처리  │  │ - 궤적 저장  │  │ - 명령 변환      │  │
│  │ - 압축 전송  │  │ - 데이터셋   │  │ - 안전 체크      │  │
│  └──────────────┘  └──────────────┘  └──────────────────┘  │
└─────────────────────┬──────────────────────────────────────┘
                      │ Serial/Ethernet
                      ▼
┌────────────────────────────────────────────────────────────┐
│                   Hardware                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────┐  │
│  │  XLeRobot    │  │  RGBD Camera │  │  Mobile Base     │  │
│  │ - 듀얼 SO100 │  │ - RealSense  │  │ - 옴니휠        │  │
│  │ - 그리퍼 x2  │  │ - 스테레오   │  │ - IMU 센서      │  │
│  └──────────────┘  └──────────────┘  └──────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

### 2.2 데이터 플로우

**Input → Processing → Output**

```python
# VR 입력 데이터
vr_input = {
    "head_pose": {
        "position": [x, y, z],
        "orientation": [qx, qy, qz, qw]
    },
    "left_hand": {
        "position": [x, y, z],
        "orientation": [qx, qy, qz, qw],
        "fingers": [...],  # 관절 각도
        "grip": 0.5        # 그리핑 강도
    },
    "right_hand": {...},
    "buttons": {
        "trigger": 0.8,
        "grip": True,
        "menu": False
    }
}

# 로봇 명령으로 변환
robot_command = {
    "left_arm": {
        "ee_position": [x', y', z'],
        "ee_orientation": [qx', qy', qz', qw'],
        "gripper": 0.5
    },
    "right_arm": {...},
    "base": {
        "linear": [vx, vy, 0],
        "angular": [0, 0, wz]
    }
}
```

---

## 3. 하드웨어 요구사항

### 3.1 VR 헤드셋

**Meta Quest 3 (권장)**
- 해상도: 2064 × 2208 per eye
- 리프레시율: 72/90/120 Hz
- 추적: Inside-out 6DOF
- 손 추적: Computer Vision 기반

**대안**
- HTC Vive Pro 2
- Varjo Aero
- Pico 4 Enterprise

### 3.2 서버 하드웨어

**최소 사양**
```
CPU: Intel i5-10400 / AMD Ryzen 5 3600
GPU: NVIDIA RTX 3060
RAM: 16 GB DDR4
Network: WiFi 6 / Gigabit Ethernet
```

**권장 사양**
```
CPU: Intel i7-12700K / AMD Ryzen 7 5800X
GPU: NVIDIA RTX 4070 Super
RAM: 32 GB DDR4-3200
Network: WiFi 6E / 2.5G Ethernet
Storage: NVMe SSD 1TB+
```

### 3.3 네트워크 요구사항

**지연시간**
- VR 제어: < 20ms (motion-to-photon)
- 로봇 명령: < 50ms
- 비디오 스트림: < 100ms

**대역폭**
- 비디오 업스트림: 50-100 Mbps
- VR 데이터: 1-5 Mbps
- 제어 명령: < 1 Mbps

---

## 4. 소프트웨어 스택

### 4.1 VR 클라이언트

**WebXR 기반**
```javascript
// Three.js + WebXR
import * as THREE from 'three';
import { VRButton } from 'three/examples/jsm/webxr/VRButton.js';

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.xr.enabled = true;

// VR 세션 시작
document.body.appendChild(VRButton.createButton(renderer));
```

**A-Frame 대안**
```html
<!DOCTYPE html>
<html>
<head>
    <script src="https://aframe.io/releases/1.4.0/aframe.min.js"></script>
</head>
<body>
    <a-scene vr-mode-ui="enabled: true">
        <a-entity id="robot-arm" 
                  gltf-model="url(robot.glb)"
                  position="0 1 -2">
        </a-entity>
    </a-scene>
</body>
</html>
```

### 4.2 서버 백엔드

**Node.js + Socket.IO**
```javascript
const express = require('express');
const http = require('http');
const socketIo = require('socket.io');

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

io.on('connection', (socket) => {
    socket.on('vr-input', (data) => {
        // VR 입력 처리
        processVRInput(data);
        
        // 로봇 제어
        sendRobotCommand(data);
    });
});
```

**Python + FastAPI 대안**
```python
from fastapi import FastAPI, WebSocket
import json

app = FastAPI()

@app.websocket("/vr")
async def vr_websocket(websocket: WebSocket):
    await websocket.accept()
    
    while True:
        data = await websocket.receive_text()
        vr_input = json.loads(data)
        
        # VR 데이터 처리
        robot_cmd = process_vr_input(vr_input)
        
        # 로봇 전송
        await send_robot_command(robot_cmd)
```

### 4.3 로봇 제어

**ROS 2 통합**
```python
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

class VRTeleopNode(Node):
    def __init__(self):
        super().__init__('vr_teleop')
        
        # Publishers
        self.arm_pub = self.create_publisher(
            PoseStamped, '/arm/target_pose', 10
        )
        
        # Subscribers
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', 
            self.joint_callback, 10
        )
    
    def vr_to_robot_pose(self, vr_pose):
        """VR 좌표 → 로봇 좌표"""
        robot_pose = PoseStamped()
        
        # 좌표계 변환
        robot_pose.pose.position.x = vr_pose['position'][0] * 0.5
        robot_pose.pose.position.y = vr_pose['position'][1] * 0.5
        robot_pose.pose.position.z = vr_pose['position'][2] * 0.5 + 0.3
        
        return robot_pose
```

---

## 5. 제어 방식

### 5.1 직접 제어 (Direct Control)

**포지션 매핑**
```
VR Hand Position → Robot EE Position
- 1:1 스케일링
- 실시간 추종
- 직관적이지만 정밀도 제한
```

**예시 코드**
```javascript
function updateRobotFromVR(vrController, robotArm) {
    // VR 컨트롤러 위치
    const vrPos = vrController.position;
    const vrRot = vrController.rotation;
    
    // 스케일링 (VR 공간 → 로봇 작업공간)
    const robotPos = {
        x: vrPos.x * 0.5,  // 50% 스케일
        y: vrPos.y * 0.5,
        z: (vrPos.z + 1.0) * 0.3  // 오프셋 적용
    };
    
    // 로봇 목표 위치 전송
    sendRobotCommand({
        type: 'ee_position',
        position: robotPos,
        orientation: vrRot
    });
}
```

### 5.2 상대 제어 (Relative Control)

**델타 제어**
```
VR Hand Movement → Robot EE Delta
- 상대적 이동량
- 누적 오차 방지
- 더 안정적
```

**예시 코드**
```javascript
let lastVRPos = null;
let robotPos = { x: 0.3, y: 0, z: 0.2 };

function updateRobotRelative(vrController) {
    const currentVRPos = vrController.position;
    
    if (lastVRPos) {
        // 델타 계산
        const delta = {
            x: currentVRPos.x - lastVRPos.x,
            y: currentVRPos.y - lastVRPos.y,
            z: currentVRPos.z - lastVRPos.z
        };
        
        // 로봇 위치 업데이트
        robotPos.x += delta.x * 0.2;  // 감쇠 적용
        robotPos.y += delta.y * 0.2;
        robotPos.z += delta.z * 0.2;
        
        sendRobotCommand({
            type: 'ee_position',
            position: robotPos
        });
    }
    
    lastVRPos = { ...currentVRPos };
}
```

### 5.3 클러치 제어 (Clutch Control)

**조건부 제어**
```
Trigger Pressed → Active Control
Trigger Released → Clutch (위치 재설정)
```

```javascript
function updateWithClutch(vrController) {
    const triggerPressed = vrController.getButton('trigger') > 0.5;
    
    if (triggerPressed) {
        // 활성 제어
        updateRobotFromVR(vrController);
        
        // 시각적 피드백
        vrController.setHaptic(0.1, 100);
    } else {
        // 클러치 모드 (위치 재설정 가능)
        showClutchIndicator(true);
    }
}
```

---

## 6. 좌표계 변환

### 6.1 VR 좌표계

**Quest 3 좌표계**
```
원점: 헤드셋 초기 위치
X축: 오른쪽 (+1m = 오른쪽 1m)
Y축: 위쪽 (+1m = 위쪽 1m)
Z축: 뒤쪽 (+1m = 사용자 뒤쪽 1m)
단위: 미터
```

### 6.2 로봇 좌표계

**XLeRobot 베이스 좌표계**
```
원점: 로봇 베이스 중심
X축: 앞쪽 (+0.3m = 앞쪽 30cm)
Y축: 왼쪽 (+0.3m = 왼쪽 30cm)
Z축: 위쪽 (+0.5m = 위쪽 50cm)
단위: 미터
```

### 6.3 변환 행렬

```python
import numpy as np

def vr_to_robot_transform():
    """VR → 로봇 좌표 변환 행렬"""
    
    # 스케일링 (VR 공간 축소)
    scale = 0.5
    
    # 회전 (VR Z축을 로봇 X축으로)
    rotation = np.array([
        [0, 0, -1],  # VR X → 로봇 -Z
        [1, 0, 0],   # VR Y → 로봇 X  
        [0, 1, 0]    # VR Z → 로봇 Y
    ])
    
    # 평행이동 (로봇 작업공간으로)
    translation = np.array([0.3, 0.0, 0.2])
    
    return scale, rotation, translation

def transform_pose(vr_pos, vr_rot):
    """VR 포즈 → 로봇 포즈"""
    scale, R, t = vr_to_robot_transform()
    
    # 위치 변환
    robot_pos = scale * (R @ vr_pos) + t
    
    # 방향 변환 (쿼터니언)
    robot_rot = quaternion_multiply(
        rotation_to_quaternion(R),
        vr_rot
    )
    
    return robot_pos, robot_rot
```

---

## 7. 안전 시스템

### 7.1 작업공간 제한

```javascript
function enforceSafetyLimits(targetPos) {
    // 작업공간 경계
    const workspace = {
        x: { min: 0.1, max: 0.5 },
        y: { min: -0.3, max: 0.3 },
        z: { min: 0.1, max: 0.4 }
    };
    
    // 클램핑
    const safePos = {
        x: Math.max(workspace.x.min, Math.min(workspace.x.max, targetPos.x)),
        y: Math.max(workspace.y.min, Math.min(workspace.y.max, targetPos.y)),
        z: Math.max(workspace.z.min, Math.min(workspace.z.max, targetPos.z))
    };
    
    // 경고 표시
    if (JSON.stringify(safePos) !== JSON.stringify(targetPos)) {
        showWorkspaceWarning();
    }
    
    return safePos;
}
```

### 7.2 비상 정지

```javascript
function initEmergencyStop() {
    // 메뉴 버튼 길게 누르기
    vrSession.addEventListener('inputsourceschange', (event) => {
        for (const inputSource of event.added) {
            inputSource.addEventListener('selectstart', (event) => {
                if (event.inputSource.profiles.includes('oculus-touch')) {
                    emergencyStopTimer = setTimeout(() => {
                        sendEmergencyStop();
                        showEmergencyNotification();
                    }, 2000); // 2초 홀드
                }
            });
            
            inputSource.addEventListener('selectend', () => {
                clearTimeout(emergencyStopTimer);
            });
        }
    });
}
```

### 7.3 충돌 감지

```python
def check_collision(robot_pos, robot_config):
    """충돌 감지"""
    
    # 자기 충돌 (팔끼리)
    if distance(left_arm_pos, right_arm_pos) < 0.15:
        return "self_collision"
    
    # 베이스 충돌
    if robot_pos[2] < 0.05:  # 너무 낮음
        return "base_collision"
    
    # 작업공간 이탈
    if not in_workspace(robot_pos):
        return "workspace_violation"
    
    return None

def handle_collision(collision_type):
    """충돌 처리"""
    # 로봇 정지
    send_stop_command()
    
    # VR 햅틱 피드백
    send_haptic_warning()
    
    # 시각적 경고
    show_collision_warning(collision_type)
```

---

## 8. 성능 최적화

### 8.1 지연시간 최소화

**Motion-to-Photon Pipeline**
```
VR Input → Network → Robot → Camera → Network → VR Display
   10ms      20ms     30ms     16ms     20ms       16ms
                    총 112ms (목표: <100ms)
```

**최적화 방법**
```javascript
// 예측 제어
function predictiveControl(vrInput, latency) {
    // 속도 추정
    const velocity = estimateVelocity(vrInput);
    
    // 미래 위치 예측
    const predictedPos = {
        x: vrInput.position.x + velocity.x * latency,
        y: vrInput.position.y + velocity.y * latency,
        z: vrInput.position.z + velocity.z * latency
    };
    
    return predictedPos;
}

// 프레임 스킵
function adaptiveFrameRate(networkLatency) {
    if (networkLatency > 50) {
        // 고지연: 30fps로 감소
        setUpdateRate(30);
    } else {
        // 저지연: 90fps 유지
        setUpdateRate(90);
    }
}
```

### 8.2 네트워크 최적화

```javascript
// 데이터 압축
function compressVRData(vrInput) {
    return {
        // 위치 (mm 정밀도로 양자화)
        pos: [
            Math.round(vrInput.position.x * 1000),
            Math.round(vrInput.position.y * 1000),
            Math.round(vrInput.position.z * 1000)
        ],
        // 회전 (압축된 쿼터니언)
        rot: compressQuaternion(vrInput.rotation),
        // 버튼 (비트마스크)
        btn: packButtons(vrInput.buttons)
    };
}

// 델타 압축
function deltaCompress(current, previous) {
    const threshold = 0.001; // 1mm
    
    if (distance(current.pos, previous.pos) < threshold) {
        return null; // 전송 스킵
    }
    
    return {
        delta_pos: subtract(current.pos, previous.pos),
        timestamp: Date.now()
    };
}
```

---

## 9. 개발 환경 설정

### 9.1 개발 도구

**Meta Quest Developer Hub**
- ADB 연결
- 앱 배포
- 성능 모니터링

**Chrome DevTools**
- WebXR 디버깅
- 네트워크 분석
- 성능 프로파일링

### 9.2 테스트 방법

**시뮬레이터 테스트**
```javascript
// WebXR Emulator 사용
import { WebXRManager } from 'three';

const manager = new WebXRManager(renderer, gl);
manager.enabled = true;

// 가상 컨트롤러 시뮬레이션
const controller = manager.getController(0);
controller.position.set(0.2, 1.0, -0.3);
```

**유닛 테스트**
```javascript
describe('VR Coordinate Transform', () => {
    test('VR to Robot position mapping', () => {
        const vrPos = { x: 0.1, y: 0.2, z: -0.3 };
        const robotPos = vrToRobotPos(vrPos);
        
        expect(robotPos.x).toBeCloseTo(0.25);
        expect(robotPos.y).toBeCloseTo(0.1);
        expect(robotPos.z).toBeCloseTo(0.35);
    });
});
```

---

## 10. 참고 자료

- [WebXR Device API](https://www.w3.org/TR/webxr/)
- [Meta Quest Developer Documentation](https://developer.oculus.com/documentation/)
- [Three.js WebXR Examples](https://threejs.org/examples/?q=webxr)
- [A-Frame VR Framework](https://aframe.io/)

---

[← 7장 목차](README.md) | [다음: 7.2 Quest3 VR 설정 →](02_quest3_setup.md)
