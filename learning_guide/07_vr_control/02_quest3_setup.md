# 7.2 Quest3 VR 설정

Meta Quest 3 헤드셋을 VR 텔레오퍼레이션용으로 설정하는 방법을 학습합니다.

## 1. Quest 3 하드웨어 설정

### 1.1 초기 설정

**박스 개봉 후**
```
1. 헤드셋 충전 (USB-C)
2. Meta Quest 모바일 앱 설치
3. Facebook/Meta 계정 연결
4. 가디언(Guardian) 경계 설정
5. 핸드 트래킹 캘리브레이션
```

**IPD(동공 간 거리) 조정**
```
설정 → 빠른 설정 → IPD 조정
- 58mm ~ 72mm 범위
- 양쪽 눈의 선명도 확인
- 편안한 착용감 확인
```

### 1.2 개발자 모드 활성화

**Step 1: Meta Quest Developer Hub 설치**
```bash
# PC에서 다운로드
https://developer.oculus.com/downloads/package/oculus-developer-hub-win/

# 또는 웹 버전 사용
https://developer.oculus.com/manage/
```

**Step 2: 개발자 계정 등록**
```
1. Meta Developer 웹사이트 방문
2. 개발자 계정 생성/로그인
3. 조직 생성 (개인 개발자도 필요)
4. 헤드셋을 조직에 등록
```

**Step 3: 헤드셋에서 개발자 모드 활성화**
```
설정 → 시스템 → 개발자
- 개발자 모드 ON
- USB 디버깅 허용
- 알 수 없는 소스 허용
```

### 1.3 PC 연결 설정

**ADB 설정**
```bash
# Android SDK Platform Tools 다운로드
https://developer.android.com/studio/releases/platform-tools

# 환경 변수 설정 (Windows)
set PATH=%PATH%;C:\platform-tools

# 또는 (Linux/Mac)
export PATH=$PATH:~/platform-tools

# 연결 확인
adb devices
```

**Quest 3 연결 확인**
```bash
# USB 연결 시
adb devices
# 출력: 1WMHH123456789 device

# 무선 연결 설정
adb tcpip 5555
adb connect 192.168.1.XXX:5555
```

---

## 2. WebXR 개발 환경

### 2.1 브라우저 설정

**Meta Browser (기본)**
```
Quest 3에서 기본 제공
WebXR 완전 지원
개발자 도구 제한적
```

**Chrome 개발자 버전**
```bash
# APK 다운로드 및 설치
adb install chrome-dev.apk

# 또는 Meta Browser에서
chrome://flags
- WebXR 실험 기능 활성화
- 개발자 모드 활성화
```

### 2.2 로컬 개발 서버

**HTTPS 필수** (WebXR 요구사항)
```bash
# Node.js 로컬 서버
npm install -g http-server
http-server -S -p 8443

# 또는 Python
python3 -m http.server 8443 --bind 0.0.0.0
```

**SSL 인증서 생성**
```bash
# OpenSSL 사용
openssl req -x509 -newkey rsa:4096 -keyout key.pem -out cert.pem -days 365 -nodes

# mkcert 사용 (권장)
brew install mkcert
mkcert -install
mkcert localhost 127.0.0.1 ::1 192.168.1.XXX
```

**Express.js HTTPS 서버**
```javascript
const express = require('express');
const https = require('https');
const fs = require('fs');

const app = express();

// 정적 파일 제공
app.use(express.static('public'));

// SSL 옵션
const options = {
  key: fs.readFileSync('key.pem'),
  cert: fs.readFileSync('cert.pem')
};

// HTTPS 서버 시작
https.createServer(options, app).listen(8443, '0.0.0.0', () => {
  console.log('HTTPS Server running on https://localhost:8443');
});
```

### 2.3 WebXR 폴리필 및 라이브러리

**기본 설정**
```html
<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>XLeRobot VR Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    
    <!-- WebXR 폴리필 -->
    <script src="https://cdn.jsdelivr.net/npm/webxr-polyfill@latest/build/webxr-polyfill.min.js"></script>
    
    <!-- Three.js -->
    <script src="https://cdn.jsdelivr.net/npm/three@0.158.0/build/three.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/three@0.158.0/examples/js/webxr/VRButton.js"></script>
    
    <style>
        body { margin: 0; background: #000; overflow: hidden; }
        canvas { display: block; }
    </style>
</head>
<body>
    <div id="info">
        <button id="enterVR">Enter VR</button>
    </div>
    
    <script>
        // WebXR 지원 확인
        if ('xr' in navigator) {
            navigator.xr.isSessionSupported('immersive-vr').then((supported) => {
                if (supported) {
                    console.log('WebXR VR supported');
                } else {
                    console.log('WebXR VR not supported');
                }
            });
        } else {
            console.log('WebXR not available');
        }
    </script>
</body>
</html>
```

---

## 3. 핸드 트래킹 설정

### 3.1 Quest 3 핸드 트래킹 활성화

**설정 방법**
```
설정 → 움직임 추적 → 핸드 트래킹
- 핸드 트래킹 ON
- 자동 전환 ON (컨트롤러 ↔ 핸드)
- 주파수: 60Hz (기본)
```

**캘리브레이션**
```
설정 → 움직임 추적 → 핸드 트래킹 재설정
1. 손바닥을 카메라에 보여주기
2. 손가락 구부리기/펴기
3. 주먹 쥐기/펴기
4. 손목 회전
```

### 3.2 WebXR 핸드 트래킹 API

**기본 구현**
```javascript
class HandTracker {
    constructor(renderer, scene) {
        this.renderer = renderer;
        this.scene = scene;
        this.hands = [];
        
        this.setupHandTracking();
    }
    
    setupHandTracking() {
        const controller1 = this.renderer.xr.getController(0);
        const controller2 = this.renderer.xr.getController(1);
        
        // 핸드 모델 로드
        const loader = new THREE.GLTFLoader();
        
        loader.load('models/hand_left.glb', (gltf) => {
            this.leftHand = gltf.scene;
            this.scene.add(this.leftHand);
        });
        
        loader.load('models/hand_right.glb', (gltf) => {
            this.rightHand = gltf.scene;
            this.scene.add(this.rightHand);
        });
        
        // 핸드 트래킹 이벤트
        this.renderer.xr.addEventListener('sessionstart', () => {
            const session = this.renderer.xr.getSession();
            
            if (session.enabledFeatures.includes('hand-tracking')) {
                console.log('Hand tracking enabled');
                this.startHandTracking(session);
            }
        });
    }
    
    startHandTracking(session) {
        session.requestAnimationFrame((time, frame) => {
            this.updateHands(frame);
            session.requestAnimationFrame(arguments.callee);
        });
    }
    
    updateHands(frame) {
        const referenceSpace = this.renderer.xr.getReferenceSpace();
        
        // 양손 처리
        for (let i = 0; i < 2; i++) {
            const inputSource = this.renderer.xr.getController(i);
            
            if (inputSource && inputSource.hand) {
                const hand = inputSource.hand;
                const handModel = i === 0 ? this.leftHand : this.rightHand;
                
                // 손 관절 위치 업데이트
                for (const [jointName, joint] of hand.entries()) {
                    const jointPose = frame.getJointPose(joint, referenceSpace);
                    
                    if (jointPose) {
                        // 관절 위치 적용
                        this.updateJoint(handModel, jointName, jointPose);
                    }
                }
            }
        }
    }
    
    updateJoint(handModel, jointName, pose) {
        const joint = handModel.getObjectByName(jointName);
        
        if (joint) {
            joint.position.copy(pose.transform.position);
            joint.quaternion.copy(pose.transform.orientation);
        }
    }
}
```

### 3.3 제스처 인식

**기본 제스처**
```javascript
class GestureRecognizer {
    constructor() {
        this.gestures = {
            pinch: false,
            fist: false,
            point: false,
            thumbsUp: false
        };
    }
    
    recognizeGestures(handJoints) {
        this.detectPinch(handJoints);
        this.detectFist(handJoints);
        this.detectPoint(handJoints);
        this.detectThumbsUp(handJoints);
        
        return this.gestures;
    }
    
    detectPinch(joints) {
        const thumb = joints['thumb-tip'];
        const index = joints['index-finger-tip'];
        
        if (thumb && index) {
            const distance = thumb.position.distanceTo(index.position);
            this.gestures.pinch = distance < 0.03; // 3cm
        }
    }
    
    detectFist(joints) {
        const fingers = [
            'index-finger-tip',
            'middle-finger-tip', 
            'ring-finger-tip',
            'pinky-finger-tip'
        ];
        
        const palm = joints['wrist'];
        let closedFingers = 0;
        
        fingers.forEach(fingerName => {
            const finger = joints[fingerName];
            if (finger && palm) {
                const distance = finger.position.distanceTo(palm.position);
                if (distance < 0.08) closedFingers++;
            }
        });
        
        this.gestures.fist = closedFingers >= 3;
    }
    
    detectPoint(joints) {
        const indexTip = joints['index-finger-tip'];
        const indexMcp = joints['index-finger-metacarpal'];
        const middleTip = joints['middle-finger-tip'];
        const middleMcp = joints['middle-finger-metacarpal'];
        
        if (indexTip && indexMcp && middleTip && middleMcp) {
            // 검지 펴짐
            const indexExtended = indexTip.position.distanceTo(indexMcp.position) > 0.06;
            
            // 중지 구부러짐
            const middleCurled = middleTip.position.distanceTo(middleMcp.position) < 0.04;
            
            this.gestures.point = indexExtended && middleCurled;
        }
    }
    
    detectThumbsUp(joints) {
        const thumbTip = joints['thumb-tip'];
        const thumbMcp = joints['thumb-metacarpal'];
        const wrist = joints['wrist'];
        
        if (thumbTip && thumbMcp && wrist) {
            // 엄지 위쪽 방향
            const thumbUp = thumbTip.position.y > thumbMcp.position.y + 0.03;
            
            // 다른 손가락 구부러짐
            const otherFingersCurled = this.detectFist(joints);
            
            this.gestures.thumbsUp = thumbUp && !otherFingersCurled;
        }
    }
}
```

---

## 4. 네트워크 설정

### 4.1 WiFi 최적화

**Quest 3 WiFi 설정**
```
설정 → WiFi
- 5GHz 네트워크 선택
- WiFi 6/6E 라우터 권장
- 채널 겹침 최소화
```

**라우터 설정**
```
QoS 설정:
- VR 트래픽 최우선순위
- 게임 모드 활성화
- 대역폭 예약 (50-100 Mbps)

WiFi 6 설정:
- 80MHz 채널 폭
- MU-MIMO 활성화
- BSS 색상 활성화
```

### 4.2 지연시간 측정

**Ping 테스트**
```bash
# Quest 3에서 PC로
ping -c 10 192.168.1.100

# PC에서 Quest 3로  
ping -c 10 192.168.1.XXX
```

**WebRTC 지연시간 측정**
```javascript
class LatencyMeasurement {
    constructor() {
        this.measurements = [];
        this.startTime = 0;
    }
    
    startMeasurement() {
        this.startTime = performance.now();
        
        // 서버로 타임스탬프 전송
        this.sendMessage({
            type: 'ping',
            timestamp: this.startTime
        });
    }
    
    receivePong(serverTimestamp) {
        const endTime = performance.now();
        const roundTripTime = endTime - this.startTime;
        
        this.measurements.push({
            rtt: roundTripTime,
            timestamp: endTime
        });
        
        // 최근 10개 평균
        if (this.measurements.length > 10) {
            this.measurements.shift();
        }
        
        const avgLatency = this.measurements.reduce((sum, m) => sum + m.rtt, 0) / this.measurements.length;
        
        console.log(`Latency: ${roundTripTime.toFixed(1)}ms (avg: ${avgLatency.toFixed(1)}ms)`);
        
        return avgLatency;
    }
}
```

---

## 5. 공간 설정

### 5.1 Guardian 경계 설정

**룸스케일 설정**
```
설정 → Guardian
1. 바닥 레벨 설정
2. 경계 그리기 (최소 2m x 2m)
3. 장애물 표시
4. 경계 알림 설정
```

**Guardian API 활용**
```javascript
// Guardian 경계 정보 가져오기
function getGuardianBounds() {
    if ('xr' in navigator && navigator.xr.getSession) {
        const session = navigator.xr.getSession();
        
        return session.requestReferenceSpace('bounded-floor').then(space => {
            const bounds = space.boundsGeometry;
            
            console.log('Guardian bounds:', bounds);
            return bounds;
        });
    }
}

// 가상 바닥 표시
function createVirtualFloor(bounds) {
    const floorGeometry = new THREE.PlaneGeometry(4, 4);
    const floorMaterial = new THREE.MeshBasicMaterial({
        color: 0x404040,
        transparent: true,
        opacity: 0.3
    });
    
    const floor = new THREE.Mesh(floorGeometry, floorMaterial);
    floor.rotation.x = -Math.PI / 2;
    floor.position.y = 0;
    
    return floor;
}
```

### 5.2 앵커 시스템

**공간 앵커 생성**
```javascript
class SpatialAnchor {
    constructor(session) {
        this.session = session;
        this.anchors = new Map();
    }
    
    async createAnchor(pose, name) {
        try {
            const anchor = await this.session.requestHitTestSourceForTransientInput({
                profile: "generic-touchscreen"
            });
            
            this.anchors.set(name, {
                anchor: anchor,
                pose: pose,
                timestamp: Date.now()
            });
            
            console.log(`Anchor '${name}' created`);
            return anchor;
        } catch (error) {
            console.error('Failed to create anchor:', error);
        }
    }
    
    getAnchor(name) {
        return this.anchors.get(name);
    }
    
    // 로봇 베이스 위치 앵커
    async setRobotBaseAnchor(robotPose) {
        const pose = new XRRigidTransform(
            robotPose.position,
            robotPose.orientation
        );
        
        return this.createAnchor(pose, 'robot_base');
    }
}
```

---

## 6. 성능 최적화

### 6.1 렌더링 최적화

**LOD (Level of Detail)**
```javascript
class LODManager {
    constructor(camera) {
        this.camera = camera;
        this.lodObjects = [];
    }
    
    addLODObject(object, distances) {
        this.lodObjects.push({
            object: object,
            distances: distances,
            currentLOD: 0
        });
    }
    
    update() {
        this.lodObjects.forEach(lodObj => {
            const distance = this.camera.position.distanceTo(lodObj.object.position);
            
            let newLOD = 0;
            for (let i = 0; i < lodObj.distances.length; i++) {
                if (distance > lodObj.distances[i]) {
                    newLOD = i + 1;
                }
            }
            
            if (newLOD !== lodObj.currentLOD) {
                this.updateLOD(lodObj, newLOD);
                lodObj.currentLOD = newLOD;
            }
        });
    }
    
    updateLOD(lodObj, level) {
        // 거리에 따른 디테일 조정
        switch(level) {
            case 0: // 가까움 - 고화질
                lodObj.object.material.map = this.highResTexture;
                break;
            case 1: // 중간 - 중화질
                lodObj.object.material.map = this.midResTexture;
                break;
            case 2: // 멀음 - 저화질
                lodObj.object.material.map = this.lowResTexture;
                break;
        }
    }
}
```

### 6.2 프레임레이트 관리

**적응형 프레임레이트**
```javascript
class FrameRateManager {
    constructor(targetFPS = 72) {
        this.targetFPS = targetFPS;
        this.targetFrameTime = 1000 / targetFPS;
        this.frameTimeHistory = [];
        this.maxHistory = 60;
    }
    
    beginFrame() {
        this.frameStart = performance.now();
    }
    
    endFrame() {
        const frameTime = performance.now() - this.frameStart;
        this.frameTimeHistory.push(frameTime);
        
        if (this.frameTimeHistory.length > this.maxHistory) {
            this.frameTimeHistory.shift();
        }
        
        // 성능 조정
        this.adjustPerformance(frameTime);
    }
    
    adjustPerformance(frameTime) {
        const avgFrameTime = this.frameTimeHistory.reduce((a, b) => a + b, 0) / this.frameTimeHistory.length;
        
        if (avgFrameTime > this.targetFrameTime * 1.2) {
            // 프레임 드롭 발생 - 품질 감소
            this.reduceQuality();
        } else if (avgFrameTime < this.targetFrameTime * 0.8) {
            // 여유 있음 - 품질 증가
            this.increaseQuality();
        }
    }
    
    reduceQuality() {
        // 렌더링 해상도 감소
        this.renderer.setPixelRatio(Math.max(0.5, this.renderer.getPixelRatio() - 0.1));
        
        // 그림자 품질 감소
        this.renderer.shadowMap.enabled = false;
    }
    
    increaseQuality() {
        // 렌더링 해상도 증가
        this.renderer.setPixelRatio(Math.min(1.0, this.renderer.getPixelRatio() + 0.1));
        
        // 그림자 활성화
        this.renderer.shadowMap.enabled = true;
    }
}
```

---

## 7. 디버깅 및 모니터링

### 7.1 VR 디버깅 도구

**Quest 3 디버깅**
```bash
# Logcat 모니터링
adb logcat | grep -i webxr

# 성능 모니터링
adb shell dumpsys VrApi

# 메모리 사용량
adb shell dumpsys meminfo com.oculus.browser
```

**Chrome DevTools**
```javascript
// VR 세션 디버깅
console.log('XR Session:', renderer.xr.getSession());

// 컨트롤러 상태
const controller = renderer.xr.getController(0);
console.log('Controller:', {
    position: controller.position,
    rotation: controller.rotation,
    connected: controller.connected
});

// 성능 메트릭
const stats = {
    fps: 1000 / deltaTime,
    drawCalls: renderer.info.render.calls,
    triangles: renderer.info.render.triangles
};
console.log('Performance:', stats);
```

### 7.2 원격 디버깅

**WebSocket 디버깅 서버**
```javascript
const WebSocket = require('ws');
const wss = new WebSocket.Server({ port: 8080 });

wss.on('connection', (ws) => {
    console.log('VR Debug client connected');
    
    ws.on('message', (data) => {
        const msg = JSON.parse(data);
        
        // 디버그 정보 로깅
        console.log(`[VR] ${msg.type}:`, msg.data);
        
        // 다른 클라이언트에게 브로드캐스트
        wss.clients.forEach(client => {
            if (client !== ws && client.readyState === WebSocket.OPEN) {
                client.send(data);
            }
        });
    });
});
```

---

## 8. 참고 자료

- [Meta Quest Developer Documentation](https://developer.oculus.com/documentation/)
- [WebXR Hand Input](https://www.w3.org/TR/webxr-hand-input-1/)
- [Three.js WebXR Examples](https://threejs.org/examples/?q=webxr)
- [Quest 3 Technical Specifications](https://www.meta.com/quest/quest-3/)

---

[← 7.1 VR 개요](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/01_vr_overview.md) | [다음: 7.3 WebRTC 통신 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/07_vr_control/03_webrtc.md)
