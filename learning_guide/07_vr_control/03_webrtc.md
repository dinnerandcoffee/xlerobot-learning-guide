# 7.3 WebRTC 통신

VR 환경에서 로봇과의 실시간 통신을 위한 WebRTC 기술을 학습합니다.

## 1. WebRTC 기본 개념

### 1.1 WebRTC vs WebSocket

**WebRTC 장점**
```
✓ P2P 직접 연결 (낮은 지연시간)
✓ 미디어 스트리밍 최적화
✓ NAT/방화벽 통과 (STUN/TURN)
✓ 암호화 기본 제공
✓ 네트워크 적응형 품질
```

**WebSocket 장점**
```
✓ 간단한 구현
✓ 서버 중심 아키텍처
✓ 메시지 순서 보장
✓ 방화벽 친화적
✓ 디버깅 용이
```

### 1.2 VR 텔레오퍼레이션에서의 활용

**데이터 흐름**
```
VR 헤드셋 ←→ WebRTC ←→ 로봇 서버
    ↓                    ↓
- 핸드 포즈           - 모터 제어
- 제스처             - 센서 데이터  
- 음성 명령          - 카메라 피드
```

---

## 2. 시그널링 서버

### 2.1 Node.js 시그널링 서버

**서버 설정**
```javascript
const express = require('express');
const WebSocket = require('ws');
const https = require('https');
const fs = require('fs');

class SignalingServer {
    constructor() {
        this.app = express();
        this.clients = new Map();
        this.rooms = new Map();
        
        this.setupHTTPS();
        this.setupWebSocket();
        this.setupRoutes();
    }
    
    setupHTTPS() {
        const options = {
            key: fs.readFileSync('key.pem'),
            cert: fs.readFileSync('cert.pem')
        };
        
        this.server = https.createServer(options, this.app);
    }
    
    setupWebSocket() {
        this.wss = new WebSocket.Server({ 
            server: this.server,
            path: '/signaling'
        });
        
        this.wss.on('connection', (ws, req) => {
            console.log('Client connected');
            
            ws.on('message', (data) => {
                this.handleMessage(ws, JSON.parse(data));
            });
            
            ws.on('close', () => {
                this.handleDisconnect(ws);
            });
        });
    }
    
    handleMessage(ws, message) {
        switch (message.type) {
            case 'join':
                this.handleJoin(ws, message);
                break;
            case 'offer':
            case 'answer':
            case 'ice-candidate':
                this.handleWebRTCMessage(ws, message);
                break;
            case 'robot-command':
                this.handleRobotCommand(ws, message);
                break;
        }
    }
    
    handleJoin(ws, message) {
        const { roomId, clientType } = message;
        
        if (!this.rooms.has(roomId)) {
            this.rooms.set(roomId, new Set());
        }
        
        const room = this.rooms.get(roomId);
        room.add(ws);
        
        ws.roomId = roomId;
        ws.clientType = clientType;
        
        // 기존 클라이언트에게 새 참가자 알림
        this.broadcastToRoom(roomId, {
            type: 'peer-joined',
            clientType: clientType
        }, ws);
        
        console.log(`${clientType} joined room ${roomId}`);
    }
    
    handleWebRTCMessage(ws, message) {
        // 같은 방의 다른 클라이언트에게 전달
        this.broadcastToRoom(ws.roomId, message, ws);
    }
    
    handleRobotCommand(ws, message) {
        // 로봇 서버에게만 전달
        this.sendToRobotServer(ws.roomId, message);
    }
    
    broadcastToRoom(roomId, message, sender) {
        const room = this.rooms.get(roomId);
        if (!room) return;
        
        room.forEach(client => {
            if (client !== sender && client.readyState === WebSocket.OPEN) {
                client.send(JSON.stringify(message));
            }
        });
    }
    
    sendToRobotServer(roomId, message) {
        const room = this.rooms.get(roomId);
        if (!room) return;
        
        room.forEach(client => {
            if (client.clientType === 'robot' && 
                client.readyState === WebSocket.OPEN) {
                client.send(JSON.stringify(message));
            }
        });
    }
    
    start(port = 8443) {
        this.server.listen(port, () => {
            console.log(`Signaling server running on https://localhost:${port}`);
        });
    }
}

// 서버 시작
const server = new SignalingServer();
server.start();
```

### 2.2 STUN/TURN 서버 설정

**Coturn 설치 (Ubuntu/Debian)**
```bash
sudo apt update
sudo apt install coturn

# 설정 파일 편집
sudo nano /etc/turnserver.conf
```

**turnserver.conf 설정**
```
# 기본 설정
listening-port=3478
listening-ip=0.0.0.0
relay-ip=YOUR_SERVER_IP
external-ip=YOUR_PUBLIC_IP

# 인증
use-auth-secret
static-auth-secret=YOUR_SECRET_KEY

# SSL/TLS
cert=/path/to/cert.pem
pkey=/path/to/key.pem

# 사용자 관리
user=guest:guest123

# 보안
denied-peer-ip=10.0.0.0-10.255.255.255
denied-peer-ip=192.168.0.0-192.168.255.255
denied-peer-ip=172.16.0.0-172.31.255.255

# 로깅
log-file=/var/log/turnserver.log
verbose
```

**서비스 시작**
```bash
sudo systemctl enable coturn
sudo systemctl start coturn
sudo systemctl status coturn
```

---

## 3. VR 클라이언트 WebRTC

### 3.1 기본 WebRTC 설정

**WebRTC 연결 관리자**
```javascript
class VRWebRTCClient {
    constructor() {
        this.pc = null;
        this.ws = null;
        this.localStream = null;
        this.remoteStream = null;
        
        this.config = {
            iceServers: [
                { urls: 'stun:stun.l.google.com:19302' },
                { 
                    urls: 'turn:your-turn-server.com:3478',
                    username: 'guest',
                    credential: 'guest123'
                }
            ]
        };
        
        this.dataChannels = new Map();
        this.setupSignaling();
    }
    
    setupSignaling() {
        this.ws = new WebSocket('wss://your-server.com:8443/signaling');
        
        this.ws.onopen = () => {
            console.log('Connected to signaling server');
            this.joinRoom('robot-control-room', 'vr-client');
        };
        
        this.ws.onmessage = (event) => {
            this.handleSignalingMessage(JSON.parse(event.data));
        };
    }
    
    joinRoom(roomId, clientType) {
        this.ws.send(JSON.stringify({
            type: 'join',
            roomId: roomId,
            clientType: clientType
        }));
    }
    
    async handleSignalingMessage(message) {
        switch (message.type) {
            case 'peer-joined':
                if (message.clientType === 'robot') {
                    await this.createOffer();
                }
                break;
            case 'offer':
                await this.handleOffer(message);
                break;
            case 'answer':
                await this.handleAnswer(message);
                break;
            case 'ice-candidate':
                await this.handleIceCandidate(message);
                break;
        }
    }
    
    async createPeerConnection() {
        this.pc = new RTCPeerConnection(this.config);
        
        // ICE 후보 전송
        this.pc.onicecandidate = (event) => {
            if (event.candidate) {
                this.ws.send(JSON.stringify({
                    type: 'ice-candidate',
                    candidate: event.candidate
                }));
            }
        };
        
        // 원격 스트림 수신
        this.pc.ontrack = (event) => {
            console.log('Received remote stream');
            this.remoteStream = event.streams[0];
            this.displayRemoteVideo();
        };
        
        // 데이터 채널 설정
        this.setupDataChannels();
        
        return this.pc;
    }
    
    setupDataChannels() {
        // 로봇 제어 채널
        const controlChannel = this.pc.createDataChannel('robot-control', {
            ordered: false,
            maxRetransmits: 0
        });
        
        controlChannel.onopen = () => {
            console.log('Control channel opened');
        };
        
        this.dataChannels.set('control', controlChannel);
        
        // 센서 데이터 채널
        const sensorChannel = this.pc.createDataChannel('sensor-data', {
            ordered: true
        });
        
        sensorChannel.onmessage = (event) => {
            this.handleSensorData(JSON.parse(event.data));
        };
        
        this.dataChannels.set('sensor', sensorChannel);
    }
    
    async createOffer() {
        await this.createPeerConnection();
        
        const offer = await this.pc.createOffer({
            offerToReceiveVideo: true,
            offerToReceiveAudio: true
        });
        
        await this.pc.setLocalDescription(offer);
        
        this.ws.send(JSON.stringify({
            type: 'offer',
            offer: offer
        }));
    }
    
    async handleOffer(message) {
        await this.createPeerConnection();
        
        await this.pc.setRemoteDescription(message.offer);
        
        const answer = await this.pc.createAnswer();
        await this.pc.setLocalDescription(answer);
        
        this.ws.send(JSON.stringify({
            type: 'answer',
            answer: answer
        }));
    }
    
    async handleAnswer(message) {
        await this.pc.setRemoteDescription(message.answer);
    }
    
    async handleIceCandidate(message) {
        await this.pc.addIceCandidate(message.candidate);
    }
}
```

### 3.2 핸드 트래킹 데이터 전송

**실시간 포즈 전송**
```javascript
class HandPoseTransmitter {
    constructor(webrtcClient) {
        this.webrtc = webrtcClient;
        this.lastSentTime = 0;
        this.sendInterval = 16; // 60 FPS
    }
    
    sendHandPose(leftHand, rightHand) {
        const now = performance.now();
        
        if (now - this.lastSentTime < this.sendInterval) {
            return;
        }
        
        const poseData = {
            timestamp: now,
            leftHand: this.serializeHand(leftHand),
            rightHand: this.serializeHand(rightHand)
        };
        
        const controlChannel = this.webrtc.dataChannels.get('control');
        if (controlChannel && controlChannel.readyState === 'open') {
            controlChannel.send(JSON.stringify({
                type: 'hand-pose',
                data: poseData
            }));
        }
        
        this.lastSentTime = now;
    }
    
    serializeHand(hand) {
        if (!hand) return null;
        
        const serialized = {
            joints: {},
            gestures: hand.gestures || {}
        };
        
        // 손 관절 위치/회전 직렬화
        Object.keys(hand.joints).forEach(jointName => {
            const joint = hand.joints[jointName];
            
            serialized.joints[jointName] = {
                position: {
                    x: joint.position.x,
                    y: joint.position.y,
                    z: joint.position.z
                },
                rotation: {
                    x: joint.quaternion.x,
                    y: joint.quaternion.y,
                    z: joint.quaternion.z,
                    w: joint.quaternion.w
                }
            };
        });
        
        return serialized;
    }
    
    // 압축된 포즈 데이터 전송 (대역폭 절약)
    sendCompressedPose(leftHand, rightHand) {
        const compressed = this.compressPoseData(leftHand, rightHand);
        
        const controlChannel = this.webrtc.dataChannels.get('control');
        if (controlChannel && controlChannel.readyState === 'open') {
            controlChannel.send(compressed);
        }
    }
    
    compressPoseData(leftHand, rightHand) {
        // 핵심 관절만 전송 (손목, 손가락 끝)
        const keyJoints = [
            'wrist',
            'thumb-tip',
            'index-finger-tip',
            'middle-finger-tip',
            'ring-finger-tip',
            'pinky-finger-tip'
        ];
        
        const compressed = new Float32Array(keyJoints.length * 2 * 7); // 2손 * 7값(pos+rot)
        let index = 0;
        
        [leftHand, rightHand].forEach(hand => {
            if (hand) {
                keyJoints.forEach(jointName => {
                    const joint = hand.joints[jointName];
                    if (joint) {
                        compressed[index++] = joint.position.x;
                        compressed[index++] = joint.position.y;
                        compressed[index++] = joint.position.z;
                        compressed[index++] = joint.quaternion.x;
                        compressed[index++] = joint.quaternion.y;
                        compressed[index++] = joint.quaternion.z;
                        compressed[index++] = joint.quaternion.w;
                    } else {
                        index += 7; // 빈 값들 건너뛰기
                    }
                });
            } else {
                index += keyJoints.length * 7;
            }
        });
        
        return compressed.buffer;
    }
}
```

### 3.3 비디오 스트림 최적화

**적응형 비트레이트**
```javascript
class AdaptiveBitrateController {
    constructor(peerConnection) {
        this.pc = peerConnection;
        this.targetBitrate = 2000000; // 2 Mbps
        this.minBitrate = 500000;     // 500 kbps
        this.maxBitrate = 5000000;    // 5 Mbps
        
        this.setupQualityMonitoring();
    }
    
    setupQualityMonitoring() {
        setInterval(() => {
            this.checkConnectionQuality();
        }, 2000);
    }
    
    async checkConnectionQuality() {
        const stats = await this.pc.getStats();
        
        let packetsLost = 0;
        let packetsReceived = 0;
        let rtt = 0;
        
        stats.forEach(report => {
            if (report.type === 'inbound-rtp' && report.mediaType === 'video') {
                packetsLost = report.packetsLost || 0;
                packetsReceived = report.packetsReceived || 0;
            }
            
            if (report.type === 'candidate-pair' && report.state === 'succeeded') {
                rtt = report.currentRoundTripTime * 1000; // ms
            }
        });
        
        const lossRate = packetsLost / (packetsLost + packetsReceived);
        
        console.log(`RTT: ${rtt}ms, Loss: ${(lossRate * 100).toFixed(1)}%`);
        
        // 품질 조정
        if (lossRate > 0.05 || rtt > 150) {
            this.reduceBitrate();
        } else if (lossRate < 0.01 && rtt < 50) {
            this.increaseBitrate();
        }
    }
    
    async reduceBitrate() {
        this.targetBitrate = Math.max(this.minBitrate, this.targetBitrate * 0.8);
        await this.applyBitrate();
        console.log(`Reduced bitrate to ${this.targetBitrate / 1000}kbps`);
    }
    
    async increaseBitrate() {
        this.targetBitrate = Math.min(this.maxBitrate, this.targetBitrate * 1.2);
        await this.applyBitrate();
        console.log(`Increased bitrate to ${this.targetBitrate / 1000}kbps`);
    }
    
    async applyBitrate() {
        const senders = this.pc.getSenders();
        
        for (const sender of senders) {
            if (sender.track && sender.track.kind === 'video') {
                const params = sender.getParameters();
                
                if (params.encodings && params.encodings.length > 0) {
                    params.encodings[0].maxBitrate = this.targetBitrate;
                    await sender.setParameters(params);
                }
            }
        }
    }
}
```

---

## 4. 로봇 서버 WebRTC

### 4.1 Python 로봇 서버

**aiortc 기반 구현**
```python
import asyncio
import json
import logging
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCIceCandidate
from aiortc.contrib.signaling import TcpSocketSignaling
import websockets

class RobotWebRTCServer:
    def __init__(self):
        self.pc = RTCPeerConnection()
        self.data_channels = {}
        self.robot_controller = None
        
        self.setup_peer_connection()
    
    def setup_peer_connection(self):
        @self.pc.on("datachannel")
        def on_datachannel(channel):
            print(f"Data channel established: {channel.label}")
            self.data_channels[channel.label] = channel
            
            @channel.on("message")
            def on_message(message):
                self.handle_data_channel_message(channel.label, message)
    
    def handle_data_channel_message(self, channel_label, message):
        if channel_label == "robot-control":
            try:
                data = json.loads(message)
                self.process_control_command(data)
            except json.JSONDecodeError:
                # 바이너리 데이터 (압축된 포즈)
                self.process_compressed_pose(message)
        
        elif channel_label == "sensor-data":
            # 센서 데이터 요청 처리
            self.send_sensor_data()
    
    def process_control_command(self, data):
        if data["type"] == "hand-pose":
            pose_data = data["data"]
            
            # 왼손/오른손 포즈 처리
            if pose_data["leftHand"]:
                self.process_hand_pose("left", pose_data["leftHand"])
            
            if pose_data["rightHand"]:
                self.process_hand_pose("right", pose_data["rightHand"])
    
    def process_hand_pose(self, hand_side, hand_data):
        if not self.robot_controller:
            return
        
        # 제스처 기반 제어
        gestures = hand_data.get("gestures", {})
        
        if gestures.get("pinch"):
            self.robot_controller.gripper_close()
        elif gestures.get("fist"):
            self.robot_controller.emergency_stop()
        elif gestures.get("point"):
            # 검지 방향으로 로봇 이동
            self.point_based_navigation(hand_data)
        
        # 손목 위치 기반 엔드이펙터 제어
        wrist = hand_data["joints"].get("wrist")
        if wrist:
            target_pos = self.vr_to_robot_coordinates(
                wrist["position"]["x"],
                wrist["position"]["y"], 
                wrist["position"]["z"]
            )
            
            self.robot_controller.move_to_position(target_pos)
    
    def vr_to_robot_coordinates(self, vr_x, vr_y, vr_z):
        # VR 좌표계를 로봇 좌표계로 변환
        # VR: Y-up, 미터 단위
        # 로봇: Z-up, 밀리미터 단위
        
        robot_x = vr_x * 1000  # m -> mm
        robot_y = -vr_z * 1000 # VR Z -> 로봇 Y (반전)
        robot_z = vr_y * 1000  # VR Y -> 로봇 Z
        
        return [robot_x, robot_y, robot_z]
    
    def send_sensor_data(self):
        sensor_channel = self.data_channels.get("sensor-data")
        if not sensor_channel or not self.robot_controller:
            return
        
        sensor_data = {
            "timestamp": time.time() * 1000,
            "joint_positions": self.robot_controller.get_joint_positions(),
            "end_effector_pose": self.robot_controller.get_ee_pose(),
            "force_feedback": self.robot_controller.get_force_data(),
            "camera_info": {
                "fps": 30,
                "resolution": "1920x1080"
            }
        }
        
        sensor_channel.send(json.dumps(sensor_data))
    
    async def create_answer(self, offer):
        await self.pc.setRemoteDescription(RTCSessionDescription(
            sdp=offer["sdp"], 
            type=offer["type"]
        ))
        
        answer = await self.pc.createAnswer()
        await self.pc.setLocalDescription(answer)
        
        return {
            "sdp": self.pc.localDescription.sdp,
            "type": self.pc.localDescription.type
        }

# WebSocket 시그널링 클라이언트
class SignalingClient:
    def __init__(self, server_url):
        self.server_url = server_url
        self.ws = None
        self.webrtc_server = RobotWebRTCServer()
    
    async def connect(self):
        self.ws = await websockets.connect(self.server_url)
        
        # 로봇으로 등록
        await self.ws.send(json.dumps({
            "type": "join",
            "roomId": "robot-control-room",
            "clientType": "robot"
        }))
        
        # 메시지 수신 대기
        async for message in self.ws:
            await self.handle_message(json.loads(message))
    
    async def handle_message(self, message):
        if message["type"] == "offer":
            answer = await self.webrtc_server.create_answer(message["offer"])
            
            await self.ws.send(json.dumps({
                "type": "answer", 
                "answer": answer
            }))
        
        elif message["type"] == "ice-candidate":
            await self.webrtc_server.pc.addIceCandidate(
                RTCIceCandidate(
                    candidate=message["candidate"]["candidate"],
                    sdpMLineIndex=message["candidate"]["sdpMLineIndex"],
                    sdpMid=message["candidate"]["sdpMid"]
                )
            )

# 서버 실행
async def main():
    client = SignalingClient("wss://your-server.com:8443/signaling")
    await client.connect()

if __name__ == "__main__":
    asyncio.run(main())
```

### 4.2 카메라 스트림 전송

**OpenCV 비디오 스트림**
```python
import cv2
from av import VideoFrame
from aiortc import VideoStreamTrack
import asyncio

class RobotCameraTrack(VideoStreamTrack):
    def __init__(self, camera_index=0):
        super().__init__()
        self.camera = cv2.VideoCapture(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        
    async def recv(self):
        pts, time_base = await self.next_timestamp()
        
        ret, frame = self.camera.read()
        if not ret:
            return None
        
        # BGR to RGB 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # VideoFrame 생성
        av_frame = VideoFrame.from_ndarray(frame, format="rgb24")
        av_frame.pts = pts
        av_frame.time_base = time_base
        
        return av_frame

# WebRTC 서버에 비디오 트랙 추가
class RobotWebRTCServerWithVideo(RobotWebRTCServer):
    def __init__(self):
        super().__init__()
        self.video_track = RobotCameraTrack()
        self.pc.addTrack(self.video_track)
```

---

## 5. 네트워크 최적화

### 5.1 지연시간 최소화

**우선순위 기반 데이터 전송**
```javascript
class PriorityDataTransmitter {
    constructor(dataChannel) {
        this.channel = dataChannel;
        this.queues = {
            critical: [],    // 비상정지, 충돌감지
            high: [],       // 핸드 포즈, 제스처
            medium: [],     // 센서 데이터
            low: []         // 로그, 디버그 정보
        };
        
        this.startTransmissionLoop();
    }
    
    send(data, priority = 'medium') {
        const message = {
            timestamp: performance.now(),
            data: data,
            priority: priority
        };
        
        this.queues[priority].push(message);
    }
    
    startTransmissionLoop() {
        const transmit = () => {
            if (this.channel.readyState !== 'open') {
                requestAnimationFrame(transmit);
                return;
            }
            
            // 우선순위 순서로 전송
            const priorities = ['critical', 'high', 'medium', 'low'];
            
            for (const priority of priorities) {
                const queue = this.queues[priority];
                
                if (queue.length > 0) {
                    const message = queue.shift();
                    
                    // 만료된 메시지 필터링 (100ms 이상 오래된 경우)
                    const age = performance.now() - message.timestamp;
                    if (age < 100 || priority === 'critical') {
                        this.channel.send(JSON.stringify(message));
                    }
                    
                    break; // 한 번에 하나씩만 전송
                }
            }
            
            requestAnimationFrame(transmit);
        };
        
        requestAnimationFrame(transmit);
    }
}
```

### 5.2 대역폭 관리

**적응형 품질 제어**
```javascript
class BandwidthManager {
    constructor(peerConnection) {
        this.pc = peerConnection;
        this.currentBandwidth = 0;
        this.targetBandwidth = 2000000; // 2 Mbps
        
        this.qualityLevels = {
            low: {
                video: { width: 640, height: 480, framerate: 15, bitrate: 500000 },
                data: { frequency: 30 } // 30 Hz
            },
            medium: {
                video: { width: 1280, height: 720, framerate: 30, bitrate: 1500000 },
                data: { frequency: 60 } // 60 Hz
            },
            high: {
                video: { width: 1920, height: 1080, framerate: 30, bitrate: 3000000 },
                data: { frequency: 90 } // 90 Hz
            }
        };
        
        this.currentQuality = 'medium';
        this.startBandwidthMonitoring();
    }
    
    startBandwidthMonitoring() {
        setInterval(async () => {
            await this.measureBandwidth();
            this.adjustQuality();
        }, 5000);
    }
    
    async measureBandwidth() {
        const stats = await this.pc.getStats();
        
        let bytesReceived = 0;
        let timestamp = 0;
        
        stats.forEach(report => {
            if (report.type === 'inbound-rtp') {
                bytesReceived += report.bytesReceived || 0;
                timestamp = report.timestamp;
            }
        });
        
        if (this.lastBytesReceived && this.lastTimestamp) {
            const bytesDiff = bytesReceived - this.lastBytesReceived;
            const timeDiff = (timestamp - this.lastTimestamp) / 1000; // seconds
            
            this.currentBandwidth = (bytesDiff * 8) / timeDiff; // bits per second
        }
        
        this.lastBytesReceived = bytesReceived;
        this.lastTimestamp = timestamp;
    }
    
    adjustQuality() {
        const utilizationRatio = this.currentBandwidth / this.targetBandwidth;
        
        if (utilizationRatio > 0.9 && this.currentQuality !== 'low') {
            // 대역폭 부족 - 품질 감소
            this.downgradeQuality();
        } else if (utilizationRatio < 0.6 && this.currentQuality !== 'high') {
            // 대역폭 여유 - 품질 증가
            this.upgradeQuality();
        }
    }
    
    downgradeQuality() {
        const levels = ['high', 'medium', 'low'];
        const currentIndex = levels.indexOf(this.currentQuality);
        
        if (currentIndex < levels.length - 1) {
            this.currentQuality = levels[currentIndex + 1];
            this.applyQualitySettings();
            console.log(`Quality downgraded to ${this.currentQuality}`);
        }
    }
    
    upgradeQuality() {
        const levels = ['low', 'medium', 'high'];
        const currentIndex = levels.indexOf(this.currentQuality);
        
        if (currentIndex < levels.length - 1) {
            this.currentQuality = levels[currentIndex + 1];
            this.applyQualitySettings();
            console.log(`Quality upgraded to ${this.currentQuality}`);
        }
    }
    
    async applyQualitySettings() {
        const settings = this.qualityLevels[this.currentQuality];
        
        // 비디오 품질 조정
        const senders = this.pc.getSenders();
        for (const sender of senders) {
            if (sender.track && sender.track.kind === 'video') {
                const params = sender.getParameters();
                
                if (params.encodings && params.encodings.length > 0) {
                    params.encodings[0].maxBitrate = settings.video.bitrate;
                    params.encodings[0].maxFramerate = settings.video.framerate;
                    await sender.setParameters(params);
                }
            }
        }
        
        // 데이터 전송 주파수 조정
        this.adjustDataFrequency(settings.data.frequency);
    }
    
    adjustDataFrequency(frequency) {
        // 핸드 포즈 전송 주파수 조정
        const interval = 1000 / frequency; // ms
        
        if (this.handPoseTransmitter) {
            this.handPoseTransmitter.setSendInterval(interval);
        }
    }
}
```

---

## 6. 에러 처리 및 복구

### 6.1 연결 복구

**자동 재연결**
```javascript
class ConnectionRecovery {
    constructor(webrtcClient) {
        this.client = webrtcClient;
        this.reconnectAttempts = 0;
        this.maxReconnectAttempts = 5;
        this.reconnectDelay = 2000;
        
        this.setupConnectionMonitoring();
    }
    
    setupConnectionMonitoring() {
        this.client.pc.onconnectionstatechange = () => {
            const state = this.client.pc.connectionState;
            console.log(`Connection state: ${state}`);
            
            switch (state) {
                case 'disconnected':
                case 'failed':
                    this.handleConnectionLoss();
                    break;
                case 'connected':
                    this.reconnectAttempts = 0;
                    console.log('Connection restored');
                    break;
            }
        };
        
        this.client.pc.oniceconnectionstatechange = () => {
            const state = this.client.pc.iceConnectionState;
            console.log(`ICE state: ${state}`);
            
            if (state === 'failed') {
                this.handleICEFailure();
            }
        };
    }
    
    async handleConnectionLoss() {
        if (this.reconnectAttempts >= this.maxReconnectAttempts) {
            console.error('Max reconnection attempts reached');
            this.showConnectionError();
            return;
        }
        
        this.reconnectAttempts++;
        console.log(`Attempting reconnection ${this.reconnectAttempts}/${this.maxReconnectAttempts}`);
        
        // 지수 백오프 대기
        const delay = this.reconnectDelay * Math.pow(2, this.reconnectAttempts - 1);
        await this.sleep(delay);
        
        try {
            await this.client.reconnect();
        } catch (error) {
            console.error('Reconnection failed:', error);
            this.handleConnectionLoss(); // 재시도
        }
    }
    
    async handleICEFailure() {
        console.log('ICE connection failed, attempting ICE restart');
        
        try {
            const offer = await this.client.pc.createOffer({ iceRestart: true });
            await this.client.pc.setLocalDescription(offer);
            
            this.client.ws.send(JSON.stringify({
                type: 'offer',
                offer: offer
            }));
        } catch (error) {
            console.error('ICE restart failed:', error);
        }
    }
    
    showConnectionError() {
        // VR UI에 연결 오류 표시
        const errorUI = document.createElement('div');
        errorUI.innerHTML = `
            <div style="position: fixed; top: 50%; left: 50%; transform: translate(-50%, -50%); 
                        background: rgba(255, 0, 0, 0.8); color: white; padding: 20px; 
                        border-radius: 10px; font-size: 18px; text-align: center;">
                <h3>연결 오류</h3>
                <p>로봇 서버와의 연결이 끊어졌습니다.</p>
                <button onclick="location.reload()">다시 시도</button>
            </div>
        `;
        document.body.appendChild(errorUI);
    }
    
    sleep(ms) {
        return new Promise(resolve => setTimeout(resolve, ms));
    }
}
```

---

## 7. 참고 자료

- [WebRTC API Documentation](https://developer.mozilla.org/en-US/docs/Web/API/WebRTC_API)
- [aiortc Python Library](https://github.com/aiortc/aiortc)
- [STUN/TURN Server Setup](https://github.com/coturn/coturn)
- [WebRTC Samples](https://webrtc.github.io/samples/)

---

[← 7.2 Quest3 설정](02_quest3_setup.md) | [다음: 7.4 VR 인터페이스 →](04_vr_interface.md)
