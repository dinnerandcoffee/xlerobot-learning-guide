# 7.5 데이터셋 기록

VR 텔레오퍼레이션을 통한 로봇 데이터셋 수집 및 머신러닝 훈련용 데이터 준비 과정을 학습합니다.

## 1. 데이터셋 기록 시스템

### 1.1 기록 데이터 구조

**수집 데이터 타입**
```javascript
const DataTypes = {
    // VR 입력 데이터
    VR_INPUT: {
        headPose: "헤드셋 위치/회전",
        leftHandPose: "왼손 포즈 (관절별)",
        rightHandPose: "오른손 포즈 (관절별)",
        gestures: "인식된 제스처",
        voiceCommands: "음성 명령",
        gazeDirection: "시선 방향"
    },
    
    // 로봇 상태 데이터
    ROBOT_STATE: {
        jointAngles: "관절 각도 (6축)",
        endEffectorPose: "엔드이펙터 위치/자세",
        gripperState: "그리퍼 개폐 상태",
        forceData: "힘/토크 센서 데이터",
        velocities: "관절 속도",
        accelerations: "관절 가속도"
    },
    
    // 환경 데이터
    ENVIRONMENT: {
        cameraImages: "RGB 카메라 이미지",
        depthImages: "깊이 이미지",
        pointClouds: "3D 포인트 클라우드",
        objectDetections: "객체 인식 결과",
        sceneDescription: "환경 설명"
    },
    
    // 작업 메타데이터
    TASK_METADATA: {
        taskType: "작업 유형 (pick, place, pour 등)",
        objectTypes: "대상 객체 타입",
        difficulty: "난이도 (1-5)",
        success: "성공/실패 여부",
        duration: "작업 소요 시간",
        operatorSkill: "조작자 숙련도"
    }
};
```

### 1.2 데이터 수집 아키텍처

**수집 시스템 구조**
```javascript
class VRDataRecorder {
    constructor() {
        this.isRecording = false;
        this.sessionId = null;
        this.dataBuffer = [];
        this.maxBufferSize = 10000; // 메모리 제한
        
        this.streams = {
            vr: new VRDataStream(),
            robot: new RobotDataStream(),
            camera: new CameraDataStream(),
            audio: new AudioDataStream()
        };
        
        this.storage = new DataStorage();
        this.synchronizer = new DataSynchronizer();
        
        this.setupRecording();
    }
    
    setupRecording() {
        // 데이터 스트림 동기화
        this.synchronizer.addStream('vr', this.streams.vr, 90); // 90 FPS
        this.synchronizer.addStream('robot', this.streams.robot, 100); // 100 Hz
        this.synchronizer.addStream('camera', this.streams.camera, 30); // 30 FPS
        this.synchronizer.addStream('audio', this.streams.audio, 16000); // 16 kHz
        
        // 동기화된 데이터 수신
        this.synchronizer.onData = (syncedData) => {
            this.processData(syncedData);
        };
    }
    
    startRecording(taskMetadata) {
        if (this.isRecording) {
            console.warn('Already recording');
            return;
        }
        
        this.sessionId = this.generateSessionId();
        this.isRecording = true;
        this.dataBuffer = [];
        
        // 메타데이터 저장
        this.storage.saveMetadata(this.sessionId, {
            ...taskMetadata,
            startTime: Date.now(),
            sessionId: this.sessionId
        });
        
        // 모든 스트림 시작
        Object.values(this.streams).forEach(stream => {
            stream.start();
        });
        
        this.synchronizer.start();
        
        console.log(`Recording started: ${this.sessionId}`);
        this.showRecordingUI();
    }
    
    stopRecording() {
        if (!this.isRecording) {
            return;
        }
        
        this.isRecording = false;
        
        // 모든 스트림 정지
        Object.values(this.streams).forEach(stream => {
            stream.stop();
        });
        
        this.synchronizer.stop();
        
        // 남은 버퍼 데이터 저장
        if (this.dataBuffer.length > 0) {
            this.flushBuffer();
        }
        
        // 세션 종료 시간 기록
        this.storage.updateMetadata(this.sessionId, {
            endTime: Date.now(),
            totalFrames: this.frameCount
        });
        
        console.log(`Recording stopped: ${this.sessionId}`);
        this.hideRecordingUI();
        
        return this.sessionId;
    }
    
    processData(syncedData) {
        if (!this.isRecording) return;
        
        const frame = {
            timestamp: performance.now(),
            frameIndex: this.frameCount++,
            sessionId: this.sessionId,
            data: syncedData
        };
        
        this.dataBuffer.push(frame);
        
        // 버퍼가 가득 차면 저장
        if (this.dataBuffer.length >= this.maxBufferSize) {
            this.flushBuffer();
        }
    }
    
    flushBuffer() {
        if (this.dataBuffer.length === 0) return;
        
        const batchData = [...this.dataBuffer];
        this.dataBuffer = [];
        
        // 비동기 저장
        this.storage.saveBatch(this.sessionId, batchData)
            .catch(error => {
                console.error('Failed to save batch:', error);
            });
    }
    
    generateSessionId() {
        const timestamp = Date.now();
        const random = Math.random().toString(36).substr(2, 5);
        return `session_${timestamp}_${random}`;
    }
}
```

---

## 2. VR 데이터 수집

### 2.1 핸드 트래킹 데이터

**정밀 핸드 포즈 기록**
```javascript
class VRDataStream {
    constructor() {
        this.handTracker = null;
        this.gazeTracker = null;
        this.isCapturing = false;
        
        this.setupTracking();
    }
    
    setupTracking() {
        this.handTracker = new HandTracker();
        this.gazeTracker = new GazeTracker();
    }
    
    start() {
        this.isCapturing = true;
        this.captureLoop();
    }
    
    stop() {
        this.isCapturing = false;
    }
    
    captureLoop() {
        if (!this.isCapturing) return;
        
        const vrData = this.captureVRData();
        
        // 데이터 콜백
        if (this.onData && vrData) {
            this.onData(vrData);
        }
        
        // 다음 프레임
        requestAnimationFrame(() => this.captureLoop());
    }
    
    captureVRData() {
        if (!this.handTracker.isTracking) return null;
        
        return {
            timestamp: performance.now(),
            
            // 헤드셋 포즈
            headPose: this.captureHeadPose(),
            
            // 핸드 포즈
            leftHand: this.captureHandData('left'),
            rightHand: this.captureHandData('right'),
            
            // 제스처
            gestures: this.captureGestures(),
            
            // 시선 추적
            gaze: this.captureGazeData(),
            
            // 컨트롤러 상태
            controllers: this.captureControllerData()
        };
    }
    
    captureHeadPose() {
        const camera = this.handTracker.camera;
        
        return {
            position: {
                x: camera.position.x,
                y: camera.position.y,
                z: camera.position.z
            },
            rotation: {
                x: camera.quaternion.x,
                y: camera.quaternion.y,
                z: camera.quaternion.z,
                w: camera.quaternion.w
            }
        };
    }
    
    captureHandData(handSide) {
        const hand = this.handTracker.getHand(handSide);
        if (!hand) return null;
        
        const handData = {
            isTracking: hand.isTracking,
            confidence: hand.confidence,
            joints: {},
            gestures: hand.gestures || {}
        };
        
        // 모든 관절 데이터 수집
        const jointNames = [
            'wrist',
            'thumb-metacarpal', 'thumb-proximal', 'thumb-distal', 'thumb-tip',
            'index-finger-metacarpal', 'index-finger-proximal', 
            'index-finger-intermediate', 'index-finger-distal', 'index-finger-tip',
            'middle-finger-metacarpal', 'middle-finger-proximal',
            'middle-finger-intermediate', 'middle-finger-distal', 'middle-finger-tip',
            'ring-finger-metacarpal', 'ring-finger-proximal',
            'ring-finger-intermediate', 'ring-finger-distal', 'ring-finger-tip',
            'pinky-finger-metacarpal', 'pinky-finger-proximal',
            'pinky-finger-intermediate', 'pinky-finger-distal', 'pinky-finger-tip'
        ];
        
        jointNames.forEach(jointName => {
            const joint = hand.joints[jointName];
            if (joint) {
                handData.joints[jointName] = {
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
                    },
                    radius: joint.radius || 0.01
                };
            }
        });
        
        return handData;
    }
    
    captureGestures() {
        return {
            leftHand: this.handTracker.getGestures('left'),
            rightHand: this.handTracker.getGestures('right'),
            bimanual: this.handTracker.getBimanualGestures()
        };
    }
    
    captureGazeData() {
        if (!this.gazeTracker.isActive) return null;
        
        return {
            direction: this.gazeTracker.getGazeDirection(),
            target: this.gazeTracker.getGazeTarget(),
            confidence: this.gazeTracker.getConfidence()
        };
    }
    
    captureControllerData() {
        return this.handTracker.controllers.map(controller => ({
            connected: controller.connected,
            position: {
                x: controller.position.x,
                y: controller.position.y,
                z: controller.position.z
            },
            rotation: {
                x: controller.quaternion.x,
                y: controller.quaternion.y,
                z: controller.quaternion.z,
                w: controller.quaternion.w
            },
            buttons: controller.buttons?.map(button => ({
                pressed: button.pressed,
                value: button.value
            })) || []
        }));
    }
}
```

### 2.2 작업 행동 라벨링

**자동 행동 분할**
```javascript
class ActionLabeler {
    constructor() {
        this.currentAction = null;
        this.actionSequence = [];
        this.actionTemplates = this.loadActionTemplates();
        
        this.velocityThreshold = 0.02; // m/s
        this.accelerationThreshold = 0.1; // m/s²
        this.idleTimeThreshold = 1000; // ms
    }
    
    loadActionTemplates() {
        return {
            'pick': {
                phases: ['approach', 'grasp', 'lift'],
                features: {
                    approach: { handClosing: false, moving: true },
                    grasp: { handClosing: true, moving: false },
                    lift: { handClosed: true, moving: true, direction: 'up' }
                }
            },
            
            'place': {
                phases: ['move', 'lower', 'release'],
                features: {
                    move: { handClosed: true, moving: true },
                    lower: { handClosed: true, moving: true, direction: 'down' },
                    release: { handOpening: true, moving: false }
                }
            },
            
            'pour': {
                phases: ['approach', 'tilt', 'pour', 'upright'],
                features: {
                    approach: { moving: true, tilting: false },
                    tilt: { moving: false, tilting: true },
                    pour: { moving: true, tilting: true },
                    upright: { moving: false, tilting: false }
                }
            }
        };
    }
    
    processFrame(vrData, robotData) {
        const features = this.extractFeatures(vrData, robotData);
        const action = this.classifyAction(features);
        
        if (action !== this.currentAction) {
            this.onActionChange(this.currentAction, action, features);
            this.currentAction = action;
        }
        
        return {
            action: action,
            phase: this.getCurrentPhase(action, features),
            confidence: this.getConfidence(action, features),
            features: features
        };
    }
    
    extractFeatures(vrData, robotData) {
        const features = {};
        
        // 손 움직임 특성
        if (vrData.rightHand) {
            const handVelocity = this.calculateHandVelocity(vrData.rightHand);
            features.handMoving = handVelocity > this.velocityThreshold;
            features.handVelocity = handVelocity;
            
            // 그립 상태
            const gripGesture = vrData.rightHand.gestures.pinch || 
                              vrData.rightHand.gestures.fist;
            features.handClosed = gripGesture;
        }
        
        // 로봇 움직임 특성
        if (robotData.endEffectorPose) {
            const eeVelocity = this.calculateEEVelocity(robotData);
            features.robotMoving = eeVelocity > this.velocityThreshold;
            features.robotVelocity = eeVelocity;
            
            // 움직임 방향
            features.movementDirection = this.getMovementDirection(robotData.endEffectorPose);
        }
        
        // 그리퍼 상태
        if (robotData.gripperState !== undefined) {
            features.gripperClosing = robotData.gripperState > 0.8;
            features.gripperOpening = robotData.gripperState < 0.2;
        }
        
        // 기울임 동작 (pour)
        if (robotData.endEffectorPose) {
            const tilt = this.calculateTilt(robotData.endEffectorPose.rotation);
            features.tilting = Math.abs(tilt) > 0.3; // 30도 이상
            features.tiltAngle = tilt;
        }
        
        return features;
    }
    
    classifyAction(features) {
        let bestMatch = null;
        let bestScore = 0;
        
        for (const [actionName, template] of Object.entries(this.actionTemplates)) {
            const score = this.calculateActionScore(features, template);
            
            if (score > bestScore) {
                bestScore = score;
                bestMatch = actionName;
            }
        }
        
        // 최소 신뢰도 임계값
        return bestScore > 0.6 ? bestMatch : 'idle';
    }
    
    calculateActionScore(features, template) {
        let totalScore = 0;
        let featureCount = 0;
        
        for (const phase of template.phases) {
            const phaseFeatures = template.features[phase];
            
            for (const [featureName, expectedValue] of Object.entries(phaseFeatures)) {
                if (features.hasOwnProperty(featureName)) {
                    const actualValue = features[featureName];
                    
                    if (typeof expectedValue === 'boolean') {
                        totalScore += (actualValue === expectedValue) ? 1 : 0;
                    } else if (typeof expectedValue === 'string') {
                        totalScore += (actualValue === expectedValue) ? 1 : 0;
                    }
                    
                    featureCount++;
                }
            }
        }
        
        return featureCount > 0 ? totalScore / featureCount : 0;
    }
    
    calculateHandVelocity(handData) {
        if (!this.previousHandData) {
            this.previousHandData = handData;
            return 0;
        }
        
        const currentPos = handData.joints.wrist.position;
        const prevPos = this.previousHandData.joints.wrist.position;
        
        const dx = currentPos.x - prevPos.x;
        const dy = currentPos.y - prevPos.y;
        const dz = currentPos.z - prevPos.z;
        
        const distance = Math.sqrt(dx*dx + dy*dy + dz*dz);
        const timeStep = 1/90; // 90 FPS
        
        this.previousHandData = handData;
        
        return distance / timeStep;
    }
    
    onActionChange(previousAction, newAction, features) {
        const timestamp = performance.now();
        
        if (previousAction) {
            // 이전 액션 종료
            this.actionSequence.push({
                action: previousAction,
                endTime: timestamp,
                duration: timestamp - (this.actionStartTime || timestamp)
            });
        }
        
        if (newAction && newAction !== 'idle') {
            // 새 액션 시작
            this.actionStartTime = timestamp;
            
            console.log(`Action changed: ${previousAction} → ${newAction}`);
        }
    }
}
```

---

## 3. 로봇 데이터 수집

### 3.1 고주파수 로봇 상태 기록

**로봇 데이터 스트림**
```javascript
class RobotDataStream {
    constructor(robotController) {
        this.robot = robotController;
        this.isCapturing = false;
        this.sampleRate = 100; // Hz
        this.sampleInterval = 1000 / this.sampleRate;
        
        this.previousState = null;
        this.setupCapture();
    }
    
    start() {
        this.isCapturing = true;
        this.startCapture();
    }
    
    stop() {
        this.isCapturing = false;
    }
    
    startCapture() {
        const capture = () => {
            if (!this.isCapturing) return;
            
            const robotData = this.captureRobotState();
            
            if (this.onData && robotData) {
                this.onData(robotData);
            }
            
            setTimeout(capture, this.sampleInterval);
        };
        
        capture();
    }
    
    captureRobotState() {
        if (!this.robot.isConnected()) return null;
        
        const state = {
            timestamp: performance.now(),
            
            // 관절 정보
            joints: this.captureJointData(),
            
            // 엔드이펙터 정보
            endEffector: this.captureEndEffectorData(),
            
            // 그리퍼 정보
            gripper: this.captureGripperData(),
            
            // 센서 정보
            sensors: this.captureSensorData(),
            
            // 제어 명령
            commands: this.captureCommandData()
        };
        
        // 속도/가속도 계산
        if (this.previousState) {
            state.derivatives = this.calculateDerivatives(state, this.previousState);
        }
        
        this.previousState = state;
        return state;
    }
    
    captureJointData() {
        const jointCount = this.robot.getJointCount();
        const joints = [];
        
        for (let i = 0; i < jointCount; i++) {
            joints.push({
                index: i,
                position: this.robot.getJointPosition(i),
                velocity: this.robot.getJointVelocity(i),
                effort: this.robot.getJointEffort(i),
                temperature: this.robot.getJointTemperature(i),
                current: this.robot.getJointCurrent(i)
            });
        }
        
        return joints;
    }
    
    captureEndEffectorData() {
        const pose = this.robot.getEndEffectorPose();
        
        return {
            position: {
                x: pose.position.x,
                y: pose.position.y,
                z: pose.position.z
            },
            rotation: {
                x: pose.rotation.x,
                y: pose.rotation.y,
                z: pose.rotation.z,
                w: pose.rotation.w
            },
            velocity: this.robot.getEndEffectorVelocity(),
            force: this.robot.getEndEffectorForce(),
            torque: this.robot.getEndEffectorTorque()
        };
    }
    
    captureGripperData() {
        return {
            position: this.robot.getGripperPosition(),
            velocity: this.robot.getGripperVelocity(),
            force: this.robot.getGripperForce(),
            isGrasping: this.robot.isGrasping(),
            width: this.robot.getGripperWidth()
        };
    }
    
    captureSensorData() {
        return {
            forceTorque: this.robot.getForceTorqueSensor(),
            tactile: this.robot.getTactileSensor(),
            proximity: this.robot.getProximitySensor(),
            imu: this.robot.getIMU(),
            encoders: this.robot.getEncoders()
        };
    }
    
    captureCommandData() {
        return {
            currentCommand: this.robot.getCurrentCommand(),
            commandQueue: this.robot.getCommandQueue(),
            executionStatus: this.robot.getExecutionStatus(),
            errors: this.robot.getErrors()
        };
    }
    
    calculateDerivatives(current, previous) {
        const dt = (current.timestamp - previous.timestamp) / 1000; // seconds
        
        const derivatives = {
            jointVelocities: [],
            jointAccelerations: [],
            endEffectorVelocity: null,
            endEffectorAcceleration: null
        };
        
        // 관절 속도/가속도
        current.joints.forEach((joint, index) => {
            const prevJoint = previous.joints[index];
            
            const velocity = (joint.position - prevJoint.position) / dt;
            const acceleration = (joint.velocity - prevJoint.velocity) / dt;
            
            derivatives.jointVelocities.push(velocity);
            derivatives.jointAccelerations.push(acceleration);
        });
        
        // 엔드이펙터 속도/가속도
        const currentPos = current.endEffector.position;
        const prevPos = previous.endEffector.position;
        
        derivatives.endEffectorVelocity = {
            x: (currentPos.x - prevPos.x) / dt,
            y: (currentPos.y - prevPos.y) / dt,
            z: (currentPos.z - prevPos.z) / dt
        };
        
        if (previous.derivatives?.endEffectorVelocity) {
            const currentVel = derivatives.endEffectorVelocity;
            const prevVel = previous.derivatives.endEffectorVelocity;
            
            derivatives.endEffectorAcceleration = {
                x: (currentVel.x - prevVel.x) / dt,
                y: (currentVel.y - prevVel.y) / dt,
                z: (currentVel.z - prevVel.z) / dt
            };
        }
        
        return derivatives;
    }
}
```

### 3.2 환경 데이터 수집

**멀티모달 센서 데이터**
```javascript
class CameraDataStream {
    constructor() {
        this.cameras = [];
        this.isCapturing = false;
        this.frameRate = 30; // FPS
        
        this.setupCameras();
    }
    
    setupCameras() {
        // 메인 RGB 카메라
        this.addCamera('rgb_main', {
            type: 'rgb',
            resolution: '1920x1080',
            position: 'end_effector'
        });
        
        // 깊이 카메라
        this.addCamera('depth_main', {
            type: 'depth',
            resolution: '640x480',
            position: 'end_effector'
        });
        
        // 전역 뷰 카메라
        this.addCamera('rgb_global', {
            type: 'rgb',
            resolution: '1280x720',
            position: 'fixed_overhead'
        });
    }
    
    addCamera(name, config) {
        const camera = new CameraCapture(config);
        this.cameras.push({
            name: name,
            config: config,
            capture: camera
        });
    }
    
    start() {
        this.isCapturing = true;
        
        // 모든 카메라 시작
        this.cameras.forEach(camera => {
            camera.capture.start();
        });
        
        this.captureLoop();
    }
    
    stop() {
        this.isCapturing = false;
        
        this.cameras.forEach(camera => {
            camera.capture.stop();
        });
    }
    
    captureLoop() {
        if (!this.isCapturing) return;
        
        const cameraData = this.captureAllCameras();
        
        if (this.onData && cameraData) {
            this.onData(cameraData);
        }
        
        setTimeout(() => this.captureLoop(), 1000 / this.frameRate);
    }
    
    captureAllCameras() {
        const data = {
            timestamp: performance.now(),
            cameras: {}
        };
        
        this.cameras.forEach(camera => {
            const frame = camera.capture.getFrame();
            if (frame) {
                data.cameras[camera.name] = {
                    type: camera.config.type,
                    resolution: camera.config.resolution,
                    position: camera.config.position,
                    frame: frame,
                    frameNumber: camera.capture.getFrameNumber(),
                    timestamp: frame.timestamp
                };
            }
        });
        
        return data;
    }
}

class CameraCapture {
    constructor(config) {
        this.config = config;
        this.isCapturing = false;
        this.frameNumber = 0;
        
        this.setupCamera();
    }
    
    setupCamera() {
        if (this.config.type === 'rgb') {
            this.setupRGBCamera();
        } else if (this.config.type === 'depth') {
            this.setupDepthCamera();
        }
    }
    
    setupRGBCamera() {
        const constraints = {
            video: {
                width: { ideal: parseInt(this.config.resolution.split('x')[0]) },
                height: { ideal: parseInt(this.config.resolution.split('x')[1]) },
                frameRate: { ideal: 30 }
            }
        };
        
        navigator.mediaDevices.getUserMedia(constraints)
            .then(stream => {
                this.stream = stream;
                this.video = document.createElement('video');
                this.video.srcObject = stream;
                this.video.play();
                
                this.canvas = document.createElement('canvas');
                this.context = this.canvas.getContext('2d');
            })
            .catch(error => {
                console.error('Camera setup failed:', error);
            });
    }
    
    setupDepthCamera() {
        // Intel RealSense 또는 Kinect 연동
        this.depthSensor = new DepthSensor(this.config);
    }
    
    start() {
        this.isCapturing = true;
    }
    
    stop() {
        this.isCapturing = false;
    }
    
    getFrame() {
        if (!this.isCapturing) return null;
        
        if (this.config.type === 'rgb') {
            return this.captureRGBFrame();
        } else if (this.config.type === 'depth') {
            return this.captureDepthFrame();
        }
        
        return null;
    }
    
    captureRGBFrame() {
        if (!this.video || this.video.readyState < 2) return null;
        
        this.canvas.width = this.video.videoWidth;
        this.canvas.height = this.video.videoHeight;
        
        this.context.drawImage(this.video, 0, 0);
        
        const imageData = this.context.getImageData(
            0, 0, this.canvas.width, this.canvas.height
        );
        
        this.frameNumber++;
        
        return {
            type: 'rgb',
            width: this.canvas.width,
            height: this.canvas.height,
            data: imageData.data,
            timestamp: performance.now(),
            frameNumber: this.frameNumber
        };
    }
    
    captureDepthFrame() {
        if (!this.depthSensor) return null;
        
        const depthData = this.depthSensor.getDepthFrame();
        this.frameNumber++;
        
        return {
            type: 'depth',
            width: depthData.width,
            height: depthData.height,
            data: depthData.depths,
            timestamp: performance.now(),
            frameNumber: this.frameNumber
        };
    }
    
    getFrameNumber() {
        return this.frameNumber;
    }
}
```

---

## 4. 데이터 저장 및 관리

### 4.1 효율적인 데이터 저장

**압축 및 인덱싱**
```javascript
class DataStorage {
    constructor() {
        this.dbName = 'VRRobotDataset';
        this.dbVersion = 1;
        this.db = null;
        
        this.compressionWorker = new Worker('compression-worker.js');
        this.initDatabase();
    }
    
    async initDatabase() {
        return new Promise((resolve, reject) => {
            const request = indexedDB.open(this.dbName, this.dbVersion);
            
            request.onerror = () => reject(request.error);
            request.onsuccess = () => {
                this.db = request.result;
                resolve(this.db);
            };
            
            request.onupgradeneeded = (event) => {
                const db = event.target.result;
                
                // 세션 메타데이터 스토어
                const sessionsStore = db.createObjectStore('sessions', {
                    keyPath: 'sessionId'
                });
                
                sessionsStore.createIndex('taskType', 'taskType', { unique: false });
                sessionsStore.createIndex('timestamp', 'startTime', { unique: false });
                sessionsStore.createIndex('success', 'success', { unique: false });
                
                // 프레임 데이터 스토어
                const framesStore = db.createObjectStore('frames', {
                    keyPath: ['sessionId', 'frameIndex']
                });
                
                framesStore.createIndex('sessionId', 'sessionId', { unique: false });
                framesStore.createIndex('timestamp', 'timestamp', { unique: false });
                
                // 압축된 이미지 스토어
                const imagesStore = db.createObjectStore('images', {
                    keyPath: ['sessionId', 'frameIndex', 'cameraName']
                });
            };
        });
    }
    
    async saveMetadata(sessionId, metadata) {
        const transaction = this.db.transaction(['sessions'], 'readwrite');
        const store = transaction.objectStore('sessions');
        
        await store.put({
            sessionId: sessionId,
            ...metadata
        });
    }
    
    async updateMetadata(sessionId, updates) {
        const transaction = this.db.transaction(['sessions'], 'readwrite');
        const store = transaction.objectStore('sessions');
        
        const session = await store.get(sessionId);
        if (session) {
            Object.assign(session, updates);
            await store.put(session);
        }
    }
    
    async saveBatch(sessionId, frames) {
        const transaction = this.db.transaction(['frames', 'images'], 'readwrite');
        const framesStore = transaction.objectStore('frames');
        const imagesStore = transaction.objectStore('images');
        
        for (const frame of frames) {
            // 이미지 데이터 분리 및 압축
            const frameData = { ...frame };
            const imagePromises = [];
            
            if (frameData.data.camera?.cameras) {
                const cameraData = frameData.data.camera.cameras;
                
                for (const [cameraName, camera] of Object.entries(cameraData)) {
                    if (camera.frame) {
                        // 이미지 압축
                        const compressed = await this.compressImage(camera.frame);
                        
                        imagePromises.push(
                            imagesStore.put({
                                sessionId: sessionId,
                                frameIndex: frame.frameIndex,
                                cameraName: cameraName,
                                type: camera.type,
                                width: camera.frame.width,
                                height: camera.frame.height,
                                compressedData: compressed,
                                timestamp: camera.timestamp
                            })
                        );
                        
                        // 원본 이미지 데이터 제거
                        delete cameraData[cameraName].frame;
                    }
                }
            }
            
            // 프레임 메타데이터 저장
            await framesStore.put(frameData);
            
            // 압축된 이미지들 저장
            await Promise.all(imagePromises);
        }
    }
    
    async compressImage(imageFrame) {
        return new Promise((resolve, reject) => {
            this.compressionWorker.postMessage({
                type: 'compress',
                data: imageFrame
            });
            
            this.compressionWorker.onmessage = (event) => {
                if (event.data.type === 'compressed') {
                    resolve(event.data.compressedData);
                } else if (event.data.type === 'error') {
                    reject(new Error(event.data.message));
                }
            };
        });
    }
    
    async exportSession(sessionId, format = 'hdf5') {
        const session = await this.getSessionMetadata(sessionId);
        const frames = await this.getAllFrames(sessionId);
        
        if (format === 'hdf5') {
            return this.exportToHDF5(session, frames);
        } else if (format === 'rosbag') {
            return this.exportToROSBag(session, frames);
        } else if (format === 'json') {
            return this.exportToJSON(session, frames);
        }
    }
    
    async exportToHDF5(session, frames) {
        // HDF5 export implementation
        const hdf5Data = {
            metadata: session,
            trajectories: this.extractTrajectories(frames),
            actions: this.extractActions(frames),
            images: await this.extractImages(frames),
            robot_states: this.extractRobotStates(frames)
        };
        
        return hdf5Data;
    }
    
    extractTrajectories(frames) {
        return frames.map(frame => ({
            timestamp: frame.timestamp,
            end_effector_pose: frame.data.robot?.endEffector,
            joint_positions: frame.data.robot?.joints?.map(j => j.position),
            hand_poses: {
                left: frame.data.vr?.leftHand,
                right: frame.data.vr?.rightHand
            }
        }));
    }
    
    async getSessionList(filters = {}) {
        const transaction = this.db.transaction(['sessions'], 'readonly');
        const store = transaction.objectStore('sessions');
        
        let cursor;
        if (filters.taskType) {
            const index = store.index('taskType');
            cursor = await index.openCursor(filters.taskType);
        } else {
            cursor = await store.openCursor();
        }
        
        const sessions = [];
        
        while (cursor) {
            const session = cursor.value;
            
            // 필터 적용
            if (this.matchesFilters(session, filters)) {
                sessions.push(session);
            }
            
            cursor = await cursor.continue();
        }
        
        return sessions;
    }
    
    matchesFilters(session, filters) {
        if (filters.success !== undefined && session.success !== filters.success) {
            return false;
        }
        
        if (filters.minDuration && session.duration < filters.minDuration) {
            return false;
        }
        
        if (filters.maxDuration && session.duration > filters.maxDuration) {
            return false;
        }
        
        return true;
    }
}
```

### 4.2 데이터 품질 관리

**품질 검증 시스템**
```javascript
class DataQualityChecker {
    constructor() {
        this.qualityMetrics = {
            completeness: 0,
            consistency: 0,
            accuracy: 0,
            reliability: 0
        };
        
        this.thresholds = {
            minFrameRate: 25, // FPS
            maxLatency: 100, // ms
            minHandTracking: 0.8, // confidence
            maxJointVelocity: 10 // rad/s
        };
    }
    
    checkSessionQuality(sessionId) {
        const session = this.loadSession(sessionId);
        const frames = this.loadFrames(sessionId);
        
        const quality = {
            sessionId: sessionId,
            overallScore: 0,
            metrics: {},
            issues: [],
            recommendations: []
        };
        
        // 데이터 완성도 검사
        quality.metrics.completeness = this.checkCompleteness(frames);
        
        // 일관성 검사
        quality.metrics.consistency = this.checkConsistency(frames);
        
        // 정확도 검사
        quality.metrics.accuracy = this.checkAccuracy(frames);
        
        // 신뢰성 검사
        quality.metrics.reliability = this.checkReliability(frames);
        
        // 전체 점수 계산
        quality.overallScore = this.calculateOverallScore(quality.metrics);
        
        // 이슈 및 권장사항 생성
        this.generateRecommendations(quality);
        
        return quality;
    }
    
    checkCompleteness(frames) {
        let score = 0;
        let totalChecks = 0;
        
        frames.forEach(frame => {
            totalChecks += 4; // VR, Robot, Camera, Audio
            
            // VR 데이터 확인
            if (frame.data.vr && frame.data.vr.leftHand && frame.data.vr.rightHand) {
                score += 1;
            }
            
            // 로봇 데이터 확인
            if (frame.data.robot && frame.data.robot.joints && frame.data.robot.endEffector) {
                score += 1;
            }
            
            // 카메라 데이터 확인
            if (frame.data.camera && Object.keys(frame.data.camera.cameras).length > 0) {
                score += 1;
            }
            
            // 오디오 데이터 확인
            if (frame.data.audio) {
                score += 1;
            }
        });
        
        return totalChecks > 0 ? score / totalChecks : 0;
    }
    
    checkConsistency(frames) {
        let consistencyScore = 1.0;
        const issues = [];
        
        for (let i = 1; i < frames.length; i++) {
            const current = frames[i];
            const previous = frames[i - 1];
            
            // 타임스탬프 일관성
            const timeDiff = current.timestamp - previous.timestamp;
            if (timeDiff < 0 || timeDiff > 200) { // 200ms 이상 차이
                issues.push(`Frame ${i}: Invalid timestamp difference ${timeDiff}ms`);
                consistencyScore -= 0.01;
            }
            
            // 관절 각도 급변 확인
            if (current.data.robot?.joints && previous.data.robot?.joints) {
                current.data.robot.joints.forEach((joint, jointIndex) => {
                    const prevJoint = previous.data.robot.joints[jointIndex];
                    if (prevJoint) {
                        const angleDiff = Math.abs(joint.position - prevJoint.position);
                        const maxChange = this.thresholds.maxJointVelocity * (timeDiff / 1000);
                        
                        if (angleDiff > maxChange) {
                            issues.push(`Frame ${i}, Joint ${jointIndex}: Excessive angle change ${angleDiff.toFixed(3)}`);
                            consistencyScore -= 0.005;
                        }
                    }
                });
            }
        }
        
        return Math.max(0, consistencyScore);
    }
    
    checkAccuracy(frames) {
        let accuracyScore = 1.0;
        
        frames.forEach((frame, index) => {
            // 핸드 트래킹 신뢰도
            if (frame.data.vr?.leftHand?.confidence < this.thresholds.minHandTracking) {
                accuracyScore -= 0.001;
            }
            
            if (frame.data.vr?.rightHand?.confidence < this.thresholds.minHandTracking) {
                accuracyScore -= 0.001;
            }
            
            // 로봇-VR 동기화 확인
            const latency = this.calculateLatency(frame);
            if (latency > this.thresholds.maxLatency) {
                accuracyScore -= 0.002;
            }
        });
        
        return Math.max(0, accuracyScore);
    }
    
    checkReliability(frames) {
        const frameRate = this.calculateFrameRate(frames);
        const reliabilityScore = Math.min(1.0, frameRate / this.thresholds.minFrameRate);
        
        return reliabilityScore;
    }
    
    calculateOverallScore(metrics) {
        const weights = {
            completeness: 0.3,
            consistency: 0.3,
            accuracy: 0.2,
            reliability: 0.2
        };
        
        let score = 0;
        for (const [metric, value] of Object.entries(metrics)) {
            score += value * weights[metric];
        }
        
        return score;
    }
    
    generateRecommendations(quality) {
        if (quality.metrics.completeness < 0.9) {
            quality.recommendations.push("데이터 완성도가 낮습니다. 센서 연결을 확인하세요.");
        }
        
        if (quality.metrics.consistency < 0.8) {
            quality.recommendations.push("데이터 일관성이 부족합니다. 네트워크 지연을 확인하세요.");
        }
        
        if (quality.metrics.accuracy < 0.8) {
            quality.recommendations.push("핸드 트래킹 정확도가 낮습니다. 조명과 카메라 위치를 확인하세요.");
        }
        
        if (quality.metrics.reliability < 0.8) {
            quality.recommendations.push("프레임 레이트가 불안정합니다. 시스템 성능을 확인하세요.");
        }
    }
}
```

---

## 5. 머신러닝 데이터셋 준비

### 5.1 데이터 전처리

**훈련용 데이터 변환**
```python
import numpy as np
import h5py
from sklearn.preprocessing import StandardScaler
import cv2

class VRDatasetProcessor:
    def __init__(self, dataset_path):
        self.dataset_path = dataset_path
        self.scaler = StandardScaler()
        self.action_encoder = self.create_action_encoder()
        
    def create_action_encoder(self):
        actions = ['idle', 'pick', 'place', 'pour', 'push', 'pull']
        return {action: idx for idx, action in enumerate(actions)}
    
    def process_session(self, session_id):
        """세션 데이터를 ML 훈련용으로 전처리"""
        
        # 원본 데이터 로드
        raw_data = self.load_raw_session(session_id)
        
        # 전처리 파이프라인
        processed = {
            'observations': self.process_observations(raw_data),
            'actions': self.process_actions(raw_data),
            'rewards': self.compute_rewards(raw_data),
            'metadata': self.extract_metadata(raw_data)
        }
        
        return processed
    
    def process_observations(self, raw_data):
        """관측 데이터 전처리"""
        
        observations = {}
        
        # 로봇 상태 벡터
        robot_states = []
        for frame in raw_data['frames']:
            if 'robot' in frame['data']:
                robot_data = frame['data']['robot']
                
                # 관절 각도 (6차원)
                joint_positions = [j['position'] for j in robot_data['joints']]
                
                # 엔드이펙터 포즈 (7차원: 위치 3 + 쿼터니언 4)
                ee_pose = robot_data['endEffector']
                ee_vector = [
                    ee_pose['position']['x'],
                    ee_pose['position']['y'], 
                    ee_pose['position']['z'],
                    ee_pose['rotation']['x'],
                    ee_pose['rotation']['y'],
                    ee_pose['rotation']['z'],
                    ee_pose['rotation']['w']
                ]
                
                # 그리퍼 상태 (1차원)
                gripper_state = [robot_data['gripper']['position']]
                
                # 상태 벡터 결합
                state_vector = joint_positions + ee_vector + gripper_state
                robot_states.append(state_vector)
        
        observations['robot_state'] = np.array(robot_states)
        
        # 이미지 처리
        observations['images'] = self.process_images(raw_data)
        
        # VR 입력 처리  
        observations['vr_input'] = self.process_vr_input(raw_data)
        
        return observations
    
    def process_images(self, raw_data):
        """이미지 데이터 전처리"""
        
        processed_images = {
            'rgb': [],
            'depth': []
        }
        
        for frame in raw_data['frames']:
            if 'camera' in frame['data']:
                cameras = frame['data']['camera']['cameras']
                
                for camera_name, camera_data in cameras.items():
                    if camera_data['type'] == 'rgb':
                        # RGB 이미지 전처리
                        img = self.decompress_image(camera_data['compressedData'])
                        img_resized = cv2.resize(img, (224, 224))  # ResNet 입력 크기
                        img_normalized = img_resized.astype(np.float32) / 255.0
                        processed_images['rgb'].append(img_normalized)
                        
                    elif camera_data['type'] == 'depth':
                        # 깊이 이미지 전처리
                        depth = self.decompress_depth(camera_data['compressedData'])
                        depth_normalized = depth / np.max(depth)  # 정규화
                        processed_images['depth'].append(depth_normalized)
        
        return {
            'rgb': np.array(processed_images['rgb']),
            'depth': np.array(processed_images['depth'])
        }
    
    def process_vr_input(self, raw_data):
        """VR 입력 데이터 전처리"""
        
        vr_inputs = []
        
        for frame in raw_data['frames']:
            if 'vr' in frame['data']:
                vr_data = frame['data']['vr']
                
                # 핸드 포즈 벡터화
                hand_vector = []
                
                for hand_side in ['leftHand', 'rightHand']:
                    if hand_side in vr_data and vr_data[hand_side]:
                        hand = vr_data[hand_side]
                        
                        # 주요 관절 위치만 사용 (차원 축소)
                        key_joints = [
                            'wrist', 'thumb-tip', 'index-finger-tip',
                            'middle-finger-tip', 'ring-finger-tip', 'pinky-finger-tip'
                        ]
                        
                        for joint_name in key_joints:
                            if joint_name in hand['joints']:
                                joint = hand['joints'][joint_name]
                                hand_vector.extend([
                                    joint['position']['x'],
                                    joint['position']['y'],
                                    joint['position']['z']
                                ])
                            else:
                                hand_vector.extend([0, 0, 0])  # 누락된 관절
                        
                        # 제스처 인코딩
                        gestures = hand.get('gestures', {})
                        gesture_vector = [
                            float(gestures.get('pinch', False)),
                            float(gestures.get('fist', False)),
                            float(gestures.get('point', False)),
                            float(gestures.get('thumbsUp', False))
                        ]
                        hand_vector.extend(gesture_vector)
                    else:
                        # 핸드 데이터 없음 - 제로 패딩
                        hand_vector.extend([0] * (6 * 3 + 4))  # 6관절*3좌표 + 4제스처
                
                vr_inputs.append(hand_vector)
        
        return np.array(vr_inputs)
    
    def process_actions(self, raw_data):
        """액션 라벨 생성"""
        
        actions = []
        
        for frame in raw_data['frames']:
            if 'action' in frame:
                action_name = frame['action']['action']
                action_idx = self.action_encoder.get(action_name, 0)  # 'idle' = 0
                actions.append(action_idx)
            else:
                actions.append(0)  # 기본값: idle
        
        return np.array(actions)
    
    def compute_rewards(self, raw_data):
        """보상 신호 계산"""
        
        rewards = []
        
        for i, frame in enumerate(raw_data['frames']):
            reward = 0.0
            
            # 작업 완료 보상
            if 'action' in frame and frame['action'].get('success', False):
                reward += 10.0
            
            # 충돌 페널티
            if 'robot' in frame['data']:
                robot_data = frame['data']['robot']
                if robot_data.get('collision', False):
                    reward -= 5.0
            
            # 움직임 효율성 (부드러운 움직임 선호)
            if i > 0:
                prev_frame = raw_data['frames'][i-1]
                if 'robot' in prev_frame['data'] and 'robot' in frame['data']:
                    velocity_penalty = self.calculate_velocity_penalty(
                        prev_frame['data']['robot'],
                        frame['data']['robot']
                    )
                    reward -= velocity_penalty
            
            rewards.append(reward)
        
        return np.array(rewards)
    
    def create_training_dataset(self, session_ids, output_path):
        """여러 세션을 결합하여 훈련 데이터셋 생성"""
        
        all_observations = []
        all_actions = []
        all_rewards = []
        
        for session_id in session_ids:
            print(f"Processing session: {session_id}")
            
            processed = self.process_session(session_id)
            
            all_observations.append(processed['observations'])
            all_actions.append(processed['actions'])
            all_rewards.append(processed['rewards'])
        
        # 데이터 결합
        combined_dataset = {
            'robot_states': np.vstack([obs['robot_state'] for obs in all_observations]),
            'images_rgb': np.vstack([obs['images']['rgb'] for obs in all_observations]),
            'images_depth': np.vstack([obs['images']['depth'] for obs in all_observations]),
            'vr_inputs': np.vstack([obs['vr_input'] for obs in all_observations]),
            'actions': np.concatenate(all_actions),
            'rewards': np.concatenate(all_rewards)
        }
        
        # 정규화
        combined_dataset['robot_states'] = self.scaler.fit_transform(
            combined_dataset['robot_states']
        )
        
        # HDF5로 저장
        self.save_dataset_hdf5(combined_dataset, output_path)
        
        return combined_dataset
    
    def save_dataset_hdf5(self, dataset, output_path):
        """HDF5 형식으로 데이터셋 저장"""
        
        with h5py.File(output_path, 'w') as f:
            # 메타데이터
            f.attrs['num_samples'] = len(dataset['actions'])
            f.attrs['action_classes'] = list(self.action_encoder.keys())
            
            # 데이터
            f.create_dataset('robot_states', data=dataset['robot_states'])
            f.create_dataset('images_rgb', data=dataset['images_rgb'])
            f.create_dataset('images_depth', data=dataset['images_depth'])
            f.create_dataset('vr_inputs', data=dataset['vr_inputs'])
            f.create_dataset('actions', data=dataset['actions'])
            f.create_dataset('rewards', data=dataset['rewards'])
            
        print(f"Dataset saved to {output_path}")
```

---

## 6. 참고 자료

- [HDF5 Python Documentation](https://docs.h5py.org/)
- [OpenCV Python Documentation](https://opencv-python-tutroals.readthedocs.io/)
- [ROS Bag Format](http://wiki.ros.org/rosbag)
- [WebRTC Recording API](https://developer.mozilla.org/en-US/docs/Web/API/MediaRecorder)

---

[← 7.4 VR 인터페이스](04_vr_interface.md) | [메인 가이드로 돌아가기](../README.md)
