# 7.4 VR 인터페이스

VR 환경에서의 직관적인 로봇 제어 인터페이스 설계 및 구현을 학습합니다.

## 1. VR UI 설계 원칙

### 1.1 VR UI 가이드라인

**공간적 UI 설계**
```
✓ 3D 공간 활용 (깊이감 제공)
✓ 팔 길이 내 배치 (80cm 이내)
✓ 시선 추적 고려
✓ 햅틱 피드백 활용
✓ 최소한의 텍스트 사용
```

**사용성 원칙**
```
✓ 직관적 제스처 매핑
✓ 즉각적인 시각적 피드백
✓ 실수 방지 및 되돌리기
✓ 피로도 최소화
✓ 접근성 고려
```

### 1.2 VR에서의 로봇 제어 모델

**제어 모드별 특성**
```javascript
const ControlModes = {
    DIRECT: {
        name: "직접 제어",
        description: "손의 움직임이 직접 로봇에 반영",
        latency: "< 50ms",
        precision: "높음",
        fatigue: "높음"
    },
    
    RELATIVE: {
        name: "상대 제어", 
        description: "손의 움직임을 상대적 변위로 변환",
        latency: "< 100ms",
        precision: "중간",
        fatigue: "낮음"
    },
    
    GESTURE: {
        name: "제스처 제어",
        description: "특정 제스처로 명령 실행",
        latency: "< 200ms",
        precision: "낮음",
        fatigue: "매우 낮음"
    }
};
```

---

## 2. 3D UI 컴포넌트

### 2.1 기본 UI 요소

**3D 버튼**
```javascript
class VR3DButton {
    constructor(text, position, size = 0.2) {
        this.mesh = this.createButtonMesh(text, size);
        this.mesh.position.copy(position);
        
        this.isPressed = false;
        this.isHovered = false;
        
        this.onClickCallbacks = [];
        this.setupInteraction();
    }
    
    createButtonMesh(text, size) {
        // 버튼 배경
        const geometry = new THREE.BoxGeometry(size * 2, size, size * 0.2);
        const material = new THREE.MeshPhongMaterial({
            color: 0x2196F3,
            transparent: true,
            opacity: 0.8
        });
        
        const button = new THREE.Mesh(geometry, material);
        
        // 텍스트 추가
        this.addText(button, text, size);
        
        return button;
    }
    
    addText(parent, text, size) {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        
        canvas.width = 512;
        canvas.height = 256;
        
        context.fillStyle = '#FFFFFF';
        context.font = 'Bold 48px Arial';
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(text, 256, 128);
        
        const texture = new THREE.CanvasTexture(canvas);
        const textMaterial = new THREE.MeshBasicMaterial({
            map: texture,
            transparent: true
        });
        
        const textGeometry = new THREE.PlaneGeometry(size * 1.8, size * 0.9);
        const textMesh = new THREE.Mesh(textGeometry, textMaterial);
        textMesh.position.z = size * 0.11;
        
        parent.add(textMesh);
    }
    
    setupInteraction() {
        this.mesh.userData = {
            interactive: true,
            onHover: (controller) => this.onHover(controller),
            onLeave: (controller) => this.onLeave(controller),
            onClick: (controller) => this.onClick(controller)
        };
    }
    
    onHover(controller) {
        if (!this.isHovered) {
            this.isHovered = true;
            this.mesh.material.color.setHex(0x4CAF50);
            this.mesh.scale.setScalar(1.1);
            
            // 햅틱 피드백
            if (controller.hapticActuators && controller.hapticActuators[0]) {
                controller.hapticActuators[0].pulse(0.3, 100);
            }
        }
    }
    
    onLeave(controller) {
        this.isHovered = false;
        this.mesh.material.color.setHex(0x2196F3);
        this.mesh.scale.setScalar(1.0);
    }
    
    onClick(controller) {
        this.isPressed = true;
        this.mesh.scale.setScalar(0.95);
        
        // 강한 햅틱 피드백
        if (controller.hapticActuators && controller.hapticActuators[0]) {
            controller.hapticActuators[0].pulse(0.8, 200);
        }
        
        // 콜백 실행
        this.onClickCallbacks.forEach(callback => callback());
        
        // 원래 크기로 복원
        setTimeout(() => {
            this.mesh.scale.setScalar(1.1);
            this.isPressed = false;
        }, 150);
    }
    
    onClick(callback) {
        this.onClickCallbacks.push(callback);
    }
}
```

**3D 슬라이더**
```javascript
class VR3DSlider {
    constructor(position, length = 0.5, orientation = 'horizontal') {
        this.length = length;
        this.value = 0.5; // 0.0 - 1.0
        
        this.group = new THREE.Group();
        this.group.position.copy(position);
        
        this.createSliderTrack();
        this.createSliderHandle();
        this.setupInteraction();
    }
    
    createSliderTrack() {
        const trackGeometry = new THREE.CylinderGeometry(0.01, 0.01, this.length);
        const trackMaterial = new THREE.MeshPhongMaterial({ color: 0x666666 });
        
        this.track = new THREE.Mesh(trackGeometry, trackMaterial);
        this.track.rotation.z = Math.PI / 2;
        this.group.add(this.track);
    }
    
    createSliderHandle() {
        const handleGeometry = new THREE.SphereGeometry(0.03);
        const handleMaterial = new THREE.MeshPhongMaterial({ color: 0xFF5722 });
        
        this.handle = new THREE.Mesh(handleGeometry, handleMaterial);
        this.updateHandlePosition();
        this.group.add(this.handle);
        
        this.handle.userData = {
            interactive: true,
            draggable: true,
            onDrag: (controller, position) => this.onDrag(controller, position)
        };
    }
    
    updateHandlePosition() {
        const x = (this.value - 0.5) * this.length;
        this.handle.position.set(x, 0, 0);
    }
    
    onDrag(controller, worldPosition) {
        // 월드 좌표를 로컬 좌표로 변환
        const localPosition = this.group.worldToLocal(worldPosition.clone());
        
        // X축 제한
        const clampedX = Math.max(-this.length/2, Math.min(this.length/2, localPosition.x));
        this.value = (clampedX / this.length) + 0.5;
        
        this.handle.position.set(clampedX, 0, 0);
        
        // 값 변경 이벤트
        this.onValueChange(this.value);
    }
    
    onValueChange(value) {
        // 오버라이드하여 사용
        console.log(`Slider value: ${value.toFixed(2)}`);
    }
    
    setValue(value) {
        this.value = Math.max(0, Math.min(1, value));
        this.updateHandlePosition();
    }
}
```

### 2.2 로봇 상태 표시 패널

**정보 표시 패널**
```javascript
class RobotStatusPanel {
    constructor(position) {
        this.group = new THREE.Group();
        this.group.position.copy(position);
        
        this.createPanel();
        this.createStatusDisplays();
        
        this.updateInterval = setInterval(() => {
            this.updateDisplays();
        }, 100);
    }
    
    createPanel() {
        const panelGeometry = new THREE.PlaneGeometry(0.8, 0.6);
        const panelMaterial = new THREE.MeshPhongMaterial({
            color: 0x1E1E1E,
            transparent: true,
            opacity: 0.9
        });
        
        this.panel = new THREE.Mesh(panelGeometry, panelMaterial);
        this.group.add(this.panel);
        
        // 테두리 추가
        const borderGeometry = new THREE.EdgesGeometry(panelGeometry);
        const borderMaterial = new THREE.LineBasicMaterial({ color: 0x00FFFF });
        const border = new THREE.LineSegments(borderGeometry, borderMaterial);
        this.group.add(border);
    }
    
    createStatusDisplays() {
        this.displays = {};
        
        // 로봇 상태 텍스트
        this.displays.status = this.createTextDisplay('상태: 연결됨', -0.3, 0.2, 0x00FF00);
        this.displays.mode = this.createTextDisplay('모드: 직접제어', -0.3, 0.1, 0xFFFFFF);
        this.displays.battery = this.createTextDisplay('배터리: 85%', -0.3, 0, 0xFFFF00);
        this.displays.latency = this.createTextDisplay('지연: 45ms', -0.3, -0.1, 0x00FFFF);
        
        // 조인트 각도 표시
        this.jointBars = [];
        for (let i = 0; i < 6; i++) {
            const bar = this.createProgressBar(0.1 + i * 0.1, -0.2, 0.08, 0.02);
            this.jointBars.push(bar);
        }
    }
    
    createTextDisplay(text, x, y, color) {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        
        canvas.width = 512;
        canvas.height = 64;
        
        context.fillStyle = `#${color.toString(16).padStart(6, '0')}`;
        context.font = 'Bold 24px Arial';
        context.fillText(text, 10, 40);
        
        const texture = new THREE.CanvasTexture(canvas);
        const material = new THREE.MeshBasicMaterial({
            map: texture,
            transparent: true
        });
        
        const geometry = new THREE.PlaneGeometry(0.25, 0.04);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(x, y, 0.01);
        
        this.group.add(mesh);
        
        return {
            mesh: mesh,
            canvas: canvas,
            context: context,
            texture: texture,
            text: text
        };
    }
    
    createProgressBar(x, y, width, height) {
        const bgGeometry = new THREE.PlaneGeometry(width, height);
        const bgMaterial = new THREE.MeshBasicMaterial({ color: 0x333333 });
        const background = new THREE.Mesh(bgGeometry, bgMaterial);
        background.position.set(x, y, 0.01);
        
        const fillGeometry = new THREE.PlaneGeometry(width * 0.8, height * 0.8);
        const fillMaterial = new THREE.MeshBasicMaterial({ color: 0x4CAF50 });
        const fill = new THREE.Mesh(fillGeometry, fillMaterial);
        fill.position.set(x, y, 0.02);
        
        this.group.add(background);
        this.group.add(fill);
        
        return {
            background: background,
            fill: fill,
            width: width,
            setValue: (value) => {
                const scale = Math.max(0, Math.min(1, value));
                fill.scale.x = scale;
                fill.position.x = x - (width * 0.8 * (1 - scale)) / 2;
            }
        };
    }
    
    updateDisplays() {
        // 로봇 서버에서 데이터 받아오기
        if (this.robotData) {
            this.updateTextDisplay(this.displays.status, 
                `상태: ${this.robotData.connected ? '연결됨' : '연결끊김'}`);
            
            this.updateTextDisplay(this.displays.latency, 
                `지연: ${this.robotData.latency}ms`);
            
            // 조인트 각도 업데이트
            if (this.robotData.jointAngles) {
                this.robotData.jointAngles.forEach((angle, index) => {
                    if (index < this.jointBars.length) {
                        const normalized = (angle + Math.PI) / (2 * Math.PI);
                        this.jointBars[index].setValue(normalized);
                    }
                });
            }
        }
    }
    
    updateTextDisplay(display, newText) {
        if (display.text !== newText) {
            display.context.clearRect(0, 0, display.canvas.width, display.canvas.height);
            display.context.fillText(newText, 10, 40);
            display.texture.needsUpdate = true;
            display.text = newText;
        }
    }
    
    setRobotData(data) {
        this.robotData = data;
    }
}
```

---

## 3. 제스처 기반 인터페이스

### 3.1 제스처 메뉴 시스템

**원형 메뉴**
```javascript
class CircularMenu {
    constructor(position) {
        this.group = new THREE.Group();
        this.group.position.copy(position);
        
        this.items = [];
        this.selectedIndex = -1;
        this.isVisible = false;
        
        this.createMenu();
    }
    
    createMenu() {
        const menuItems = [
            { text: '이동', icon: '➤', action: 'move' },
            { text: '회전', icon: '↻', action: 'rotate' },
            { text: '그립', icon: '✋', action: 'grip' },
            { text: '정지', icon: '■', action: 'stop' },
            { text: '홈', icon: '🏠', action: 'home' },
            { text: '설정', icon: '⚙', action: 'settings' }
        ];
        
        const radius = 0.15;
        const angleStep = (Math.PI * 2) / menuItems.length;
        
        menuItems.forEach((item, index) => {
            const angle = index * angleStep;
            const x = Math.cos(angle) * radius;
            const y = Math.sin(angle) * radius;
            
            const menuItem = this.createMenuItem(item, x, y, index);
            this.items.push(menuItem);
            this.group.add(menuItem.mesh);
        });
        
        // 중앙 원
        this.createCenterCircle();
        
        this.group.visible = false;
    }
    
    createMenuItem(item, x, y, index) {
        const geometry = new THREE.CircleGeometry(0.04, 16);
        const material = new THREE.MeshPhongMaterial({
            color: 0x2196F3,
            transparent: true,
            opacity: 0.8
        });
        
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.set(x, y, 0);
        
        // 아이콘 텍스트 추가
        this.addIconText(mesh, item.icon, 0.03);
        
        mesh.userData = {
            index: index,
            action: item.action,
            interactive: true
        };
        
        return {
            mesh: mesh,
            item: item,
            originalColor: 0x2196F3,
            selectedColor: 0x4CAF50
        };
    }
    
    createCenterCircle() {
        const geometry = new THREE.CircleGeometry(0.02, 16);
        const material = new THREE.MeshPhongMaterial({ color: 0xFFFFFF });
        
        this.centerCircle = new THREE.Mesh(geometry, material);
        this.centerCircle.position.z = 0.01;
        this.group.add(this.centerCircle);
    }
    
    addIconText(parent, icon, size) {
        const canvas = document.createElement('canvas');
        const context = canvas.getContext('2d');
        
        canvas.width = 128;
        canvas.height = 128;
        
        context.fillStyle = '#FFFFFF';
        context.font = `${size * 1000}px Arial`;
        context.textAlign = 'center';
        context.textBaseline = 'middle';
        context.fillText(icon, 64, 64);
        
        const texture = new THREE.CanvasTexture(canvas);
        const material = new THREE.MeshBasicMaterial({
            map: texture,
            transparent: true
        });
        
        const geometry = new THREE.PlaneGeometry(size * 2, size * 2);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.position.z = 0.01;
        
        parent.add(mesh);
    }
    
    show(position) {
        this.group.position.copy(position);
        this.group.visible = true;
        this.isVisible = true;
        
        // 페이드 인 애니메이션
        this.group.scale.setScalar(0.1);
        
        const animate = () => {
            this.group.scale.setScalar(
                Math.min(1.0, this.group.scale.x + 0.1)
            );
            
            if (this.group.scale.x < 1.0) {
                requestAnimationFrame(animate);
            }
        };
        
        animate();
    }
    
    hide() {
        this.isVisible = false;
        
        const animate = () => {
            this.group.scale.setScalar(
                Math.max(0.0, this.group.scale.x - 0.1)
            );
            
            if (this.group.scale.x > 0.0) {
                requestAnimationFrame(animate);
            } else {
                this.group.visible = false;
            }
        };
        
        animate();
    }
    
    selectItem(handPosition) {
        if (!this.isVisible) return;
        
        const localPos = this.group.worldToLocal(handPosition.clone());
        const distance = Math.sqrt(localPos.x * localPos.x + localPos.y * localPos.y);
        
        if (distance < 0.02) {
            // 중앙 선택 - 메뉴 취소
            this.selectedIndex = -1;
            return null;
        }
        
        // 각도 계산
        const angle = Math.atan2(localPos.y, localPos.x);
        const normalizedAngle = angle < 0 ? angle + Math.PI * 2 : angle;
        
        const itemIndex = Math.floor(normalizedAngle / (Math.PI * 2 / this.items.length));
        
        // 이전 선택 해제
        if (this.selectedIndex !== -1 && this.selectedIndex !== itemIndex) {
            this.items[this.selectedIndex].mesh.material.color.setHex(
                this.items[this.selectedIndex].originalColor
            );
        }
        
        // 새 선택
        if (itemIndex !== this.selectedIndex && itemIndex < this.items.length) {
            this.selectedIndex = itemIndex;
            this.items[itemIndex].mesh.material.color.setHex(
                this.items[itemIndex].selectedColor
            );
            
            return this.items[itemIndex].item.action;
        }
        
        return null;
    }
}
```

### 3.2 공간 인터랙션

**3D 로봇 모델 조작**
```javascript
class RobotModelController {
    constructor(robotModel) {
        this.robotModel = robotModel;
        this.joints = [];
        this.selectedJoint = null;
        
        this.setupJointInteraction();
        this.createVisualizationHelpers();
    }
    
    setupJointInteraction() {
        this.robotModel.traverse((child) => {
            if (child.userData.joint) {
                this.setupJoint(child);
            }
        });
    }
    
    setupJoint(jointMesh) {
        // 조인트 표시를 위한 기즈모 생성
        const gizmo = this.createJointGizmo();
        jointMesh.add(gizmo);
        
        jointMesh.userData.interactive = true;
        jointMesh.userData.onSelect = () => {
            this.selectJoint(jointMesh);
        };
        
        this.joints.push({
            mesh: jointMesh,
            gizmo: gizmo,
            originalPosition: jointMesh.position.clone(),
            currentAngle: 0,
            limits: jointMesh.userData.limits || { min: -Math.PI, max: Math.PI }
        });
    }
    
    createJointGizmo() {
        const group = new THREE.Group();
        
        // 회전 링
        const ringGeometry = new THREE.TorusGeometry(0.05, 0.005, 8, 16);
        const ringMaterial = new THREE.MeshBasicMaterial({ 
            color: 0xFFFF00,
            transparent: true,
            opacity: 0.7
        });
        
        const ring = new THREE.Mesh(ringGeometry, ringMaterial);
        group.add(ring);
        
        // 각도 표시
        const arrowGeometry = new THREE.ConeGeometry(0.01, 0.03, 8);
        const arrowMaterial = new THREE.MeshBasicMaterial({ color: 0xFF0000 });
        const arrow = new THREE.Mesh(arrowGeometry, arrowMaterial);
        arrow.position.set(0.05, 0, 0);
        arrow.rotation.z = -Math.PI / 2;
        
        group.add(arrow);
        group.visible = false;
        
        return group;
    }
    
    selectJoint(jointMesh) {
        // 이전 선택 해제
        if (this.selectedJoint) {
            this.selectedJoint.gizmo.visible = false;
        }
        
        // 새 조인트 선택
        this.selectedJoint = this.joints.find(j => j.mesh === jointMesh);
        if (this.selectedJoint) {
            this.selectedJoint.gizmo.visible = true;
        }
    }
    
    updateJointAngle(handPosition, handRotation) {
        if (!this.selectedJoint) return;
        
        const joint = this.selectedJoint;
        
        // 손의 회전을 조인트 각도로 변환
        const euler = new THREE.Euler().setFromQuaternion(handRotation);
        let targetAngle = euler.y; // Y축 회전 사용
        
        // 각도 제한 적용
        targetAngle = Math.max(joint.limits.min, 
                      Math.min(joint.limits.max, targetAngle));
        
        // 부드러운 보간
        const lerpFactor = 0.1;
        joint.currentAngle = THREE.MathUtils.lerp(
            joint.currentAngle, 
            targetAngle, 
            lerpFactor
        );
        
        // 조인트 회전 적용
        joint.mesh.rotation.y = joint.currentAngle;
        
        // 실제 로봇에 명령 전송
        this.sendJointCommand(joint.mesh.userData.jointIndex, joint.currentAngle);
        
        // 기즈모 업데이트
        joint.gizmo.children[1].rotation.z = -Math.PI / 2 + joint.currentAngle;
    }
    
    sendJointCommand(jointIndex, angle) {
        if (this.robotController) {
            this.robotController.sendCommand({
                type: 'joint_angle',
                joint: jointIndex,
                angle: angle,
                timestamp: performance.now()
            });
        }
    }
    
    createVisualizationHelpers() {
        // 작업 공간 표시
        this.createWorkspaceVisualization();
        
        // 엔드이펙터 궤적 표시
        this.createTrajectoryVisualization();
    }
    
    createWorkspaceVisualization() {
        const geometry = new THREE.SphereGeometry(0.8, 16, 16);
        const material = new THREE.MeshBasicMaterial({
            color: 0x00FFFF,
            transparent: true,
            opacity: 0.1,
            wireframe: true
        });
        
        this.workspaceSphere = new THREE.Mesh(geometry, material);
        this.robotModel.add(this.workspaceSphere);
    }
    
    createTrajectoryVisualization() {
        const points = [];
        this.trajectoryGeometry = new THREE.BufferGeometry().setFromPoints(points);
        
        const material = new THREE.LineBasicMaterial({ 
            color: 0xFF00FF,
            linewidth: 3
        });
        
        this.trajectoryLine = new THREE.Line(this.trajectoryGeometry, material);
        this.robotModel.add(this.trajectoryLine);
        
        this.trajectoryPoints = [];
        this.maxTrajectoryPoints = 100;
    }
    
    updateTrajectory(endEffectorPosition) {
        this.trajectoryPoints.push(endEffectorPosition.clone());
        
        if (this.trajectoryPoints.length > this.maxTrajectoryPoints) {
            this.trajectoryPoints.shift();
        }
        
        this.trajectoryGeometry.setFromPoints(this.trajectoryPoints);
        this.trajectoryGeometry.attributes.position.needsUpdate = true;
    }
}
```

---

## 4. 멀티모달 인터페이스

### 4.1 음성 명령 통합

**음성 인식 시스템**
```javascript
class VoiceCommandSystem {
    constructor() {
        this.recognition = new (window.SpeechRecognition || window.webkitSpeechRecognition)();
        this.isListening = false;
        this.commands = new Map();
        
        this.setupRecognition();
        this.registerCommands();
    }
    
    setupRecognition() {
        this.recognition.continuous = true;
        this.recognition.interimResults = true;
        this.recognition.lang = 'ko-KR';
        
        this.recognition.onresult = (event) => {
            this.handleSpeechResult(event);
        };
        
        this.recognition.onerror = (event) => {
            console.error('Speech recognition error:', event.error);
        };
        
        this.recognition.onend = () => {
            if (this.isListening) {
                this.recognition.start(); // 자동 재시작
            }
        };
    }
    
    registerCommands() {
        // 기본 명령어 등록
        this.addCommand(['정지', '스톱', '멈춰'], () => {
            this.robotController.emergencyStop();
            this.showFeedback('긴급 정지');
        });
        
        this.addCommand(['홈 포지션', '원점 복귀', '홈'], () => {
            this.robotController.goHome();
            this.showFeedback('홈 포지션으로 이동');
        });
        
        this.addCommand(['그립 열어', '손 열어', '그리퍼 오픈'], () => {
            this.robotController.openGripper();
            this.showFeedback('그리퍼 열기');
        });
        
        this.addCommand(['그립 닫아', '손 닫아', '그리퍼 클로즈'], () => {
            this.robotController.closeGripper();
            this.showFeedback('그리퍼 닫기');
        });
        
        this.addCommand(['속도 느리게', '슬로우'], () => {
            this.robotController.setSpeed(0.3);
            this.showFeedback('속도 30%로 설정');
        });
        
        this.addCommand(['속도 빠르게', '패스트'], () => {
            this.robotController.setSpeed(0.8);
            this.showFeedback('속도 80%로 설정');
        });
    }
    
    addCommand(phrases, callback) {
        phrases.forEach(phrase => {
            this.commands.set(phrase.toLowerCase(), callback);
        });
    }
    
    handleSpeechResult(event) {
        const results = event.results;
        const lastResult = results[results.length - 1];
        
        if (lastResult.isFinal) {
            const transcript = lastResult[0].transcript.toLowerCase().trim();
            console.log('Speech:', transcript);
            
            // 명령어 매칭
            for (const [phrase, callback] of this.commands) {
                if (transcript.includes(phrase)) {
                    callback();
                    break;
                }
            }
        }
    }
    
    startListening() {
        if (!this.isListening) {
            this.isListening = true;
            this.recognition.start();
            console.log('Voice recognition started');
        }
    }
    
    stopListening() {
        this.isListening = false;
        this.recognition.stop();
        console.log('Voice recognition stopped');
    }
    
    showFeedback(message) {
        // VR 환경에 피드백 표시
        const feedback = new VRFeedback(message);
        feedback.show();
    }
}
```

### 4.2 햅틱 피드백

**힘 피드백 시스템**
```javascript
class HapticFeedbackSystem {
    constructor(controllers) {
        this.controllers = controllers;
        this.feedbackQueue = [];
        this.isActive = true;
        
        this.setupFeedbackTypes();
        this.startFeedbackLoop();
    }
    
    setupFeedbackTypes() {
        this.feedbackTypes = {
            COLLISION: {
                intensity: 1.0,
                duration: 500,
                pattern: [200, 100, 200, 100, 200]
            },
            
            CONTACT: {
                intensity: 0.6,
                duration: 200,
                pattern: [200]
            },
            
            BUTTON_PRESS: {
                intensity: 0.4,
                duration: 100,
                pattern: [100]
            },
            
            GRIP_FEEDBACK: {
                intensity: 0.8,
                duration: 300,
                pattern: [300]
            },
            
            FORCE_LIMIT: {
                intensity: 0.9,
                duration: 1000,
                pattern: [100, 50, 100, 50, 100, 50]
            }
        };
    }
    
    triggerFeedback(controllerIndex, type, customParams = {}) {
        if (!this.isActive) return;
        
        const controller = this.controllers[controllerIndex];
        if (!controller || !controller.hapticActuators || 
            !controller.hapticActuators[0]) {
            return;
        }
        
        const feedback = {
            controller: controller,
            type: type,
            params: { ...this.feedbackTypes[type], ...customParams },
            startTime: performance.now()
        };
        
        this.feedbackQueue.push(feedback);
    }
    
    startFeedbackLoop() {
        const processFeedback = () => {
            const now = performance.now();
            
            this.feedbackQueue = this.feedbackQueue.filter(feedback => {
                const elapsed = now - feedback.startTime;
                
                if (elapsed < feedback.params.duration) {
                    this.executeFeedback(feedback, elapsed);
                    return true; // 계속 유지
                } else {
                    return false; // 제거
                }
            });
            
            requestAnimationFrame(processFeedback);
        };
        
        processFeedback();
    }
    
    executeFeedback(feedback, elapsed) {
        const actuator = feedback.controller.hapticActuators[0];
        const pattern = feedback.params.pattern;
        
        if (pattern.length === 1) {
            // 단순 진동
            actuator.pulse(feedback.params.intensity, pattern[0]);
        } else {
            // 패턴 진동
            const patternIndex = Math.floor(elapsed / 150) % pattern.length;
            const isOn = patternIndex % 2 === 0;
            
            if (isOn) {
                actuator.pulse(feedback.params.intensity, 150);
            }
        }
    }
    
    // 로봇 센서 데이터 기반 피드백
    updateFromSensorData(sensorData) {
        if (sensorData.forceData) {
            this.processForceData(sensorData.forceData);
        }
        
        if (sensorData.collision) {
            this.triggerFeedback(0, 'COLLISION');
            this.triggerFeedback(1, 'COLLISION');
        }
        
        if (sensorData.gripperContact) {
            this.triggerFeedback(1, 'CONTACT'); // 오른손에만
        }
    }
    
    processForceData(forceData) {
        const maxForce = 50; // N
        const forceRatio = Math.min(forceData.magnitude / maxForce, 1.0);
        
        if (forceRatio > 0.8) {
            // 힘 제한 경고
            this.triggerFeedback(0, 'FORCE_LIMIT');
            this.triggerFeedback(1, 'FORCE_LIMIT');
        } else if (forceRatio > 0.3) {
            // 접촉 피드백
            this.triggerFeedback(1, 'CONTACT', {
                intensity: forceRatio * 0.8
            });
        }
    }
    
    setActive(active) {
        this.isActive = active;
        
        if (!active) {
            // 모든 피드백 정지
            this.feedbackQueue = [];
        }
    }
}
```

---

## 5. 사용자 설정 및 커스터마이제이션

### 5.1 설정 패널

**VR 설정 인터페이스**
```javascript
class VRSettingsPanel {
    constructor() {
        this.group = new THREE.Group();
        this.settings = this.loadSettings();
        
        this.createSettingsUI();
        this.group.visible = false;
    }
    
    createSettingsUI() {
        // 배경 패널
        const panelGeometry = new THREE.PlaneGeometry(1.0, 1.2);
        const panelMaterial = new THREE.MeshPhongMaterial({
            color: 0x263238,
            transparent: true,
            opacity: 0.95
        });
        
        const panel = new THREE.Mesh(panelGeometry, panelMaterial);
        this.group.add(panel);
        
        // 설정 카테고리들
        this.createControlSettings();
        this.createDisplaySettings();
        this.createHapticSettings();
        this.createSafetySettings();
    }
    
    createControlSettings() {
        const y = 0.4;
        
        // 제어 감도
        this.sensitivitySlider = new VR3DSlider(
            new THREE.Vector3(-0.2, y, 0.01), 0.3
        );
        this.sensitivitySlider.setValue(this.settings.controlSensitivity);
        this.sensitivitySlider.onValueChange = (value) => {
            this.settings.controlSensitivity = value;
            this.updateSetting('controlSensitivity', value);
        };
        this.group.add(this.sensitivitySlider.group);
        
        // 제어 모드 버튼들
        const modes = ['직접', '상대', '제스처'];
        modes.forEach((mode, index) => {
            const button = new VR3DButton(
                mode,
                new THREE.Vector3(-0.3 + index * 0.3, y - 0.15, 0.01),
                0.1
            );
            
            button.onClick(() => {
                this.setControlMode(index);
            });
            
            this.group.add(button.mesh);
        });
    }
    
    createDisplaySettings() {
        const y = 0.1;
        
        // UI 크기 조절
        this.uiScaleSlider = new VR3DSlider(
            new THREE.Vector3(-0.2, y, 0.01), 0.3
        );
        this.uiScaleSlider.setValue(this.settings.uiScale);
        this.uiScaleSlider.onValueChange = (value) => {
            this.settings.uiScale = value;
            this.updateUIScale(value);
        };
        this.group.add(this.uiScaleSlider.group);
        
        // 정보 표시 토글
        const infoToggle = new VR3DButton(
            this.settings.showInfo ? 'ON' : 'OFF',
            new THREE.Vector3(0.2, y, 0.01),
            0.1
        );
        
        infoToggle.onClick(() => {
            this.settings.showInfo = !this.settings.showInfo;
            this.updateInfoDisplay();
        });
        
        this.group.add(infoToggle.mesh);
    }
    
    createHapticSettings() {
        const y = -0.2;
        
        // 햅틱 강도
        this.hapticIntensitySlider = new VR3DSlider(
            new THREE.Vector3(-0.2, y, 0.01), 0.3
        );
        this.hapticIntensitySlider.setValue(this.settings.hapticIntensity);
        this.hapticIntensitySlider.onValueChange = (value) => {
            this.settings.hapticIntensity = value;
            this.updateHapticIntensity(value);
        };
        this.group.add(this.hapticIntensitySlider.group);
    }
    
    createSafetySettings() {
        const y = -0.5;
        
        // 안전 영역 설정
        this.safetyZoneSlider = new VR3DSlider(
            new THREE.Vector3(-0.2, y, 0.01), 0.3
        );
        this.safetyZoneSlider.setValue(this.settings.safetyZoneSize);
        this.safetyZoneSlider.onValueChange = (value) => {
            this.settings.safetyZoneSize = value;
            this.updateSafetyZone(value);
        };
        this.group.add(this.safetyZoneSlider.group);
    }
    
    loadSettings() {
        const defaultSettings = {
            controlSensitivity: 0.5,
            controlMode: 0, // 0: 직접, 1: 상대, 2: 제스처
            uiScale: 1.0,
            showInfo: true,
            hapticIntensity: 0.7,
            safetyZoneSize: 0.8
        };
        
        const saved = localStorage.getItem('vrRobotSettings');
        return saved ? { ...defaultSettings, ...JSON.parse(saved) } : defaultSettings;
    }
    
    saveSettings() {
        localStorage.setItem('vrRobotSettings', JSON.stringify(this.settings));
    }
    
    updateSetting(key, value) {
        this.settings[key] = value;
        this.saveSettings();
        
        // 실시간 적용
        this.applySettings();
    }
    
    applySettings() {
        // 각 설정을 시스템에 적용
        if (this.controlSystem) {
            this.controlSystem.setSensitivity(this.settings.controlSensitivity);
            this.controlSystem.setMode(this.settings.controlMode);
        }
        
        if (this.hapticSystem) {
            this.hapticSystem.setIntensity(this.settings.hapticIntensity);
        }
        
        if (this.safetySystem) {
            this.safetySystem.setZoneSize(this.settings.safetyZoneSize);
        }
    }
    
    show() {
        this.group.visible = true;
        this.applySettings();
    }
    
    hide() {
        this.group.visible = false;
        this.saveSettings();
    }
}
```

---

## 6. 참고 자료

- [WebXR Design Guidelines](https://immersive-web.github.io/webxr-design/)
- [VR Interface Design Best Practices](https://developer.oculus.com/design/)
- [Three.js VR Examples](https://threejs.org/examples/?q=webxr)
- [Hand Tracking API](https://www.w3.org/TR/webxr-hand-input-1/)

---

[← 7.3 WebRTC 통신](03_webrtc.md) | [다음: 7.5 데이터셋 기록 →](05_dataset_recording.md)
