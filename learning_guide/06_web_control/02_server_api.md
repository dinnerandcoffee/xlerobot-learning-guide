# 6.2 서버 API 이해하기

FastAPI를 사용한 로봇 제어 서버 구현 방법을 학습합니다.

## 1. FastAPI 기초

### 1.1 핵심 개념

```python
from fastapi import FastAPI

app = FastAPI()

@app.get("/")           # HTTP GET
async def read_root():
    return {"Hello": "World"}

@app.post("/items")     # HTTP POST
async def create_item(item: dict):
    return item
```

**주요 특징**
- 자동 API 문서 생성 (/docs)
- 타입 힌팅 + Pydantic 검증
- async/await 지원
- 빠른 성능 (Starlette 기반)

### 1.2 Pydantic 모델

```python
from pydantic import BaseModel, Field

class RobotCommand(BaseModel):
    """로봇 명령 스키마"""
    command_type: str = Field(..., description="명령 타입")
    value: float = Field(default=0.0, ge=-1.0, le=1.0)
    duration: float = Field(default=1.0, gt=0)
    
    class Config:
        json_schema_extra = {
            "example": {
                "command_type": "move_forward",
                "value": 0.5,
                "duration": 2.0
            }
        }

@app.post("/command")
async def send_command(cmd: RobotCommand):
    # FastAPI가 자동으로 검증
    return {"received": cmd.dict()}
```

---

## 2. 로봇 제어 API

### 2.1 엔드포인트 설계

```python
# server/api/robot.py
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import sys
sys.path.append('../../src')
from robots.so100 import SO100Arm

router = APIRouter(prefix="/api/robot", tags=["robot"])

# 전역 로봇 인스턴스 (나중에 의존성 주입으로 개선)
robot = None

class JointPosition(BaseModel):
    """조인트 각도"""
    joint_id: int
    angle: float  # degrees

class EEPosition(BaseModel):
    """엔드 이펙터 위치"""
    x: float
    y: float
    z: Optional[float] = None

class GripperCommand(BaseModel):
    """그리퍼 명령"""
    action: str  # "open" or "close"

# === 연결 관리 ===

@router.post("/connect")
async def connect(port: str = "/dev/ttyUSB0"):
    """로봇 연결"""
    global robot
    try:
        robot = SO100Arm(port=port)
        return {"status": "connected", "port": port}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/disconnect")
async def disconnect():
    """로봇 연결 해제"""
    global robot
    if robot:
        robot.cleanup()
        robot = None
        return {"status": "disconnected"}
    raise HTTPException(status_code=400, detail="No active connection")

@router.get("/status")
async def get_status():
    """로봇 상태 조회"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")
    
    return {
        "connected": True,
        "joint_count": 6,
        "gripper_state": "unknown"  # 실제로는 읽기
    }

# === 조인트 제어 ===

@router.post("/joint")
async def set_joint(pos: JointPosition):
    """단일 조인트 제어"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")
    
    try:
        robot.set_joint_angle(pos.joint_id, pos.angle)
        return {"joint_id": pos.joint_id, "angle": pos.angle}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/joint/{joint_id}")
async def get_joint(joint_id: int):
    """조인트 각도 읽기"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")
    
    angle = robot.get_joint_angle(joint_id)
    return {"joint_id": joint_id, "angle": angle}

# === 엔드 이펙터 제어 ===

@router.post("/ee")
async def set_ee_position(pos: EEPosition):
    """엔드 이펙터 위치 제어"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")
    
    try:
        if pos.z is not None:
            robot.set_ee_position([pos.x, pos.y, pos.z])
        else:
            robot.set_ee_position([pos.x, pos.y])
        
        return {"x": pos.x, "y": pos.y, "z": pos.z}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/ee")
async def get_ee_position():
    """엔드 이펙터 위치 읽기"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")
    
    pos = robot.get_ee_position()
    return {"x": pos[0], "y": pos[1], "z": pos[2] if len(pos) > 2 else None}

# === 그리퍼 제어 ===

@router.post("/gripper")
async def control_gripper(cmd: GripperCommand):
    """그리퍼 제어"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")
    
    if cmd.action == "open":
        robot.open_gripper()
    elif cmd.action == "close":
        robot.close_gripper()
    else:
        raise HTTPException(status_code=400, detail="Invalid action")
    
    return {"action": cmd.action}

# === 프리셋 동작 ===

@router.post("/home")
async def go_home():
    """홈 위치로 이동"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")
    
    robot.go_home()
    return {"status": "homing"}

@router.post("/sleep")
async def go_sleep():
    """슬립 자세"""
    if not robot:
        raise HTTPException(status_code=400, detail="Robot not connected")
    
    robot.go_sleep()
    return {"status": "sleeping"}
```

### 2.2 메인 앱에 등록

```python
# server/main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from api import robot

app = FastAPI(
    title="XLeRobot Web Control API",
    version="1.0.0",
    description="로봇 원격 제어 REST API"
)

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 라우터 등록
app.include_router(robot.router)

@app.get("/")
async def root():
    return {
        "name": "XLeRobot Web Control",
        "version": "1.0.0",
        "docs": "/docs"
    }

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
```

---

## 3. WebSocket 통신

### 3.1 기본 WebSocket

```python
# server/api/websocket.py
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
from typing import List
import json
import asyncio

router = APIRouter()

class ConnectionManager:
    """WebSocket 연결 관리자"""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        """클라이언트 연결"""
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"Client connected. Total: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        """클라이언트 연결 해제"""
        self.active_connections.remove(websocket)
        print(f"Client disconnected. Total: {len(self.active_connections)}")
    
    async def send_personal(self, message: dict, websocket: WebSocket):
        """특정 클라이언트에게 전송"""
        await websocket.send_json(message)
    
    async def broadcast(self, message: dict):
        """모든 클라이언트에게 전송"""
        for connection in self.active_connections:
            await connection.send_json(message)

manager = ConnectionManager()

@router.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """WebSocket 엔드포인트"""
    await manager.connect(websocket)
    
    try:
        while True:
            # 클라이언트로부터 메시지 수신
            data = await websocket.receive_json()
            
            print(f"Received: {data}")
            
            # 메시지 타입에 따라 처리
            msg_type = data.get("type")
            
            if msg_type == "ping":
                # Pong 응답
                await manager.send_personal({"type": "pong"}, websocket)
            
            elif msg_type == "joystick":
                # 조이스틱 입력 처리
                x = data.get("x", 0)
                y = data.get("y", 0)
                
                # 로봇 제어 (예시)
                # robot.move(x, y)
                
                # 응답
                await manager.send_personal({
                    "type": "joystick_ack",
                    "x": x,
                    "y": y
                }, websocket)
            
            elif msg_type == "command":
                # 명령 처리
                command = data.get("command")
                # 실행...
                
                await manager.send_personal({
                    "type": "command_result",
                    "success": True
                }, websocket)
            
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception as e:
        print(f"WebSocket error: {e}")
        manager.disconnect(websocket)
```

**main.py에 등록**
```python
from api import websocket

app.include_router(websocket.router)
```

### 3.2 실시간 로봇 상태 브로드캐스트

```python
# server/core/robot_monitor.py
import asyncio
from api.websocket import manager

async def robot_status_loop(robot):
    """로봇 상태 주기적 전송"""
    while True:
        if robot and manager.active_connections:
            # 로봇 상태 읽기
            ee_pos = robot.get_ee_position()
            
            # 모든 클라이언트에게 전송
            await manager.broadcast({
                "type": "robot_status",
                "ee_position": {
                    "x": ee_pos[0],
                    "y": ee_pos[1],
                    "z": ee_pos[2] if len(ee_pos) > 2 else None
                },
                "timestamp": asyncio.get_event_loop().time()
            })
        
        await asyncio.sleep(0.1)  # 10Hz

# main.py에서 백그라운드 태스크로 실행
@app.on_event("startup")
async def startup_event():
    # 백그라운드 태스크 시작
    asyncio.create_task(robot_status_loop(robot))
```

---

## 4. 비디오 스트리밍

### 4.1 MJPEG 스트리밍

```python
# server/api/camera.py
from fastapi import APIRouter
from fastapi.responses import StreamingResponse
import cv2
import asyncio

router = APIRouter(prefix="/api/camera", tags=["camera"])

class VideoCamera:
    """비디오 카메라"""
    
    def __init__(self, camera_id=0):
        self.cap = cv2.VideoCapture(camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    def get_frame(self):
        """프레임 읽기"""
        ret, frame = self.cap.read()
        if not ret:
            return None
        
        # JPEG 인코딩
        ret, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ret:
            return None
        
        return jpeg.tobytes()
    
    def cleanup(self):
        """정리"""
        self.cap.release()

# 전역 카메라
camera = VideoCamera()

def generate_frames():
    """프레임 생성기 (MJPEG)"""
    while True:
        frame = camera.get_frame()
        
        if frame is None:
            continue
        
        # MJPEG 형식
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@router.get("/stream")
async def video_stream():
    """비디오 스트림"""
    return StreamingResponse(
        generate_frames(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )

@router.get("/snapshot")
async def snapshot():
    """스냅샷"""
    frame = camera.get_frame()
    
    if frame is None:
        return {"error": "Failed to capture"}
    
    return StreamingResponse(
        iter([frame]),
        media_type="image/jpeg"
    )
```

**main.py에 등록**
```python
from api import camera

app.include_router(camera.router)
```

**클라이언트 (HTML)**
```html
<!-- 비디오 피드 -->
<img src="http://localhost:8000/api/camera/stream" alt="Video Feed">
```

---

## 5. 의존성 주입

### 5.1 로봇 의존성

```python
# server/core/dependencies.py
from fastapi import Depends, HTTPException
from typing import Annotated

class RobotConnection:
    """로봇 연결 싱글톤"""
    
    _instance = None
    _robot = None
    
    @classmethod
    def get_instance(cls):
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance
    
    def connect(self, port="/dev/ttyUSB0"):
        """연결"""
        if self._robot is None:
            from robots.so100 import SO100Arm
            self._robot = SO100Arm(port=port)
    
    def get_robot(self):
        """로봇 인스턴스 반환"""
        if self._robot is None:
            raise HTTPException(status_code=503, detail="Robot not connected")
        return self._robot
    
    def disconnect(self):
        """연결 해제"""
        if self._robot:
            self._robot.cleanup()
            self._robot = None

def get_robot_connection():
    """의존성 함수"""
    return RobotConnection.get_instance()

def get_robot(
    conn: Annotated[RobotConnection, Depends(get_robot_connection)]
):
    """로봇 인스턴스 의존성"""
    return conn.get_robot()

# 사용
from core.dependencies import get_robot
from robots.so100 import SO100Arm

@router.post("/ee")
async def set_ee(
    pos: EEPosition,
    robot: Annotated[SO100Arm, Depends(get_robot)]
):
    """의존성 주입 사용"""
    robot.set_ee_position([pos.x, pos.y])
    return {"status": "ok"}
```

---

## 6. 에러 처리

### 6.1 커스텀 예외

```python
# server/core/exceptions.py
from fastapi import HTTPException

class RobotNotConnectedException(HTTPException):
    def __init__(self):
        super().__init__(
            status_code=503,
            detail="Robot is not connected. Please connect first."
        )

class InvalidCommandException(HTTPException):
    def __init__(self, message: str):
        super().__init__(
            status_code=400,
            detail=f"Invalid command: {message}"
        )

class RobotTimeoutException(HTTPException):
    def __init__(self):
        super().__init__(
            status_code=504,
            detail="Robot command timeout"
        )
```

### 6.2 전역 예외 핸들러

```python
# main.py
from fastapi import Request
from fastapi.responses import JSONResponse

@app.exception_handler(Exception)
async def global_exception_handler(request: Request, exc: Exception):
    """전역 예외 처리"""
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal Server Error",
            "message": str(exc),
            "path": str(request.url)
        }
    )
```

---

## 7. 로깅

### 7.1 구조화된 로깅

```python
# server/core/logging_config.py
import logging
import sys

def setup_logging():
    """로깅 설정"""
    
    # 포맷
    formatter = logging.Formatter(
        '[%(asctime)s] %(levelname)s [%(name)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # 콘솔 핸들러
    console_handler = logging.StreamHandler(sys.stdout)
    console_handler.setFormatter(formatter)
    
    # 파일 핸들러
    file_handler = logging.FileHandler('robot_server.log')
    file_handler.setFormatter(formatter)
    
    # 루트 로거
    logger = logging.getLogger()
    logger.setLevel(logging.INFO)
    logger.addHandler(console_handler)
    logger.addHandler(file_handler)
    
    return logger

# main.py
from core.logging_config import setup_logging

logger = setup_logging()

@app.on_event("startup")
async def startup():
    logger.info("Server starting...")

@app.on_event("shutdown")
async def shutdown():
    logger.info("Server shutting down...")
```

### 7.2 요청 로깅 미들웨어

```python
from fastapi import Request
import time

@app.middleware("http")
async def log_requests(request: Request, call_next):
    """요청 로깅"""
    start_time = time.time()
    
    response = await call_next(request)
    
    process_time = time.time() - start_time
    
    logger.info(
        f"{request.method} {request.url.path} "
        f"completed in {process_time:.3f}s "
        f"status={response.status_code}"
    )
    
    return response
```

---

## 8. 테스트

### 8.1 pytest 설정

```python
# tests/test_robot_api.py
from fastapi.testclient import TestClient
from main import app

client = TestClient(app)

def test_root():
    """루트 엔드포인트 테스트"""
    response = client.get("/")
    assert response.status_code == 200
    assert "name" in response.json()

def test_robot_status_not_connected():
    """로봇 미연결 상태 테스트"""
    response = client.get("/api/robot/status")
    assert response.status_code in [400, 503]

def test_set_ee_position():
    """EE 위치 설정 테스트 (Mock)"""
    # 먼저 연결
    client.post("/api/robot/connect", params={"port": "/dev/null"})
    
    # EE 위치 설정
    response = client.post("/api/robot/ee", json={
        "x": 0.1,
        "y": 0.2,
        "z": 0.3
    })
    
    assert response.status_code == 200
```

---

## 9. 참고 자료

- [FastAPI Tutorial](https://fastapi.tiangolo.com/tutorial/)
- [Pydantic Models](https://docs.pydantic.dev/)
- [WebSocket in FastAPI](https://fastapi.tiangolo.com/advanced/websockets/)
- [OpenCV Video Streaming](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)

---

[← 6.1 개요](01_overview.md) | [다음: 6.3 클라이언트 구성 →](03_client_setup.md)
