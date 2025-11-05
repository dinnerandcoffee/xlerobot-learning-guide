# 5.5 핸드 카메라 활용

로봇 팔 끝(엔드 이펙터)에 부착된 RGBD 카메라를 활용하여 정밀한 물체 조작을 구현합니다.

## 1. 핸드 카메라 개요

### 1.1 Eye-in-Hand 장점

```
✓ 동적 시야: 로봇이 이동하면서 다양한 각도에서 관찰
✓ 폐쇄 루프: 실시간으로 그리퍼와 물체 간 거리 확인
✓ 정밀 조작: 가까운 거리에서 세밀한 작업 가능
✗ 캘리브레이션 필요: 카메라-그리퍼 변환 행렬 필요
```

### 1.2 XLeRobot 하드웨어

**RGBD Gimbal 구성**
- Intel RealSense D435 또는 OAK-D Lite
- 2축 짐벌 (Pan/Tilt)
- SO100 그리퍼 상단 마운트

---

## 2. Intel RealSense 설정

### 2.1 설치

```bash
# librealsense SDK
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils

# Python wrapper
pip install pyrealsense2

# 테스트
realsense-viewer
```

### 2.2 기본 사용법

```python
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseRGBD:
    """RealSense RGBD 카메라 래퍼"""
    
    def __init__(self, width=640, height=480, fps=30):
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # RGB 스트림
        self.config.enable_stream(
            rs.stream.color, width, height, rs.format.bgr8, fps
        )
        
        # Depth 스트림
        self.config.enable_stream(
            rs.stream.depth, width, height, rs.format.z16, fps
        )
        
        # 시작
        self.profile = self.pipeline.start(self.config)
        
        # 정렬 객체 (depth → color)
        self.align = rs.align(rs.stream.color)
    
    def get_frames(self):
        """RGB + Depth 프레임"""
        frames = self.pipeline.wait_for_frames()
        
        # Depth를 Color에 정렬
        aligned_frames = self.align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return None, None
        
        # NumPy 배열
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        return color_image, depth_image, depth_frame
    
    def get_intrinsics(self):
        """카메라 내부 파라미터"""
        color_stream = self.profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
        return intrinsics
    
    def cleanup(self):
        self.pipeline.stop()

# 사용
camera = RealSenseRGBD()

try:
    while True:
        color, depth, depth_frame = camera.get_frames()
        
        if color is None:
            continue
        
        # Depth 시각화 (0~10m → 0~255)
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth, alpha=0.03),
            cv2.COLORMAP_JET
        )
        
        # 나란히 표시
        images = np.hstack((color, depth_colormap))
        cv2.imshow('RealSense', images)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    camera.cleanup()
    cv2.destroyAllWindows()
```

---

## 3. YOLO + Depth 통합

### 3.1 3D 위치 추출

```python
from ultralytics import YOLO

class YOLODepth:
    """YOLO + Depth 통합"""
    
    def __init__(self, yolo_model_path='yolov8n.pt'):
        self.yolo = YOLO(yolo_model_path)
        self.camera = RealSenseRGBD()
        self.intrinsics = self.camera.get_intrinsics()
    
    def get_3d_position(self, px, py, depth_frame):
        """
        2D 픽셀 + Depth → 3D 좌표 (카메라 기준)
        
        Args:
            px, py: 픽셀 좌표
            depth_frame: RealSense depth frame
        
        Returns:
            (x, y, z): 3D 좌표 (m)
        """
        depth = depth_frame.get_distance(int(px), int(py))
        
        if depth == 0:
            return None
        
        # 픽셀 → 3D (RealSense 함수 사용)
        point_3d = rs.rs2_deproject_pixel_to_point(
            self.intrinsics, [px, py], depth
        )
        
        return point_3d  # [x, y, z] in meters
    
    def detect_objects_3d(self, target_classes=None):
        """
        객체 감지 + 3D 위치
        
        Returns:
            list of (class_name, bbox, position_3d)
        """
        color, depth, depth_frame = self.camera.get_frames()
        
        if color is None:
            return []
        
        # YOLO 감지
        results = self.yolo(color, classes=target_classes)
        
        detections = []
        
        for box in results[0].boxes:
            class_id = int(box.cls[0])
            class_name = self.yolo.names[class_id]
            
            # 바운딩 박스 중심
            cx, cy = box.xywh[0][:2].tolist()
            
            # 3D 위치
            pos_3d = self.get_3d_position(cx, cy, depth_frame)
            
            if pos_3d is not None:
                detections.append({
                    'class': class_name,
                    'bbox': box.xyxy[0].tolist(),
                    'position_3d': pos_3d,
                    'confidence': float(box.conf[0])
                })
        
        return detections, color
    
    def cleanup(self):
        self.camera.cleanup()

# 사용
yolo_depth = YOLODepth()

while True:
    detections, color = yolo_depth.detect_objects_3d([39, 41])  # bottle, cup
    
    # 결과 표시
    for det in detections:
        x, y, z = det['position_3d']
        class_name = det['class']
        
        print(f"{class_name}: ({x:.3f}, {y:.3f}, {z:.3f}) m")
        
        # 이미지에 3D 좌표 표시
        x1, y1, x2, y2 = [int(v) for v in det['bbox']]
        cv2.rectangle(color, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            color,
            f"{class_name} ({z:.2f}m)",
            (x1, y1 - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2
        )
    
    cv2.imshow('YOLO + Depth', color)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

yolo_depth.cleanup()
```

---

## 4. 로봇 제어 통합

### 4.1 카메라 → 로봇 좌표 변환

```python
import numpy as np

class HandEyeTransform:
    """핸드 카메라 → 로봇 베이스 좌표 변환"""
    
    def __init__(self):
        # 카메라 → 그리퍼 변환 (캘리브레이션 필요)
        # 예시: 카메라가 그리퍼 위 5cm, 앞쪽 3cm
        self.cam_to_gripper = np.array([
            [1, 0, 0, 0.03],   # x: 앞쪽 3cm
            [0, 1, 0, 0.00],   # y: 0cm
            [0, 0, 1, 0.05],   # z: 위쪽 5cm
            [0, 0, 0, 1]
        ])
    
    def camera_to_robot(self, point_cam, gripper_pose):
        """
        카메라 좌표 → 로봇 베이스 좌표
        
        Args:
            point_cam: 카메라 좌표 (x, y, z)
            gripper_pose: 그리퍼 위치/방향 (x, y, z, rx, ry, rz)
        
        Returns:
            로봇 베이스 좌표 (x, y, z)
        """
        # 카메라 좌표를 homogeneous 형태로
        point_cam_h = np.array([*point_cam, 1])
        
        # 카메라 → 그리퍼
        point_gripper_h = self.cam_to_gripper @ point_cam_h
        
        # 그리퍼 → 로봇 베이스 (간단히 더하기)
        # 실제로는 회전 행렬도 고려해야 함
        point_robot = point_gripper_h[:3] + gripper_pose[:3]
        
        return point_robot.tolist()

# 통합 제어 클래스
class VisionGuidedGrasp:
    """비전 가이드 그립"""
    
    def __init__(self, robot, yolo_depth):
        self.robot = robot
        self.vision = yolo_depth
        self.transform = HandEyeTransform()
    
    def find_and_grasp(self, target_class='cup'):
        """객체 찾아서 집기"""
        print(f"Searching for {target_class}...")
        
        while True:
            # 객체 감지
            detections, color = self.vision.detect_objects_3d()
            
            # 타겟 찾기
            target = None
            for det in detections:
                if det['class'] == target_class:
                    target = det
                    break
            
            if target is None:
                cv2.imshow('Search', color)
                if cv2.waitKey(100) & 0xFF == ord('q'):
                    return False
                continue
            
            # 3D 위치 (카메라 기준)
            pos_cam = target['position_3d']
            
            # 그리퍼 현재 위치
            gripper_pose = self.robot.get_ee_position()
            
            # 로봇 좌표로 변환
            pos_robot = self.transform.camera_to_robot(pos_cam, gripper_pose)
            
            print(f"Target at camera: {pos_cam}")
            print(f"Target at robot: {pos_robot}")
            
            # 접근
            approach_pos = pos_robot.copy()
            approach_pos[2] += 0.05  # 5cm 위에서 접근
            
            self.robot.set_ee_position(approach_pos)
            time.sleep(1.0)
            
            # 하강
            self.robot.set_ee_position(pos_robot)
            time.sleep(1.0)
            
            # 집기
            self.robot.close_gripper()
            time.sleep(0.5)
            
            # 들어올리기
            self.robot.set_ee_position(approach_pos)
            
            print(f"Grasped {target_class}!")
            return True
    
    def cleanup(self):
        self.vision.cleanup()

# 사용
robot = SO100Arm(port="/dev/ttyUSB0")
yolo_depth = YOLODepth()
vision_grasp = VisionGuidedGrasp(robot, yolo_depth)

try:
    vision_grasp.find_and_grasp('cup')
    vision_grasp.find_and_grasp('bottle')
finally:
    vision_grasp.cleanup()
    robot.cleanup()
```

---

## 5. 짐벌 제어

### 5.1 Pan/Tilt 카메라

```python
class GimbalCamera:
    """RGBD 짐벌 카메라"""
    
    def __init__(self, pan_motor_id=7, tilt_motor_id=8):
        # Dynamixel 모터 (예시)
        from dynamixel_sdk import *
        
        self.pan_id = pan_motor_id
        self.tilt_id = tilt_motor_id
        
        # 포트 초기화 (실제 코드는 더 복잡)
        self.portHandler = PortHandler('/dev/ttyUSB1')
        self.packetHandler = PacketHandler(2.0)
        
        # RealSense
        self.camera = RealSenseRGBD()
    
    def set_pan_tilt(self, pan_deg, tilt_deg):
        """
        Pan/Tilt 각도 설정
        
        Args:
            pan_deg: Pan 각도 (-90 ~ 90)
            tilt_deg: Tilt 각도 (-45 ~ 45)
        """
        # 각도 → Dynamixel 값
        pan_value = int((pan_deg + 90) / 180 * 4095)
        tilt_value = int((tilt_deg + 45) / 90 * 4095)
        
        # 모터 제어
        self.packetHandler.write4ByteTxRx(
            self.portHandler, self.pan_id, 116, pan_value
        )
        self.packetHandler.write4ByteTxRx(
            self.portHandler, self.tilt_id, 116, tilt_value
        )
    
    def track_object(self, yolo_model, target_class='person'):
        """객체 추적 (Pan/Tilt)"""
        kp = 0.05  # 비례 게인
        
        while True:
            color, depth, depth_frame = self.camera.get_frames()
            
            # 객체 감지
            results = yolo_model(color)
            
            # 타겟 찾기
            target_box = None
            for box in results[0].boxes:
                if yolo_model.names[int(box.cls[0])] == target_class:
                    target_box = box
                    break
            
            if target_box is None:
                cv2.imshow('Tracking', color)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
            
            # 이미지 중심에서 오차
            cx, cy = target_box.xywh[0][:2].tolist()
            error_x = cx - 320  # 이미지 중심 (640/2)
            error_y = cy - 240  # (480/2)
            
            # Pan/Tilt 조정
            pan_deg = -error_x * kp
            tilt_deg = error_y * kp
            
            self.set_pan_tilt(pan_deg, tilt_deg)
            
            # 시각화
            annotated = results[0].plot()
            cv2.circle(annotated, (320, 240), 5, (0, 0, 255), -1)
            cv2.imshow('Tracking', annotated)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    
    def cleanup(self):
        self.camera.cleanup()
        self.portHandler.closePort()
```

---

## 6. OAK-D 대안

### 6.1 DepthAI 설치

```bash
pip install depthai
```

### 6.2 OAK-D 사용법

```python
import depthai as dai

class OAKDCamera:
    """OAK-D RGBD 카메라"""
    
    def __init__(self):
        self.pipeline = dai.Pipeline()
        
        # RGB 카메라
        cam_rgb = self.pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(640, 480)
        
        # Depth (스테레오)
        mono_left = self.pipeline.create(dai.node.MonoCamera)
        mono_right = self.pipeline.create(dai.node.MonoCamera)
        stereo = self.pipeline.create(dai.node.StereoDepth)
        
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        
        # 출력
        xout_rgb = self.pipeline.create(dai.node.XLinkOut)
        xout_depth = self.pipeline.create(dai.node.XLinkOut)
        
        xout_rgb.setStreamName("rgb")
        xout_depth.setStreamName("depth")
        
        cam_rgb.preview.link(xout_rgb.input)
        stereo.depth.link(xout_depth.input)
        
        # 디바이스 시작
        self.device = dai.Device(self.pipeline)
        
        self.q_rgb = self.device.getOutputQueue("rgb", maxSize=4, blocking=False)
        self.q_depth = self.device.getOutputQueue("depth", maxSize=4, blocking=False)
    
    def get_frames(self):
        """RGB + Depth"""
        rgb_frame = self.q_rgb.get()
        depth_frame = self.q_depth.get()
        
        color = rgb_frame.getCvFrame()
        depth = depth_frame.getFrame()
        
        return color, depth
    
    def cleanup(self):
        self.device.close()

# 사용 (RealSense와 유사)
oak = OAKDCamera()

while True:
    color, depth = oak.get_frames()
    
    # Depth 시각화
    depth_vis = (depth / depth.max() * 255).astype(np.uint8)
    depth_colormap = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
    
    cv2.imshow('OAK-D', np.hstack((color, depth_colormap)))
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

oak.cleanup()
```

---

## 7. 실전 예제: 정밀 조립

### 7.1 페그-인-홀 (Peg-in-Hole)

```python
class PegInHole:
    """핸드 카메라 활용 정밀 조립"""
    
    def __init__(self, robot, yolo_depth):
        self.robot = robot
        self.vision = yolo_depth
    
    def find_hole_center(self):
        """구멍 중심 찾기 (세그멘테이션)"""
        yolo_seg = YOLO('yolov8n-seg.pt')
        
        detections, color = self.vision.detect_objects_3d()
        
        # 구멍 클래스 찾기 (예: 'donut')
        for det in detections:
            if det['class'] == 'donut':
                # 세그멘테이션 마스크
                results = yolo_seg(color)
                mask = results[0].masks[0].data[0].cpu().numpy()
                
                # 중심 계산
                M = cv2.moments((mask * 255).astype(np.uint8))
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                
                # 3D 위치
                _, _, depth_frame = self.vision.camera.get_frames()
                pos_3d = self.vision.get_3d_position(cx, cy, depth_frame)
                
                return pos_3d
        
        return None
    
    def insert_peg(self):
        """페그 삽입"""
        # 1. 구멍 찾기
        hole_pos = self.find_hole_center()
        
        if hole_pos is None:
            print("Hole not found")
            return False
        
        print(f"Hole at: {hole_pos}")
        
        # 2. 위에서 접근
        approach_pos = hole_pos.copy()
        approach_pos[2] += 0.1  # 10cm 위
        
        self.robot.set_ee_position(approach_pos)
        time.sleep(1.0)
        
        # 3. 천천히 하강 (힘 센싱 권장)
        for z_offset in np.linspace(0.1, 0.0, 20):
            current = approach_pos.copy()
            current[2] = hole_pos[2] + z_offset
            self.robot.set_ee_position(current)
            time.sleep(0.1)
        
        print("Peg inserted!")
        return True
```

---

## 8. 참고 자료

- [Intel RealSense Documentation](https://dev.intelrealsense.com/docs)
- [DepthAI (OAK-D)](https://docs.luxonis.com/)
- [Hand-Eye Calibration OpenCV](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b)

---

[← 5.4 비전 기반 로봇 제어](04_vision_control.md) | [목차로 돌아가기](README.md)
