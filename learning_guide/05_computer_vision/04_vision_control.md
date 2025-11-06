# 5.4 비전 기반 로봇 제어

YOLO 객체 감지 결과를 로봇 제어 명령으로 변환하는 방법을 학습합니다.

## 1. 비전-로봇 통합 개요

### 1.1 제어 파이프라인

```
카메라 → YOLO 감지 → 픽셀 좌표 → 로봇 좌표 → 모터 명령
  ↓         ↓            ↓            ↓           ↓
 RGB    바운딩박스   (px, py)    (x, y, z)   조인트 각도
```

### 1.2 좌표계 변환

```
1. 이미지 좌표 (픽셀): (0, 0) = 좌상단
2. 카메라 좌표 (m): (0, 0, 0) = 카메라 중심
3. 로봇 베이스 좌표 (m): (0, 0, 0) = 로봇 베이스
```

---

## 2. 카메라 캘리브레이션

### 2.1 픽셀 → 미터 변환

```python
class CameraCalibration:
    """카메라 캘리브레이션 정보"""
    
    def __init__(self, focal_length, sensor_size, image_size,
                 camera_height):
        """
        Args:
            focal_length: 초점 거리 (mm)
            sensor_size: 센서 크기 (mm, mm)
            image_size: 이미지 크기 (px, px)
            camera_height: 카메라 높이 (m)
        """
        self.focal_length = focal_length
        self.sensor_width, self.sensor_height = sensor_size
        self.image_width, self.image_height = image_size
        self.camera_height = camera_height
        
        # 픽셀당 실제 크기
        self.px_to_mm_x = self.sensor_width / self.image_width
        self.px_to_mm_y = self.sensor_height / self.image_height
    
    def pixel_to_world(self, px, py):
        """
        픽셀 좌표 → 월드 좌표 (바닥면 기준)
        
        Args:
            px, py: 픽셀 좌표
        
        Returns:
            x, y: 월드 좌표 (m)
        """
        # 이미지 중심에서의 오프셋 (픽셀)
        cx = px - self.image_width / 2
        cy = py - self.image_height / 2
        
        # mm 단위로 변환
        mm_x = cx * self.px_to_mm_x
        mm_y = cy * self.px_to_mm_y
        
        # 실제 거리 계산 (유사 삼각형)
        scale = self.camera_height / (self.focal_length / 1000)
        
        x = mm_x * scale / 1000  # m
        y = mm_y * scale / 1000  # m
        
        return x, y

# 사용 예시
calib = CameraCalibration(
    focal_length=3.6,           # mm (웹캠 예시)
    sensor_size=(6.17, 4.55),  # mm
    image_size=(640, 480),      # px
    camera_height=0.5           # m (바닥에서 50cm)
)

# 픽셀 (320, 240) → 월드 좌표
x, y = calib.pixel_to_world(320, 240)
print(f"World: ({x:.3f}, {y:.3f}) m")
```

### 2.2 간단한 비율 기반 변환

```python
def simple_pixel_to_robot(px, py, image_size=(640, 480),
                          workspace=(0.3, 0.3)):
    """
    간단한 비율 변환 (평면 가정)
    
    Args:
        px, py: 픽셀 좌표
        image_size: 이미지 크기 (width, height)
        workspace: 작업 공간 크기 (width_m, height_m)
    
    Returns:
        x, y: 로봇 좌표 (m)
    """
    img_w, img_h = image_size
    ws_w, ws_h = workspace
    
    # 정규화 (0~1)
    norm_x = px / img_w
    norm_y = py / img_h
    
    # 로봇 좌표 (-ws/2 ~ +ws/2)
    x = (norm_x - 0.5) * ws_w
    y = (norm_y - 0.5) * ws_h
    
    return x, y
```

---

## 3. YOLO + 로봇 제어

### 3.1 기본 구조

```python
from ultralytics import YOLO
import cv2
import sys
sys.path.append('../src')
from robots.so100 import SO100Arm

# YOLO 모델
yolo_model = YOLO('yolov8n.pt')

# 로봇
robot = SO100Arm(port="/dev/ttyUSB0")

# 카메라
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    
    # YOLO 감지
    results = yolo_model(frame, classes=[39])  # bottle
    
    if len(results[0].boxes) > 0:
        # 첫 번째 병
        box = results[0].boxes[0]
        cx, cy = box.xywh[0][:2].tolist()
        
        # 픽셀 → 로봇 좌표
        x, y = simple_pixel_to_robot(cx, cy)
        
        # 로봇 이동
        robot.set_ee_position([x, y])
        
        print(f"Bottle at pixel ({cx:.0f}, {cy:.0f}) → robot ({x:.3f}, {y:.3f})")
    
    # 화면 표시
    annotated = results[0].plot()
    cv2.imshow('Vision Control', annotated)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
robot.cleanup()
```

---

## 4. 시각 서보잉 (Visual Servoing)

### 4.1 위치 기반 서보 (PBVS)

```python
class VisualServo:
    """시각 기반 로봇 제어"""
    
    def __init__(self, robot, yolo_model, target_class='bottle'):
        self.robot = robot
        self.yolo = yolo_model
        self.target_class = target_class
        
        # 제어 게인
        self.kp_x = 0.5
        self.kp_y = 0.5
        
        # 목표 위치 (이미지 중심)
        self.target_px = 320
        self.target_py = 240
    
    def update(self, frame):
        """프레임 업데이트 → 로봇 이동"""
        # 객체 감지
        results = self.yolo(frame)
        
        # 타겟 찾기
        target_box = self._find_target(results)
        
        if target_box is None:
            return False
        
        # 현재 위치 (픽셀)
        cx, cy = target_box.xywh[0][:2].tolist()
        
        # 오차 계산
        error_x = cx - self.target_px
        error_y = cy - self.target_py
        
        # P 제어
        dx = -error_x * self.kp_x * 0.001  # m
        dy = -error_y * self.kp_y * 0.001  # m
        
        # 로봇 이동 (상대)
        current_pos = self.robot.get_ee_position()
        new_pos = [current_pos[0] + dx, current_pos[1] + dy]
        
        self.robot.set_ee_position(new_pos)
        
        print(f"Error: ({error_x:.0f}, {error_y:.0f}) px → Move: ({dx:.3f}, {dy:.3f}) m")
        
        # 수렴 확인
        if abs(error_x) < 10 and abs(error_y) < 10:
            return True  # 도달
        
        return False
    
    def _find_target(self, results):
        """타겟 클래스 찾기"""
        for box in results[0].boxes:
            class_name = self.yolo.names[int(box.cls[0])]
            if class_name == self.target_class:
                return box
        return None

# 사용
servo = VisualServo(robot, yolo_model, 'bottle')

while cap.isOpened():
    ret, frame = cap.read()
    
    # 시각 서보 업데이트
    reached = servo.update(frame)
    
    if reached:
        print("Target reached!")
        robot.close_gripper()
        break
```

### 4.2 이미지 기반 서보 (IBVS)

```python
def image_based_servo(robot, yolo, cap, target_size=100):
    """
    이미지 기반 서보: 객체 크기가 목표 크기가 될 때까지 이동
    
    Args:
        target_size: 목표 바운딩 박스 크기 (px)
    """
    kp_z = 0.001  # Z축 제어 게인
    
    while cap.isOpened():
        ret, frame = cap.read()
        results = yolo(frame, classes=[39])  # bottle
        
        if len(results[0].boxes) == 0:
            continue
        
        box = results[0].boxes[0]
        w, h = box.xywh[0][2:].tolist()
        size = (w + h) / 2  # 평균 크기
        
        # 크기 오차
        error_size = target_size - size
        
        # Z축 이동 (가까워지기/멀어지기)
        dz = error_size * kp_z
        
        current_pos = robot.get_ee_position()
        new_pos = [current_pos[0], current_pos[1] + dz]
        robot.set_ee_position(new_pos)
        
        print(f"Size: {size:.0f} px (target: {target_size}) → dz: {dz:.3f} m")
        
        # 도달 확인
        if abs(error_size) < 5:
            print("Target size reached!")
            break
        
        cv2.imshow('IBVS', results[0].plot())
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
```

---

## 5. 멀티 객체 제어

### 5.1 순차 집기

```python
def sequential_grasp(robot, yolo, cap, target_classes=['cup', 'bottle']):
    """여러 객체 순차적으로 집기"""
    
    for target_class in target_classes:
        print(f"\n=== Grasping {target_class} ===")
        
        # 객체 찾기
        found = False
        while not found:
            ret, frame = cap.read()
            results = yolo(frame)
            
            for box in results[0].boxes:
                class_name = yolo.names[int(box.cls[0])]
                
                if class_name == target_class:
                    # 좌표 추출
                    cx, cy = box.xywh[0][:2].tolist()
                    x, y = simple_pixel_to_robot(cx, cy)
                    
                    # 이동
                    robot.set_ee_position([x, y])
                    time.sleep(1.0)
                    
                    # 집기
                    robot.close_gripper()
                    time.sleep(0.5)
                    
                    # 들어올리기
                    robot.set_ee_position([x, y + 0.05])
                    
                    # 놓기 (고정 위치)
                    robot.set_ee_position([0.2, 0.1])
                    robot.open_gripper()
                    
                    found = True
                    break
            
            cv2.imshow('Sequential Grasp', results[0].plot())
            if cv2.waitKey(1) & 0xFF == ord('q'):
                return
```

---

## 6. 깊이 정보 활용 (RGBD)

### 6.1 Intel RealSense 통합

```python
import pyrealsense2 as rs
import numpy as np

class RealSenseCamera:
    """RealSense RGBD 카메라"""
    
    def __init__(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        self.pipeline.start(config)
    
    def get_frames(self):
        """RGB + Depth 프레임 반환"""
        frames = self.pipeline.wait_for_frames()
        
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            return None, None
        
        # numpy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())
        
        return color_image, depth_image
    
    def get_3d_point(self, px, py, depth_image):
        """2D 픽셀 + 깊이 → 3D 좌표"""
        depth = depth_image[int(py), int(px)] * 0.001  # mm → m
        
        if depth == 0:
            return None
        
        # 카메라 내부 파라미터 (예시)
        fx = 615.0  # focal length x
        fy = 615.0  # focal length y
        cx = 320.0  # principal point x
        cy = 240.0  # principal point y
        
        # 3D 좌표 계산
        x = (px - cx) * depth / fx
        y = (py - cy) * depth / fy
        z = depth
        
        return (x, y, z)
    
    def cleanup(self):
        """정리"""
        self.pipeline.stop()

# 사용
camera = RealSenseCamera()

while True:
    color, depth = camera.get_frames()
    
    # YOLO 감지
    results = yolo_model(color, classes=[39])
    
    if len(results[0].boxes) > 0:
        box = results[0].boxes[0]
        cx, cy = box.xywh[0][:2].tolist()
        
        # 3D 좌표
        point_3d = camera.get_3d_point(cx, cy, depth)
        
        if point_3d is not None:
            x, y, z = point_3d
            print(f"Object at 3D: ({x:.3f}, {y:.3f}, {z:.3f}) m")
            
            # 로봇 제어
            robot.set_ee_position([x, y, z])
    
    cv2.imshow('RGBD', results[0].plot())
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.cleanup()
```

---

## 7. 핸드-아이 캘리브레이션

### 7.1 Eye-in-Hand vs Eye-to-Hand

**Eye-to-Hand (고정 카메라)**
```
     [Camera]
        ↓
    작업 공간
        ↓
     [Robot]
```

**Eye-in-Hand (로봇 팔 끝에 카메라)**
```
[Robot] ← [Camera]
   ↓
작업 공간
```

### 7.2 변환 행렬 캘리브레이션

```python
import numpy as np

def hand_eye_calibration(camera_poses, robot_poses):
    """
    핸드-아이 캘리브레이션 (단순 버전)
    
    Args:
        camera_poses: 카메라에서 본 객체 위치 리스트 [(x,y,z), ...]
        robot_poses: 로봇 좌표계 객체 위치 리스트 [(x,y,z), ...]
    
    Returns:
        변환 행렬 (3x3)
    """
    camera_poses = np.array(camera_poses)
    robot_poses = np.array(robot_poses)
    
    # 중심 제거
    cam_mean = camera_poses.mean(axis=0)
    rob_mean = robot_poses.mean(axis=0)
    
    cam_centered = camera_poses - cam_mean
    rob_centered = robot_poses - rob_mean
    
    # 스케일 계산
    scale = np.linalg.norm(rob_centered) / np.linalg.norm(cam_centered)
    
    # 회전 행렬 추정 (SVD)
    H = cam_centered.T @ rob_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # 이동 벡터
    t = rob_mean - scale * R @ cam_mean
    
    return R * scale, t

# 데이터 수집
camera_points = []
robot_points = []

print("Move robot to 5 different positions and press Enter")
for i in range(5):
    input(f"Position {i+1}: ")
    
    # 카메라에서 객체 위치
    ret, frame = cap.read()
    results = yolo_model(frame)
    if len(results[0].boxes) > 0:
        cx, cy = results[0].boxes[0].xywh[0][:2].tolist()
        cam_x, cam_y = simple_pixel_to_robot(cx, cy)
        camera_points.append([cam_x, cam_y, 0])
    
    # 로봇 현재 위치
    robot_pos = robot.get_ee_position()
    robot_points.append(robot_pos)

# 캘리브레이션
R, t = hand_eye_calibration(camera_points, robot_points)
print(f"Rotation:\n{R}")
print(f"Translation: {t}")
```

---

## 8. 참고 자료

- [Visual Servoing Tutorial](https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-ibvs.html)
- [Hand-Eye Calibration](https://www.mathworks.com/help/vision/ug/hand-eye-calibration.html)
- [PyRealSense2](https://github.com/IntelRealSense/librealsense/tree/master/wrappers/python)

---

[← 5.3 세그멘테이션](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/03_segmentation.md) | [다음: 5.5 핸드 카메라 활용 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/05_hand_camera.md)
