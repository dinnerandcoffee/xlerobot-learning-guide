# 8.3 프로젝트 3: 픽앤플레이스 시스템

컴퓨터 비전과 모션 플래닝을 결합한 완전한 픽앤플레이스 시스템 구현 프로젝트입니다.

## 🎯 프로젝트 목표

- 물체 감지 및 인식 시스템 구축
- 3D 포즈 추정 구현
- 모션 플래닝 및 궤적 생성
- 그리퍼 제어 최적화
- 완전 자동화된 픽앤플레이스 파이프라인

**난이도**: ⭐⭐⭐ (고급)  
**소요 시간**: 4시간  
**선수 지식**: 1-5장

---

## 1. 시스템 아키텍처

### 1.1 전체 파이프라인

```
┌─────────────────────────────────────────────────────┐
│              픽앤플레이스 시스템                      │
├─────────────────────────────────────────────────────┤
│                                                     │
│  1. 인식 단계                                        │
│     ├─ 카메라 입력                                   │
│     ├─ 물체 감지 (YOLO/Detectron2)                  │
│     ├─ 3D 포즈 추정 (Depth + RGB)                   │
│     └─ 물체 분류 및 선택                             │
│                                                     │
│  2. 플래닝 단계                                      │
│     ├─ 접근 경로 계획                                │
│     ├─ 그래스핑 포인트 계산                          │
│     ├─ 충돌 회피 경로 생성                           │
│     └─ 궤적 최적화                                   │
│                                                     │
│  3. 실행 단계                                        │
│     ├─ 접근 (Approach)                              │
│     ├─ 그래스핑 (Grasp)                             │
│     ├─ 리프트 (Lift)                                │
│     ├─ 이동 (Transfer)                              │
│     ├─ 플레이스 (Place)                             │
│     └─ 복귀 (Retreat)                               │
│                                                     │
│  4. 검증 단계                                        │
│     ├─ 그래스핑 성공 확인                            │
│     ├─ 위치 정확도 검증                              │
│     └─ 오류 처리                                     │
│                                                     │
└─────────────────────────────────────────────────────┘
```

### 1.2 핵심 컴포넌트

**시스템 구조**
```python
import numpy as np
import cv2
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum

class PickPlaceState(Enum):
    """픽앤플레이스 상태"""
    IDLE = 0
    DETECTING = 1
    PLANNING = 2
    APPROACHING = 3
    GRASPING = 4
    LIFTING = 5
    TRANSFERRING = 6
    PLACING = 7
    RETREATING = 8
    COMPLETED = 9
    FAILED = 10

@dataclass
class DetectedObject:
    """감지된 물체 정보"""
    class_name: str
    confidence: float
    bbox_2d: np.ndarray  # [x1, y1, x2, y2]
    position_3d: np.ndarray  # [x, y, z]
    orientation_3d: np.ndarray  # rotation matrix or quaternion
    dimensions: np.ndarray  # [width, height, depth]
    grasp_points: List[np.ndarray]  # possible grasp positions
    
@dataclass
class GraspPose:
    """그래스핑 포즈"""
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # rotation matrix
    approach_vector: np.ndarray  # approach direction
    grasp_width: float  # gripper opening
    quality_score: float  # grasp quality

@dataclass
class Trajectory:
    """로봇 궤적"""
    waypoints: List[np.ndarray]  # joint angles
    timestamps: List[float]
    velocities: List[np.ndarray]
    accelerations: List[np.ndarray]

class PickAndPlaceSystem:
    """픽앤플레이스 통합 시스템"""
    
    def __init__(self, robot, camera):
        self.robot = robot
        self.camera = camera
        
        # 하위 시스템들
        self.detector = ObjectDetector()
        self.pose_estimator = PoseEstimator()
        self.grasp_planner = GraspPlanner()
        self.motion_planner = MotionPlanner(robot)
        
        # 상태
        self.state = PickPlaceState.IDLE
        self.current_object: Optional[DetectedObject] = None
        self.current_grasp: Optional[GraspPose] = None
        self.current_trajectory: Optional[Trajectory] = None
        
        # 설정
        self.config = {
            'approach_distance': 0.10,  # 10cm
            'lift_height': 0.15,        # 15cm
            'grasp_force': 50.0,        # N
            'safety_margin': 0.02,      # 2cm
        }
        
    def execute_pick_and_place(self, target_class: str, place_position: np.ndarray):
        """픽앤플레이스 실행"""
        print(f"🎯 픽앤플레이스 시작: {target_class}")
        
        try:
            # 1. 물체 감지
            self.state = PickPlaceState.DETECTING
            detected_object = self.detect_object(target_class)
            
            if detected_object is None:
                raise Exception(f"물체를 찾을 수 없음: {target_class}")
            
            self.current_object = detected_object
            print(f"✓ 물체 감지 완료: {detected_object.class_name} "
                  f"(신뢰도: {detected_object.confidence:.2f})")
            
            # 2. 그래스핑 계획
            self.state = PickPlaceState.PLANNING
            grasp_pose = self.plan_grasp(detected_object)
            self.current_grasp = grasp_pose
            print(f"✓ 그래스핑 계획 완료 (품질: {grasp_pose.quality_score:.2f})")
            
            # 3. 픽 실행
            self.execute_pick(grasp_pose)
            print("✓ 픽 완료")
            
            # 4. 플레이스 실행
            self.execute_place(place_position)
            print("✓ 플레이스 완료")
            
            self.state = PickPlaceState.COMPLETED
            print("🎉 픽앤플레이스 성공!")
            
            return True
            
        except Exception as e:
            self.state = PickPlaceState.FAILED
            print(f"❌ 실패: {e}")
            self.robot.stop()
            return False
    
    def detect_object(self, target_class: str) -> Optional[DetectedObject]:
        """물체 감지"""
        # 구현은 섹션 2에서
        pass
    
    def plan_grasp(self, obj: DetectedObject) -> GraspPose:
        """그래스핑 계획"""
        # 구현은 섹션 3에서
        pass
    
    def execute_pick(self, grasp_pose: GraspPose):
        """픽 실행"""
        # 구현은 섹션 4에서
        pass
    
    def execute_place(self, position: np.ndarray):
        """플레이스 실행"""
        # 구현은 섹션 5에서
        pass
```

---

## 2. 물체 감지 및 3D 포즈 추정

### 2.1 YOLO 기반 물체 감지

**YOLOv8 통합**
```python
from ultralytics import YOLO
import torch

class ObjectDetector:
    def __init__(self):
        # YOLOv8 모델 로드
        self.model = YOLO('yolov8n.pt')  # nano 버전
        
        # 관심 클래스
        self.target_classes = [
            'bottle', 'cup', 'bowl', 'banana', 'apple',
            'orange', 'book', 'cell phone', 'keyboard', 'mouse'
        ]
        
        # 신뢰도 임계값
        self.confidence_threshold = 0.5
        
    def detect(self, rgb_image: np.ndarray) -> List[dict]:
        """물체 감지"""
        # YOLO 추론
        results = self.model(rgb_image, verbose=False)
        
        detections = []
        
        for result in results:
            boxes = result.boxes
            
            for i, box in enumerate(boxes):
                # 클래스 및 신뢰도
                class_id = int(box.cls[0])
                confidence = float(box.conf[0])
                class_name = self.model.names[class_id]
                
                # 필터링
                if confidence < self.confidence_threshold:
                    continue
                if class_name not in self.target_classes:
                    continue
                
                # 바운딩 박스
                xyxy = box.xyxy[0].cpu().numpy()
                
                detection = {
                    'class_name': class_name,
                    'confidence': confidence,
                    'bbox': xyxy,  # [x1, y1, x2, y2]
                }
                
                detections.append(detection)
        
        return detections
    
    def visualize(self, image: np.ndarray, detections: List[dict]) -> np.ndarray:
        """감지 결과 시각화"""
        vis_image = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox'].astype(int)
            
            # 바운딩 박스
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 레이블
            label = f"{det['class_name']} {det['confidence']:.2f}"
            cv2.putText(vis_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        return vis_image
```

### 2.2 3D 포즈 추정

**RGB-D 기반 3D 위치 추정**
```python
class PoseEstimator:
    def __init__(self, camera_intrinsics):
        """
        camera_intrinsics: 3x3 카메라 내부 파라미터 행렬
        """
        self.K = camera_intrinsics
        self.fx = camera_intrinsics[0, 0]
        self.fy = camera_intrinsics[1, 1]
        self.cx = camera_intrinsics[0, 2]
        self.cy = camera_intrinsics[1, 2]
        
    def estimate_3d_pose(self, detection: dict, depth_image: np.ndarray) -> DetectedObject:
        """2D 감지 + 깊이 → 3D 포즈"""
        
        # 바운딩 박스 중심
        x1, y1, x2, y2 = detection['bbox']
        cx_2d = (x1 + x2) / 2
        cy_2d = (y1 + y2) / 2
        
        # 깊이 값 추출 (중앙 영역의 중간값)
        bbox_depth = depth_image[int(y1):int(y2), int(x1):int(x2)]
        
        # 이상치 제거 후 중간값 사용
        valid_depths = bbox_depth[bbox_depth > 0]
        if len(valid_depths) == 0:
            raise ValueError("깊이 데이터 없음")
        
        depth = np.median(valid_depths)
        
        # 2D → 3D 변환 (카메라 좌표계)
        x_cam = (cx_2d - self.cx) * depth / self.fx
        y_cam = (cy_2d - self.cy) * depth / self.fy
        z_cam = depth
        
        position_camera = np.array([x_cam, y_cam, z_cam])
        
        # 카메라 → 로봇 베이스 좌표 변환
        position_base = self.camera_to_base(position_camera)
        
        # 물체 크기 추정
        width_pixels = x2 - x1
        height_pixels = y2 - y1
        
        width_meters = width_pixels * depth / self.fx
        height_meters = height_pixels * depth / self.fy
        
        # 대략적인 깊이 (물체 종류에 따라 조정 가능)
        depth_meters = min(width_meters, height_meters) * 0.8
        
        dimensions = np.array([width_meters, height_meters, depth_meters])
        
        # 기본 방향 (위에서 잡기)
        orientation = np.eye(3)
        
        # 그래스핑 포인트 생성
        grasp_points = self.generate_grasp_points(
            position_base, orientation, dimensions
        )
        
        return DetectedObject(
            class_name=detection['class_name'],
            confidence=detection['confidence'],
            bbox_2d=detection['bbox'],
            position_3d=position_base,
            orientation_3d=orientation,
            dimensions=dimensions,
            grasp_points=grasp_points
        )
    
    def camera_to_base(self, pos_camera: np.ndarray) -> np.ndarray:
        """카메라 좌표 → 로봇 베이스 좌표"""
        # 카메라 위치 (로봇 베이스 기준)
        camera_position = np.array([0.0, 0.0, 0.5])  # 예시
        
        # 카메라 회전 (아래를 향함)
        camera_rotation = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        # 변환
        pos_base = camera_rotation @ pos_camera + camera_position
        
        return pos_base
    
    def generate_grasp_points(self, position: np.ndarray, 
                             orientation: np.ndarray,
                             dimensions: np.ndarray) -> List[np.ndarray]:
        """가능한 그래스핑 포인트 생성"""
        grasp_points = []
        
        # 위에서 잡기
        top_grasp = position.copy()
        top_grasp[2] += dimensions[2] / 2
        grasp_points.append(top_grasp)
        
        # 측면에서 잡기 (양쪽)
        side_offset = dimensions[0] / 2
        for sign in [-1, 1]:
            side_grasp = position.copy()
            side_grasp[0] += sign * side_offset
            grasp_points.append(side_grasp)
        
        return grasp_points
    
    def visualize_3d(self, rgb_image: np.ndarray, obj: DetectedObject) -> np.ndarray:
        """3D 정보 오버레이"""
        vis = rgb_image.copy()
        
        # 바운딩 박스
        x1, y1, x2, y2 = obj.bbox_2d.astype(int)
        cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # 3D 정보 텍스트
        pos = obj.position_3d
        info_text = [
            f"Class: {obj.class_name}",
            f"Pos: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})m",
            f"Size: {obj.dimensions[0]:.2f}x{obj.dimensions[1]:.2f}x{obj.dimensions[2]:.2f}m"
        ]
        
        y_offset = y1 - 40
        for text in info_text:
            cv2.putText(vis, text, (x1, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            y_offset += 15
        
        # 그래스핑 포인트 표시 (카메라 투영)
        for gp in obj.grasp_points:
            # 3D → 2D 투영
            gp_2d = self.project_to_2d(gp)
            if gp_2d is not None:
                gp_x, gp_y = gp_2d.astype(int)
                cv2.circle(vis, (gp_x, gp_y), 5, (0, 0, 255), -1)
        
        return vis
    
    def project_to_2d(self, point_3d: np.ndarray) -> Optional[np.ndarray]:
        """3D 포인트 → 2D 이미지 투영"""
        # 베이스 → 카메라 좌표 변환
        point_cam = self.base_to_camera(point_3d)
        
        if point_cam[2] <= 0:
            return None
        
        # 투영
        x = self.fx * point_cam[0] / point_cam[2] + self.cx
        y = self.fy * point_cam[1] / point_cam[2] + self.cy
        
        return np.array([x, y])
    
    def base_to_camera(self, pos_base: np.ndarray) -> np.ndarray:
        """로봇 베이스 → 카메라 좌표"""
        # camera_to_base의 역변환
        camera_position = np.array([0.0, 0.0, 0.5])
        camera_rotation = np.array([
            [1, 0, 0],
            [0, -1, 0],
            [0, 0, -1]
        ])
        
        pos_cam = camera_rotation.T @ (pos_base - camera_position)
        return pos_cam
```

---

## 3. 그래스핑 계획

### 3.1 그래스핑 포즈 계산

**Grasp Planner**
```python
from scipy.spatial.transform import Rotation as R

class GraspPlanner:
    def __init__(self):
        self.gripper_width_range = (0.0, 0.08)  # 0-8cm
        
    def plan_grasp(self, obj: DetectedObject) -> GraspPose:
        """최적 그래스핑 포즈 계획"""
        
        # 모든 가능한 그래스핑 포즈 생성
        candidate_grasps = []
        
        for grasp_point in obj.grasp_points:
            # 여러 접근 각도 시도
            for approach_angle in [0, 45, 90, 135, 180]:
                grasp = self.create_grasp_pose(
                    grasp_point, 
                    obj.orientation_3d,
                    approach_angle,
                    obj.dimensions
                )
                
                if grasp is not None:
                    # 그래스핑 품질 평가
                    grasp.quality_score = self.evaluate_grasp(grasp, obj)
                    candidate_grasps.append(grasp)
        
        if not candidate_grasps:
            raise Exception("실행 가능한 그래스핑 포즈 없음")
        
        # 최고 품질의 그래스핑 선택
        best_grasp = max(candidate_grasps, key=lambda g: g.quality_score)
        
        return best_grasp
    
    def create_grasp_pose(self, position: np.ndarray, 
                         obj_orientation: np.ndarray,
                         approach_angle_deg: float,
                         dimensions: np.ndarray) -> Optional[GraspPose]:
        """그래스핑 포즈 생성"""
        
        # 접근 방향 (Z축 기준 회전)
        approach_angle_rad = np.deg2rad(approach_angle_deg)
        
        # 기본: 위에서 아래로 접근
        approach_vector = np.array([0, 0, -1])
        
        # 회전 적용
        rotation = R.from_euler('z', approach_angle_rad)
        approach_vector = rotation.apply(approach_vector)
        
        # 그리퍼 방향 (접근 방향에 수직)
        gripper_orientation = self.compute_gripper_orientation(approach_vector)
        
        # 그래스핑 폭 계산
        grasp_width = min(dimensions[0], dimensions[1])
        
        # 그리퍼 범위 체크
        if not (self.gripper_width_range[0] <= grasp_width <= self.gripper_width_range[1]):
            return None
        
        return GraspPose(
            position=position.copy(),
            orientation=gripper_orientation,
            approach_vector=approach_vector,
            grasp_width=grasp_width,
            quality_score=0.0  # 나중에 계산
        )
    
    def compute_gripper_orientation(self, approach_vector: np.ndarray) -> np.ndarray:
        """접근 벡터로부터 그리퍼 방향 계산"""
        
        # Z축이 접근 방향을 향하도록
        z_axis = -approach_vector / np.linalg.norm(approach_vector)
        
        # X축 (임의 선택, 중력 방향에 수직)
        if abs(z_axis[2]) < 0.9:
            x_axis = np.cross([0, 0, 1], z_axis)
        else:
            x_axis = np.cross([1, 0, 0], z_axis)
        x_axis = x_axis / np.linalg.norm(x_axis)
        
        # Y축 (오른손 법칙)
        y_axis = np.cross(z_axis, x_axis)
        
        # 회전 행렬
        rotation_matrix = np.column_stack([x_axis, y_axis, z_axis])
        
        return rotation_matrix
    
    def evaluate_grasp(self, grasp: GraspPose, obj: DetectedObject) -> float:
        """그래스핑 품질 평가 (0~1)"""
        
        score = 0.0
        
        # 1. 접근 가능성 (위에서 접근이 더 좋음)
        approach_score = max(0, grasp.approach_vector[2])  # Z 성분
        score += 0.3 * approach_score
        
        # 2. 그래스핑 폭 (중간 크기가 안정적)
        optimal_width = (self.gripper_width_range[0] + self.gripper_width_range[1]) / 2
        width_score = 1.0 - abs(grasp.grasp_width - optimal_width) / optimal_width
        score += 0.3 * width_score
        
        # 3. 물체 중심과의 거리 (중심에 가까울수록 좋음)
        distance = np.linalg.norm(grasp.position - obj.position_3d)
        distance_score = np.exp(-distance * 10)
        score += 0.2 * distance_score
        
        # 4. 안정성 (테이블 위의 물체는 위에서 잡기)
        if obj.position_3d[2] > 0:  # 테이블 위
            if grasp.approach_vector[2] < -0.5:  # 위에서 접근
                score += 0.2
        
        return np.clip(score, 0, 1)
```

---

## 4. 모션 플래닝 및 궤적 생성

### 4.1 RRT 기반 경로 계획

**RRT Path Planner**
```python
import random
from scipy.spatial import KDTree

class MotionPlanner:
    def __init__(self, robot):
        self.robot = robot
        
        # 관절 한계
        self.joint_limits = robot.get_joint_limits()
        
        # RRT 파라미터
        self.rrt_max_iterations = 1000
        self.rrt_step_size = 0.1  # rad
        self.rrt_goal_bias = 0.1
        
    def plan_to_pose(self, target_pos: np.ndarray, 
                    target_rot: np.ndarray) -> Optional[Trajectory]:
        """목표 포즈로 경로 계획"""
        
        # 역기구학으로 목표 관절 각도 계산
        target_joints = self.robot.inverse_kinematics(target_pos, target_rot)
        
        if target_joints is None:
            print("역기구학 실패")
            return None
        
        # 현재 관절 각도
        start_joints = self.robot.get_joint_angles()
        
        # RRT로 경로 계획
        path = self.rrt_plan(start_joints, target_joints)
        
        if path is None:
            print("경로 계획 실패")
            return None
        
        # 궤적 생성 (시간 파라미터화)
        trajectory = self.create_trajectory(path)
        
        return trajectory
    
    def rrt_plan(self, start: np.ndarray, goal: np.ndarray) -> Optional[List[np.ndarray]]:
        """RRT 경로 계획"""
        
        # 트리 초기화
        tree = [start]
        parent = {0: None}
        
        for iteration in range(self.rrt_max_iterations):
            # 랜덤 샘플 (goal bias 적용)
            if random.random() < self.rrt_goal_bias:
                sample = goal
            else:
                sample = self.random_configuration()
            
            # 가장 가까운 노드 찾기
            nearest_idx = self.nearest_node(tree, sample)
            nearest = tree[nearest_idx]
            
            # 새 노드로 확장
            new_node = self.steer(nearest, sample)
            
            # 충돌 체크
            if self.is_collision_free(nearest, new_node):
                # 트리에 추가
                new_idx = len(tree)
                tree.append(new_node)
                parent[new_idx] = nearest_idx
                
                # 목표 도달 확인
                if np.linalg.norm(new_node - goal) < self.rrt_step_size:
                    # 경로 추출
                    path = self.extract_path(tree, parent, new_idx)
                    path.append(goal)
                    return path
        
        print(f"RRT: {self.rrt_max_iterations}회 반복 후 실패")
        return None
    
    def random_configuration(self) -> np.ndarray:
        """랜덤 관절 설정"""
        config = np.random.uniform(
            self.joint_limits[:, 0],
            self.joint_limits[:, 1]
        )
        return config
    
    def nearest_node(self, tree: List[np.ndarray], sample: np.ndarray) -> int:
        """가장 가까운 노드 찾기"""
        tree_array = np.array(tree)
        distances = np.linalg.norm(tree_array - sample, axis=1)
        return np.argmin(distances)
    
    def steer(self, from_node: np.ndarray, to_node: np.ndarray) -> np.ndarray:
        """노드 확장"""
        direction = to_node - from_node
        distance = np.linalg.norm(direction)
        
        if distance < self.rrt_step_size:
            return to_node
        
        direction = direction / distance
        new_node = from_node + direction * self.rrt_step_size
        
        # 관절 한계 클리핑
        new_node = np.clip(new_node, 
                          self.joint_limits[:, 0],
                          self.joint_limits[:, 1])
        
        return new_node
    
    def is_collision_free(self, config1: np.ndarray, config2: np.ndarray) -> bool:
        """충돌 체크 (간단한 버전)"""
        
        # 중간 지점들 체크
        steps = 10
        for i in range(steps + 1):
            t = i / steps
            config = config1 + t * (config2 - config1)
            
            # 자체 충돌 체크
            if self.is_self_collision(config):
                return False
            
            # 환경 충돌 체크
            if self.is_environment_collision(config):
                return False
        
        return True
    
    def is_self_collision(self, config: np.ndarray) -> bool:
        """자체 충돌 체크"""
        # 간단한 버전: 관절 한계만 체크
        return np.any(config < self.joint_limits[:, 0]) or \
               np.any(config > self.joint_limits[:, 1])
    
    def is_environment_collision(self, config: np.ndarray) -> bool:
        """환경 충돌 체크"""
        # 시뮬레이션 환경에서 체크
        # (실제로는 MuJoCo collision detection 사용)
        return False
    
    def extract_path(self, tree: List[np.ndarray], 
                    parent: dict, node_idx: int) -> List[np.ndarray]:
        """경로 추출"""
        path = []
        current = node_idx
        
        while current is not None:
            path.append(tree[current])
            current = parent[current]
        
        path.reverse()
        return path
    
    def create_trajectory(self, path: List[np.ndarray], 
                         max_velocity: float = 1.0,
                         max_acceleration: float = 2.0) -> Trajectory:
        """경로를 시간 파라미터화된 궤적으로 변환"""
        
        if len(path) < 2:
            raise ValueError("경로가 너무 짧음")
        
        waypoints = []
        timestamps = [0.0]
        velocities = []
        accelerations = []
        
        # 첫 웨이포인트
        waypoints.append(path[0])
        velocities.append(np.zeros_like(path[0]))
        accelerations.append(np.zeros_like(path[0]))
        
        current_time = 0.0
        
        # 각 세그먼트마다 시간 계산
        for i in range(1, len(path)):
            prev = path[i - 1]
            current = path[i]
            
            # 거리
            distance = np.linalg.norm(current - prev)
            
            # 시간 (삼각형 속도 프로파일)
            segment_time = 2 * distance / max_velocity
            current_time += segment_time
            
            waypoints.append(current)
            timestamps.append(current_time)
            
            # 속도 (간단한 추정)
            velocity = (current - prev) / segment_time
            velocities.append(velocity)
            
            # 가속도 (간단한 추정)
            if i > 1:
                accel = (velocities[-1] - velocities[-2]) / segment_time
            else:
                accel = velocities[-1] / segment_time
            accelerations.append(accel)
        
        return Trajectory(
            waypoints=waypoints,
            timestamps=timestamps,
            velocities=velocities,
            accelerations=accelerations
        )
```

---

## 5. 픽앤플레이스 실행

### 5.1 픽 시퀀스

**Pick Execution**
```python
def execute_pick(self, grasp_pose: GraspPose):
    """픽 시퀀스 실행"""
    
    print("📍 Step 1: 접근 위치로 이동")
    self.state = PickPlaceState.APPROACHING
    
    # 접근 위치 (그래스핑 위치에서 약간 떨어진 곳)
    approach_pos = grasp_pose.position + \
                   grasp_pose.approach_vector * self.config['approach_distance']
    
    # 접근 위치로 경로 계획 및 이동
    approach_traj = self.motion_planner.plan_to_pose(
        approach_pos, 
        grasp_pose.orientation
    )
    
    if approach_traj is None:
        raise Exception("접근 경로 계획 실패")
    
    self.execute_trajectory(approach_traj)
    print("  ✓ 접근 위치 도달")
    
    # 그리퍼 열기
    self.robot.set_gripper_position(0.0)
    time.sleep(0.5)
    
    print("📍 Step 2: 물체로 이동")
    # 직선 이동 (그래스핑 위치로)
    self.move_linear(grasp_pose.position, grasp_pose.orientation)
    print("  ✓ 그래스핑 위치 도달")
    
    print("📍 Step 3: 그리퍼 닫기")
    self.state = PickPlaceState.GRASPING
    
    # 그리퍼 닫기
    target_gripper_pos = grasp_pose.grasp_width / self.robot.max_gripper_width
    self.robot.set_gripper_position(target_gripper_pos)
    time.sleep(1.0)
    
    # 그래스핑 성공 확인
    if not self.verify_grasp():
        raise Exception("그래스핑 실패")
    
    print("  ✓ 물체 잡음")
    
    print("📍 Step 4: 리프트")
    self.state = PickPlaceState.LIFTING
    
    # 위로 들어올리기
    lift_pos = grasp_pose.position.copy()
    lift_pos[2] += self.config['lift_height']
    
    self.move_linear(lift_pos, grasp_pose.orientation)
    print("  ✓ 리프트 완료")

def verify_grasp(self) -> bool:
    """그래스핑 성공 확인"""
    
    # 그리퍼 위치 확인
    gripper_pos = self.robot.get_gripper_position()
    
    # 완전히 닫히지 않았으면 물체를 잡은 것
    if gripper_pos > 0.1:  # 10% 이상 열려있음
        return True
    
    # 추가: 힘 센서가 있다면 접촉력 확인
    # contact_force = self.robot.get_gripper_force()
    # if contact_force > threshold:
    #     return True
    
    return False

def move_linear(self, target_pos: np.ndarray, target_rot: np.ndarray, 
                duration: float = 2.0):
    """직선 경로로 이동"""
    
    current_pos, current_rot = self.robot.get_end_effector_pose()
    
    steps = int(duration * 60)  # 60 Hz
    
    for i in range(steps):
        t = (i + 1) / steps
        
        # 위치 보간
        interp_pos = current_pos + t * (target_pos - current_pos)
        
        # 회전 보간 (SLERP)
        from scipy.spatial.transform import Rotation as R
        rot_current = R.from_matrix(current_rot)
        rot_target = R.from_matrix(target_rot)
        
        rot_interp = rot_current.slerp(rot_target, t)
        interp_rot = rot_interp.as_matrix()
        
        # 역기구학
        joint_angles = self.robot.inverse_kinematics(interp_pos, interp_rot)
        
        if joint_angles is not None:
            self.robot.set_joint_angles(joint_angles)
        
        time.sleep(1.0 / 60.0)

def execute_trajectory(self, trajectory: Trajectory):
    """궤적 실행"""
    
    start_time = time.time()
    
    for i, (waypoint, timestamp) in enumerate(zip(trajectory.waypoints, 
                                                   trajectory.timestamps)):
        # 타이밍 맞추기
        while time.time() - start_time < timestamp:
            time.sleep(0.001)
        
        # 관절 각도 설정
        self.robot.set_joint_angles(waypoint)
```

### 5.2 플레이스 시퀀스

**Place Execution**
```python
def execute_place(self, place_position: np.ndarray):
    """플레이스 시퀀스 실행"""
    
    print("📍 Step 5: 플레이스 위치로 이동")
    self.state = PickPlaceState.TRANSFERRING
    
    # 플레이스 위치 위로 이동
    transfer_pos = place_position.copy()
    transfer_pos[2] += self.config['lift_height']
    
    # 방향 (위에서 아래로)
    place_orientation = np.eye(3)
    
    # 경로 계획 및 실행
    transfer_traj = self.motion_planner.plan_to_pose(
        transfer_pos,
        place_orientation
    )
    
    if transfer_traj is None:
        raise Exception("이동 경로 계획 실패")
    
    self.execute_trajectory(transfer_traj)
    print("  ✓ 플레이스 위치 상공 도달")
    
    print("📍 Step 6: 하강")
    self.state = PickPlaceState.PLACING
    
    # 천천히 하강
    self.move_linear(place_position, place_orientation, duration=2.0)
    print("  ✓ 플레이스 위치 도달")
    
    print("📍 Step 7: 물체 놓기")
    # 그리퍼 열기
    self.robot.set_gripper_position(0.0)
    time.sleep(1.0)
    print("  ✓ 물체 놓음")
    
    print("📍 Step 8: 복귀")
    self.state = PickPlaceState.RETREATING
    
    # 위로 복귀
    retreat_pos = place_position.copy()
    retreat_pos[2] += self.config['lift_height']
    
    self.move_linear(retreat_pos, place_orientation, duration=1.5)
    print("  ✓ 복귀 완료")
```

---

## 6. 통합 예제

### 6.1 완전한 픽앤플레이스 데모

**메인 스크립트**
```python
import time
import numpy as np
import cv2

def main():
    """픽앤플레이스 데모"""
    
    print("=" * 60)
    print("  XLeRobot 픽앤플레이스 시스템")
    print("=" * 60)
    
    # 1. 로봇 및 카메라 초기화
    print("\n🤖 시스템 초기화...")
    robot = XLeRobotMuJoCo()
    camera = RGBDCamera()
    
    # 2. 픽앤플레이스 시스템 생성
    system = PickAndPlaceSystem(robot, camera)
    
    # 3. 홈 포지션으로 이동
    print("🏠 홈 포지션으로 이동")
    robot.go_home()
    time.sleep(2)
    
    # 4. 물체 감지
    print("\n📷 물체 감지 중...")
    rgb_image = camera.get_rgb()
    depth_image = camera.get_depth()
    
    # YOLO 감지
    detections = system.detector.detect(rgb_image)
    print(f"  발견된 물체: {len(detections)}개")
    
    for det in detections:
        print(f"    - {det['class_name']} (신뢰도: {det['confidence']:.2f})")
    
    # 시각화
    vis_image = system.detector.visualize(rgb_image, detections)
    cv2.imshow("Detections", vis_image)
    cv2.waitKey(2000)
    
    # 5. 첫 번째 물체 선택
    if not detections:
        print("❌ 물체를 찾을 수 없습니다")
        return
    
    target_detection = detections[0]
    print(f"\n🎯 타겟: {target_detection['class_name']}")
    
    # 6. 3D 포즈 추정
    print("📐 3D 포즈 추정 중...")
    camera_intrinsics = camera.get_intrinsics()
    pose_estimator = PoseEstimator(camera_intrinsics)
    
    detected_object = pose_estimator.estimate_3d_pose(
        target_detection, 
        depth_image
    )
    
    print(f"  위치: {detected_object.position_3d}")
    print(f"  크기: {detected_object.dimensions}")
    
    # 7. 플레이스 위치 설정
    place_position = np.array([0.4, -0.3, 0.05])  # 테이블의 다른 위치
    
    print(f"\n📍 플레이스 위치: {place_position}")
    
    # 8. 픽앤플레이스 실행
    print("\n🚀 픽앤플레이스 시작!\n")
    
    success = system.execute_pick_and_place(
        target_detection['class_name'],
        place_position
    )
    
    if success:
        print("\n" + "=" * 60)
        print("  ✅ 픽앤플레이스 성공!")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print("  ❌ 픽앤플레이스 실패")
        print("=" * 60)
    
    # 9. 홈으로 복귀
    print("\n🏠 홈 포지션으로 복귀")
    robot.go_home()
    
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
```

### 6.2 연속 픽앤플레이스

**다중 물체 처리**
```python
def continuous_pick_and_place():
    """연속 픽앤플레이스 (테이블 정리)"""
    
    system = PickAndPlaceSystem(robot, camera)
    
    # 소스 영역과 타겟 영역 정의
    source_area = {
        'x_range': (0.2, 0.5),
        'y_range': (-0.2, 0.2)
    }
    
    target_positions = [
        np.array([0.4, -0.4, 0.05]),
        np.array([0.4, -0.5, 0.05]),
        np.array([0.5, -0.4, 0.05]),
        np.array([0.5, -0.5, 0.05]),
    ]
    
    placed_count = 0
    
    while placed_count < len(target_positions):
        # 물체 감지
        rgb = camera.get_rgb()
        depth = camera.get_depth()
        
        detections = system.detector.detect(rgb)
        
        if not detections:
            print("더 이상 물체가 없습니다")
            break
        
        # 소스 영역 내의 물체 필터링
        valid_objects = []
        
        for det in detections:
            obj = pose_estimator.estimate_3d_pose(det, depth)
            
            if (source_area['x_range'][0] <= obj.position_3d[0] <= source_area['x_range'][1] and
                source_area['y_range'][0] <= obj.position_3d[1] <= source_area['y_range'][1]):
                valid_objects.append((det, obj))
        
        if not valid_objects:
            print("소스 영역에 물체가 없습니다")
            break
        
        # 가장 가까운 물체 선택
        det, obj = min(valid_objects, key=lambda x: np.linalg.norm(x[1].position_3d[:2]))
        
        # 픽앤플레이스 실행
        success = system.execute_pick_and_place(
            det['class_name'],
            target_positions[placed_count]
        )
        
        if success:
            placed_count += 1
            print(f"✅ {placed_count}/{len(target_positions)} 완료")
        else:
            print("❌ 실패, 다음 물체로...")
            continue
    
    print(f"\n🎉 총 {placed_count}개 물체 정리 완료!")
```

---

## 7. 고급 기능

### 7.1 학습 기반 그래스핑

**간단한 그래스핑 네트워크**
```python
import torch
import torch.nn as nn

class GraspNet(nn.Module):
    """그래스핑 품질 예측 네트워크"""
    
    def __init__(self):
        super().__init__()
        
        # 이미지 특징 추출
        self.conv = nn.Sequential(
            nn.Conv2d(4, 32, 3, padding=1),  # RGB + Depth
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(32, 64, 3, padding=1),
            nn.ReLU(),
            nn.MaxPool2d(2),
            nn.Conv2d(64, 128, 3, padding=1),
            nn.ReLU(),
            nn.AdaptiveAvgPool2d(1)
        )
        
        # 그래스핑 품질 예측
        self.fc = nn.Sequential(
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Sigmoid()
        )
    
    def forward(self, rgbd_patch):
        """
        rgbd_patch: (B, 4, H, W) - RGB + Depth
        returns: (B, 1) - grasp quality score
        """
        features = self.conv(rgbd_patch)
        features = features.view(features.size(0), -1)
        quality = self.fc(features)
        return quality
```

---

## ✅ 프로젝트 3 완료 체크리스트

- [ ] YOLO 물체 감지 구현
- [ ] RGB-D 기반 3D 포즈 추정
- [ ] 그래스핑 포즈 계획
- [ ] RRT 경로 계획 구현
- [ ] 픽 시퀀스 실행
- [ ] 플레이스 시퀀스 실행
- [ ] 완전한 픽앤플레이스 파이프라인
- [ ] 연속 픽앤플레이스 (다중 물체)

## 🎓 학습 정리

1. **물체 인식**: YOLO + 깊이 정보로 3D 포즈 추정
2. **그래스핑**: 최적 그래스핑 포즈 계산 및 품질 평가
3. **모션 플래닝**: RRT 알고리즘으로 충돌 회피 경로 생성
4. **실행 제어**: 정밀한 픽앤플레이스 시퀀스 실행
5. **시스템 통합**: 모든 컴포넌트를 하나의 파이프라인으로

---

[← 8.2 커스텀 제어](02_custom_control.md) | [다음: 8.4 자율 네비게이션 →](04_navigation.md)
