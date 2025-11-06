# 8.4 프로젝트 4: 자율 네비게이션

SLAM, 경로 계획, 장애물 회피를 포함한 완전 자율 네비게이션 시스템 구현 프로젝트입니다.

## 🎯 프로젝트 목표

- SLAM (동시적 위치추정 및 지도작성) 구현
- 전역 경로 계획 (A*, Dijkstra)
- 지역 경로 계획 (DWA, TEB)
- 장애물 감지 및 회피
- 로컬라이제이션 및 맵핑
- 완전 자율 네비게이션 시스템

**난이도**: ⭐⭐⭐⭐ (전문가)  
**소요 시간**: 1일  
**선수 지식**: 1-5장

---

## 1. 시스템 아키텍처

### 1.1 네비게이션 스택 구조

```
┌─────────────────────────────────────────────────────────┐
│           자율 네비게이션 시스템                          │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  센서 레이어                                             │
│  ├─ LiDAR / 깊이 카메라                                 │
│  ├─ IMU (관성 측정 장치)                                │
│  ├─ 휠 오도메트리                                        │
│  └─ RGB 카메라                                          │
│                                                         │
│  인식 레이어                                             │
│  ├─ SLAM (gmapping, Cartographer)                      │
│  ├─ 로컬라이제이션 (AMCL, EKF)                          │
│  └─ 장애물 감지                                         │
│                                                         │
│  계획 레이어                                             │
│  ├─ 전역 플래너 (A*, Dijkstra, RRT*)                   │
│  ├─ 지역 플래너 (DWA, TEB, MPC)                        │
│  └─ 복구 동작                                           │
│                                                         │
│  제어 레이어                                             │
│  ├─ 속도 제어                                           │
│  ├─ 궤적 추종                                           │
│  └─ 안전 모니터링                                       │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 1.2 핵심 데이터 구조

**기본 클래스 정의**
```python
import numpy as np
from dataclasses import dataclass
from typing import List, Tuple, Optional
from enum import Enum
import matplotlib.pyplot as plt
from scipy.ndimage import distance_transform_edt

class NavigationState(Enum):
    """네비게이션 상태"""
    IDLE = 0
    PLANNING = 1
    NAVIGATING = 2
    STUCK = 3
    REACHED = 4
    FAILED = 5

@dataclass
class Pose2D:
    """2D 포즈 (x, y, theta)"""
    x: float
    y: float
    theta: float  # 라디안
    
    def to_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta])
    
    def distance_to(self, other: 'Pose2D') -> float:
        return np.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

@dataclass
class Velocity:
    """속도 명령"""
    linear: float   # m/s
    angular: float  # rad/s

@dataclass
class LaserScan:
    """LiDAR 스캔 데이터"""
    ranges: np.ndarray      # 거리 (m)
    angle_min: float        # 시작 각도 (rad)
    angle_max: float        # 끝 각도 (rad)
    angle_increment: float  # 각도 증분 (rad)
    range_min: float        # 최소 거리
    range_max: float        # 최대 거리
    
    def get_angles(self) -> np.ndarray:
        num_points = len(self.ranges)
        return np.linspace(self.angle_min, self.angle_max, num_points)
    
    def to_cartesian(self) -> np.ndarray:
        """극좌표 → 직교좌표 변환"""
        angles = self.get_angles()
        x = self.ranges * np.cos(angles)
        y = self.ranges * np.sin(angles)
        return np.column_stack([x, y])

class OccupancyGrid:
    """점유 격자 지도"""
    
    def __init__(self, width: int, height: int, resolution: float, origin: Pose2D):
        """
        width, height: 격자 크기 (셀 개수)
        resolution: 격자 해상도 (m/cell)
        origin: 지도 원점 (world 좌표계)
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        
        # 점유 확률 (0: 자유, 100: 점유, -1: 미지)
        self.data = np.full((height, width), -1, dtype=np.int8)
        
    def world_to_grid(self, x: float, y: float) -> Tuple[int, int]:
        """월드 좌표 → 격자 좌표"""
        gx = int((x - self.origin.x) / self.resolution)
        gy = int((y - self.origin.y) / self.resolution)
        return gx, gy
    
    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        """격자 좌표 → 월드 좌표"""
        x = gx * self.resolution + self.origin.x
        y = gy * self.resolution + self.origin.y
        return x, y
    
    def is_valid(self, gx: int, gy: int) -> bool:
        """격자 좌표 유효성 검사"""
        return 0 <= gx < self.width and 0 <= gy < self.height
    
    def is_occupied(self, gx: int, gy: int, threshold: int = 50) -> bool:
        """점유 여부 확인"""
        if not self.is_valid(gx, gy):
            return True
        return self.data[gy, gx] >= threshold
    
    def set_occupied(self, gx: int, gy: int, value: int = 100):
        """점유 설정"""
        if self.is_valid(gx, gy):
            self.data[gy, gx] = value
    
    def set_free(self, gx: int, gy: int, value: int = 0):
        """자유 공간 설정"""
        if self.is_valid(gx, gy):
            self.data[gy, gx] = value
    
    def visualize(self, robot_pose: Optional[Pose2D] = None, 
                  path: Optional[List[Tuple[int, int]]] = None):
        """지도 시각화"""
        plt.figure(figsize=(10, 10))
        
        # 지도 표시
        display_map = self.data.copy().astype(float)
        display_map[display_map == -1] = 50  # 미지 영역은 회색
        
        plt.imshow(display_map, cmap='gray_r', origin='lower',
                  extent=[self.origin.x, 
                         self.origin.x + self.width * self.resolution,
                         self.origin.y,
                         self.origin.y + self.height * self.resolution])
        
        # 경로 표시
        if path:
            path_x = [self.grid_to_world(gx, gy)[0] for gx, gy in path]
            path_y = [self.grid_to_world(gx, gy)[1] for gx, gy in path]
            plt.plot(path_x, path_y, 'b-', linewidth=2, label='Path')
        
        # 로봇 위치 표시
        if robot_pose:
            plt.plot(robot_pose.x, robot_pose.y, 'ro', markersize=10, label='Robot')
            
            # 방향 표시
            arrow_length = 0.3
            dx = arrow_length * np.cos(robot_pose.theta)
            dy = arrow_length * np.sin(robot_pose.theta)
            plt.arrow(robot_pose.x, robot_pose.y, dx, dy, 
                     head_width=0.1, head_length=0.1, fc='r', ec='r')
        
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Occupancy Grid Map')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.show()
```

---

## 2. SLAM (동시적 위치추정 및 지도작성)

### 2.1 2D LiDAR SLAM

**Grid-based SLAM**
```python
class GridSLAM:
    """그리드 기반 SLAM"""
    
    def __init__(self, map_size: Tuple[int, int], resolution: float):
        self.resolution = resolution
        
        # 지도 초기화
        origin = Pose2D(-map_size[0] * resolution / 2,
                       -map_size[1] * resolution / 2,
                       0.0)
        self.map = OccupancyGrid(map_size[0], map_size[1], resolution, origin)
        
        # 로봇 포즈
        self.robot_pose = Pose2D(0.0, 0.0, 0.0)
        
        # 파티클 필터 (로컬라이제이션용)
        self.num_particles = 100
        self.particles = self.initialize_particles()
        
        # 이전 스캔
        self.prev_scan = None
        
    def initialize_particles(self) -> np.ndarray:
        """파티클 초기화"""
        particles = np.zeros((self.num_particles, 3))
        
        # 초기 위치 주변에 분산
        particles[:, 0] = np.random.normal(0, 0.1, self.num_particles)  # x
        particles[:, 1] = np.random.normal(0, 0.1, self.num_particles)  # y
        particles[:, 2] = np.random.normal(0, 0.1, self.num_particles)  # theta
        
        return particles
    
    def update(self, scan: LaserScan, odom: Pose2D):
        """SLAM 업데이트"""
        
        # 1. 예측 단계 (오도메트리)
        self.predict(odom)
        
        # 2. 스캔 매칭으로 포즈 보정
        if self.prev_scan is not None:
            corrected_pose = self.scan_matching(scan, self.prev_scan)
            if corrected_pose is not None:
                self.robot_pose = corrected_pose
        
        # 3. 지도 업데이트
        self.update_map(scan, self.robot_pose)
        
        # 4. 이전 스캔 저장
        self.prev_scan = scan
        
        return self.robot_pose, self.map
    
    def predict(self, odom: Pose2D):
        """오도메트리 기반 예측"""
        # 간단한 버전: 오도메트리 그대로 사용
        self.robot_pose = odom
        
        # 실제로는 노이즈 모델 적용 및 파티클 업데이트
        # self.particles = self.motion_model(self.particles, odom)
    
    def scan_matching(self, current_scan: LaserScan, 
                     prev_scan: LaserScan) -> Optional[Pose2D]:
        """스캔 매칭으로 포즈 추정 (ICP 간단 버전)"""
        
        # 현재 스캔을 포인트 클라우드로 변환
        current_points = current_scan.to_cartesian()
        prev_points = prev_scan.to_cartesian()
        
        # 유효한 포인트만 사용
        valid_current = current_scan.ranges < current_scan.range_max
        valid_prev = prev_scan.ranges < prev_scan.range_max
        
        current_points = current_points[valid_current]
        prev_points = prev_points[valid_prev]
        
        if len(current_points) < 10 or len(prev_points) < 10:
            return None
        
        # 간단한 ICP (반복 최근접점)
        best_pose = self.icp(prev_points, current_points, max_iterations=10)
        
        return best_pose
    
    def icp(self, source: np.ndarray, target: np.ndarray, 
            max_iterations: int = 10) -> Pose2D:
        """Iterative Closest Point"""
        
        # 초기 변환
        dx, dy, dtheta = 0.0, 0.0, 0.0
        
        for iteration in range(max_iterations):
            # 변환 적용
            transformed = self.transform_points(source, dx, dy, dtheta)
            
            # 최근접 점 찾기
            correspondences = self.find_correspondences(transformed, target)
            
            if len(correspondences) < 5:
                break
            
            # 변환 계산
            delta = self.compute_transformation(correspondences)
            
            dx += delta[0]
            dy += delta[1]
            dtheta += delta[2]
            
            # 수렴 확인
            if np.linalg.norm(delta[:2]) < 0.001 and abs(delta[2]) < 0.01:
                break
        
        # 새 포즈 계산
        new_pose = Pose2D(
            self.robot_pose.x + dx,
            self.robot_pose.y + dy,
            self.robot_pose.theta + dtheta
        )
        
        return new_pose
    
    def transform_points(self, points: np.ndarray, 
                        dx: float, dy: float, dtheta: float) -> np.ndarray:
        """포인트 변환"""
        cos_t = np.cos(dtheta)
        sin_t = np.sin(dtheta)
        
        rotation = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
        translation = np.array([dx, dy])
        
        return points @ rotation.T + translation
    
    def find_correspondences(self, source: np.ndarray, 
                           target: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray]]:
        """최근접 점 대응 찾기"""
        from scipy.spatial import KDTree
        
        tree = KDTree(target)
        correspondences = []
        
        for point in source:
            distance, idx = tree.query(point)
            
            if distance < 0.3:  # 30cm 이내만 매칭
                correspondences.append((point, target[idx]))
        
        return correspondences
    
    def compute_transformation(self, correspondences: List) -> np.ndarray:
        """대응점으로부터 변환 계산"""
        
        source_points = np.array([c[0] for c in correspondences])
        target_points = np.array([c[1] for c in correspondences])
        
        # 중심점
        source_center = np.mean(source_points, axis=0)
        target_center = np.mean(target_points, axis=0)
        
        # 중심 정렬
        source_centered = source_points - source_center
        target_centered = target_points - target_center
        
        # SVD로 회전 계산
        H = source_centered.T @ target_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # 회전 각도
        dtheta = np.arctan2(R[1, 0], R[0, 0])
        
        # 이동
        translation = target_center - R @ source_center
        
        return np.array([translation[0], translation[1], dtheta])
    
    def update_map(self, scan: LaserScan, pose: Pose2D):
        """스캔으로 지도 업데이트"""
        
        # 로봇 위치 (격자 좌표)
        robot_gx, robot_gy = self.map.world_to_grid(pose.x, pose.y)
        
        # 각 스캔 포인트 처리
        angles = scan.get_angles()
        
        for i, (distance, angle) in enumerate(zip(scan.ranges, angles)):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            
            # 글로벌 각도
            global_angle = pose.theta + angle
            
            # 장애물 위치 (월드 좌표)
            obstacle_x = pose.x + distance * np.cos(global_angle)
            obstacle_y = pose.y + distance * np.sin(global_angle)
            
            # 격자 좌표
            obstacle_gx, obstacle_gy = self.map.world_to_grid(obstacle_x, obstacle_y)
            
            # 레이 트레이싱 (브레젠햄)
            free_cells = self.bresenham(robot_gx, robot_gy, obstacle_gx, obstacle_gy)
            
            # 자유 공간 업데이트
            for gx, gy in free_cells[:-1]:
                if self.map.is_valid(gx, gy):
                    current_value = self.map.data[gy, gx]
                    if current_value == -1:
                        self.map.set_free(gx, gy, 0)
                    elif current_value > 0:
                        self.map.data[gy, gx] = max(0, current_value - 5)
            
            # 장애물 업데이트
            if self.map.is_valid(obstacle_gx, obstacle_gy):
                current_value = self.map.data[obstacle_gy, obstacle_gx]
                if current_value == -1:
                    self.map.set_occupied(obstacle_gx, obstacle_gy, 100)
                else:
                    self.map.data[obstacle_gy, obstacle_gx] = min(100, current_value + 10)
    
    def bresenham(self, x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
        """브레젠햄 알고리즘 (선 그리기)"""
        points = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
            
            e2 = 2 * err
            
            if e2 > -dy:
                err -= dy
                x += sx
            
            if e2 < dx:
                err += dx
                y += sy
        
        return points
```

---

## 3. 경로 계획

### 3.1 A* 전역 경로 계획

**A* Path Planner**
```python
import heapq
from typing import List, Tuple, Optional

class AStarPlanner:
    """A* 경로 계획"""
    
    def __init__(self, occupancy_map: OccupancyGrid):
        self.map = occupancy_map
        
    def plan(self, start: Pose2D, goal: Pose2D) -> Optional[List[Pose2D]]:
        """A* 경로 계획"""
        
        # 격자 좌표로 변환
        start_gx, start_gy = self.map.world_to_grid(start.x, start.y)
        goal_gx, goal_gy = self.map.world_to_grid(goal.x, goal.y)
        
        # 유효성 검사
        if not self.map.is_valid(start_gx, start_gy):
            print(f"시작 위치 무효: ({start_gx}, {start_gy})")
            return None
        
        if not self.map.is_valid(goal_gx, goal_gy):
            print(f"목표 위치 무효: ({goal_gx}, {goal_gy})")
            return None
        
        if self.map.is_occupied(goal_gx, goal_gy):
            print("목표 위치가 점유됨")
            return None
        
        # A* 알고리즘
        open_set = []
        heapq.heappush(open_set, (0, (start_gx, start_gy)))
        
        came_from = {}
        g_score = {(start_gx, start_gy): 0}
        f_score = {(start_gx, start_gy): self.heuristic(start_gx, start_gy, goal_gx, goal_gy)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            # 목표 도달
            if current == (goal_gx, goal_gy):
                # 경로 재구성
                path = self.reconstruct_path(came_from, current)
                
                # 월드 좌표로 변환
                world_path = []
                for gx, gy in path:
                    x, y = self.map.grid_to_world(gx, gy)
                    world_path.append(Pose2D(x, y, 0.0))
                
                # 경로 스무딩
                world_path = self.smooth_path(world_path)
                
                return world_path
            
            # 이웃 탐색
            for neighbor in self.get_neighbors(current[0], current[1]):
                # 점유 체크
                if self.map.is_occupied(neighbor[0], neighbor[1]):
                    continue
                
                # g 점수 계산
                tentative_g = g_score[current] + self.distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(
                        neighbor[0], neighbor[1], goal_gx, goal_gy
                    )
                    
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print("경로를 찾을 수 없음")
        return None
    
    def get_neighbors(self, gx: int, gy: int) -> List[Tuple[int, int]]:
        """이웃 셀 (8방향)"""
        neighbors = []
        
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                
                nx, ny = gx + dx, gy + dy
                
                if self.map.is_valid(nx, ny):
                    neighbors.append((nx, ny))
        
        return neighbors
    
    def heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """휴리스틱 (유클리드 거리)"""
        return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def distance(self, p1: Tuple[int, int], p2: Tuple[int, int]) -> float:
        """두 점 사이 거리"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        return np.sqrt(dx*dx + dy*dy)
    
    def reconstruct_path(self, came_from: dict, 
                        current: Tuple[int, int]) -> List[Tuple[int, int]]:
        """경로 재구성"""
        path = [current]
        
        while current in came_from:
            current = came_from[current]
            path.append(current)
        
        path.reverse()
        return path
    
    def smooth_path(self, path: List[Pose2D]) -> List[Pose2D]:
        """경로 스무딩"""
        if len(path) <= 2:
            return path
        
        smoothed = [path[0]]
        
        i = 0
        while i < len(path) - 1:
            # 직선으로 갈 수 있는 가장 먼 점 찾기
            for j in range(len(path) - 1, i, -1):
                if self.is_line_free(path[i], path[j]):
                    smoothed.append(path[j])
                    i = j
                    break
            else:
                i += 1
        
        return smoothed
    
    def is_line_free(self, p1: Pose2D, p2: Pose2D) -> bool:
        """두 점 사이 직선이 장애물 없는지 확인"""
        gx1, gy1 = self.map.world_to_grid(p1.x, p1.y)
        gx2, gy2 = self.map.world_to_grid(p2.x, p2.y)
        
        # 브레젠햄으로 선 상의 모든 셀 체크
        slam = GridSLAM((100, 100), self.map.resolution)
        line_cells = slam.bresenham(gx1, gy1, gx2, gy2)
        
        for gx, gy in line_cells:
            if self.map.is_occupied(gx, gy):
                return False
        
        return True
```

### 3.2 Dynamic Window Approach (DWA)

**DWA Local Planner**
```python
class DWAPlanner:
    """Dynamic Window Approach 지역 경로 계획"""
    
    def __init__(self):
        # 로봇 파라미터
        self.max_speed = 0.5  # m/s
        self.max_angular_speed = 1.0  # rad/s
        self.max_accel = 0.2  # m/s^2
        self.max_angular_accel = 0.5  # rad/s^2
        
        # DWA 파라미터
        self.dt = 0.1  # 시간 스텝
        self.predict_time = 3.0  # 예측 시간
        self.v_resolution = 0.05  # 속도 해상도
        self.w_resolution = 0.1  # 각속도 해상도
        
        # 비용 함수 가중치
        self.heading_weight = 0.3
        self.distance_weight = 0.1
        self.velocity_weight = 0.6
        
    def plan(self, current_pose: Pose2D, current_vel: Velocity,
             goal: Pose2D, obstacles: np.ndarray) -> Velocity:
        """DWA 경로 계획"""
        
        # Dynamic Window 계산
        dw = self.calculate_dynamic_window(current_vel)
        
        # 모든 가능한 (v, w) 조합 평가
        best_vel = Velocity(0.0, 0.0)
        best_score = -float('inf')
        
        v_samples = np.arange(dw[0], dw[1], self.v_resolution)
        w_samples = np.arange(dw[2], dw[3], self.w_resolution)
        
        for v in v_samples:
            for w in w_samples:
                # 궤적 시뮬레이션
                trajectory = self.simulate_trajectory(current_pose, v, w)
                
                # 충돌 체크
                if self.check_collision(trajectory, obstacles):
                    continue
                
                # 비용 계산
                heading_score = self.heading_cost(trajectory, goal)
                distance_score = self.distance_cost(trajectory, obstacles)
                velocity_score = self.velocity_cost(v)
                
                total_score = (self.heading_weight * heading_score +
                             self.distance_weight * distance_score +
                             self.velocity_weight * velocity_score)
                
                if total_score > best_score:
                    best_score = total_score
                    best_vel = Velocity(v, w)
        
        return best_vel
    
    def calculate_dynamic_window(self, current_vel: Velocity) -> Tuple[float, float, float, float]:
        """Dynamic Window 계산 [v_min, v_max, w_min, w_max]"""
        
        # 속도 제한
        v_min = max(0, current_vel.linear - self.max_accel * self.dt)
        v_max = min(self.max_speed, current_vel.linear + self.max_accel * self.dt)
        
        # 각속도 제한
        w_min = max(-self.max_angular_speed, 
                   current_vel.angular - self.max_angular_accel * self.dt)
        w_max = min(self.max_angular_speed,
                   current_vel.angular + self.max_angular_accel * self.dt)
        
        return (v_min, v_max, w_min, w_max)
    
    def simulate_trajectory(self, pose: Pose2D, v: float, w: float) -> np.ndarray:
        """궤적 시뮬레이션"""
        
        num_steps = int(self.predict_time / self.dt)
        trajectory = np.zeros((num_steps, 3))
        
        x, y, theta = pose.x, pose.y, pose.theta
        
        for i in range(num_steps):
            x += v * np.cos(theta) * self.dt
            y += v * np.sin(theta) * self.dt
            theta += w * self.dt
            
            trajectory[i] = [x, y, theta]
        
        return trajectory
    
    def check_collision(self, trajectory: np.ndarray, 
                       obstacles: np.ndarray, safety_margin: float = 0.3) -> bool:
        """충돌 체크"""
        
        if len(obstacles) == 0:
            return False
        
        for pose in trajectory:
            # 각 장애물과의 거리
            distances = np.linalg.norm(obstacles[:, :2] - pose[:2], axis=1)
            
            if np.any(distances < safety_margin):
                return True
        
        return False
    
    def heading_cost(self, trajectory: np.ndarray, goal: Pose2D) -> float:
        """목표 방향 비용 (높을수록 좋음)"""
        
        # 마지막 포즈
        final_pose = trajectory[-1]
        
        # 목표 방향
        dx = goal.x - final_pose[0]
        dy = goal.y - final_pose[1]
        goal_angle = np.arctan2(dy, dx)
        
        # 각도 차이
        angle_diff = abs(self.normalize_angle(goal_angle - final_pose[2]))
        
        # 0~1 스케일 (각도 차이가 작을수록 높음)
        return 1.0 - angle_diff / np.pi
    
    def distance_cost(self, trajectory: np.ndarray, obstacles: np.ndarray) -> float:
        """장애물 거리 비용 (높을수록 좋음)"""
        
        if len(obstacles) == 0:
            return 1.0
        
        min_distance = float('inf')
        
        for pose in trajectory:
            distances = np.linalg.norm(obstacles[:, :2] - pose[:2], axis=1)
            min_distance = min(min_distance, np.min(distances))
        
        # 0~1 스케일
        return min(1.0, min_distance / 2.0)
    
    def velocity_cost(self, v: float) -> float:
        """속도 비용 (빠를수록 좋음)"""
        return v / self.max_speed
    
    def normalize_angle(self, angle: float) -> float:
        """각도 정규화 [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
```

---

## 4. 통합 네비게이션 시스템

### 4.1 자율 네비게이션 컨트롤러

**Navigation Controller**
```python
import time

class NavigationController:
    """자율 네비게이션 통합 시스템"""
    
    def __init__(self, robot, lidar):
        self.robot = robot
        self.lidar = lidar
        
        # SLAM
        self.slam = GridSLAM(map_size=(200, 200), resolution=0.05)
        
        # 경로 계획
        self.global_planner = None  # SLAM 후 초기화
        self.local_planner = DWAPlanner()
        
        # 상태
        self.state = NavigationState.IDLE
        self.current_goal = None
        self.global_path = None
        
        # 제어
        self.control_rate = 10  # Hz
        self.goal_tolerance = 0.2  # m
        
    def navigate_to(self, goal: Pose2D) -> bool:
        """목표 위치로 네비게이션"""
        
        print(f"🎯 목표: ({goal.x:.2f}, {goal.y:.2f})")
        
        self.current_goal = goal
        self.state = NavigationState.PLANNING
        
        # 전역 경로 계획
        print("📍 전역 경로 계획 중...")
        self.global_planner = AStarPlanner(self.slam.map)
        
        current_pose = self.robot.get_pose()
        self.global_path = self.global_planner.plan(current_pose, goal)
        
        if self.global_path is None:
            print("❌ 경로 계획 실패")
            self.state = NavigationState.FAILED
            return False
        
        print(f"✓ 경로 길이: {len(self.global_path)} 웨이포인트")
        
        # 경로 추종
        self.state = NavigationState.NAVIGATING
        
        rate = 1.0 / self.control_rate
        stuck_counter = 0
        
        while True:
            loop_start = time.time()
            
            # 현재 상태
            current_pose = self.robot.get_pose()
            current_vel = self.robot.get_velocity()
            
            # SLAM 업데이트
            scan = self.lidar.get_scan()
            self.slam.update(scan, current_pose)
            
            # 목표 도달 확인
            if current_pose.distance_to(goal) < self.goal_tolerance:
                print("✅ 목표 도달!")
                self.robot.stop()
                self.state = NavigationState.REACHED
                return True
            
            # 장애물 추출
            obstacles = self.extract_obstacles(scan, current_pose)
            
            # 지역 목표 (전역 경로 상의 다음 포인트)
            local_goal = self.get_local_goal(current_pose, self.global_path)
            
            # DWA 지역 경로 계획
            cmd_vel = self.local_planner.plan(
                current_pose, current_vel, local_goal, obstacles
            )
            
            # 속도 명령 전송
            self.robot.set_velocity(cmd_vel.linear, cmd_vel.angular)
            
            # 정체 감지
            if cmd_vel.linear < 0.01 and cmd_vel.angular < 0.01:
                stuck_counter += 1
                
                if stuck_counter > 30:  # 3초
                    print("⚠️ 정체 감지, 복구 동작...")
                    self.recovery_behavior()
                    stuck_counter = 0
            else:
                stuck_counter = 0
            
            # 주기 유지
            elapsed = time.time() - loop_start
            if elapsed < rate:
                time.sleep(rate - elapsed)
        
        return False
    
    def extract_obstacles(self, scan: LaserScan, pose: Pose2D) -> np.ndarray:
        """스캔에서 장애물 추출"""
        
        obstacles = []
        angles = scan.get_angles()
        
        for distance, angle in zip(scan.ranges, angles):
            if distance < scan.range_min or distance > scan.range_max:
                continue
            
            # 글로벌 좌표
            global_angle = pose.theta + angle
            x = pose.x + distance * np.cos(global_angle)
            y = pose.y + distance * np.sin(global_angle)
            
            obstacles.append([x, y])
        
        return np.array(obstacles) if obstacles else np.array([]).reshape(0, 2)
    
    def get_local_goal(self, current_pose: Pose2D, 
                      global_path: List[Pose2D], lookahead: float = 1.0) -> Pose2D:
        """전역 경로에서 지역 목표 추출"""
        
        # 현재 위치에서 lookahead 거리만큼 떨어진 경로 상의 점
        for i, waypoint in enumerate(global_path):
            if current_pose.distance_to(waypoint) > lookahead:
                return waypoint
        
        # 경로 끝
        return global_path[-1]
    
    def recovery_behavior(self):
        """복구 동작"""
        
        print("🔄 복구: 제자리 회전")
        
        # 360도 회전하며 지도 업데이트
        for _ in range(36):
            self.robot.set_velocity(0.0, 0.5)  # 제자리 회전
            time.sleep(0.1)
            
            # SLAM 업데이트
            scan = self.lidar.get_scan()
            pose = self.robot.get_pose()
            self.slam.update(scan, pose)
        
        self.robot.stop()
        
        # 경로 재계획
        print("📍 경로 재계획...")
        self.global_planner = AStarPlanner(self.slam.map)
        current_pose = self.robot.get_pose()
        self.global_path = self.global_planner.plan(current_pose, self.current_goal)
    
    def visualize(self):
        """현재 상태 시각화"""
        
        current_pose = self.robot.get_pose()
        
        # 격자 경로 변환
        grid_path = None
        if self.global_path:
            grid_path = [
                self.slam.map.world_to_grid(p.x, p.y) 
                for p in self.global_path
            ]
        
        self.slam.map.visualize(current_pose, grid_path)
```

---

## 5. 실습 예제

### 5.1 완전 자율 네비게이션

**메인 데모**
```python
def main_navigation_demo():
    """자율 네비게이션 데모"""
    
    print("=" * 60)
    print("  XLeRobot 자율 네비게이션 시스템")
    print("=" * 60)
    
    # 로봇 및 센서 초기화
    robot = XLeRobotMobile()
    lidar = SimulatedLidar()
    
    # 네비게이션 컨트롤러
    nav = NavigationController(robot, lidar)
    
    # 초기 SLAM (환경 스캔)
    print("\n🗺️  환경 매핑 중...")
    
    for i in range(100):
        # 천천히 회전하며 스캔
        robot.set_velocity(0.1, 0.3)
        time.sleep(0.1)
        
        scan = lidar.get_scan()
        pose = robot.get_pose()
        nav.slam.update(scan, pose)
        
        if i % 20 == 0:
            print(f"  {i}% 완료...")
    
    robot.stop()
    print("✓ 초기 지도 생성 완료")
    
    # 지도 시각화
    nav.visualize()
    
    # 목표 위치들
    waypoints = [
        Pose2D(2.0, 0.0, 0.0),
        Pose2D(2.0, 2.0, np.pi/2),
        Pose2D(0.0, 2.0, np.pi),
        Pose2D(0.0, 0.0, -np.pi/2),
    ]
    
    # 각 웨이포인트로 네비게이션
    for i, goal in enumerate(waypoints):
        print(f"\n{'='*60}")
        print(f"  웨이포인트 {i+1}/{len(waypoints)}")
        print(f"{'='*60}")
        
        success = nav.navigate_to(goal)
        
        if success:
            print(f"✅ 웨이포인트 {i+1} 도달")
        else:
            print(f"❌ 웨이포인트 {i+1} 실패")
            break
        
        time.sleep(2)
    
    # 최종 지도
    print("\n🗺️  최종 지도")
    nav.visualize()
    
    print("\n" + "="*60)
    print("  네비게이션 완료!")
    print("="*60)

if __name__ == "__main__":
    main_navigation_demo()
```

### 5.2 대화형 네비게이션

**Interactive Navigation**
```python
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

def interactive_navigation():
    """마우스 클릭으로 목표 설정"""
    
    robot = XLeRobotMobile()
    lidar = SimulatedLidar()
    nav = NavigationController(robot, lidar)
    
    # 초기 매핑
    print("환경 매핑 중...")
    for _ in range(50):
        robot.set_velocity(0.1, 0.2)
        time.sleep(0.1)
        scan = lidar.get_scan()
        pose = robot.get_pose()
        nav.slam.update(scan, pose)
    
    robot.stop()
    
    # 대화형 플롯
    fig, ax = plt.subplots(figsize=(10, 10))
    
    def onclick(event):
        """마우스 클릭 이벤트"""
        if event.xdata is not None and event.ydata is not None:
            goal = Pose2D(event.xdata, event.ydata, 0.0)
            
            print(f"\n🎯 새 목표: ({goal.x:.2f}, {goal.y:.2f})")
            
            # 목표 표시
            circle = Circle((goal.x, goal.y), 0.1, color='red', fill=True)
            ax.add_patch(circle)
            plt.draw()
            
            # 네비게이션 (별도 스레드에서)
            import threading
            thread = threading.Thread(target=nav.navigate_to, args=(goal,))
            thread.start()
    
    fig.canvas.mpl_connect('button_press_event', onclick)
    
    # 실시간 업데이트
    while True:
        ax.clear()
        
        # 지도 표시
        map_data = nav.slam.map.data.copy().astype(float)
        map_data[map_data == -1] = 50
        
        ax.imshow(map_data, cmap='gray_r', origin='lower',
                 extent=[nav.slam.map.origin.x,
                        nav.slam.map.origin.x + nav.slam.map.width * nav.slam.map.resolution,
                        nav.slam.map.origin.y,
                        nav.slam.map.origin.y + nav.slam.map.height * nav.slam.map.resolution])
        
        # 로봇 위치
        pose = robot.get_pose()
        ax.plot(pose.x, pose.y, 'ro', markersize=10)
        
        # 방향
        arrow_len = 0.3
        ax.arrow(pose.x, pose.y, 
                arrow_len * np.cos(pose.theta),
                arrow_len * np.sin(pose.theta),
                head_width=0.1, color='red')
        
        # 경로
        if nav.global_path:
            path_x = [p.x for p in nav.global_path]
            path_y = [p.y for p in nav.global_path]
            ax.plot(path_x, path_y, 'b-', linewidth=2)
        
        plt.pause(0.1)
```

---

## ✅ 프로젝트 4 완료 체크리스트

- [ ] LiDAR 데이터 처리
- [ ] Grid-based SLAM 구현
- [ ] A* 전역 경로 계획
- [ ] DWA 지역 경로 계획
- [ ] 장애물 감지 및 회피
- [ ] 자율 네비게이션 통합
- [ ] 복구 동작 구현
- [ ] 대화형 네비게이션 테스트

## 🎓 학습 정리

1. **SLAM**: 동시적 위치추정 및 지도 작성
2. **전역 계획**: A* 알고리즘으로 최적 경로 찾기
3. **지역 계획**: DWA로 동적 장애물 회피
4. **센서 융합**: LiDAR + 오도메트리 통합
5. **시스템 통합**: 완전 자율 네비게이션 파이프라인

---

[← 8.3 픽앤플레이스](03_pick_and_place.md) | [다음: 8.5 가정용 작업 →](05_household_tasks.md)
