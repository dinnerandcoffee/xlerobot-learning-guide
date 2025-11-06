# 8.5 프로젝트 5: 가정용 작업 자동화

복잡한 가정용 작업을 수행하는 통합 로봇 시스템 구현 프로젝트입니다.

## 🎯 프로젝트 목표

- 테이블 청소 자동화
- 물체 정리 및 분류
- 주방 보조 작업
- 복잡한 작업 시퀀스 실행
- 상황 인식 및 의사 결정
- 실패 처리 및 복구

**난이도**: ⭐⭐⭐⭐⭐ (마스터)  
**소요 시간**: 3일  
**선수 지식**: 1-7장, 프로젝트 1-4

---

## 1. 시스템 아키텍처

### 1.1 가정용 로봇 통합 시스템

```
┌─────────────────────────────────────────────────────────┐
│           가정용 작업 자동화 시스템                       │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  인지 레이어 (Perception)                                │
│  ├─ 환경 이해 (Scene Understanding)                     │
│  ├─ 물체 인식 및 분류                                    │
│  ├─ 공간 매핑                                           │
│  └─ 상태 모니터링                                       │
│                                                         │
│  작업 레이어 (Task Planning)                             │
│  ├─ 작업 분해 (Task Decomposition)                      │
│  ├─ 우선순위 관리                                       │
│  ├─ 스케줄링                                            │
│  └─ 의사 결정                                           │
│                                                         │
│  실행 레이어 (Execution)                                 │
│  ├─ 네비게이션 (프로젝트 4)                             │
│  ├─ 조작 (프로젝트 3)                                   │
│  ├─ 인간-로봇 상호작용                                  │
│  └─ 안전 제어                                           │
│                                                         │
│  학습 레이어 (Learning)                                  │
│  ├─ 경험 기반 개선                                      │
│  ├─ 사용자 선호도 학습                                  │
│  └─ 적응적 행동                                         │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

### 1.2 핵심 데이터 구조

**작업 관리 시스템**
```python
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Callable
from enum import Enum
import time

class TaskStatus(Enum):
    """작업 상태"""
    PENDING = 0      # 대기 중
    IN_PROGRESS = 1  # 진행 중
    COMPLETED = 2    # 완료
    FAILED = 3       # 실패
    CANCELLED = 4    # 취소됨

class TaskPriority(Enum):
    """작업 우선순위"""
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    URGENT = 3

@dataclass
class Task:
    """작업 정의"""
    task_id: str
    name: str
    description: str
    priority: TaskPriority
    estimated_duration: float  # seconds
    prerequisites: List[str] = field(default_factory=list)
    subtasks: List['Task'] = field(default_factory=list)
    execution_function: Optional[Callable] = None
    status: TaskStatus = TaskStatus.PENDING
    progress: float = 0.0  # 0.0 ~ 1.0
    retry_count: int = 0
    max_retries: int = 3
    
    def execute(self, robot, context: Dict):
        """작업 실행"""
        if self.execution_function is None:
            raise ValueError(f"실행 함수가 정의되지 않음: {self.name}")
        
        self.status = TaskStatus.IN_PROGRESS
        
        try:
            result = self.execution_function(robot, context)
            
            if result:
                self.status = TaskStatus.COMPLETED
                self.progress = 1.0
                return True
            else:
                self.status = TaskStatus.FAILED
                return False
                
        except Exception as e:
            print(f"❌ 작업 실패: {self.name} - {e}")
            self.status = TaskStatus.FAILED
            return False

@dataclass
class SceneObject:
    """장면 내 물체"""
    object_id: str
    class_name: str
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray
    dimensions: np.ndarray  # [width, height, depth]
    confidence: float
    is_movable: bool = True
    category: str = "unknown"  # tableware, food, cleaning, etc.
    
@dataclass
class WorkArea:
    """작업 영역"""
    name: str
    bounds: np.ndarray  # [x_min, x_max, y_min, y_max, z_min, z_max]
    objects: List[SceneObject] = field(default_factory=list)
    is_accessible: bool = True
    purpose: str = "general"  # dining, kitchen, storage, etc.

class HouseholdRobotSystem:
    """가정용 로봇 통합 시스템"""
    
    def __init__(self, robot, sensors):
        self.robot = robot
        self.sensors = sensors
        
        # 하위 시스템들
        from project_3 import PickAndPlaceSystem
        from project_4 import NavigationController
        
        self.manipulation = PickAndPlaceSystem(robot, sensors['camera'])
        self.navigation = NavigationController(robot, sensors['lidar'])
        
        # 작업 관리
        self.task_queue: List[Task] = []
        self.completed_tasks: List[Task] = []
        self.current_task: Optional[Task] = None
        
        # 환경 모델
        self.work_areas: Dict[str, WorkArea] = {}
        self.known_objects: Dict[str, SceneObject] = {}
        
        # 상태
        self.is_busy = False
        self.battery_level = 100.0
        self.error_count = 0
        
    def add_task(self, task: Task):
        """작업 추가"""
        self.task_queue.append(task)
        
        # 우선순위로 정렬
        self.task_queue.sort(key=lambda t: t.priority.value, reverse=True)
        
        print(f"📋 작업 추가: {task.name} (우선순위: {task.priority.name})")
    
    def execute_tasks(self):
        """작업 큐 실행"""
        print("🤖 작업 실행 시작")
        
        while self.task_queue:
            # 배터리 체크
            if self.battery_level < 20:
                print("🔋 배터리 부족, 충전 필요")
                self.go_to_charging_station()
                break
            
            # 다음 작업
            task = self.task_queue.pop(0)
            self.current_task = task
            
            print(f"\n{'='*60}")
            print(f"  실행: {task.name}")
            print(f"  우선순위: {task.priority.name}")
            print(f"  예상 시간: {task.estimated_duration:.1f}초")
            print(f"{'='*60}")
            
            # 전제 조건 확인
            if not self.check_prerequisites(task):
                print(f"⚠️ 전제 조건 미충족: {task.name}")
                task.status = TaskStatus.FAILED
                continue
            
            # 작업 실행
            success = self.execute_single_task(task)
            
            if success:
                print(f"✅ 완료: {task.name}")
                self.completed_tasks.append(task)
            else:
                print(f"❌ 실패: {task.name}")
                
                # 재시도
                if task.retry_count < task.max_retries:
                    task.retry_count += 1
                    print(f"🔄 재시도 {task.retry_count}/{task.max_retries}")
                    self.task_queue.insert(0, task)  # 앞에 다시 추가
                else:
                    print(f"❌ 최대 재시도 횟수 초과")
                    task.status = TaskStatus.FAILED
        
        self.current_task = None
        print("\n✅ 모든 작업 완료!")
        
        # 요약
        self.print_summary()
    
    def execute_single_task(self, task: Task) -> bool:
        """단일 작업 실행"""
        
        # 서브태스크가 있으면 순차 실행
        if task.subtasks:
            for i, subtask in enumerate(task.subtasks):
                print(f"  🔸 서브태스크 {i+1}/{len(task.subtasks)}: {subtask.name}")
                
                success = self.execute_single_task(subtask)
                
                if not success:
                    return False
                
                # 진행률 업데이트
                task.progress = (i + 1) / len(task.subtasks)
            
            task.status = TaskStatus.COMPLETED
            return True
        
        # 단일 작업 실행
        context = {
            'work_areas': self.work_areas,
            'known_objects': self.known_objects,
            'robot': self.robot,
            'sensors': self.sensors
        }
        
        return task.execute(self.robot, context)
    
    def check_prerequisites(self, task: Task) -> bool:
        """전제 조건 확인"""
        for prereq_id in task.prerequisites:
            # 완료된 작업 확인
            completed_ids = [t.task_id for t in self.completed_tasks]
            
            if prereq_id not in completed_ids:
                return False
        
        return True
    
    def print_summary(self):
        """작업 요약 출력"""
        print("\n" + "="*60)
        print("  작업 요약")
        print("="*60)
        
        completed = len([t for t in self.completed_tasks if t.status == TaskStatus.COMPLETED])
        failed = len([t for t in self.completed_tasks if t.status == TaskStatus.FAILED])
        
        print(f"완료: {completed}")
        print(f"실패: {failed}")
        print(f"배터리: {self.battery_level:.1f}%")
        print("="*60)
    
    def go_to_charging_station(self):
        """충전소로 이동"""
        print("🔌 충전소로 이동 중...")
        # 구현 생략
```

---

## 2. 작업 1: 테이블 청소

### 2.1 테이블 청소 시스템

**Table Cleaning Task**
```python
class TableCleaningTask:
    """테이블 청소 작업"""
    
    def __init__(self, robot_system: HouseholdRobotSystem):
        self.system = robot_system
        self.robot = robot_system.robot
        
    def create_task(self, table_area: WorkArea) -> Task:
        """테이블 청소 작업 생성"""
        
        # 서브태스크들
        subtasks = [
            Task(
                task_id="detect_items",
                name="테이블 위 물체 감지",
                description="테이블 위의 모든 물체를 감지하고 분류",
                priority=TaskPriority.HIGH,
                estimated_duration=10.0,
                execution_function=self.detect_table_items
            ),
            Task(
                task_id="classify_items",
                name="물체 분류",
                description="물체를 유지/제거/재배치로 분류",
                priority=TaskPriority.MEDIUM,
                estimated_duration=5.0,
                execution_function=self.classify_items
            ),
            Task(
                task_id="remove_trash",
                name="쓰레기 제거",
                description="쓰레기를 쓰레기통으로 이동",
                priority=TaskPriority.HIGH,
                estimated_duration=30.0,
                execution_function=self.remove_trash_items
            ),
            Task(
                task_id="organize_items",
                name="물체 정리",
                description="물체를 적절한 위치로 이동",
                priority=TaskPriority.MEDIUM,
                estimated_duration=60.0,
                execution_function=self.organize_items
            ),
            Task(
                task_id="wipe_table",
                name="테이블 닦기",
                description="테이블 표면을 닦기",
                priority=TaskPriority.LOW,
                estimated_duration=20.0,
                execution_function=self.wipe_table_surface
            ),
        ]
        
        main_task = Task(
            task_id="clean_table",
            name="테이블 청소",
            description=f"{table_area.name} 완전 청소",
            priority=TaskPriority.HIGH,
            estimated_duration=125.0,
            subtasks=subtasks
        )
        
        return main_task
    
    def detect_table_items(self, robot, context: Dict) -> bool:
        """테이블 위 물체 감지"""
        print("  📷 물체 감지 중...")
        
        # 카메라 이미지 획득
        rgb = context['sensors']['camera'].get_rgb()
        depth = context['sensors']['camera'].get_depth()
        
        # 물체 감지
        detections = self.system.manipulation.detector.detect(rgb)
        
        # 3D 포즈 추정
        detected_objects = []
        
        for det in detections:
            obj_3d = self.system.manipulation.pose_estimator.estimate_3d_pose(
                det, depth
            )
            
            # 테이블 위에 있는지 확인
            table_height = 0.75  # 75cm
            if abs(obj_3d.position_3d[2] - table_height) < 0.1:
                scene_obj = SceneObject(
                    object_id=f"obj_{len(detected_objects)}",
                    class_name=obj_3d.class_name,
                    position=obj_3d.position_3d,
                    orientation=obj_3d.orientation_3d,
                    dimensions=obj_3d.dimensions,
                    confidence=obj_3d.confidence
                )
                detected_objects.append(scene_obj)
        
        # 컨텍스트에 저장
        context['detected_objects'] = detected_objects
        
        print(f"  ✓ {len(detected_objects)}개 물체 발견")
        
        return True
    
    def classify_items(self, robot, context: Dict) -> bool:
        """물체 분류"""
        print("  🏷️  물체 분류 중...")
        
        detected_objects = context.get('detected_objects', [])
        
        # 분류 규칙
        trash_items = ['bottle', 'cup', 'wrapper']
        keep_items = ['laptop', 'book', 'phone']
        organize_items = ['remote', 'pen', 'notebook']
        
        classified = {
            'trash': [],
            'keep': [],
            'organize': []
        }
        
        for obj in detected_objects:
            if obj.class_name in trash_items:
                obj.category = 'trash'
                classified['trash'].append(obj)
            elif obj.class_name in keep_items:
                obj.category = 'keep'
                classified['keep'].append(obj)
            else:
                obj.category = 'organize'
                classified['organize'].append(obj)
        
        context['classified_objects'] = classified
        
        print(f"  ✓ 쓰레기: {len(classified['trash'])}, "
              f"보관: {len(classified['keep'])}, "
              f"정리: {len(classified['organize'])}")
        
        return True
    
    def remove_trash_items(self, robot, context: Dict) -> bool:
        """쓰레기 제거"""
        print("  🗑️  쓰레기 제거 중...")
        
        classified = context.get('classified_objects', {})
        trash_items = classified.get('trash', [])
        
        # 쓰레기통 위치
        trash_bin_position = np.array([1.5, -1.0, 0.3])
        
        for i, obj in enumerate(trash_items):
            print(f"    {i+1}/{len(trash_items)}: {obj.class_name}")
            
            # 픽앤플레이스
            success = self.system.manipulation.execute_pick_and_place(
                obj.class_name,
                trash_bin_position
            )
            
            if not success:
                print(f"    ⚠️ 실패: {obj.class_name}")
                continue
            
            time.sleep(1)
        
        print(f"  ✓ {len(trash_items)}개 쓰레기 제거 완료")
        
        return True
    
    def organize_items(self, robot, context: Dict) -> bool:
        """물체 정리"""
        print("  📦 물체 정리 중...")
        
        classified = context.get('classified_objects', {})
        organize_items = classified.get('organize', [])
        
        # 정리 위치 (테이블 한쪽)
        organize_positions = [
            np.array([0.8, 0.3, 0.75]),
            np.array([0.8, 0.4, 0.75]),
            np.array([0.9, 0.3, 0.75]),
            np.array([0.9, 0.4, 0.75]),
        ]
        
        for i, obj in enumerate(organize_items):
            if i >= len(organize_positions):
                break
            
            print(f"    {i+1}/{len(organize_items)}: {obj.class_name}")
            
            # 목표 위치
            target_pos = organize_positions[i]
            
            # 픽앤플레이스
            success = self.system.manipulation.execute_pick_and_place(
                obj.class_name,
                target_pos
            )
            
            if not success:
                print(f"    ⚠️ 실패: {obj.class_name}")
                continue
            
            time.sleep(1)
        
        print(f"  ✓ {len(organize_items)}개 물체 정리 완료")
        
        return True
    
    def wipe_table_surface(self, robot, context: Dict) -> bool:
        """테이블 표면 닦기"""
        print("  🧹 테이블 닦는 중...")
        
        # 천 또는 스펀지 잡기
        cloth_position = np.array([1.0, 0.5, 0.75])
        
        # 그리퍼로 천 잡기 (간단한 버전)
        robot.move_to_position(cloth_position)
        robot.close_gripper()
        
        # 지그재그 패턴으로 닦기
        table_area = [
            (0.3, 0.0), (0.7, 0.0),
            (0.7, 0.2), (0.3, 0.2),
            (0.3, 0.4), (0.7, 0.4),
        ]
        
        table_height = 0.76  # 테이블보다 살짝 높게
        
        for x, y in table_area:
            position = np.array([x, y, table_height])
            robot.move_to_position(position)
            time.sleep(0.5)
        
        # 천 놓기
        robot.open_gripper()
        
        print("  ✓ 테이블 청소 완료")
        
        return True
```

---

## 3. 작업 2: 물체 정리 및 분류

### 3.1 지능형 물체 정리

**Object Sorting System**
```python
class ObjectSortingTask:
    """물체 정리 및 분류 작업"""
    
    def __init__(self, robot_system: HouseholdRobotSystem):
        self.system = robot_system
        self.robot = robot_system.robot
        
        # 카테고리별 저장 위치
        self.storage_locations = {
            'books': np.array([2.0, 1.0, 0.5]),
            'electronics': np.array([2.0, 0.5, 0.5]),
            'kitchen': np.array([1.5, 0.0, 0.5]),
            'toys': np.array([2.0, 1.5, 0.3]),
            'clothes': np.array([1.8, 1.0, 0.7]),
            'tools': np.array([2.2, 0.5, 0.4]),
        }
        
    def create_task(self) -> Task:
        """물체 정리 작업 생성"""
        
        return Task(
            task_id="sort_objects",
            name="물체 정리 및 분류",
            description="방에 흩어진 물체들을 카테고리별로 정리",
            priority=TaskPriority.MEDIUM,
            estimated_duration=180.0,
            execution_function=self.execute_sorting
        )
    
    def execute_sorting(self, robot, context: Dict) -> bool:
        """물체 정리 실행"""
        print("  🔍 물체 스캔 중...")
        
        # 1. 방 전체 스캔
        objects = self.scan_room(context)
        
        print(f"  ✓ {len(objects)}개 물체 발견")
        
        # 2. 카테고리별 분류
        categorized = self.categorize_objects(objects)
        
        # 3. 각 카테고리별로 정리
        for category, items in categorized.items():
            if not items:
                continue
            
            print(f"\n  📦 {category} 정리 중... ({len(items)}개)")
            
            target_location = self.storage_locations.get(
                category, 
                np.array([2.0, 0.0, 0.5])  # 기본 위치
            )
            
            for i, obj in enumerate(items):
                print(f"    {i+1}/{len(items)}: {obj.class_name}")
                
                # 약간씩 위치 조정 (겹치지 않게)
                offset = np.array([0, i * 0.1, 0])
                place_pos = target_location + offset
                
                # 물체 잡고 이동
                success = self.system.manipulation.execute_pick_and_place(
                    obj.class_name,
                    place_pos
                )
                
                if not success:
                    print(f"    ⚠️ 실패: {obj.class_name}")
                    continue
                
                time.sleep(0.5)
        
        print("  ✅ 물체 정리 완료")
        
        return True
    
    def scan_room(self, context: Dict) -> List[SceneObject]:
        """방 전체 스캔"""
        
        # 여러 위치에서 스캔
        scan_positions = [
            Pose2D(0.0, 0.0, 0.0),
            Pose2D(1.0, 1.0, np.pi/2),
            Pose2D(2.0, 0.0, np.pi),
            Pose2D(1.0, -1.0, -np.pi/2),
        ]
        
        all_objects = []
        
        for pos in scan_positions:
            # 위치로 이동
            self.system.navigation.navigate_to(pos)
            
            # 물체 감지
            rgb = context['sensors']['camera'].get_rgb()
            depth = context['sensors']['camera'].get_depth()
            
            detections = self.system.manipulation.detector.detect(rgb)
            
            for det in detections:
                obj = self.system.manipulation.pose_estimator.estimate_3d_pose(
                    det, depth
                )
                
                scene_obj = SceneObject(
                    object_id=f"obj_{len(all_objects)}",
                    class_name=obj.class_name,
                    position=obj.position_3d,
                    orientation=obj.orientation_3d,
                    dimensions=obj.dimensions,
                    confidence=obj.confidence
                )
                
                all_objects.append(scene_obj)
        
        # 중복 제거 (같은 위치의 물체)
        unique_objects = self.remove_duplicates(all_objects)
        
        return unique_objects
    
    def remove_duplicates(self, objects: List[SceneObject], 
                         threshold: float = 0.2) -> List[SceneObject]:
        """중복 물체 제거"""
        
        unique = []
        
        for obj in objects:
            is_duplicate = False
            
            for unique_obj in unique:
                distance = np.linalg.norm(obj.position - unique_obj.position)
                
                if distance < threshold and obj.class_name == unique_obj.class_name:
                    is_duplicate = True
                    break
            
            if not is_duplicate:
                unique.append(obj)
        
        return unique
    
    def categorize_objects(self, objects: List[SceneObject]) -> Dict[str, List[SceneObject]]:
        """물체 카테고리 분류"""
        
        # 분류 규칙
        category_mapping = {
            'book': 'books',
            'laptop': 'electronics',
            'phone': 'electronics',
            'remote': 'electronics',
            'bottle': 'kitchen',
            'cup': 'kitchen',
            'bowl': 'kitchen',
            'teddy bear': 'toys',
            'frisbee': 'toys',
        }
        
        categorized = {cat: [] for cat in self.storage_locations.keys()}
        categorized['misc'] = []  # 기타
        
        for obj in objects:
            category = category_mapping.get(obj.class_name, 'misc')
            obj.category = category
            categorized[category].append(obj)
        
        return categorized
```

---

## 4. 작업 3: 주방 보조

### 4.1 주방 작업 시스템

**Kitchen Assistant**
```python
class KitchenAssistantTask:
    """주방 보조 작업"""
    
    def __init__(self, robot_system: HouseholdRobotSystem):
        self.system = robot_system
        self.robot = robot_system.robot
        
    def create_dishwashing_task(self) -> Task:
        """설거지 보조 작업"""
        
        subtasks = [
            Task(
                task_id="collect_dishes",
                name="식기 수집",
                description="테이블에서 사용한 식기 수집",
                priority=TaskPriority.HIGH,
                estimated_duration=60.0,
                execution_function=self.collect_dishes
            ),
            Task(
                task_id="load_dishwasher",
                name="식기세척기 적재",
                description="식기를 식기세척기에 배치",
                priority=TaskPriority.MEDIUM,
                estimated_duration=90.0,
                execution_function=self.load_dishwasher
            ),
        ]
        
        return Task(
            task_id="dishwashing",
            name="설거지 보조",
            description="식사 후 설거지 작업",
            priority=TaskPriority.MEDIUM,
            estimated_duration=150.0,
            subtasks=subtasks
        )
    
    def create_meal_prep_task(self) -> Task:
        """식사 준비 보조"""
        
        subtasks = [
            Task(
                task_id="get_ingredients",
                name="재료 가져오기",
                description="필요한 재료를 냉장고/선반에서 가져오기",
                priority=TaskPriority.HIGH,
                estimated_duration=120.0,
                execution_function=self.get_ingredients
            ),
            Task(
                task_id="arrange_workspace",
                name="작업 공간 정리",
                description="조리대 위 정리 및 도구 배치",
                priority=TaskPriority.MEDIUM,
                estimated_duration=60.0,
                execution_function=self.arrange_workspace
            ),
        ]
        
        return Task(
            task_id="meal_prep",
            name="식사 준비 보조",
            description="요리를 위한 준비 작업",
            priority=TaskPriority.HIGH,
            estimated_duration=180.0,
            subtasks=subtasks
        )
    
    def collect_dishes(self, robot, context: Dict) -> bool:
        """식기 수집"""
        print("  🍽️  식기 수집 중...")
        
        # 식탁 위 식기 감지
        rgb = context['sensors']['camera'].get_rgb()
        depth = context['sensors']['camera'].get_depth()
        
        detections = self.system.manipulation.detector.detect(rgb)
        
        # 식기 필터링
        dish_classes = ['cup', 'bowl', 'plate', 'fork', 'knife', 'spoon']
        dishes = []
        
        for det in detections:
            if det['class_name'] in dish_classes:
                obj = self.system.manipulation.pose_estimator.estimate_3d_pose(
                    det, depth
                )
                dishes.append(obj)
        
        print(f"  ✓ {len(dishes)}개 식기 발견")
        
        # 싱크대 위치
        sink_position = np.array([1.8, 0.0, 0.85])
        
        # 각 식기를 싱크대로 이동
        for i, dish in enumerate(dishes):
            print(f"    {i+1}/{len(dishes)}: {dish.class_name}")
            
            # 쌓기 (높이 조정)
            place_pos = sink_position.copy()
            place_pos[2] += i * 0.05  # 5cm씩 높이 증가
            
            success = self.system.manipulation.execute_pick_and_place(
                dish.class_name,
                place_pos
            )
            
            if not success:
                print(f"    ⚠️ 실패: {dish.class_name}")
            
            time.sleep(1)
        
        print("  ✅ 식기 수집 완료")
        
        return True
    
    def load_dishwasher(self, robot, context: Dict) -> bool:
        """식기세척기 적재"""
        print("  �� 식기세척기 적재 중...")
        
        # 싱크대의 식기들
        sink_position = np.array([1.8, 0.0, 0.85])
        
        # 식기세척기 랙 위치들
        dishwasher_positions = [
            np.array([2.0, -0.3, 0.6]),  # 아래 랙
            np.array([2.0, -0.2, 0.6]),
            np.array([2.0, -0.1, 0.6]),
            np.array([2.0, 0.0, 0.6]),
            np.array([2.0, -0.3, 0.9]),  # 위 랙
            np.array([2.0, -0.2, 0.9]),
        ]
        
        # 식기 이동
        for i, target_pos in enumerate(dishwasher_positions):
            print(f"    {i+1}/{len(dishwasher_positions)} 위치")
            
            # 싱크대에서 잡기
            pick_pos = sink_position.copy()
            pick_pos[2] += (len(dishwasher_positions) - i - 1) * 0.05
            
            robot.move_to_position(pick_pos)
            robot.close_gripper()
            
            # 식기세척기로 이동
            robot.move_to_position(target_pos)
            robot.open_gripper()
            
            time.sleep(1)
        
        print("  ✅ 식기세척기 적재 완료")
        
        return True
    
    def get_ingredients(self, robot, context: Dict) -> bool:
        """재료 가져오기"""
        print("  �� 재료 가져오는 중...")
        
        # 필요한 재료 리스트 (예시)
        ingredients = [
            {'name': 'apple', 'location': np.array([2.5, 1.0, 1.2])},  # 냉장고
            {'name': 'bottle', 'location': np.array([2.5, 1.1, 1.2])},
            {'name': 'bowl', 'location': np.array([2.2, 0.5, 0.8])},   # 선반
        ]
        
        # 조리대 위치
        counter_position = np.array([1.5, 0.0, 0.85])
        
        for i, ingredient in enumerate(ingredients):
            print(f"    {i+1}/{len(ingredients)}: {ingredient['name']}")
            
            # 재료 위치로 이동
            self.system.navigation.navigate_to(
                Pose2D(ingredient['location'][0], 
                      ingredient['location'][1], 
                      0.0)
            )
            
            # 재료 잡기
            robot.move_to_position(ingredient['location'])
            robot.close_gripper()
            
            # 조리대로 이동
            place_pos = counter_position.copy()
            place_pos[0] += i * 0.15  # 옆으로 배치
            
            self.system.navigation.navigate_to(
                Pose2D(place_pos[0], place_pos[1], 0.0)
            )
            
            robot.move_to_position(place_pos)
            robot.open_gripper()
            
            time.sleep(1)
        
        print("  ✅ 재료 준비 완료")
        
        return True
    
    def arrange_workspace(self, robot, context: Dict) -> bool:
        """작업 공간 정리"""
        print("  🧹 작업 공간 정리 중...")
        
        # 조리대 청소 (간단한 버전)
        # 실제로는 물체 감지 → 분류 → 정리
        
        counter_area = WorkArea(
            name="counter",
            bounds=np.array([1.2, 1.8, -0.3, 0.3, 0.8, 1.0])
        )
        
        # 조리대 위 물체 감지
        rgb = context['sensors']['camera'].get_rgb()
        depth = context['sensors']['camera'].get_depth()
        
        detections = self.system.manipulation.detector.detect(rgb)
        
        # 필요 없는 물체 제거
        unwanted_classes = ['bottle', 'wrapper', 'book']
        
        for det in detections:
            if det['class_name'] in unwanted_classes:
                obj = self.system.manipulation.pose_estimator.estimate_3d_pose(
                    det, depth
                )
                
                # 다른 곳으로 이동
                storage_pos = np.array([2.0, 1.0, 0.5])
                
                self.system.manipulation.execute_pick_and_place(
                    obj.class_name,
                    storage_pos
                )
        
        print("  ✅ 작업 공간 정리 완료")
        
        return True
```

---

## 5. 통합 데모

### 5.1 일일 가사 루틴

**Daily Routine**
```python
def daily_household_routine():
    """일일 가사 루틴"""
    
    print("=" * 70)
    print("  🏠 XLeRobot 가정용 작업 자동화 시스템")
    print("  일일 가사 루틴")
    print("=" * 70)
    
    # 로봇 시스템 초기화
    robot = XLeRobot()
    sensors = {
        'camera': RGBDCamera(),
        'lidar': SimulatedLidar()
    }
    
    system = HouseholdRobotSystem(robot, sensors)
    
    # 작업 영역 정의
    system.work_areas = {
        'dining_table': WorkArea(
            name="식탁",
            bounds=np.array([0.3, 1.0, -0.4, 0.4, 0.7, 0.8]),
            purpose="dining"
        ),
        'kitchen_counter': WorkArea(
            name="조리대",
            bounds=np.array([1.2, 1.8, -0.3, 0.3, 0.8, 1.0]),
            purpose="kitchen"
        ),
        'living_room': WorkArea(
            name="거실",
            bounds=np.array([0.0, 3.0, -2.0, 2.0, 0.0, 0.5]),
            purpose="living"
        ),
    }
    
    # 작업 생성자들
    table_cleaner = TableCleaningTask(system)
    object_sorter = ObjectSortingTask(system)
    kitchen_assistant = KitchenAssistantTask(system)
    
    # 아침 루틴
    print("\n🌅 아침 루틴 시작")
    print("-" * 70)
    
    morning_tasks = [
        kitchen_assistant.create_meal_prep_task(),
        table_cleaner.create_task(system.work_areas['dining_table']),
    ]
    
    for task in morning_tasks:
        system.add_task(task)
    
    system.execute_tasks()
    
    # 점심 루틴
    print("\n☀️ 점심 루틴 시작")
    print("-" * 70)
    
    lunch_tasks = [
        kitchen_assistant.create_dishwashing_task(),
        table_cleaner.create_task(system.work_areas['dining_table']),
    ]
    
    for task in lunch_tasks:
        system.add_task(task)
    
    system.execute_tasks()
    
    # 저녁 루틴
    print("\n🌙 저녁 루틴 시작")
    print("-" * 70)
    
    evening_tasks = [
        kitchen_assistant.create_dishwashing_task(),
        object_sorter.create_task(),
        table_cleaner.create_task(system.work_areas['dining_table']),
    ]
    
    for task in evening_tasks:
        system.add_task(task)
    
    system.execute_tasks()
    
    # 최종 요약
    print("\n" + "=" * 70)
    print("  📊 일일 작업 요약")
    print("=" * 70)
    
    total_tasks = len(system.completed_tasks)
    successful = len([t for t in system.completed_tasks 
                     if t.status == TaskStatus.COMPLETED])
    
    print(f"총 작업: {total_tasks}")
    print(f"성공: {successful}")
    print(f"성공률: {successful/total_tasks*100:.1f}%")
    print(f"배터리 잔량: {system.battery_level:.1f}%")
    print("=" * 70)

if __name__ == "__main__":
    daily_household_routine()
```

### 5.2 대화형 작업 시스템

**Voice Command Interface**
```python
def voice_command_household_system():
    """음성 명령 기반 가사 시스템"""
    
    import speech_recognition as sr
    
    robot = XLeRobot()
    sensors = {'camera': RGBDCamera(), 'lidar': SimulatedLidar()}
    system = HouseholdRobotSystem(robot, sensors)
    
    # 작업 매핑
    task_creators = {
        '테이블 청소': lambda: TableCleaningTask(system).create_task(
            system.work_areas['dining_table']
        ),
        '물체 정리': lambda: ObjectSortingTask(system).create_task(),
        '설거지': lambda: KitchenAssistantTask(system).create_dishwashing_task(),
        '식사 준비': lambda: KitchenAssistantTask(system).create_meal_prep_task(),
    }
    
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()
    
    print("🎤 음성 명령 대기 중...")
    print("명령어: '테이블 청소', '물체 정리', '설거지', '식사 준비', '종료'")
    
    while True:
        with microphone as source:
            recognizer.adjust_for_ambient_noise(source)
            
            try:
                audio = recognizer.listen(source, timeout=5)
                command = recognizer.recognize_google(audio, language='ko-KR')
                
                print(f"\n📝 명령: {command}")
                
                if '종료' in command:
                    print("👋 시스템 종료")
                    break
                
                # 작업 매칭
                task_created = False
                for task_name, creator in task_creators.items():
                    if task_name in command:
                        task = creator()
                        system.add_task(task)
                        task_created = True
                        break
                
                if task_created:
                    print("✅ 작업 추가됨")
                    
                    # 즉시 실행 여부
                    if '지금' in command or '바로' in command:
                        system.execute_tasks()
                else:
                    print("❓ 명령을 이해하지 못했습니다")
                    
            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                print("❓ 음성을 인식하지 못했습니다")
            except Exception as e:
                print(f"❌ 오류: {e}")
```

---

## ✅ 프로젝트 5 완료 체크리스트

- [ ] 테이블 청소 시스템 구현
- [ ] 물체 정리 및 분류 구현
- [ ] 주방 보조 작업 구현
- [ ] 작업 관리 시스템 구현
- [ ] 일일 루틴 자동화
- [ ] 음성 명령 인터페이스
- [ ] 실패 처리 및 복구
- [ ] 통합 테스트

## 🎓 학습 정리

1. **작업 계획**: 복잡한 작업을 서브태스크로 분해
2. **시스템 통합**: 네비게이션 + 조작 + 인식 통합
3. **상황 인식**: 환경 이해 및 적응적 행동
4. **안정성**: 실패 처리 및 복구 메커니즘
5. **실용성**: 실제 가정 환경에서 사용 가능한 시스템

---

[← 8.4 자율 네비게이션](04_navigation.md) | [다음: 8.6 다음 단계 →](06_next_steps.md)
