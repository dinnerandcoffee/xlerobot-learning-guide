# 5.2 실시간 객체 추적

YOLO 감지 결과를 기반으로 객체를 추적하는 방법을 학습합니다.

## 1. 객체 추적이란?

### 1.1 감지 vs 추적

**객체 감지 (Detection)**
```
프레임마다 독립적으로 객체 찾기
→ ID 일관성 없음
→ 빠르지만 불안정
```

**객체 추적 (Tracking)**
```
프레임 간 객체 연결
→ 고유 ID 부여
→ 궤적 생성 가능
→ 안정적인 제어
```

### 1.2 추적의 장점

- **ID 유지**: 동일 객체에 일관된 ID
- **예측 가능**: 다음 위치 예측
- **안정성**: 일시적 가림 대응
- **궤적 분석**: 이동 경로 파악

---

## 2. YOLO + Tracking

### 2.1 YOLOv8 내장 추적기

```python
from ultralytics import YOLO
import cv2

# 모델 로드
model = YOLO('yolov8n.pt')

# 비디오 열기
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # 추적 활성화
    results = model.track(frame, persist=True)
    
    # 결과 시각화
    annotated = results[0].plot()
    cv2.imshow('Tracking', annotated)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

**`persist=True`**: 프레임 간 ID 유지

### 2.2 추적 ID 추출

```python
results = model.track(frame, persist=True)

for box in results[0].boxes:
    # 추적 ID
    if box.id is not None:
        track_id = int(box.id[0])
    else:
        track_id = -1  # 추적 실패
    
    # 좌표
    x1, y1, x2, y2 = box.xyxy[0].tolist()
    
    # 클래스
    class_name = model.names[int(box.cls[0])]
    
    print(f"ID {track_id}: {class_name} at ({x1:.0f}, {y1:.0f})")
```

---

## 3. 추적 알고리즘

### 3.1 지원 알고리즘

| 알고리즘 | 속도 | 정확도 | 특징 |
|---------|------|-------|------|
| **BoT-SORT** | 빠름 | 높음 | 기본값, 권장 |
| ByteTrack | 매우 빠름 | 중간 | 경량 |
| DeepSORT | 느림 | 매우 높음 | Re-ID 기능 |

### 3.2 추적기 선택

```python
# BoT-SORT (기본값)
results = model.track(frame, tracker='botsort.yaml')

# ByteTrack
results = model.track(frame, tracker='bytetrack.yaml')
```

**추적기 설정 파일 위치:**
```
~/.config/Ultralytics/trackers/
├── botsort.yaml
└── bytetrack.yaml
```

---

## 4. 특정 객체 추적

### 4.1 단일 객체 추적

```python
class SingleObjectTracker:
    """화면 중심에서 가장 가까운 객체 추적"""
    
    def __init__(self, model, target_class='bottle'):
        self.model = model
        self.target_class = target_class
        self.tracked_id = None
    
    def update(self, frame):
        """프레임 업데이트"""
        # YOLO 추적
        results = self.model.track(frame, persist=True)
        
        if len(results[0].boxes) == 0:
            return None
        
        # 타겟 클래스만 필터링
        target_id = self._get_class_id(self.target_class)
        
        # 기존 추적 ID가 있으면 우선
        if self.tracked_id is not None:
            box = self._find_by_id(results, self.tracked_id)
            if box is not None:
                return box
        
        # 없으면 가장 가까운 객체
        closest = self._find_closest(results, target_id)
        if closest is not None and closest.id is not None:
            self.tracked_id = int(closest.id[0])
        
        return closest
    
    def _get_class_id(self, class_name):
        """클래스 이름 → ID"""
        for id, name in self.model.names.items():
            if name == class_name:
                return id
        return None
    
    def _find_by_id(self, results, track_id):
        """추적 ID로 박스 찾기"""
        for box in results[0].boxes:
            if box.id is not None and int(box.id[0]) == track_id:
                return box
        return None
    
    def _find_closest(self, results, class_id):
        """화면 중심에서 가장 가까운 박스"""
        img_h, img_w = results[0].orig_shape
        center_x, center_y = img_w / 2, img_h / 2
        
        min_distance = float('inf')
        closest_box = None
        
        for box in results[0].boxes:
            if int(box.cls[0]) != class_id:
                continue
            
            cx, cy = box.xywh[0][:2].tolist()
            distance = ((cx - center_x)**2 + (cy - center_y)**2)**0.5
            
            if distance < min_distance:
                min_distance = distance
                closest_box = box
        
        return closest_box

# 사용 예시
tracker = SingleObjectTracker(model, 'bottle')

while True:
    ret, frame = cap.read()
    
    # 추적 업데이트
    box = tracker.update(frame)
    
    if box is not None:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        track_id = int(box.id[0]) if box.id is not None else -1
        
        # 박스 그리기
        cv2.rectangle(frame, (int(x1), int(y1)), 
                     (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(frame, f"ID: {track_id}", 
                   (int(x1), int(y1)-10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow('Single Object Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

---

## 5. 궤적 추적

### 5.1 궤적 저장

```python
from collections import defaultdict
import numpy as np

class TrajectoryTracker:
    """객체별 이동 궤적 저장"""
    
    def __init__(self, max_length=100):
        self.trajectories = defaultdict(list)
        self.max_length = max_length
    
    def update(self, results):
        """추적 결과로 궤적 업데이트"""
        for box in results[0].boxes:
            if box.id is None:
                continue
            
            track_id = int(box.id[0])
            cx, cy = box.xywh[0][:2].tolist()
            
            # 궤적 추가
            self.trajectories[track_id].append((int(cx), int(cy)))
            
            # 최대 길이 제한
            if len(self.trajectories[track_id]) > self.max_length:
                self.trajectories[track_id].pop(0)
    
    def draw(self, frame):
        """궤적 그리기"""
        for track_id, points in self.trajectories.items():
            if len(points) < 2:
                continue
            
            # 선 그리기
            for i in range(1, len(points)):
                # 투명도 (최근일수록 진함)
                alpha = i / len(points)
                color = (0, int(255 * alpha), 0)
                
                cv2.line(frame, points[i-1], points[i], 
                        color, 2)
        
        return frame

# 사용
traj_tracker = TrajectoryTracker()

while cap.isOpened():
    ret, frame = cap.read()
    
    # 추적
    results = model.track(frame, persist=True)
    
    # 궤적 업데이트
    traj_tracker.update(results)
    
    # 시각화
    annotated = results[0].plot()
    annotated = traj_tracker.draw(annotated)
    
    cv2.imshow('Trajectory', annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

---

## 6. 고급 추적 기능

### 6.1 객체 진입/퇴장 감지

```python
class EntryExitCounter:
    """특정 영역 진입/퇴장 카운트"""
    
    def __init__(self, line_y):
        """
        Args:
            line_y: 기준선 Y 좌표
        """
        self.line_y = line_y
        self.tracked_objects = {}  # {id: last_y}
        self.entry_count = 0
        self.exit_count = 0
    
    def update(self, results):
        """프레임 업데이트"""
        current_ids = set()
        
        for box in results[0].boxes:
            if box.id is None:
                continue
            
            track_id = int(box.id[0])
            cy = box.xywh[0][1].item()
            
            current_ids.add(track_id)
            
            if track_id in self.tracked_objects:
                last_y = self.tracked_objects[track_id]
                
                # 위에서 아래로 (진입)
                if last_y < self.line_y <= cy:
                    self.entry_count += 1
                    print(f"Entry: ID {track_id}")
                
                # 아래에서 위로 (퇴장)
                elif last_y > self.line_y >= cy:
                    self.exit_count += 1
                    print(f"Exit: ID {track_id}")
            
            self.tracked_objects[track_id] = cy
        
        # 사라진 객체 제거
        disappeared = set(self.tracked_objects.keys()) - current_ids
        for tid in disappeared:
            del self.tracked_objects[tid]
    
    def draw(self, frame):
        """카운트 표시"""
        h, w = frame.shape[:2]
        
        # 기준선
        cv2.line(frame, (0, self.line_y), (w, self.line_y), 
                (255, 0, 0), 2)
        
        # 카운트
        text = f"IN: {self.entry_count} | OUT: {self.exit_count}"
        cv2.putText(frame, text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        return frame

# 사용
counter = EntryExitCounter(line_y=300)

while cap.isOpened():
    ret, frame = cap.read()
    results = model.track(frame, persist=True)
    
    counter.update(results)
    annotated = results[0].plot()
    annotated = counter.draw(annotated)
    
    cv2.imshow('Entry/Exit Counter', annotated)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

### 6.2 속도 추정

```python
class SpeedEstimator:
    """객체 이동 속도 추정"""
    
    def __init__(self, fps=30, pixel_to_meter=0.01):
        """
        Args:
            fps: 카메라 프레임레이트
            pixel_to_meter: 픽셀 → 미터 변환 (캘리브레이션 필요)
        """
        self.fps = fps
        self.pixel_to_meter = pixel_to_meter
        self.prev_positions = {}
        self.speeds = {}
    
    def update(self, results):
        """속도 계산"""
        for box in results[0].boxes:
            if box.id is None:
                continue
            
            track_id = int(box.id[0])
            cx, cy = box.xywh[0][:2].tolist()
            
            if track_id in self.prev_positions:
                px, py = self.prev_positions[track_id]
                
                # 픽셀 거리
                pixel_dist = ((cx - px)**2 + (cy - py)**2)**0.5
                
                # 실제 거리 (m)
                meter_dist = pixel_dist * self.pixel_to_meter
                
                # 속도 (m/s)
                speed = meter_dist * self.fps
                
                self.speeds[track_id] = speed
            
            self.prev_positions[track_id] = (cx, cy)
    
    def get_speed(self, track_id):
        """특정 객체 속도 반환"""
        return self.speeds.get(track_id, 0.0)

# 사용
speed_est = SpeedEstimator(fps=30)

while cap.isOpened():
    ret, frame = cap.read()
    results = model.track(frame, persist=True)
    
    speed_est.update(results)
    
    # 속도 표시
    for box in results[0].boxes:
        if box.id is not None:
            tid = int(box.id[0])
            speed = speed_est.get_speed(tid)
            
            x1, y1 = box.xyxy[0][:2].tolist()
            text = f"ID{tid}: {speed*100:.1f} cm/s"
            cv2.putText(frame, text, (int(x1), int(y1)-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow('Speed Estimation', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

---

## 7. 비디오 저장

### 7.1 추적 결과 저장

```python
import cv2
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
cap = cv2.VideoCapture('input.mp4')

# 비디오 라이터
fps = int(cap.get(cv2.CAP_PROP_FPS))
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, fps, (w, h))

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # 추적
    results = model.track(frame, persist=True)
    annotated = results[0].plot()
    
    # 저장
    out.write(annotated)

cap.release()
out.release()
print("Video saved: output.mp4")
```

---

## 8. 성능 최적화

### 8.1 트래커 파라미터 튜닝

```yaml
# botsort.yaml
tracker_type: botsort
track_high_thresh: 0.5     # 높은 신뢰도 임계값
track_low_thresh: 0.1      # 낮은 신뢰도 임계값
new_track_thresh: 0.6      # 새 트랙 생성 임계값
track_buffer: 30           # 트랙 유지 프레임 수
match_thresh: 0.8          # 매칭 임계값
```

### 8.2 프레임 스킵

```python
# 2프레임마다 추적
frame_count = 0
last_results = None

while cap.isOpened():
    ret, frame = cap.read()
    frame_count += 1
    
    if frame_count % 2 == 0:
        # 추적 실행
        last_results = model.track(frame, persist=True)
    
    # 마지막 결과 사용
    if last_results is not None:
        annotated = last_results[0].plot()
        cv2.imshow('Frame Skip', annotated)
```

---

## 9. 디버깅

### 9.1 추적 정보 로깅

```python
import logging

logging.basicConfig(level=logging.INFO)

while cap.isOpened():
    ret, frame = cap.read()
    results = model.track(frame, persist=True, verbose=True)
    
    # 추적 통계
    total = len(results[0].boxes)
    tracked = sum(1 for b in results[0].boxes if b.id is not None)
    
    logging.info(f"Total: {total}, Tracked: {tracked}")
```

### 9.2 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| ID 자주 바뀜 | 낮은 conf | `conf=0.5` 상향 |
| 트랙 끊김 | 짧은 buffer | `track_buffer=60` |
| 느린 속도 | 무거운 모델 | `yolov8n.pt` 사용 |
| 잘못된 매칭 | 높은 match_thresh | 값 낮춤 (0.7) |

---

## 10. 참고 자료

- [YOLOv8 Tracking 문서](https://docs.ultralytics.com/modes/track/)
- [BoT-SORT 논문](https://arxiv.org/abs/2206.14651)
- [ByteTrack 논문](https://arxiv.org/abs/2110.06864)
- [DeepSORT](https://github.com/nwojke/deep_sort)

---

[← 5.1 YOLO 기초](01_yolo_basics.md) | [다음: 5.3 세그멘테이션 →](03_segmentation.md)
