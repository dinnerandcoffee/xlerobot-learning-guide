# 5.1 YOLO 객체 감지 기초

YOLOv8을 사용한 실시간 객체 감지의 기초를 학습합니다.

## 1. YOLO란?

### 1.1 개요

**YOLO (You Only Look Once)**
- 실시간 객체 감지 알고리즘
- 단일 신경망으로 이미지 전체를 한 번에 처리
- 빠른 속도 + 높은 정확도

```
입력 이미지 → CNN → 바운딩 박스 + 클래스 확률 → 출력
   (640×640)     |        (여러 객체 동시 감지)      (JSON)
```

### 1.2 YOLO 버전 비교

| 버전 | 출시 | 특징 | 속도 (FPS) |
|------|------|------|-----------|
| YOLOv5 | 2020 | PyTorch 구현, 쉬운 사용 | ~140 |
| YOLOv7 | 2022 | 정확도 향상 | ~160 |
| **YOLOv8** | 2023 | Ultralytics, SOTA | ~200 |
| YOLOv9 | 2024 | 효율성 개선 | ~220 |

**XLeRobot 권장: YOLOv8** (안정성 + 성능)

---

## 2. 환경 설정

### 2.1 패키지 설치

```bash
# YOLOv8 (Ultralytics)
pip install ultralytics

# 추가 의존성
pip install opencv-python
pip install numpy
pip install pillow
```

**requirements.txt:**
```txt
ultralytics>=8.0.0
opencv-python>=4.8.0
numpy>=1.24.0
torch>=2.0.0
torchvision>=0.15.0
```

### 2.2 설치 확인

```python
import ultralytics
from ultralytics import YOLO
import cv2

print(f"Ultralytics version: {ultralytics.__version__}")
print(f"OpenCV version: {cv2.__version__}")

# YOLO 모델 다운로드 (최초 1회)
model = YOLO('yolov8n.pt')  # nano 모델 (가장 빠름)
print("YOLO model loaded successfully!")
```

---

## 3. YOLO 모델 선택

### 3.1 사전 훈련 모델

| 모델 | 크기 | mAP | 속도 | 용도 |
|------|------|-----|------|------|
| yolov8n.pt | 3.2MB | 37.3 | 가장 빠름 | 실시간 |
| yolov8s.pt | 11.2MB | 44.9 | 빠름 | 일반 |
| yolov8m.pt | 25.9MB | 50.2 | 중간 | 균형 |
| yolov8l.pt | 43.7MB | 52.9 | 느림 | 고정확도 |
| yolov8x.pt | 68.2MB | 53.9 | 가장 느림 | 최고 성능 |

**XLeRobot 추천:**
- Jetson Nano: `yolov8n.pt` (nano)
- 일반 PC: `yolov8s.pt` (small)
- GPU 있음: `yolov8m.pt` (medium)

### 3.2 모델 다운로드

```python
from ultralytics import YOLO

# 자동 다운로드 (~/.ultralytics/)
model = YOLO('yolov8n.pt')

# 또는 수동 다운로드
# https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
```

---

## 4. 기본 사용법

### 4.1 이미지 감지

```python
from ultralytics import YOLO
import cv2

# 모델 로드
model = YOLO('yolov8n.pt')

# 이미지 로드
image = cv2.imread('input.jpg')

# 객체 감지
results = model(image)

# 결과 시각화
annotated = results[0].plot()
cv2.imshow('YOLO Detection', annotated)
cv2.waitKey(0)
```

### 4.2 웹캠 실시간 감지

```python
import cv2
from ultralytics import YOLO

# 모델 로드
model = YOLO('yolov8n.pt')

# 웹캠 열기
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # YOLO 감지
    results = model(frame)
    
    # 결과 시각화
    annotated = results[0].plot()
    
    # 화면 표시
    cv2.imshow('YOLO Webcam', annotated)
    
    # 'q' 키로 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## 5. 결과 파싱

### 5.1 Results 객체 구조

```python
results = model(image)

# 첫 번째 결과 (이미지 1개)
r = results[0]

# 주요 속성
print(r.boxes)      # 바운딩 박스
print(r.masks)      # 세그멘테이션 마스크 (seg 모델만)
print(r.keypoints)  # 키포인트 (pose 모델만)
print(r.probs)      # 분류 확률 (cls 모델만)
```

### 5.2 바운딩 박스 정보

```python
# 모든 박스 순회
for box in results[0].boxes:
    # 좌표 (x1, y1, x2, y2)
    x1, y1, x2, y2 = box.xyxy[0].tolist()
    
    # 중심점 + 크기 (x_center, y_center, width, height)
    cx, cy, w, h = box.xywh[0].tolist()
    
    # 신뢰도
    confidence = box.conf[0].item()
    
    # 클래스 ID
    class_id = int(box.cls[0].item())
    
    # 클래스 이름
    class_name = model.names[class_id]
    
    print(f"{class_name}: {confidence:.2f} at ({cx:.0f}, {cy:.0f})")
```

### 5.3 특정 클래스 필터링

```python
# COCO 클래스 (80개)
# 0: person, 39: bottle, 41: cup, 56: chair, 등

# 사람만 감지
results = model(image, classes=[0])

# 여러 클래스 감지 (사람, 병, 컵)
results = model(image, classes=[0, 39, 41])

# 신뢰도 임계값
results = model(image, conf=0.5)  # 50% 이상만
```

---

## 6. COCO 데이터셋 클래스

### 6.1 주요 클래스 목록

```python
COCO_CLASSES = {
    0: 'person',
    1: 'bicycle',
    2: 'car',
    # ... (총 80개)
    39: 'bottle',
    40: 'wine glass',
    41: 'cup',
    42: 'fork',
    43: 'knife',
    44: 'spoon',
    45: 'bowl',
    # ...
    56: 'chair',
    57: 'couch',
    # ...
}

# 모델에서 가져오기
model = YOLO('yolov8n.pt')
print(model.names)  # {0: 'person', 1: 'bicycle', ...}
```

### 6.2 클래스 이름으로 필터링

```python
def get_class_id(model, class_name):
    """클래스 이름 → ID 변환"""
    for id, name in model.names.items():
        if name == class_name:
            return id
    return None

# 사용 예시
bottle_id = get_class_id(model, 'bottle')
results = model(image, classes=[bottle_id])
```

---

## 7. 성능 최적화

### 7.1 추론 속도 향상

```python
# 1. 이미지 크기 조정
results = model(image, imgsz=320)  # 기본 640 → 320

# 2. 배치 처리
images = [img1, img2, img3]
results = model(images)  # 한 번에 처리

# 3. Half precision (FP16)
model = YOLO('yolov8n.pt')
model.fuse()  # 레이어 융합
results = model(image, half=True)  # GPU 필요
```

### 7.2 속도 측정

```python
import time

model = YOLO('yolov8n.pt')
cap = cv2.VideoCapture(0)

fps_list = []

for _ in range(100):  # 100프레임 평균
    ret, frame = cap.read()
    
    start = time.time()
    results = model(frame)
    end = time.time()
    
    fps = 1 / (end - start)
    fps_list.append(fps)

print(f"Average FPS: {sum(fps_list) / len(fps_list):.1f}")
```

---

## 8. 실전 예제

### 8.1 객체 카운터

```python
from ultralytics import YOLO
import cv2

model = YOLO('yolov8n.pt')
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # 감지
    results = model(frame, classes=[0])  # 사람만
    
    # 카운트
    person_count = len(results[0].boxes)
    
    # 텍스트 표시
    cv2.putText(frame, f"People: {person_count}", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                1, (0, 255, 0), 2)
    
    # 바운딩 박스 그리기
    annotated = results[0].plot()
    cv2.imshow('People Counter', annotated)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

### 8.2 가장 가까운 객체 찾기

```python
def find_closest_object(results, target_class='bottle'):
    """화면 중심에서 가장 가까운 객체"""
    model = results[0].model
    
    # 타겟 클래스 ID
    target_id = None
    for id, name in model.names.items():
        if name == target_class:
            target_id = id
            break
    
    if target_id is None:
        return None
    
    # 이미지 중심
    img_h, img_w = results[0].orig_shape
    center_x, center_y = img_w / 2, img_h / 2
    
    closest_box = None
    min_distance = float('inf')
    
    for box in results[0].boxes:
        if int(box.cls[0]) != target_id:
            continue
        
        # 객체 중심
        cx, cy = box.xywh[0][:2].tolist()
        
        # 거리 계산
        distance = ((cx - center_x)**2 + (cy - center_y)**2)**0.5
        
        if distance < min_distance:
            min_distance = distance
            closest_box = box
    
    return closest_box

# 사용
results = model(frame)
bottle = find_closest_object(results, 'bottle')

if bottle is not None:
    x1, y1, x2, y2 = bottle.xyxy[0].tolist()
    print(f"Closest bottle at ({x1:.0f}, {y1:.0f})")
```

---

## 9. 커스텀 모델 학습

### 9.1 데이터셋 준비

```yaml
# dataset.yaml
path: /path/to/dataset
train: images/train
val: images/val

nc: 3  # 클래스 개수
names: ['cup', 'bottle', 'phone']
```

**폴더 구조:**
```
dataset/
├── images/
│   ├── train/
│   │   ├── img001.jpg
│   │   └── ...
│   └── val/
│       └── ...
└── labels/
    ├── train/
    │   ├── img001.txt  (YOLO 형식)
    │   └── ...
    └── val/
        └── ...
```

### 9.2 학습 실행

```python
from ultralytics import YOLO

# 사전 훈련 모델 로드
model = YOLO('yolov8n.pt')

# 파인튜닝
results = model.train(
    data='dataset.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    device=0  # GPU ID (CPU: 'cpu')
)

# 최고 모델 저장됨: runs/detect/train/weights/best.pt
```

### 9.3 커스텀 모델 사용

```python
# 학습된 모델 로드
model = YOLO('runs/detect/train/weights/best.pt')

# 추론
results = model('test_image.jpg')
```

---

## 10. 디버깅 및 문제 해결

### 10.1 일반적인 오류

| 증상 | 원인 | 해결 |
|------|------|------|
| 느린 속도 | 큰 모델/이미지 | `yolov8n.pt`, `imgsz=320` |
| GPU 미사용 | CUDA 미설치 | `torch.cuda.is_available()` 확인 |
| 낮은 정확도 | 부적절한 모델 | 더 큰 모델 또는 파인튜닝 |
| 메모리 부족 | 배치 크기 과다 | `batch=8` → `batch=4` |

### 10.2 디버깅 팁

```python
# 1. 상세 출력
results = model(image, verbose=True)

# 2. 결과 저장
results[0].save('output.jpg')

# 3. 원본 이미지 크기 확인
print(results[0].orig_shape)  # (height, width)

# 4. GPU 사용 확인
import torch
print(f"CUDA available: {torch.cuda.is_available()}")
print(f"Device: {next(model.model.parameters()).device}")
```

---

## 11. 참고 자료

- [Ultralytics YOLOv8 공식 문서](https://docs.ultralytics.com/)
- [YOLO GitHub](https://github.com/ultralytics/ultralytics)
- [COCO 데이터셋](https://cocodataset.org/)
- [YOLOv8 논문](https://arxiv.org/abs/2305.09972)

---

[← 5장 목차](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/README.md) | [다음: 5.2 실시간 객체 추적 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/02_object_tracking.md)
