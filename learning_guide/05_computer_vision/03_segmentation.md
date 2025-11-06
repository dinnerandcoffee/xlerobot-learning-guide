# 5.3 세그멘테이션 활용

YOLO 세그멘테이션을 사용한 픽셀 단위 객체 분할 방법을 학습합니다.

## 1. 세그멘테이션이란?

### 1.1 바운딩 박스 vs 세그멘테이션

**바운딩 박스 (Detection)**
```
┌─────────┐
│  🍎    │  사과 영역 + 배경 포함
│         │  직사각형 영역
└─────────┘
```

**세그멘테이션 (Segmentation)**
```
   🍎      픽셀 단위 정확한 경계
  🍎🍎     마스크 형태
   🍎      배경 제외
```

### 1.2 세그멘테이션 장점

- **정확한 경계**: 픽셀 단위 객체 영역
- **면적 계산**: 실제 객체 크기 측정
- **겹침 처리**: 여러 객체 구분
- **정밀 제어**: 로봇 그리핑 최적화

---

## 2. YOLOv8 세그멘테이션

### 2.1 모델 선택

| 모델 | 크기 | mAP-box | mAP-seg | 속도 |
|------|------|---------|---------|------|
| yolov8n-seg.pt | 6.7MB | 36.7 | 30.5 | 가장 빠름 |
| yolov8s-seg.pt | 21.5MB | 44.6 | 36.8 | 빠름 |
| yolov8m-seg.pt | 49.9MB | 49.9 | 40.8 | 중간 |
| yolov8l-seg.pt | 83.6MB | 52.3 | 42.6 | 느림 |
| yolov8x-seg.pt | 130.5MB | 53.4 | 43.4 | 가장 느림 |

**XLeRobot 권장**: `yolov8n-seg.pt` (실시간) 또는 `yolov8s-seg.pt` (균형)

### 2.2 기본 사용법

```python
from ultralytics import YOLO
import cv2

# 세그멘테이션 모델 로드
model = YOLO('yolov8n-seg.pt')

# 이미지 로드
image = cv2.imread('input.jpg')

# 세그멘테이션 실행
results = model(image)

# 결과 시각화 (마스크 포함)
annotated = results[0].plot()
cv2.imshow('Segmentation', annotated)
cv2.waitKey(0)
```

---

## 3. 마스크 데이터 추출

### 3.1 마스크 정보

```python
results = model(image)

# 첫 번째 결과
r = results[0]

# 마스크 확인
if r.masks is not None:
    print(f"Total objects: {len(r.masks)}")
    
    for i, mask in enumerate(r.masks):
        # 마스크 배열 (H×W)
        mask_array = mask.data[0].cpu().numpy()
        
        # 클래스
        class_id = int(r.boxes[i].cls[0])
        class_name = model.names[class_id]
        
        print(f"Object {i}: {class_name}")
        print(f"Mask shape: {mask_array.shape}")
```

### 3.2 바이너리 마스크 생성

```python
import numpy as np

def get_binary_mask(results, index=0):
    """특정 객체의 바이너리 마스크 반환"""
    if results[0].masks is None:
        return None
    
    # 마스크 데이터 (0~1 범위)
    mask = results[0].masks[index].data[0].cpu().numpy()
    
    # 바이너리 변환 (0 or 255)
    binary_mask = (mask > 0.5).astype(np.uint8) * 255
    
    return binary_mask

# 사용
results = model(image)
mask = get_binary_mask(results, index=0)

if mask is not None:
    cv2.imshow('Binary Mask', mask)
    cv2.waitKey(0)
```

---

## 4. 마스크 활용

### 4.1 객체 영역 추출

```python
def extract_object(image, mask):
    """마스크를 사용해 객체만 추출"""
    # 3채널로 확장
    mask_3ch = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    # 마스크 적용 (배경 제거)
    masked_image = cv2.bitwise_and(image, mask_3ch)
    
    return masked_image

# 사용
results = model(image)
mask = get_binary_mask(results, 0)
object_only = extract_object(image, mask)

cv2.imshow('Extracted Object', object_only)
```

### 4.2 면적 계산

```python
def calculate_area(mask, pixel_to_cm2=0.01):
    """마스크 면적 계산 (cm²)"""
    # 픽셀 개수
    pixel_count = np.sum(mask > 0)
    
    # 실제 면적 (캘리브레이션 필요)
    area_cm2 = pixel_count * pixel_to_cm2
    
    return area_cm2

# 사용
results = model(image)
for i, mask_data in enumerate(results[0].masks):
    mask = (mask_data.data[0].cpu().numpy() > 0.5).astype(np.uint8) * 255
    area = calculate_area(mask)
    
    class_name = model.names[int(results[0].boxes[i].cls[0])]
    print(f"{class_name}: {area:.1f} cm²")
```

### 4.3 중심점 찾기

```python
def find_mask_center(mask):
    """마스크의 무게중심 (centroid)"""
    # 모멘트 계산
    M = cv2.moments(mask)
    
    if M['m00'] == 0:
        return None
    
    # 중심 좌표
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    
    return (cx, cy)

# 사용
results = model(image)
mask = get_binary_mask(results, 0)
center = find_mask_center(mask)

if center is not None:
    cv2.circle(image, center, 5, (0, 255, 0), -1)
    print(f"Center: {center}")
```

---

## 5. 여러 객체 세그멘테이션

### 5.1 클래스별 마스크

```python
def get_masks_by_class(results, target_class='person'):
    """특정 클래스의 모든 마스크 반환"""
    if results[0].masks is None:
        return []
    
    masks = []
    
    for i, box in enumerate(results[0].boxes):
        class_id = int(box.cls[0])
        class_name = results[0].names[class_id]
        
        if class_name == target_class:
            mask = results[0].masks[i].data[0].cpu().numpy()
            binary_mask = (mask > 0.5).astype(np.uint8) * 255
            masks.append(binary_mask)
    
    return masks

# 사용
results = model(image)
person_masks = get_masks_by_class(results, 'person')

print(f"Found {len(person_masks)} person(s)")
```

### 5.2 마스크 합성

```python
def combine_masks(masks):
    """여러 마스크를 하나로 합침"""
    if len(masks) == 0:
        return None
    
    # 첫 번째 마스크로 초기화
    combined = masks[0].copy()
    
    # OR 연산으로 합침
    for mask in masks[1:]:
        combined = cv2.bitwise_or(combined, mask)
    
    return combined

# 사용
all_masks = get_masks_by_class(results, 'person')
combined = combine_masks(all_masks)

if combined is not None:
    cv2.imshow('Combined Mask', combined)
```

---

## 6. 세그멘테이션 + 추적

### 6.1 마스크와 추적 ID

```python
from ultralytics import YOLO
import cv2

model = YOLO('yolov8n-seg.pt')
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    
    # 세그멘테이션 + 추적
    results = model.track(frame, persist=True)
    
    if results[0].masks is not None:
        for i, (box, mask) in enumerate(zip(results[0].boxes, 
                                            results[0].masks)):
            # 추적 ID
            if box.id is not None:
                track_id = int(box.id[0])
            else:
                track_id = -1
            
            # 마스크 추출
            mask_arr = (mask.data[0].cpu().numpy() > 0.5).astype(np.uint8) * 255
            
            # 중심점
            center = find_mask_center(mask_arr)
            
            if center is not None:
                # ID 표시
                cv2.putText(frame, f"ID: {track_id}", center,
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    annotated = results[0].plot()
    cv2.imshow('Seg + Track', annotated)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## 7. 로봇 응용

### 7.1 그리핑 포인트 찾기

```python
def find_grasp_point(mask):
    """마스크에서 최적 그리핑 지점 찾기"""
    # 윤곽선 찾기
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                   cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 0:
        return None
    
    # 가장 큰 윤곽선
    largest = max(contours, key=cv2.contourArea)
    
    # 최소 외접 원
    (cx, cy), radius = cv2.minEnclosingCircle(largest)
    
    return (int(cx), int(cy)), int(radius)

# 사용
results = model(image)
mask = get_binary_mask(results, 0)

grasp_info = find_grasp_point(mask)
if grasp_info is not None:
    center, radius = grasp_info
    
    # 시각화
    cv2.circle(image, center, radius, (0, 255, 0), 2)
    cv2.circle(image, center, 5, (0, 0, 255), -1)
    
    print(f"Grasp at: {center}, size: {radius}px")
```

### 7.2 객체 방향 추정

```python
def estimate_orientation(mask):
    """객체 방향 (각도) 추정"""
    # 윤곽선
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, 
                                   cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 0:
        return None
    
    largest = max(contours, key=cv2.contourArea)
    
    # 최소 외접 사각형
    rect = cv2.minAreaRect(largest)
    angle = rect[2]
    
    # 박스 좌표
    box = cv2.boxPoints(rect)
    box = np.int0(box)
    
    return angle, box

# 사용
results = model(image)
mask = get_binary_mask(results, 0)

orientation = estimate_orientation(mask)
if orientation is not None:
    angle, box = orientation
    
    # 그리기
    cv2.drawContours(image, [box], 0, (0, 255, 0), 2)
    print(f"Orientation: {angle:.1f}°")
```

---

## 8. 실전 예제

### 8.1 컵 감지 및 그리핑

```python
from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO('yolov8n-seg.pt')

def find_cup_grasp(image):
    """컵 찾아서 그리핑 포인트 반환"""
    results = model(image, classes=[41])  # cup = 41
    
    if results[0].masks is None or len(results[0].masks) == 0:
        return None
    
    # 첫 번째 컵
    mask = (results[0].masks[0].data[0].cpu().numpy() > 0.5).astype(np.uint8) * 255
    
    # 그리핑 포인트
    grasp = find_grasp_point(mask)
    
    if grasp is not None:
        center, radius = grasp
        return {
            'position': center,
            'size': radius,
            'mask': mask
        }
    
    return None

# 실시간 감지
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    
    cup_info = find_cup_grasp(frame)
    
    if cup_info is not None:
        # 그리핑 포인트 표시
        cv2.circle(frame, cup_info['position'], 
                  cup_info['size'], (0, 255, 0), 2)
        cv2.circle(frame, cup_info['position'], 
                  5, (0, 0, 255), -1)
        
        # 좌표 출력
        x, y = cup_info['position']
        cv2.putText(frame, f"Grasp: ({x}, {y})", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow('Cup Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

---

## 9. 성능 최적화

### 9.1 마스크 해상도 조정

```python
# 낮은 해상도로 추론 (빠름)
results = model(image, imgsz=320)  # 기본 640 → 320

# 마스크 원본 크기로 복원
for mask in results[0].masks:
    # 자동으로 원본 크기에 맞춰짐
    mask_array = mask.data[0].cpu().numpy()
```

### 9.2 GPU 사용

```python
import torch

# GPU 확인
if torch.cuda.is_available():
    model = YOLO('yolov8n-seg.pt').to('cuda')
    print("Using GPU")
else:
    model = YOLO('yolov8n-seg.pt')
    print("Using CPU")

# 추론
results = model(image)
```

---

## 10. 커스텀 세그멘테이션 학습

### 10.1 데이터셋 준비

```yaml
# dataset.yaml
path: /path/to/dataset
train: images/train
val: images/val

nc: 2
names: ['cup', 'bottle']
```

**폴더 구조:**
```
dataset/
├── images/
│   ├── train/
│   └── val/
└── labels/
    ├── train/
    │   └── img001.txt  # 세그멘테이션 포맷
    └── val/
```

**라벨 형식 (YOLO seg):**
```
class_id x1 y1 x2 y2 x3 y3 ... (polygon points, normalized)
0 0.1 0.2 0.3 0.2 0.3 0.4 0.1 0.4
```

### 10.2 학습

```python
from ultralytics import YOLO

model = YOLO('yolov8n-seg.pt')

results = model.train(
    data='dataset.yaml',
    epochs=100,
    imgsz=640,
    batch=8
)
```

---

## 11. 디버깅

### 11.1 마스크 시각화

```python
def visualize_mask(image, mask, alpha=0.5):
    """마스크를 반투명 오버레이"""
    # 컬러 마스크 (녹색)
    colored_mask = np.zeros_like(image)
    colored_mask[mask > 0] = [0, 255, 0]
    
    # 블렌딩
    overlay = cv2.addWeighted(image, 1-alpha, colored_mask, alpha, 0)
    
    return overlay

# 사용
results = model(image)
mask = get_binary_mask(results, 0)
vis = visualize_mask(image, mask, alpha=0.3)

cv2.imshow('Mask Overlay', vis)
```

### 11.2 문제 해결

| 증상 | 원인 | 해결 |
|------|------|------|
| 마스크 없음 | detection 모델 사용 | `-seg.pt` 모델 사용 |
| 경계 부정확 | 낮은 conf | `conf=0.6` 상향 |
| 느린 속도 | 큰 이미지 | `imgsz=320` 축소 |
| 메모리 부족 | batch 과다 | `batch=4` 축소 |

---

## 12. 참고 자료

- [YOLOv8 Segmentation](https://docs.ultralytics.com/tasks/segment/)
- [Instance Segmentation](https://paperswithcode.com/task/instance-segmentation)
- [OpenCV Contours](https://docs.opencv.org/4.x/d4/d73/tutorial_py_contours_begin.html)

---

[← 5.2 객체 추적](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/02_object_tracking.md) | [다음: 5.4 비전 기반 로봇 제어 →](https://github.com/dinnerandcoffee/xlerobot-learning-guide/blob/main/learning_guide/05_computer_vision/04_vision_control.md)
