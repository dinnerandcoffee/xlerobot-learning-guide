# 8.1 프로젝트 1: 첫 시뮬레이션 실행

XLeRobot의 첫 시뮬레이션을 실행하고 기본적인 로봇 움직임을 체험해보는 입문 프로젝트입니다.

## 🎯 프로젝트 목표

- 시뮬레이션 환경 설정 및 실행
- 기본 로봇 제어 체험
- 시뮬레이션 파라미터 조정
- 카메라 뷰 및 센서 데이터 확인

**난이도**: ⭐ (초급)  
**소요 시간**: 30분  
**선수 지식**: 1-3장

---

## 1. 시뮬레이션 환경 선택

### 1.1 사용 가능한 시뮬레이터

XLeRobot은 여러 시뮬레이터를 지원합니다:

```
📦 MuJoCo (권장)
├── 빠른 물리 시뮬레이션
├── 정확한 접촉 모델링
└── 가벼운 렌더링

📦 Isaac Sim
├── 고품질 렌더링
├── GPU 가속 지원
└── 대규모 환경 지원

📦 ManiSkill
├── 조작 작업 특화
├── 다양한 벤치마크
└── 강화학습 최적화
```

### 1.2 MuJoCo 시뮬레이션 (추천)

**1단계: 환경 준비**
```bash
# 프로젝트 디렉토리로 이동
cd ~/XLeRobot

# MuJoCo 시뮬레이션 확인
ls simulation/mujoco/
```

**2단계: 첫 시뮬레이션 실행**
```bash
# Python 환경 활성화
conda activate xlerobot

# 기본 시뮬레이션 실행
cd simulation/mujoco
python xlerobot_mujoco.py
```

**3단계: 시뮬레이션 창 확인**
- 로봇 모델이 표시되는지 확인
- 마우스로 뷰 조정 가능
- 키보드로 기본 제어 가능

---

## 2. 기본 제어 실습

### 2.1 키보드 제어

시뮬레이션이 실행되면 다음 키로 제어 가능합니다:

```
🎮 기본 제어키
├── W/S: 전진/후진
├── A/D: 좌회전/우회전
├── Q/E: 관절 1 제어
├── R/F: 관절 2 제어
├── T/G: 관절 3 제어
└── SPACE: 정지
```

**실습 1: 기본 움직임**
```python
# xlerobot_mujoco.py 내용 확인
import mujoco
import numpy as np
from mujoco import viewer

class XLeRobotMuJoCo:
    def __init__(self, xml_path="xlerobot.xml"):
        # 모델 로드
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # 초기 설정
        self.setup_robot()
        
    def setup_robot(self):
        """로봇 초기 설정"""
        # 관절 위치 초기화
        self.data.qpos[0:7] = [0, 0, 0, 0, 0, 0, 0]  # 7-DOF 암
        
        # 그리퍼 초기화
        if self.model.njnt > 7:
            self.data.qpos[7:9] = [0.0, 0.0]  # 그리퍼
        
        # Forward kinematics 계산
        mujoco.mj_forward(self.model, self.data)
        
    def step_simulation(self, control_input=None):
        """시뮬레이션 한 스텝 실행"""
        if control_input is not None:
            self.data.ctrl[:] = control_input
            
        # 물리 시뮬레이션 스텝
        mujoco.mj_step(self.model, self.data)
        
    def get_end_effector_pose(self):
        """엔드 이펙터 위치 반환"""
        ee_id = self.model.body("end_effector").id
        ee_pos = self.data.xpos[ee_id].copy()
        ee_rot = self.data.xmat[ee_id].copy().reshape(3, 3)
        
        return ee_pos, ee_rot
        
    def get_joint_angles(self):
        """현재 관절 각도 반환"""
        return self.data.qpos[:7].copy()

# 메인 실행 함수
def main():
    # 로봇 초기화
    robot = XLeRobotMuJoCo()
    
    # 뷰어 시작
    with mujoco.viewer.launch_passive(robot.model, robot.data) as viewer:
        print("🤖 XLeRobot 시뮬레이션 시작!")
        print("키보드 제어:")
        print("  WASD: 베이스 이동")
        print("  QE/RF/TG: 관절 제어")
        print("  ESC: 종료")
        
        while viewer.is_running():
            # 키보드 입력 처리
            control = process_keyboard_input(viewer)
            
            # 시뮬레이션 스텝
            robot.step_simulation(control)
            
            # 뷰어 업데이트
            viewer.sync()
            
            # 60 FPS로 제한
            time.sleep(1.0 / 60.0)

if __name__ == "__main__":
    main()
```

### 2.2 프로그래밍 방식 제어

**실습 2: 코드로 제어하기**
```python
import time
import numpy as np

def demo_movements(robot):
    """데모 움직임 시퀀스"""
    
    print("🚀 데모 시작: 기본 움직임")
    
    # 1. 홈 포지션으로 이동
    home_position = [0, -0.5, 0, -1.5, 0, 1.0, 0]
    move_to_joint_position(robot, home_position, duration=3.0)
    
    # 2. 간단한 웨이브 동작
    wave_positions = [
        [0, -0.5, 0, -1.5, 0, 1.0, 1.57],  # 손목 회전
        [0, -0.5, 0, -1.5, 0, 1.0, -1.57], # 반대 방향
        [0, -0.5, 0, -1.5, 0, 1.0, 0]      # 원위치
    ]
    
    for pos in wave_positions:
        move_to_joint_position(robot, pos, duration=1.0)
        time.sleep(0.5)
    
    # 3. 픽앤플레이스 모션
    pick_demo(robot)
    
    print("✅ 데모 완료!")

def move_to_joint_position(robot, target_pos, duration=2.0):
    """관절 위치로 부드럽게 이동"""
    start_pos = robot.get_joint_angles()
    steps = int(duration * 60)  # 60 FPS
    
    for i in range(steps):
        t = i / steps
        # 부드러운 보간 (cubic ease-in-out)
        smooth_t = 3*t*t - 2*t*t*t
        
        current_pos = start_pos + smooth_t * (np.array(target_pos) - start_pos)
        
        # 위치 제어 (간단한 PD 제어)
        control = pd_control(robot, current_pos)
        robot.step_simulation(control)
        
        time.sleep(1.0 / 60.0)

def pd_control(robot, target_pos, kp=100, kd=10):
    """PD 제어기"""
    current_pos = robot.get_joint_angles()
    current_vel = robot.data.qvel[:7]
    
    error = target_pos - current_pos
    error_dot = -current_vel
    
    control = kp * error + kd * error_dot
    
    # 토크 제한
    return np.clip(control, -50, 50)

def pick_demo(robot):
    """간단한 픽 동작 시연"""
    print("📦 픽 동작 시연")
    
    # 1. 물체 위로 이동 (사전 정의된 위치)
    approach_pos = [0, -0.3, 0.5, -1.2, 0, 0.9, 0]
    move_to_joint_position(robot, approach_pos, duration=2.0)
    
    # 2. 물체로 하강
    grasp_pos = [0, -0.1, 0.8, -1.5, 0, 1.4, 0]
    move_to_joint_position(robot, grasp_pos, duration=1.5)
    
    # 3. 그리퍼 닫기 (시뮬레이션)
    print("🤏 그리퍼 닫기")
    time.sleep(1.0)
    
    # 4. 리프트
    lift_pos = [0, -0.3, 0.5, -1.2, 0, 0.9, 0]
    move_to_joint_position(robot, lift_pos, duration=2.0)
    
    # 5. 플레이스 위치로 이동
    place_pos = [1.0, -0.3, 0.5, -1.2, 0, 0.9, 0]
    move_to_joint_position(robot, place_pos, duration=2.0)
    
    # 6. 하강 및 플레이스
    place_down_pos = [1.0, -0.1, 0.8, -1.5, 0, 1.4, 0]
    move_to_joint_position(robot, place_down_pos, duration=1.5)
    
    # 7. 그리퍼 열기
    print("✋ 그리퍼 열기")
    time.sleep(1.0)
    
    # 8. 홈으로 복귀
    move_to_joint_position(robot, [0, -0.5, 0, -1.5, 0, 1.0, 0], duration=3.0)
```

---

## 3. 시뮬레이션 파라미터 조정

### 3.1 XML 설정 파일 수정

**xlerobot.xml 파일 이해**
```xml
<mujoco model="XLeRobot">
    <compiler angle="radian" />
    
    <!-- 물리 설정 -->
    <option timestep="0.001" integrator="RK4">
        <flag warmstart="enable" />
    </option>
    
    <!-- 시각 설정 -->
    <visual>
        <rgba haze="0.15 0.25 0.35 1" />
        <quality shadowsize="2048" />
        <map force="0.1" zfar="30" />
    </visual>
    
    <!-- 자산 정의 -->
    <asset>
        <texture name="grid" type="2d" builtin="checker" width="512" height="512" 
                 rgb1=".1 .2 .3" rgb2=".2 .3 .4" />
        <material name="grid" texture="grid" texrepeat="1 1" reflectance=".2" />
    </asset>
    
    <!-- 월드 정의 -->
    <worldbody>
        <!-- 바닥 -->
        <geom name="floor" size="0 0 .05" type="plane" material="grid" />
        
        <!-- 조명 -->
        <light name="light1" pos="0 0 6" dir="0 0 -1" diffuse="1 1 1" />
        <light name="light2" pos="3 3 6" dir="-1 -1 -1" diffuse="0.5 0.5 0.5" />
        
        <!-- 로봇 베이스 -->
        <body name="base_link" pos="0 0 0.1">
            <!-- 베이스 형상 -->
            <geom name="base" type="cylinder" size="0.1 0.05" 
                  rgba="0.2 0.2 0.2 1" mass="2.0" />
            
            <!-- 첫 번째 관절 -->
            <joint name="joint1" type="hinge" axis="0 0 1" 
                   range="-3.14 3.14" damping="0.1" />
            
            <!-- 첫 번째 링크 -->
            <body name="link1" pos="0 0 0.1">
                <geom name="link1_visual" type="cylinder" size="0.05 0.15" 
                      rgba="0.8 0.2 0.2 1" mass="1.0" />
                
                <!-- 두 번째 관절 -->
                <joint name="joint2" type="hinge" axis="0 1 0" 
                       range="-1.57 1.57" damping="0.1" />
                
                <!-- 추가 링크들... -->
                <!-- (실제 파일에는 모든 7개 관절과 링크가 정의됨) -->
            </body>
        </body>
        
        <!-- 환경 객체들 -->
        <body name="cube1" pos="0.3 0.3 0.525">
            <geom name="cube1_geom" type="box" size="0.025 0.025 0.025" 
                  rgba="1 0 0 1" mass="0.1" />
            <joint name="cube1_joint" type="free" />
        </body>
        
        <body name="cube2" pos="-0.3 0.3 0.525">
            <geom name="cube2_geom" type="box" size="0.025 0.025 0.025" 
                  rgba="0 1 0 1" mass="0.1" />
            <joint name="cube2_joint" type="free" />
        </body>
    </worldbody>
    
    <!-- 액추에이터 정의 -->
    <actuator>
        <motor name="motor1" joint="joint1" gear="100" />
        <motor name="motor2" joint="joint2" gear="100" />
        <motor name="motor3" joint="joint3" gear="50" />
        <motor name="motor4" joint="joint4" gear="50" />
        <motor name="motor5" joint="joint5" gear="25" />
        <motor name="motor6" joint="joint6" gear="25" />
        <motor name="motor7" joint="joint7" gear="25" />
    </actuator>
</mujoco>
```

### 3.2 시뮬레이션 설정 커스터마이징

**설정 변경 예제**
```python
def customize_simulation():
    """시뮬레이션 설정 커스터마이즈"""
    
    # XML 파일 로드 및 수정
    xml_content = load_xml_template()
    
    # 물리 파라미터 조정
    physics_settings = {
        'timestep': 0.001,      # 시뮬레이션 스텝 크기
        'gravity': -9.81,       # 중력 가속도
        'iterations': 50,       # 제약 해결 반복 횟수
        'tolerance': 1e-10      # 수렴 허용 오차
    }
    
    # 로봇 파라미터 조정
    robot_settings = {
        'joint_damping': 0.1,   # 관절 댐핑
        'joint_stiffness': 0,   # 관절 강성
        'friction': [0.7, 0.005, 0.0001],  # 마찰 계수
        'mass_scaling': 1.0     # 질량 스케일링
    }
    
    # 환경 설정
    environment_settings = {
        'num_objects': 5,       # 환경 내 객체 수
        'object_mass': 0.1,     # 객체 질량
        'table_height': 0.8,    # 테이블 높이
        'workspace_size': 1.0   # 작업 공간 크기
    }
    
    return generate_custom_xml(physics_settings, robot_settings, environment_settings)

def load_xml_template():
    """XML 템플릿 로드"""
    with open('xlerobot_template.xml', 'r') as f:
        return f.read()

def generate_custom_xml(physics, robot, env):
    """커스텀 XML 생성"""
    xml_template = """
    <mujoco model="XLeRobot_Custom">
        <compiler angle="radian" />
        
        <option timestep="{timestep}" gravity="0 0 {gravity}">
            <flag warmstart="enable" />
        </option>
        
        <!-- 로봇 정의 (동적 생성) -->
        <!-- 환경 객체 (동적 생성) -->
        
    </mujoco>
    """
    
    # 템플릿에 값 대입
    custom_xml = xml_template.format(
        timestep=physics['timestep'],
        gravity=physics['gravity']
    )
    
    # 추가 요소들 동적으로 생성
    custom_xml += generate_robot_xml(robot)
    custom_xml += generate_environment_xml(env)
    
    return custom_xml
```

---

## 4. 센서 데이터 및 카메라 뷰

### 4.1 센서 데이터 수집

**센서 정보 모니터링**
```python
class SensorMonitor:
    def __init__(self, robot):
        self.robot = robot
        self.sensor_data = {}
        
    def update_sensors(self):
        """모든 센서 데이터 업데이트"""
        
        # 관절 센서
        self.sensor_data['joint_positions'] = self.robot.get_joint_angles()
        self.sensor_data['joint_velocities'] = self.robot.data.qvel[:7]
        self.sensor_data['joint_torques'] = self.robot.data.qfrc_actuator[:7]
        
        # 엔드 이펙터 정보
        ee_pos, ee_rot = self.robot.get_end_effector_pose()
        self.sensor_data['ee_position'] = ee_pos
        self.sensor_data['ee_rotation'] = ee_rot
        
        # 접촉 센서
        self.sensor_data['contacts'] = self.get_contact_forces()
        
        # IMU 데이터 (시뮬레이션)
        self.sensor_data['imu'] = self.get_imu_data()
        
        return self.sensor_data
    
    def get_contact_forces(self):
        """접촉력 정보"""
        contacts = []
        
        for i in range(self.robot.data.ncon):
            contact = self.robot.data.contact[i]
            
            contact_info = {
                'geom1': contact.geom1,
                'geom2': contact.geom2,
                'pos': contact.pos.copy(),
                'normal': contact.frame[:3].copy(),
                'force': np.linalg.norm(contact.force)
            }
            contacts.append(contact_info)
            
        return contacts
    
    def get_imu_data(self):
        """IMU 데이터 시뮬레이션"""
        # 베이스 링크의 가속도 및 각속도
        base_id = self.robot.model.body("base_link").id
        
        linear_acc = self.robot.data.cacc[base_id][:3]
        angular_vel = self.robot.data.cvel[base_id][3:]
        
        return {
            'linear_acceleration': linear_acc,
            'angular_velocity': angular_vel
        }
    
    def print_sensor_summary(self):
        """센서 데이터 요약 출력"""
        print("\n📊 센서 데이터 요약")
        print("-" * 40)
        
        # 관절 정보
        joint_pos = self.sensor_data['joint_positions']
        print(f"관절 위치: {joint_pos}")
        
        # 엔드 이펙터 위치
        ee_pos = self.sensor_data['ee_position']
        print(f"EE 위치: [{ee_pos[0]:.3f}, {ee_pos[1]:.3f}, {ee_pos[2]:.3f}]")
        
        # 접촉 정보
        contacts = self.sensor_data['contacts']
        print(f"접촉 수: {len(contacts)}")
        
        if contacts:
            max_force = max(c['force'] for c in contacts)
            print(f"최대 접촉력: {max_force:.3f} N")
```

### 4.2 카메라 시스템

**가상 카메라 설정**
```python
class CameraSystem:
    def __init__(self, robot):
        self.robot = robot
        self.cameras = {}
        self.setup_cameras()
        
    def setup_cameras(self):
        """카메라 설정"""
        
        # 1. 손목 카메라 (엔드 이펙터)
        self.cameras['wrist'] = {
            'position': [0, 0, 0],  # 상대적 위치
            'orientation': [1, 0, 0, 0],
            'fov': 60,
            'resolution': (640, 480)
        }
        
        # 2. 고정 오버헤드 카메라
        self.cameras['overhead'] = {
            'position': [0, -2, 3],
            'target': [0, 0, 0.5],
            'fov': 45,
            'resolution': (1280, 720)
        }
        
        # 3. 사이드 뷰 카메라
        self.cameras['side'] = {
            'position': [2, 0, 1],
            'target': [0, 0, 0.5],
            'fov': 45,
            'resolution': (640, 480)
        }
    
    def capture_image(self, camera_name):
        """카메라 이미지 캡처"""
        if camera_name not in self.cameras:
            return None
            
        camera = self.cameras[camera_name]
        
        # MuJoCo 렌더러로 이미지 생성
        renderer = mujoco.Renderer(
            self.robot.model, 
            height=camera['resolution'][1],
            width=camera['resolution'][0]
        )
        
        # 카메라 설정
        if camera_name == 'wrist':
            # 엔드 이펙터 기준 카메라
            ee_pos, ee_rot = self.robot.get_end_effector_pose()
            renderer.update_scene(
                self.robot.data,
                camera=camera_name
            )
        else:
            # 고정 카메라
            renderer.update_scene(
                self.robot.data,
                camera=camera_name
            )
        
        # 이미지 렌더링
        rgb_image = renderer.render()
        
        return rgb_image
    
    def capture_depth(self, camera_name):
        """깊이 이미지 캡처"""
        # 깊이 렌더링 설정
        renderer = mujoco.Renderer(self.robot.model)
        renderer.enable_depth = True
        
        # 깊이 이미지 생성
        renderer.update_scene(self.robot.data, camera=camera_name)
        depth_image = renderer.render_depth()
        
        return depth_image
    
    def save_images(self, prefix="frame"):
        """모든 카메라 이미지 저장"""
        import cv2
        
        for camera_name in self.cameras:
            # RGB 이미지
            rgb_img = self.capture_image(camera_name)
            if rgb_img is not None:
                filename = f"{prefix}_{camera_name}_rgb.jpg"
                cv2.imwrite(filename, cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))
            
            # 깊이 이미지
            depth_img = self.capture_depth(camera_name)
            if depth_img is not None:
                filename = f"{prefix}_{camera_name}_depth.png"
                cv2.imwrite(filename, (depth_img * 255).astype(np.uint8))
```

---

## 5. 실습 과제

### 과제 1: 기본 제어 마스터하기
```python
def exercise_1_basic_control():
    """과제 1: 기본 제어 연습"""
    
    print("🎯 과제 1: 기본 제어 마스터하기")
    print("목표: 로봇을 원하는 위치로 정확히 이동시키기")
    
    robot = XLeRobotMuJoCo()
    
    # 목표 위치들
    target_positions = [
        [0, 0, 0, 0, 0, 0, 0],           # 홈
        [1.57, 0, 0, 0, 0, 0, 0],       # 90도 회전
        [0, 1.57, 0, 0, 0, 0, 0],       # 팔 들기
        [0, 0, 1.57, 0, 0, 0, 0],       # 팔꿈치 굽히기
    ]
    
    for i, target in enumerate(target_positions):
        print(f"단계 {i+1}: {target}")
        move_to_joint_position(robot, target, duration=2.0)
        
        # 정확도 체크
        current = robot.get_joint_angles()
        error = np.linalg.norm(np.array(target) - current)
        
        print(f"위치 오차: {error:.4f} rad")
        if error < 0.1:
            print("✅ 성공!")
        else:
            print("❌ 다시 시도하세요")
        
        time.sleep(1.0)

def exercise_2_trajectory_following():
    """과제 2: 궤적 추종"""
    
    print("🎯 과제 2: 원형 궤적 그리기")
    print("목표: 엔드 이펙터로 원형 궤적 그리기")
    
    robot = XLeRobotMuJoCo()
    
    # 원형 궤적 생성
    center = [0.3, 0, 0.6]
    radius = 0.1
    num_points = 50
    
    trajectory = []
    for i in range(num_points):
        angle = 2 * np.pi * i / num_points
        x = center[0] + radius * np.cos(angle)
        y = center[1] + radius * np.sin(angle)
        z = center[2]
        
        trajectory.append([x, y, z])
    
    # 궤적 실행
    for point in trajectory:
        # 역기구학으로 관절 각도 계산 (간단한 버전)
        joint_angles = simple_inverse_kinematics(robot, point)
        
        if joint_angles is not None:
            move_to_joint_position(robot, joint_angles, duration=0.1)
        
        time.sleep(0.1)

def simple_inverse_kinematics(robot, target_pos):
    """간단한 역기구학 (수치적 방법)"""
    # 현재 관절 각도
    current_joints = robot.get_joint_angles()
    
    # 목표 위치와 현재 위치의 차이
    current_ee_pos, _ = robot.get_end_effector_pose()
    error = np.array(target_pos) - current_ee_pos
    
    # 자코비안 계산 (수치적 미분)
    jacobian = compute_jacobian(robot, current_joints)
    
    # 의사역행렬로 관절 속도 계산
    joint_velocity = np.linalg.pinv(jacobian) @ error
    
    # 새로운 관절 각도
    new_joints = current_joints + 0.1 * joint_velocity
    
    return new_joints

def compute_jacobian(robot, joint_angles):
    """자코비안 행렬 계산 (수치적 미분)"""
    epsilon = 1e-6
    jacobian = np.zeros((3, len(joint_angles)))
    
    # 현재 엔드 이펙터 위치
    original_pos, _ = robot.get_end_effector_pose()
    
    for i in range(len(joint_angles)):
        # 관절 각도 미소 변화
        perturbed_joints = joint_angles.copy()
        perturbed_joints[i] += epsilon
        
        # 변화된 위치에서의 엔드 이펙터 위치
        robot.data.qpos[:len(joint_angles)] = perturbed_joints
        mujoco.mj_forward(robot.model, robot.data)
        
        perturbed_pos, _ = robot.get_end_effector_pose()
        
        # 자코비안 열 계산
        jacobian[:, i] = (perturbed_pos - original_pos) / epsilon
    
    # 원래 상태로 복원
    robot.data.qpos[:len(joint_angles)] = joint_angles
    mujoco.mj_forward(robot.model, robot.data)
    
    return jacobian
```

---

## 6. 시뮬레이션 확장

### 6.1 Isaac Sim 실행

**Isaac Sim 버전 실행**
```bash
# Isaac Sim 디렉토리로 이동
cd ~/XLeRobot/simulation/Isaac_sim

# 스크립트 실행
python run_xlerobot_sim.py
```

**Isaac Sim 특징**
```python
# Isaac Sim 전용 기능들
import omni.isaac.core
from omni.isaac.core import World
from omni.isaac.core.robots import Robot

class XLeRobotIsaacSim:
    def __init__(self):
        # Isaac Sim 월드 생성
        self.world = World(stage_units_in_meters=1.0)
        
        # 로봇 로드
        self.robot = self.world.scene.add(
            Robot(
                prim_path="/World/XLeRobot",
                name="xlerobot",
                usd_path="path/to/xlerobot.usd"
            )
        )
        
        # 고품질 렌더링 설정
        self.setup_high_quality_rendering()
        
    def setup_high_quality_rendering(self):
        """고품질 렌더링 설정"""
        import omni.isaac.core.utils.render as render_utils
        
        # RTX 레이트레이싱 활성화
        render_utils.set_render_mode("RayTracedLighting")
        
        # 그림자 품질 향상
        render_utils.set_shadow_quality("Ultra")
        
        # 안티앨리어싱
        render_utils.set_anti_aliasing("TAA")
```

### 6.2 ManiSkill 환경

**ManiSkill 실행**
```bash
cd ~/XLeRobot/simulation/Maniskill
python run_xlerobot_sim.py
```

**ManiSkill 작업들**
```python
# 사용 가능한 작업들
available_tasks = [
    "PickCube-v0",
    "StackCube-v0", 
    "PegInsertionSide-v0",
    "PlugCharger-v0",
    "TurnFaucet-v0"
]

# 작업 실행 예제
import gymnasium as gym
import mani_skill.envs

def run_maniskill_task(task_name):
    env = gym.make(
        task_name,
        obs_mode="rgbd",
        control_mode="pd_joint_delta_pos",
        render_mode="human"
    )
    
    obs, info = env.reset()
    
    for step in range(1000):
        # 랜덤 액션 또는 학습된 정책
        action = env.action_space.sample()
        
        obs, reward, terminated, truncated, info = env.step(action)
        
        if terminated or truncated:
            obs, info = env.reset()
    
    env.close()
```

---

## 7. 문제 해결 및 팁

### 7.1 일반적인 문제들

**문제 1: 시뮬레이션이 시작되지 않음**
```
해결책:
1. conda 환경 확인: conda activate xlerobot
2. 패키지 설치: pip install mujoco
3. XML 파일 경로 확인
4. 권한 문제 확인
```

**문제 2: 로봇이 불안정하게 움직임**
```
해결책:
1. timestep 줄이기 (0.001 → 0.0005)
2. 관절 댐핑 증가
3. PD 제어 게인 조정
4. 질량 분포 확인
```

**문제 3: 렌더링이 느림**
```
해결책:
1. 해상도 낮추기
2. 그림자 끄기
3. 복잡한 지오메트리 단순화
4. GPU 사용 확인
```

### 7.2 성능 최적화

**최적화 팁**
```python
def optimize_simulation():
    """시뮬레이션 최적화 설정"""
    
    # 1. 물리 최적화
    model.opt.timestep = 0.002  # 타임스텝 증가
    model.opt.iterations = 20   # 반복 횟수 감소
    
    # 2. 렌더링 최적화
    scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False  # 그림자 끄기
    scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False  # 반사 끄기
    
    # 3. 접촉 최적화
    model.opt.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL  # 간단한 접촉 모델
    
    # 4. 메모리 최적화
    model.opt.memory = 1000000  # 메모리 제한
```

---

## ✅ 프로젝트 1 완료 체크리스트

- [ ] MuJoCo 시뮬레이션 실행 성공
- [ ] 키보드로 로봇 제어 가능
- [ ] 프로그래밍으로 관절 제어 구현
- [ ] 센서 데이터 읽기 성공
- [ ] 카메라 이미지 캡처 성공
- [ ] 기본 실습 과제 완료
- [ ] XML 파일 수정 경험
- [ ] 다른 시뮬레이터 실행 시도

## 🎓 학습 정리

1. **시뮬레이션 기초**: MuJoCo를 통한 로봇 시뮬레이션
2. **제어 방법**: 키보드 및 프로그래밍 제어
3. **센서 활용**: 관절, 접촉, 카메라 센서
4. **설정 조정**: XML 파일을 통한 커스터마이징
5. **문제 해결**: 일반적인 이슈 대응 방법

---

[← 8장 메인](README.md) | [다음: 8.2 커스텀 제어 스크립트 →](02_custom_control.md)
