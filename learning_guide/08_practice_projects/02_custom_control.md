# 8.2 프로젝트 2: 커스텀 제어 스크립트

다양한 입력 장치와 제어 방식을 활용한 커스텀 제어 시스템 구현 프로젝트입니다.

## 🎯 프로젝트 목표

- 키보드 제어 시스템 구축
- 조이스틱/게임패드 통합
- 음성 명령 제어 구현
- GUI 제어 인터페이스 개발
- 실시간 파라미터 조정

**난이도**: ⭐⭐ (중급)  
**소요 시간**: 2시간  
**선수 지식**: 1-4장

---

## 1. 키보드 제어 시스템

### 1.1 고급 키보드 제어

**다층 키보드 매핑**
```python
import pygame
import numpy as np
from enum import Enum

class ControlMode(Enum):
    """제어 모드"""
    JOINT = 1           # 관절 제어
    CARTESIAN = 2       # 데카르트 좌표 제어
    GRIPPER = 3         # 그리퍼 제어
    BASE = 4            # 베이스 이동

class KeyboardController:
    def __init__(self, robot):
        self.robot = robot
        self.mode = ControlMode.JOINT
        
        # Pygame 초기화
        pygame.init()
        self.screen = pygame.display.set_mode((400, 300))
        pygame.display.set_caption("XLeRobot Keyboard Control")
        
        # 제어 속도
        self.joint_speed = 0.1      # rad/s
        self.cartesian_speed = 0.01 # m/s
        self.base_speed = 0.2       # m/s
        self.gripper_speed = 0.05   # 0-1
        
        # 현재 상태
        self.current_joint_idx = 0  # 제어 중인 관절
        self.gripper_position = 0.0
        
        # 키 매핑
        self.setup_key_mapping()
        
    def setup_key_mapping(self):
        """키 매핑 설정"""
        self.key_map = {
            # 모드 전환
            pygame.K_1: lambda: self.switch_mode(ControlMode.JOINT),
            pygame.K_2: lambda: self.switch_mode(ControlMode.CARTESIAN),
            pygame.K_3: lambda: self.switch_mode(ControlMode.GRIPPER),
            pygame.K_4: lambda: self.switch_mode(ControlMode.BASE),
            
            # 관절 선택 (숫자키 패드)
            pygame.K_KP0: lambda: self.select_joint(0),
            pygame.K_KP1: lambda: self.select_joint(1),
            pygame.K_KP2: lambda: self.select_joint(2),
            pygame.K_KP3: lambda: self.select_joint(3),
            pygame.K_KP4: lambda: self.select_joint(4),
            pygame.K_KP5: lambda: self.select_joint(5),
            pygame.K_KP6: lambda: self.select_joint(6),
            
            # 속도 조정
            pygame.K_PLUS: self.increase_speed,
            pygame.K_MINUS: self.decrease_speed,
            
            # 홈 포지션
            pygame.K_h: self.go_home,
            
            # 긴급 정지
            pygame.K_SPACE: self.emergency_stop,
        }
    
    def switch_mode(self, mode):
        """제어 모드 전환"""
        self.mode = mode
        print(f"🔄 제어 모드: {mode.name}")
        
    def select_joint(self, joint_idx):
        """제어할 관절 선택"""
        self.current_joint_idx = joint_idx
        print(f"🎯 관절 {joint_idx} 선택")
        
    def increase_speed(self):
        """제어 속도 증가"""
        if self.mode == ControlMode.JOINT:
            self.joint_speed *= 1.2
            print(f"⬆️ 관절 속도: {self.joint_speed:.3f} rad/s")
        elif self.mode == ControlMode.CARTESIAN:
            self.cartesian_speed *= 1.2
            print(f"⬆️ 직교 속도: {self.cartesian_speed:.4f} m/s")
            
    def decrease_speed(self):
        """제어 속도 감소"""
        if self.mode == ControlMode.JOINT:
            self.joint_speed *= 0.8
            print(f"⬇️ 관절 속도: {self.joint_speed:.3f} rad/s")
        elif self.mode == ControlMode.CARTESIAN:
            self.cartesian_speed *= 0.8
            print(f"⬇️ 직교 속도: {self.cartesian_speed:.4f} m/s")
    
    def process_input(self):
        """입력 처리"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
                
            # 키 이벤트 처리
            if event.type == pygame.KEYDOWN:
                if event.key in self.key_map:
                    self.key_map[event.key]()
        
        # 연속 키 입력 (방향키)
        keys = pygame.key.get_pressed()
        
        if self.mode == ControlMode.JOINT:
            self.control_joint(keys)
        elif self.mode == ControlMode.CARTESIAN:
            self.control_cartesian(keys)
        elif self.mode == ControlMode.GRIPPER:
            self.control_gripper(keys)
        elif self.mode == ControlMode.BASE:
            self.control_base(keys)
            
        return True
    
    def control_joint(self, keys):
        """관절 제어"""
        delta = 0
        
        if keys[pygame.K_UP] or keys[pygame.K_w]:
            delta = self.joint_speed
        elif keys[pygame.K_DOWN] or keys[pygame.K_s]:
            delta = -self.joint_speed
            
        if delta != 0:
            current_angles = self.robot.get_joint_angles()
            current_angles[self.current_joint_idx] += delta * 0.016  # ~60fps
            
            # 관절 한계 체크
            current_angles = self.clip_joint_angles(current_angles)
            
            # 로봇에 적용
            self.robot.set_joint_angles(current_angles)
    
    def control_cartesian(self, keys):
        """데카르트 좌표 제어"""
        delta_pos = np.zeros(3)
        
        # X축 (전후)
        if keys[pygame.K_w]:
            delta_pos[0] = self.cartesian_speed
        elif keys[pygame.K_s]:
            delta_pos[0] = -self.cartesian_speed
            
        # Y축 (좌우)
        if keys[pygame.K_a]:
            delta_pos[1] = self.cartesian_speed
        elif keys[pygame.K_d]:
            delta_pos[1] = -self.cartesian_speed
            
        # Z축 (상하)
        if keys[pygame.K_q]:
            delta_pos[2] = self.cartesian_speed
        elif keys[pygame.K_e]:
            delta_pos[2] = -self.cartesian_speed
        
        if np.any(delta_pos != 0):
            # 현재 위치
            current_pos, current_rot = self.robot.get_end_effector_pose()
            
            # 목표 위치
            target_pos = current_pos + delta_pos * 0.016
            
            # 역기구학으로 관절 각도 계산
            joint_angles = self.robot.inverse_kinematics(target_pos, current_rot)
            
            if joint_angles is not None:
                self.robot.set_joint_angles(joint_angles)
    
    def control_gripper(self, keys):
        """그리퍼 제어"""
        if keys[pygame.K_o]:  # Open
            self.gripper_position = max(0, self.gripper_position - self.gripper_speed * 0.016)
        elif keys[pygame.K_c]:  # Close
            self.gripper_position = min(1, self.gripper_position + self.gripper_speed * 0.016)
            
        self.robot.set_gripper_position(self.gripper_position)
    
    def control_base(self, keys):
        """베이스 이동 제어"""
        linear = 0
        angular = 0
        
        if keys[pygame.K_w]:
            linear = self.base_speed
        elif keys[pygame.K_s]:
            linear = -self.base_speed
            
        if keys[pygame.K_a]:
            angular = self.base_speed
        elif keys[pygame.K_d]:
            angular = -self.base_speed
            
        if linear != 0 or angular != 0:
            self.robot.set_base_velocity(linear, angular)
    
    def clip_joint_angles(self, angles):
        """관절 각도 제한"""
        joint_limits = self.robot.get_joint_limits()
        return np.clip(angles, joint_limits[:, 0], joint_limits[:, 1])
    
    def go_home(self):
        """홈 포지션으로 이동"""
        print("🏠 홈 포지션으로 이동")
        home_position = [0, -0.5, 0, -1.5, 0, 1.0, 0]
        self.robot.move_to_joint_position(home_position, duration=3.0)
    
    def emergency_stop(self):
        """긴급 정지"""
        print("🛑 긴급 정지!")
        self.robot.stop()
    
    def render_ui(self):
        """UI 렌더링"""
        self.screen.fill((30, 30, 30))
        
        font = pygame.font.Font(None, 24)
        
        # 현재 모드 표시
        mode_text = font.render(f"Mode: {self.mode.name}", True, (255, 255, 255))
        self.screen.blit(mode_text, (10, 10))
        
        # 현재 관절 표시
        if self.mode == ControlMode.JOINT:
            joint_text = font.render(f"Joint: {self.current_joint_idx}", True, (255, 255, 0))
            self.screen.blit(joint_text, (10, 40))
        
        # 속도 표시
        speed_text = font.render(f"Speed: {self.joint_speed:.3f}", True, (0, 255, 0))
        self.screen.blit(speed_text, (10, 70))
        
        # 키 안내
        help_texts = [
            "1-4: Mode Switch",
            "W/S/A/D: Movement",
            "Q/E: Up/Down",
            "H: Home",
            "SPACE: Stop"
        ]
        
        y_pos = 120
        for text in help_texts:
            help_surface = font.render(text, True, (150, 150, 150))
            self.screen.blit(help_surface, (10, y_pos))
            y_pos += 25
        
        pygame.display.flip()
    
    def run(self):
        """메인 루프"""
        clock = pygame.time.Clock()
        running = True
        
        print("⌨️  키보드 제어 시작!")
        print("도움말:")
        print("  1-4: 제어 모드 전환")
        print("  W/A/S/D: 이동")
        print("  Q/E: 상하")
        print("  H: 홈 포지션")
        print("  SPACE: 긴급 정지")
        
        while running:
            running = self.process_input()
            self.render_ui()
            clock.tick(60)
        
        pygame.quit()

# 사용 예제
def main():
    robot = XLeRobotMuJoCo()
    controller = KeyboardController(robot)
    controller.run()

if __name__ == "__main__":
    main()
```

---

## 2. 조이스틱 제어

### 2.1 Xbox/PlayStation 컨트롤러 통합

**조이스틱 매핑**
```python
class JoystickController:
    def __init__(self, robot):
        self.robot = robot
        
        # Pygame 조이스틱 초기화
        pygame.init()
        pygame.joystick.init()
        
        # 조이스틱 연결 확인
        if pygame.joystick.get_count() == 0:
            raise RuntimeError("조이스틱이 연결되지 않았습니다!")
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        print(f"🎮 연결됨: {self.joystick.get_name()}")
        print(f"  축: {self.joystick.get_numaxes()}")
        print(f"  버튼: {self.joystick.get_numbuttons()}")
        print(f"  햇: {self.joystick.get_numhats()}")
        
        # 제어 설정
        self.deadzone = 0.1         # 데드존
        self.cartesian_scale = 0.3  # 직교 제어 스케일
        self.angular_scale = 1.0    # 각도 제어 스케일
        
        # 버튼 매핑 (Xbox 컨트롤러 기준)
        self.button_map = {
            0: self.button_a,       # A: 그리퍼 닫기
            1: self.button_b,       # B: 그리퍼 열기
            2: self.button_x,       # X: 모드 전환
            3: self.button_y,       # Y: 홈 포지션
            4: self.button_lb,      # LB: 속도 감소
            5: self.button_rb,      # RB: 속도 증가
            6: self.button_back,    # Back: 데이터 저장
            7: self.button_start,   # Start: 긴급 정지
        }
        
        self.mode = ControlMode.CARTESIAN
        self.gripper_state = 0.0
        
    def apply_deadzone(self, value):
        """데드존 적용"""
        if abs(value) < self.deadzone:
            return 0.0
        return value
    
    def process_input(self):
        """입력 처리"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            # 버튼 이벤트
            if event.type == pygame.JOYBUTTONDOWN:
                button_id = event.button
                if button_id in self.button_map:
                    self.button_map[button_id]()
        
        # 아날로그 스틱 입력
        if self.mode == ControlMode.CARTESIAN:
            self.control_cartesian_joystick()
        elif self.mode == ControlMode.JOINT:
            self.control_joint_joystick()
        
        return True
    
    def control_cartesian_joystick(self):
        """조이스틱으로 데카르트 제어"""
        # 왼쪽 스틱: X, Y 이동
        left_x = self.apply_deadzone(self.joystick.get_axis(0))
        left_y = self.apply_deadzone(self.joystick.get_axis(1))
        
        # 오른쪽 스틱: Z 이동, 회전
        right_x = self.apply_deadzone(self.joystick.get_axis(2))
        right_y = self.apply_deadzone(self.joystick.get_axis(3))
        
        # 트리거: 추가 제어
        left_trigger = self.joystick.get_axis(4)   # -1 ~ 1
        right_trigger = self.joystick.get_axis(5)  # -1 ~ 1
        
        # 위치 변화량 계산
        delta_pos = np.array([
            -left_y * self.cartesian_scale,   # 전후
            left_x * self.cartesian_scale,    # 좌우
            -right_y * self.cartesian_scale   # 상하
        ]) * 0.016  # 60fps 기준
        
        # 회전 변화량 (간단한 yaw 제어)
        delta_yaw = right_x * self.angular_scale * 0.016
        
        if np.any(delta_pos != 0) or delta_yaw != 0:
            # 현재 포즈
            current_pos, current_rot = self.robot.get_end_effector_pose()
            
            # 목표 위치
            target_pos = current_pos + delta_pos
            
            # 회전 적용 (간단한 Z축 회전)
            from scipy.spatial.transform import Rotation as R
            current_euler = R.from_matrix(current_rot).as_euler('xyz')
            current_euler[2] += delta_yaw
            target_rot = R.from_euler('xyz', current_euler).as_matrix()
            
            # 역기구학
            joint_angles = self.robot.inverse_kinematics(target_pos, target_rot)
            
            if joint_angles is not None:
                self.robot.set_joint_angles(joint_angles)
    
    def control_joint_joystick(self):
        """조이스틱으로 관절 제어"""
        # D-Pad로 관절 선택
        hat = self.joystick.get_hat(0)
        
        # 왼쪽 스틱으로 선택된 관절 제어
        left_y = self.apply_deadzone(self.joystick.get_axis(1))
        
        if left_y != 0:
            current_angles = self.robot.get_joint_angles()
            # 마지막 선택된 관절 제어 (구현 필요)
            # current_angles[selected_joint] += left_y * speed
            self.robot.set_joint_angles(current_angles)
    
    # 버튼 핸들러들
    def button_a(self):
        """A 버튼: 그리퍼 닫기"""
        print("🤏 그리퍼 닫기")
        self.gripper_state = 1.0
        self.robot.set_gripper_position(self.gripper_state)
    
    def button_b(self):
        """B 버튼: 그리퍼 열기"""
        print("✋ 그리퍼 열기")
        self.gripper_state = 0.0
        self.robot.set_gripper_position(self.gripper_state)
    
    def button_x(self):
        """X 버튼: 모드 전환"""
        modes = list(ControlMode)
        current_idx = modes.index(self.mode)
        self.mode = modes[(current_idx + 1) % len(modes)]
        print(f"🔄 모드: {self.mode.name}")
    
    def button_y(self):
        """Y 버튼: 홈 포지션"""
        print("🏠 홈 포지션")
        self.robot.go_home()
    
    def button_lb(self):
        """LB: 속도 감소"""
        self.cartesian_scale *= 0.8
        print(f"⬇️ 속도: {self.cartesian_scale:.2f}")
    
    def button_rb(self):
        """RB: 속도 증가"""
        self.cartesian_scale *= 1.2
        print(f"⬆️ 속도: {self.cartesian_scale:.2f}")
    
    def button_back(self):
        """Back: 현재 포즈 저장"""
        pose = self.robot.get_joint_angles()
        print(f"💾 포즈 저장: {pose}")
        # 파일에 저장 (구현 필요)
    
    def button_start(self):
        """Start: 긴급 정지"""
        print("🛑 긴급 정지!")
        self.robot.stop()
    
    def run(self):
        """메인 루프"""
        clock = pygame.time.Clock()
        running = True
        
        print("🎮 조이스틱 제어 시작!")
        
        while running:
            running = self.process_input()
            clock.tick(60)
        
        pygame.quit()
```

### 2.2 JoyCon 제어 (Nintendo Switch)

**JoyCon 통합**
```python
import sys
sys.path.append('../../software')
from joyconrobotics import JoyconRobotics

class JoyconController:
    def __init__(self, robot):
        self.robot = robot
        
        # JoyCon 연결
        print("🎮 JoyCon 연결 중...")
        self.joycon = JoyconRobotics()
        
        # 콜백 등록
        self.joycon.on_button_pressed = self.on_button
        self.joycon.on_stick_moved = self.on_stick
        self.joycon.on_gyro = self.on_gyro
        
        # 상태
        self.control_enabled = False
        self.use_gyro = False
        
    def on_button(self, button):
        """버튼 이벤트"""
        if button == 'A':
            self.control_enabled = not self.control_enabled
            print(f"{'▶️ ' if self.control_enabled else '⏸️ '} 제어 {'활성화' if self.control_enabled else '비활성화'}")
        
        elif button == 'B':
            self.robot.go_home()
            print("🏠 홈 포지션")
        
        elif button == 'X':
            self.use_gyro = not self.use_gyro
            print(f"🔄 자이로 제어: {'ON' if self.use_gyro else 'OFF'}")
        
        elif button == 'Y':
            # 현재 위치 저장
            self.save_waypoint()
        
        elif button == 'L':
            self.robot.set_gripper_position(0.0)
            print("✋ 그리퍼 열기")
        
        elif button == 'R':
            self.robot.set_gripper_position(1.0)
            print("🤏 그리퍼 닫기")
        
        elif button == 'PLUS':
            self.robot.stop()
            print("🛑 정지")
    
    def on_stick(self, x, y):
        """스틱 이벤트"""
        if not self.control_enabled:
            return
        
        # 직교 좌표 제어
        delta_pos = np.array([
            -y * 0.01,  # 전후
            x * 0.01,   # 좌우
            0.0
        ])
        
        current_pos, current_rot = self.robot.get_end_effector_pose()
        target_pos = current_pos + delta_pos
        
        joint_angles = self.robot.inverse_kinematics(target_pos, current_rot)
        if joint_angles is not None:
            self.robot.set_joint_angles(joint_angles)
    
    def on_gyro(self, accel, gyro):
        """자이로 센서 이벤트"""
        if not self.control_enabled or not self.use_gyro:
            return
        
        # 자이로 데이터로 회전 제어
        # (구현 예제)
        rotation_delta = np.array(gyro) * 0.001
        
        current_pos, current_rot = self.robot.get_end_effector_pose()
        
        from scipy.spatial.transform import Rotation as R
        delta_rot = R.from_rotvec(rotation_delta)
        new_rot = delta_rot.as_matrix() @ current_rot
        
        joint_angles = self.robot.inverse_kinematics(current_pos, new_rot)
        if joint_angles is not None:
            self.robot.set_joint_angles(joint_angles)
    
    def save_waypoint(self):
        """현재 위치를 웨이포인트로 저장"""
        pose = self.robot.get_joint_angles()
        timestamp = time.time()
        
        waypoint = {
            'timestamp': timestamp,
            'joint_angles': pose.tolist(),
            'ee_pose': self.robot.get_end_effector_pose()
        }
        
        # JSON으로 저장
        filename = f"waypoint_{int(timestamp)}.json"
        with open(filename, 'w') as f:
            json.dump(waypoint, f, indent=2)
        
        print(f"💾 웨이포인트 저장: {filename}")
    
    def run(self):
        """메인 루프"""
        print("🎮 JoyCon 제어 시작!")
        print("버튼:")
        print("  A: 제어 ON/OFF")
        print("  B: 홈 포지션")
        print("  X: 자이로 제어")
        print("  Y: 웨이포인트 저장")
        print("  L/R: 그리퍼")
        
        self.joycon.start()
        
        try:
            while True:
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("\n종료 중...")
            self.joycon.stop()
```

---

## 3. 음성 명령 제어

### 3.1 음성 인식 시스템

**음성 명령 구현**
```python
import speech_recognition as sr
import pyttsx3
import threading

class VoiceController:
    def __init__(self, robot):
        self.robot = robot
        
        # 음성 인식기
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        
        # 음성 합성기
        self.tts_engine = pyttsx3.init()
        self.tts_engine.setProperty('rate', 150)
        
        # 명령어 매핑
        self.commands = {
            # 기본 동작
            '홈': self.go_home,
            '정지': self.stop,
            '시작': self.start,
            
            # 그리퍼
            '잡아': self.close_gripper,
            '놓아': self.open_gripper,
            '그리퍼 닫아': self.close_gripper,
            '그리퍼 열어': self.open_gripper,
            
            # 이동
            '위로': lambda: self.move_direction('up'),
            '아래로': lambda: self.move_direction('down'),
            '앞으로': lambda: self.move_direction('forward'),
            '뒤로': lambda: self.move_direction('backward'),
            '왼쪽으로': lambda: self.move_direction('left'),
            '오른쪽으로': lambda: self.move_direction('right'),
            
            # 사전 정의된 포즈
            '준비 자세': self.pose_ready,
            '대기 자세': self.pose_standby,
            '픽업 자세': self.pose_pickup,
            
            # 작업
            '물체 잡아': self.task_pick,
            '물체 놓아': self.task_place,
            '테이블 청소': self.task_clean_table,
        }
        
        # 상태
        self.listening = False
        self.active = False
        
    def speak(self, text):
        """음성 출력"""
        print(f"🔊 {text}")
        self.tts_engine.say(text)
        self.tts_engine.runAndWait()
    
    def listen_command(self):
        """음성 명령 듣기"""
        with self.microphone as source:
            print("🎤 듣는 중...")
            self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
            
            try:
                audio = self.recognizer.listen(source, timeout=5)
                print("🔄 인식 중...")
                
                # Google Speech Recognition 사용
                text = self.recognizer.recognize_google(audio, language='ko-KR')
                print(f"📝 인식: {text}")
                
                return text
                
            except sr.WaitTimeoutError:
                print("⏱️ 타임아웃")
                return None
            except sr.UnknownValueError:
                print("❓ 인식 실패")
                return None
            except sr.RequestError as e:
                print(f"❌ 오류: {e}")
                return None
    
    def process_command(self, command_text):
        """명령 처리"""
        if not command_text:
            return
        
        command_text = command_text.lower().strip()
        
        # 정확한 매칭
        if command_text in self.commands:
            self.speak(f"{command_text} 실행합니다")
            self.commands[command_text]()
            return True
        
        # 부분 매칭
        for key, func in self.commands.items():
            if key in command_text:
                self.speak(f"{key} 실행합니다")
                func()
                return True
        
        # 숫자 파싱 (예: "관절 1을 30도로")
        if '관절' in command_text and '도' in command_text:
            self.parse_joint_command(command_text)
            return True
        
        self.speak("명령을 이해하지 못했습니다")
        return False
    
    def parse_joint_command(self, text):
        """관절 명령 파싱"""
        import re
        
        # 숫자 추출
        numbers = re.findall(r'\d+', text)
        
        if len(numbers) >= 2:
            joint_idx = int(numbers[0])
            angle_deg = int(numbers[1])
            
            angle_rad = np.deg2rad(angle_deg)
            
            current_angles = self.robot.get_joint_angles()
            if 0 <= joint_idx < len(current_angles):
                current_angles[joint_idx] = angle_rad
                self.robot.set_joint_angles(current_angles)
                
                self.speak(f"관절 {joint_idx}을 {angle_deg}도로 설정했습니다")
    
    # 명령 구현들
    def go_home(self):
        self.robot.go_home()
    
    def stop(self):
        self.robot.stop()
        self.active = False
    
    def start(self):
        self.active = True
    
    def close_gripper(self):
        self.robot.set_gripper_position(1.0)
    
    def open_gripper(self):
        self.robot.set_gripper_position(0.0)
    
    def move_direction(self, direction):
        """방향으로 이동"""
        distance = 0.05  # 5cm
        
        current_pos, current_rot = self.robot.get_end_effector_pose()
        delta = np.zeros(3)
        
        if direction == 'up':
            delta[2] = distance
        elif direction == 'down':
            delta[2] = -distance
        elif direction == 'forward':
            delta[0] = distance
        elif direction == 'backward':
            delta[0] = -distance
        elif direction == 'left':
            delta[1] = distance
        elif direction == 'right':
            delta[1] = -distance
        
        target_pos = current_pos + delta
        joint_angles = self.robot.inverse_kinematics(target_pos, current_rot)
        
        if joint_angles is not None:
            self.robot.set_joint_angles(joint_angles)
    
    def pose_ready(self):
        """준비 자세"""
        pose = [0, -0.5, 0, -1.5, 0, 1.0, 0]
        self.robot.move_to_joint_position(pose, duration=2.0)
    
    def pose_standby(self):
        """대기 자세"""
        pose = [0, -1.0, 0, -2.0, 0, 1.5, 0]
        self.robot.move_to_joint_position(pose, duration=2.0)
    
    def pose_pickup(self):
        """픽업 자세"""
        pose = [0, -0.3, 0.5, -1.2, 0, 0.9, 0]
        self.robot.move_to_joint_position(pose, duration=2.0)
    
    def task_pick(self):
        """물체 잡기 작업"""
        self.speak("물체를 잡겠습니다")
        # 픽 시퀀스 실행
        self.pose_pickup()
        time.sleep(2)
        self.close_gripper()
        time.sleep(1)
        self.move_direction('up')
        self.speak("완료했습니다")
    
    def task_place(self):
        """물체 놓기 작업"""
        self.speak("물체를 놓겠습니다")
        self.move_direction('down')
        time.sleep(1)
        self.open_gripper()
        time.sleep(1)
        self.move_direction('up')
        self.speak("완료했습니다")
    
    def task_clean_table(self):
        """테이블 청소 작업"""
        self.speak("테이블 청소를 시작합니다")
        # 청소 루틴 구현
        # ...
        self.speak("청소를 완료했습니다")
    
    def run(self):
        """메인 루프"""
        self.speak("음성 제어 시스템을 시작합니다")
        self.speak("명령을 말씀해주세요")
        
        self.listening = True
        
        while self.listening:
            command = self.listen_command()
            
            if command:
                if '종료' in command or '그만' in command:
                    self.speak("음성 제어를 종료합니다")
                    break
                
                self.process_command(command)

# 사용 예제
def main():
    robot = XLeRobotMuJoCo()
    voice_controller = VoiceController(robot)
    voice_controller.run()
```

---

## 4. GUI 제어 인터페이스

### 4.1 PyQt 기반 GUI

**GUI 컨트롤러**
```python
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QPushButton, QSlider, QLabel, 
                             QTabWidget, QGroupBox, QGridLayout)
from PyQt5.QtCore import Qt, QTimer
import sys

class RobotControlGUI(QMainWindow):
    def __init__(self, robot):
        super().__init__()
        self.robot = robot
        
        self.setWindowTitle("XLeRobot Control Panel")
        self.setGeometry(100, 100, 1200, 800)
        
        # 메인 위젯
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        
        # 레이아웃
        layout = QVBoxLayout()
        main_widget.setLayout(layout)
        
        # 탭 위젯
        tabs = QTabWidget()
        layout.addWidget(tabs)
        
        # 탭들 추가
        tabs.addTab(self.create_joint_control_tab(), "관절 제어")
        tabs.addTab(self.create_cartesian_control_tab(), "직교 제어")
        tabs.addTab(self.create_task_control_tab(), "작업 제어")
        tabs.addTab(self.create_monitoring_tab(), "모니터링")
        
        # 타이머 (상태 업데이트)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_status)
        self.timer.start(50)  # 20Hz
        
        # 슬라이더 저장
        self.joint_sliders = []
        
    def create_joint_control_tab(self):
        """관절 제어 탭"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # 각 관절마다 슬라이더 생성
        for i in range(7):
            group = QGroupBox(f"관절 {i+1}")
            group_layout = QHBoxLayout()
            
            # 슬라이더
            slider = QSlider(Qt.Horizontal)
            slider.setMinimum(-180)
            slider.setMaximum(180)
            slider.setValue(0)
            slider.valueChanged.connect(
                lambda value, idx=i: self.on_joint_slider_changed(idx, value)
            )
            
            # 값 레이블
            label = QLabel("0°")
            slider.valueChanged.connect(lambda value, lbl=label: lbl.setText(f"{value}°"))
            
            group_layout.addWidget(slider)
            group_layout.addWidget(label)
            
            group.setLayout(group_layout)
            layout.addWidget(group)
            
            self.joint_sliders.append(slider)
        
        # 버튼들
        button_layout = QHBoxLayout()
        
        home_btn = QPushButton("홈 포지션")
        home_btn.clicked.connect(self.go_home)
        button_layout.addWidget(home_btn)
        
        reset_btn = QPushButton("리셋")
        reset_btn.clicked.connect(self.reset_joints)
        button_layout.addWidget(reset_btn)
        
        layout.addLayout(button_layout)
        
        return tab
    
    def create_cartesian_control_tab(self):
        """직교 좌표 제어 탭"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # 위치 제어
        pos_group = QGroupBox("위치 제어")
        pos_layout = QGridLayout()
        
        directions = [
            ("↑ 위로", lambda: self.move_cartesian(0, 0, 0.01)),
            ("↓ 아래로", lambda: self.move_cartesian(0, 0, -0.01)),
            ("← 왼쪽", lambda: self.move_cartesian(0, 0.01, 0)),
            ("→ 오른쪽", lambda: self.move_cartesian(0, -0.01, 0)),
            ("↗ 앞으로", lambda: self.move_cartesian(0.01, 0, 0)),
            ("↙ 뒤로", lambda: self.move_cartesian(-0.01, 0, 0)),
        ]
        
        positions = [(0, 0), (1, 0), (0, 1), (1, 1), (0, 2), (1, 2)]
        
        for (text, func), (row, col) in zip(directions, positions):
            btn = QPushButton(text)
            btn.clicked.connect(func)
            pos_layout.addWidget(btn, row, col)
        
        pos_group.setLayout(pos_layout)
        layout.addWidget(pos_group)
        
        # 그리퍼 제어
        gripper_group = QGroupBox("그리퍼")
        gripper_layout = QHBoxLayout()
        
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setMinimum(0)
        self.gripper_slider.setMaximum(100)
        self.gripper_slider.valueChanged.connect(self.on_gripper_changed)
        
        gripper_label = QLabel("0%")
        self.gripper_slider.valueChanged.connect(
            lambda v: gripper_label.setText(f"{v}%")
        )
        
        gripper_layout.addWidget(QLabel("열림"))
        gripper_layout.addWidget(self.gripper_slider)
        gripper_layout.addWidget(QLabel("닫힘"))
        gripper_layout.addWidget(gripper_label)
        
        gripper_group.setLayout(gripper_layout)
        layout.addWidget(gripper_group)
        
        layout.addStretch()
        
        return tab
    
    def create_task_control_tab(self):
        """작업 제어 탭"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        tasks = [
            ("🏠 홈 포지션", self.go_home),
            ("📦 픽 데모", self.demo_pick),
            ("�� 플레이스 데모", self.demo_place),
            ("⭕ 원형 궤적", self.demo_circle),
            ("📍 웨이포인트 저장", self.save_waypoint),
            ("▶️ 웨이포인트 재생", self.replay_waypoints),
            ("🧹 테이블 청소", self.demo_clean_table),
        ]
        
        for text, func in tasks:
            btn = QPushButton(text)
            btn.setMinimumHeight(50)
            btn.clicked.connect(func)
            layout.addWidget(btn)
        
        layout.addStretch()
        
        return tab
    
    def create_monitoring_tab(self):
        """모니터링 탭"""
        tab = QWidget()
        layout = QVBoxLayout()
        tab.setLayout(layout)
        
        # 상태 표시
        self.status_labels = {}
        
        status_items = [
            "관절 위치",
            "EE 위치",
            "EE 회전",
            "그리퍼 상태",
            "접촉력",
        ]
        
        for item in status_items:
            group = QGroupBox(item)
            group_layout = QVBoxLayout()
            
            label = QLabel("N/A")
            label.setStyleSheet("font-family: monospace;")
            group_layout.addWidget(label)
            
            group.setLayout(group_layout)
            layout.addWidget(group)
            
            self.status_labels[item] = label
        
        layout.addStretch()
        
        return tab
    
    def on_joint_slider_changed(self, joint_idx, value_deg):
        """관절 슬라이더 변경"""
        value_rad = np.deg2rad(value_deg)
        
        current_angles = self.robot.get_joint_angles()
        current_angles[joint_idx] = value_rad
        
        self.robot.set_joint_angles(current_angles)
    
    def on_gripper_changed(self, value):
        """그리퍼 슬라이더 변경"""
        gripper_pos = value / 100.0
        self.robot.set_gripper_position(gripper_pos)
    
    def move_cartesian(self, dx, dy, dz):
        """직교 좌표 이동"""
        current_pos, current_rot = self.robot.get_end_effector_pose()
        target_pos = current_pos + np.array([dx, dy, dz])
        
        joint_angles = self.robot.inverse_kinematics(target_pos, current_rot)
        if joint_angles is not None:
            self.robot.set_joint_angles(joint_angles)
    
    def go_home(self):
        """홈 포지션"""
        self.robot.go_home()
    
    def reset_joints(self):
        """관절 리셋"""
        for slider in self.joint_sliders:
            slider.setValue(0)
    
    def demo_pick(self):
        """픽 데모"""
        # 구현 (프로젝트 1 참조)
        pass
    
    def demo_place(self):
        """플레이스 데모"""
        pass
    
    def demo_circle(self):
        """원형 궤적 데모"""
        pass
    
    def save_waypoint(self):
        """웨이포인트 저장"""
        pass
    
    def replay_waypoints(self):
        """웨이포인트 재생"""
        pass
    
    def demo_clean_table(self):
        """테이블 청소 데모"""
        pass
    
    def update_status(self):
        """상태 업데이트"""
        # 관절 위치
        joint_pos = self.robot.get_joint_angles()
        joint_text = ", ".join([f"{np.rad2deg(a):.1f}°" for a in joint_pos])
        self.status_labels["관절 위치"].setText(joint_text)
        
        # EE 위치
        ee_pos, ee_rot = self.robot.get_end_effector_pose()
        ee_pos_text = f"X: {ee_pos[0]:.3f}, Y: {ee_pos[1]:.3f}, Z: {ee_pos[2]:.3f}"
        self.status_labels["EE 위치"].setText(ee_pos_text)
        
        # EE 회전
        from scipy.spatial.transform import Rotation as R
        ee_euler = R.from_matrix(ee_rot).as_euler('xyz', degrees=True)
        ee_rot_text = f"R: {ee_euler[0]:.1f}°, P: {ee_euler[1]:.1f}°, Y: {ee_euler[2]:.1f}°"
        self.status_labels["EE 회전"].setText(ee_rot_text)

def main():
    app = QApplication(sys.argv)
    robot = XLeRobotMuJoCo()
    gui = RobotControlGUI(robot)
    gui.show()
    sys.exit(app.exec_())
```

---

## 5. 웹 기반 제어 (참조: 6장)

6장에서 다룬 웹 제어 시스템을 활용할 수 있습니다:

```bash
# 웹 제어 서버 시작
cd ../../web_control/server
python main.py

# 브라우저에서 접속
# http://localhost:8000
```

---

## 6. 실습 과제

### 과제 1: 멀티모달 제어
```python
class MultiModalController:
    """여러 입력 방식을 동시에 지원"""
    
    def __init__(self, robot):
        self.robot = robot
        
        # 각 컨트롤러 초기화
        self.keyboard = KeyboardController(robot)
        self.joystick = JoystickController(robot)
        self.voice = VoiceController(robot)
        
        # 우선순위
        self.priority = {
            'voice': 3,      # 가장 높음
            'joystick': 2,
            'keyboard': 1    # 가장 낮음
        }
        
        self.active_controller = None
        
    def run(self):
        """모든 컨트롤러 동시 실행"""
        # 각각 별도 스레드에서 실행
        # 우선순위에 따라 제어권 전환
        pass
```

### 과제 2: 제스처 제어
```python
import cv2
import mediapipe as mp

class GestureController:
    """손 제스처로 로봇 제어"""
    
    def __init__(self, robot):
        self.robot = robot
        
        # MediaPipe 손 인식
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.5
        )
        
    def recognize_gesture(self, hand_landmarks):
        """제스처 인식"""
        # 손가락 상태 확인
        # 제스처 분류
        # 로봇 명령 생성
        pass
```

---

## ✅ 프로젝트 2 완료 체크리스트

- [ ] 키보드 제어 구현
- [ ] 조이스틱 제어 구현
- [ ] 음성 명령 제어 구현
- [ ] GUI 인터페이스 개발
- [ ] 멀티모달 제어 통합
- [ ] 제스처 제어 실험
- [ ] 실시간 파라미터 조정 구현

## 🎓 학습 정리

1. **다양한 입력**: 키보드, 조이스틱, 음성, GUI
2. **제어 모드**: 관절, 직교, 그리퍼, 베이스
3. **사용자 경험**: 직관적 인터페이스 설계
4. **멀티모달**: 여러 입력 방식 통합
5. **확장성**: 새로운 제어 방식 추가 용이

---

[← 8.1 첫 시뮬레이션](01_first_simulation.md) | [다음: 8.3 픽앤플레이스 →](03_pick_and_place.md)
