import time
from PyQt5.QtCore import QThread, pyqtSignal
from my_ros2_assignment.robot.controller import RobotController


class MoveThread(QThread):
    """로봇 이동을 백그라운드에서 실행하는 QThread 클래스"""

    # GUI로 전송할 시그널
    log_message = pyqtSignal(str)
    joint_updated = pyqtSignal(list)
    position_updated = pyqtSignal(float, float, float)
    movement_started = pyqtSignal()
    movement_finished = pyqtSignal()
    movement_error = pyqtSignal(str)
    status_changed = pyqtSignal(str)  # 대기, 이동 중, 완료, 정지, 오류
    position_before_move = pyqtSignal(float, float, float, float, float, float)  # 이동 전 위치 기록용

    def __init__(self, controller: RobotController, node):
        super().__init__()
        self._controller = controller
        self._node = node
        self._targets = []      # [(x, y, z), ...]
        self._velocity = 100.0
        self._acceleration = 100.0
        self._is_absolute = True
        self._stop_flag = False

    def set_targets(self, targets: list, velocity: float,
                    acceleration: float, is_absolute: bool):
        self._targets = targets
        self._velocity = velocity
        self._acceleration = acceleration
        self._is_absolute = is_absolute
        self._stop_flag = False

    def stop(self):
        """이동 중지 요청"""
        self._stop_flag = True
        self._controller.stop_movement()
        self.log_message.emit('정지 요청됨 - 현재 위치에서 멈춤')

    def run(self):
        """스레드 메인 루프 - 좌표 목록을 순회하며 이동"""
        self.movement_started.emit()
        self.status_changed.emit('이동 중')

        total = len(self._targets)
        self.log_message.emit(f'총 {total}개 목표 이동 시작')

        for i, target in enumerate(self._targets):
            if self._stop_flag:
                self.log_message.emit('사용자에 의해 정지됨')
                self.status_changed.emit('정지')
                self.movement_finished.emit()
                return

            x, y, z, rx, ry, rz = target
            self.log_message.emit(f'[{i+1}/{total}] 목표 pos({x:.1f}, {y:.1f}, {z:.1f}) rot({rx:.0f}, {ry:.0f}, {rz:.0f}) 이동 중...')

            # MoveLine 서비스 호출
            future = self._controller.move_to_position(
                x, y, z,
                rx=rx, ry=ry, rz=rz,
                velocity=self._velocity,
                acceleration=self._acceleration,
                is_absolute=self._is_absolute
            )

            if future is None:
                self.log_message.emit(f'[{i+1}/{total}] 서비스 호출 실패')
                self.movement_error.emit('move_line 서비스 호출 실패')
                self.status_changed.emit('오류')
                self.movement_finished.emit()
                return

            # 서비스 응답 대기 (메인 스레드에서 spin 중이므로 여기서는 폴링만)
            timeout_time = time.time() + 30.0  # 30초 타임아웃
            while not future.done():
                if self._stop_flag:
                    self.log_message.emit('정지 요청으로 대기 중단')
                    self.status_changed.emit('정지')
                    self.movement_finished.emit()
                    return

                if time.time() > timeout_time:
                    self.log_message.emit(f'[{i+1}/{total}] 타임아웃')
                    self.movement_error.emit('이동 타임아웃')
                    self.status_changed.emit('오류')
                    self.movement_finished.emit()
                    return

                # 관절 상태만 업데이트 (메인 스레드가 구독 콜백 처리)
                joints = self._controller.get_current_joints()
                self.joint_updated.emit(joints)
                time.sleep(0.1)

            # 응답 확인
            try:
                result = future.result()
                if result.success:
                    self.log_message.emit(f'[{i+1}/{total}] 목표 도달 완료')
                    # 성공한 위치를 이전 위치로 기록
                    self.position_before_move.emit(x, y, z, rx, ry, rz)
                else:
                    self.log_message.emit(f'[{i+1}/{total}] 이동 실패')
                    self.movement_error.emit(f'목표 {i+1} 이동 실패')
            except Exception as e:
                self.log_message.emit(f'[{i+1}/{total}] 오류: {str(e)}')
                self.movement_error.emit(str(e))

            time.sleep(0.5)

        self.log_message.emit('모든 목표 이동 완료')
        self.status_changed.emit('완료')
        self.movement_finished.emit()
