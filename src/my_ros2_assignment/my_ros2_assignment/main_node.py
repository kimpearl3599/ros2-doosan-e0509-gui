import sys
import signal
import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

from my_ros2_assignment.gui.main_window import MainWindow
from my_ros2_assignment.robot.controller import RobotController
from my_ros2_assignment.threads.move_thread import MoveThread


class RobotGUINode(Node):
    """ROS2 노드 + PyQt5 GUI 통합"""

    def __init__(self):
        super().__init__('my_ros2_assignment')
        self.get_logger().info('두산 E0509 제어 GUI 노드 시작')

        # 로봇 컨트롤러 생성
        self.controller = RobotController(self)

        # GUI 생성
        self.window = MainWindow()

        # 이동 스레드 (아직 시작 안 함)
        self._move_thread = None

        # 대기 중인 Future들 (폴링으로 처리)
        self._pending_pose_future = None
        self._pending_pose_type = None
        self._pending_validate_future = None
        self._pending_validate_coord = None
        self._pending_return_future = None
        self._pending_return_type = None  # 'previous' or 'home'

        # GUI 시그널 연결
        self.window.request_move.connect(self._on_request_move)
        self.window.request_stop.connect(self._on_request_stop)
        self.window.request_validate.connect(self._on_request_validate)
        self.window.request_ready_pose.connect(self._on_request_ready_pose)
        self.window.request_return_previous.connect(self._on_request_return_previous)
        self.window.request_home_return.connect(self._on_request_home_return)

        # 상태 업데이트 타이머 (500ms 주기)
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(500)

        self.window.append_log('시스템 초기화 완료')
        self.window.append_log('에뮬레이터와 Gazebo가 실행 중이어야 합니다.')
        self.window.show()

    def _on_request_move(self, targets, velocity, acceleration, is_absolute):
        """실행 버튼 클릭 시 호출"""
        if self._move_thread is not None and self._move_thread.isRunning():
            self.window.append_log('이미 이동 중입니다.')
            return

        if not self.controller.is_connected:
            self.window.append_log('로봇이 연결되지 않았습니다.')
            return

        self._move_thread = MoveThread(self.controller, self)

        # 스레드 시그널 → GUI 슬롯 연결
        self._move_thread.log_message.connect(self.window.append_log)
        self._move_thread.joint_updated.connect(self.window.update_joints)
        self._move_thread.position_updated.connect(self.window.update_ee_position)
        self._move_thread.status_changed.connect(self.window.update_motion_status)
        self._move_thread.movement_started.connect(lambda: self.window.set_moving_state(True))
        self._move_thread.movement_finished.connect(lambda: self.window.set_moving_state(False))
        self._move_thread.movement_error.connect(
            lambda msg: self.window.append_log(f'오류: {msg}')
        )

        self._move_thread.set_targets(targets, velocity, acceleration, is_absolute)
        self._move_thread.start()

    def _on_request_stop(self):
        """정지 버튼 클릭 시 호출"""
        if self._move_thread is not None and self._move_thread.isRunning():
            self._move_thread.stop()

    def _on_request_validate(self, x, y, z, rx, ry, rz):
        """좌표 검증 요청 - Ikin 서비스로 도달 가능 여부 확인"""
        if not self.controller.is_connected:
            # 로봇 미연결 시 기본 거리 검증만 수행
            reach = (x**2 + y**2 + z**2) ** 0.5
            if reach > 509:
                self.window.on_validation_result(False, '로봇 미연결 (거리 초과)')
            else:
                self.window.on_validation_result(True, '로봇 미연결 (거리 검증만)')
            return

        future = self.controller.check_reachability(x, y, z, rx, ry, rz)
        if future is None:
            self.window.on_validation_result(True, 'Ikin 서비스 불가 (기본 추가)')
            return

        # Future 저장 (폴링으로 처리)
        self._pending_validate_future = future
        self._pending_validate_coord = (x, y, z, rx, ry, rz)

    def _on_request_ready_pose(self):
        """작업 자세로 전환 요청"""
        if not self.controller.is_connected:
            self.window.on_pose_change_result(False, 'ready')
            self.window.append_log('로봇이 연결되지 않았습니다.')
            return

        future = self.controller.move_to_ready_position()
        if future is None:
            self.window.on_pose_change_result(False, 'ready')
            return

        # Future 저장 (폴링으로 처리)
        self._pending_pose_future = future
        self._pending_pose_type = 'ready'

    def _on_request_return_previous(self):
        """이전 위치로 복귀 요청"""
        if self._move_thread is not None and self._move_thread.isRunning():
            self.window.append_log('이동 중에는 이전 위치 복귀가 불가능합니다.')
            return

        if not self.controller.is_connected:
            self.window.append_log('로봇이 연결되지 않았습니다.')
            return

        prev_pos = self.window.get_previous_position()
        if prev_pos is None:
            self.window.append_log('저장된 이전 위치가 없습니다.')
            return

        x, y, z, rx, ry, rz = prev_pos
        self.window.append_log(f'이전 위치로 복귀 중: ({x:.1f}, {y:.1f}, {z:.1f})')
        self.window.update_motion_status('이전 위치 복귀 중')

        future = self.controller.move_to_position(
            x, y, z, rx, ry, rz,
            velocity=100.0, acceleration=100.0,
            is_absolute=True
        )

        if future is None:
            self.window.append_log('이동 서비스 호출 실패')
            self.window.update_motion_status('오류')
            return

        self._pending_return_future = future
        self._pending_return_type = 'previous'

    def _on_request_home_return(self):
        """홈 위치(작업 자세)로 복귀 요청"""
        if self._move_thread is not None and self._move_thread.isRunning():
            self.window.append_log('이동 중에는 홈 복귀가 불가능합니다.')
            return

        if not self.controller.is_connected:
            self.window.append_log('로봇이 연결되지 않았습니다.')
            return

        self.window.append_log('홈 위치(작업 자세)로 복귀 중...')
        self.window.update_motion_status('홈 복귀 중')

        # 작업 자세(싱귤러리티 회피)로 이동
        future = self.controller.move_to_ready_position(velocity=30.0, acceleration=30.0)

        if future is None:
            self.window.append_log('이동 서비스 호출 실패')
            self.window.update_motion_status('오류')
            return

        self._pending_return_future = future
        self._pending_return_type = 'home'

    def _update_status(self):
        """주기적으로 로봇 상태 업데이트"""
        # 연결 상태
        self.window.update_connection_status(self.controller.is_connected)

        # 대기 중인 자세 전환 Future 확인
        if self._pending_pose_future is not None:
            if self._pending_pose_future.done():
                try:
                    result = self._pending_pose_future.result()
                    success = result.success if hasattr(result, 'success') else (result.res == 0)
                    self.window.on_pose_change_result(success, self._pending_pose_type)
                except Exception as e:
                    self.window.append_log(f'자세 전환 오류: {str(e)}')
                    self.window.on_pose_change_result(False, self._pending_pose_type)
                finally:
                    self._pending_pose_future = None
                    self._pending_pose_type = None

        # 대기 중인 좌표 검증 Future 확인
        if self._pending_validate_future is not None:
            if self._pending_validate_future.done():
                try:
                    result = self._pending_validate_future.result()
                    if result.success:
                        self.window.on_validation_result(True, '')
                    else:
                        self.window.on_validation_result(False, '역기구학 실패 - 싱귤러리티 또는 범위 초과')
                except Exception as e:
                    self.window.on_validation_result(False, f'검증 오류: {str(e)}')
                finally:
                    self._pending_validate_future = None
                    self._pending_validate_coord = None

        # 대기 중인 복귀 Future 확인 (이전 위치 / 홈)
        if self._pending_return_future is not None:
            if self._pending_return_future.done():
                try:
                    result = self._pending_return_future.result()
                    success = result.success if hasattr(result, 'success') else True
                    if success:
                        if self._pending_return_type == 'previous':
                            self.window.append_log('이전 위치 복귀 완료')
                        else:
                            self.window.append_log('홈 복귀 완료')
                        self.window.update_motion_status('대기')
                    else:
                        self.window.append_log('복귀 실패')
                        self.window.update_motion_status('오류')
                except Exception as e:
                    self.window.append_log(f'복귀 오류: {str(e)}')
                    self.window.update_motion_status('오류')
                finally:
                    self._pending_return_future = None
                    self._pending_return_type = None

        if not self.controller.is_connected:
            return

        # 관절 각도 업데이트
        joints = self.controller.get_current_joints()
        self.window.update_joints(joints)

        # FK로 EE 위치 계산
        future = self.controller._get_position_by_fkin()
        if future is not None:
            future.add_done_callback(self._on_fkin_result)

    def _on_fkin_result(self, future):
        """FK 결과 콜백"""
        try:
            result = future.result()
            if result.success:
                pos = result.conv_posx
                self.window.update_ee_position(pos[0], pos[1], pos[2])
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)

    app = QApplication(sys.argv)

    node = RobotGUINode()

    # QTimer로 ROS2 spin_once를 주기적으로 호출
    ros_timer = QTimer()
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01))
    ros_timer.start(10)  # 10ms 주기

    # Ctrl+C 시그널 처리
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    # Qt 이벤트 루프 실행
    exit_code = app.exec_()

    # 정리
    node.get_logger().info('노드 종료 중...')
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
