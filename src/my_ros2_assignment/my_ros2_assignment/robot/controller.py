import time
import numpy as np
from typing import List, Tuple, Optional
from rclpy.node import Node
from sensor_msgs.msg import JointState
from dsr_msgs2.srv import MoveLine, MoveJoint, GetCurrentPosx, Fkin, MoveHome, Ikin, MoveStop


class RobotController:
    """두산 E0509 로봇 제어 클래스 (dsr_msgs2 서비스 사용)"""

    # 두산 로봇은 위치 단위: mm, 각도 단위: deg
    JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    CONNECTION_TIMEOUT = 3.0  # 3초 동안 메시지 없으면 연결 끊김 판정

    def __init__(self, node: Node):
        self._node = node
        self._current_joints = [0.0] * 6
        self._connected = False
        self._last_msg_time = 0.0  # 마지막 메시지 수신 시간

        # joint_states 구독
        self._joint_sub = node.create_subscription(
            JointState,
            '/dsr01/joint_states',
            self._joint_state_callback,
            10
        )

        # 서비스 클라이언트 생성
        self._cli_move_line = node.create_client(MoveLine, '/dsr01/motion/move_line')
        self._cli_move_joint = node.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self._cli_get_posx = node.create_client(GetCurrentPosx, '/dsr01/aux_control/get_current_posx')
        self._cli_fkin = node.create_client(Fkin, '/dsr01/motion/fkin')
        self._cli_move_home = node.create_client(MoveHome, '/dsr01/motion/move_home')
        self._cli_ikin = node.create_client(Ikin, '/dsr01/motion/ikin')
        self._cli_move_stop = node.create_client(MoveStop, '/dsr01/motion/move_stop')

    def _joint_state_callback(self, msg: JointState):
        """관절 상태 콜백 - 관절 순서를 정렬하여 저장"""
        self._last_msg_time = time.time()
        self._connected = True
        joints = [0.0] * 6
        for i, name in enumerate(msg.name):
            if name in self.JOINT_NAMES:
                idx = self.JOINT_NAMES.index(name)
                joints[idx] = np.degrees(msg.position[i])  # rad -> deg
        self._current_joints = joints

    @property
    def is_connected(self) -> bool:
        """연결 상태 확인 (타임아웃 기반)"""
        if not self._connected:
            return False
        # 마지막 메시지 이후 타임아웃 체크
        if self._last_msg_time > 0:
            elapsed = time.time() - self._last_msg_time
            if elapsed > self.CONNECTION_TIMEOUT:
                self._connected = False
                return False
        return True

    @property
    def seconds_since_last_message(self) -> float:
        """마지막 메시지 수신 후 경과 시간"""
        if self._last_msg_time == 0:
            return float('inf')
        return time.time() - self._last_msg_time

    def get_current_joints(self) -> List[float]:
        """현재 6개 관절 각도 반환 (deg)"""
        return list(self._current_joints)

    def get_current_position(self) -> Optional[Tuple[float, float, float]]:
        """현재 End-Effector XYZ 위치 반환 (mm)
        서비스 호출이 불가능하면 FK로 계산"""
        req = GetCurrentPosx.Request()
        req.ref = 0  # 베이스 좌표계

        if not self._cli_get_posx.wait_for_service(timeout_sec=1.0):
            return self._get_position_by_fkin()

        future = self._cli_get_posx.call_async(req)
        return future

    def get_current_position_sync(self) -> Optional[Tuple[float, float, float]]:
        """FK를 이용한 동기식 위치 계산"""
        return self._get_position_by_fkin()

    def _get_position_by_fkin(self) -> Optional[Tuple[float, float, float]]:
        """FK 서비스로 현재 위치 계산"""
        if not self._cli_fkin.wait_for_service(timeout_sec=1.0):
            return None
        req = Fkin.Request()
        req.pos = [float(j) for j in self._current_joints]
        req.ref = 0
        future = self._cli_fkin.call_async(req)
        return future

    def move_to_position(self, x: float, y: float, z: float,
                         rx: float = 0.0, ry: float = 180.0, rz: float = 0.0,
                         velocity: float = 100.0, acceleration: float = 100.0,
                         is_absolute: bool = True):
        """직선 이동 명령 (MoveLine 서비스)

        Args:
            x, y, z: 목표 위치 (mm)
            rx, ry, rz: 목표 자세 (deg)
            velocity: 속도 (mm/s)
            acceleration: 가속도 (mm/s²)
            is_absolute: True=절대좌표, False=상대좌표
        Returns:
            Future 객체
        """
        if not self._cli_move_line.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().error('move_line 서비스 사용 불가')
            return None

        req = MoveLine.Request()
        req.pos = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        req.vel = [float(velocity), float(velocity)]
        req.acc = [float(acceleration), float(acceleration)]
        req.time = 0.0
        req.radius = 0.0
        req.ref = 0   # 베이스 좌표계
        req.mode = 0 if is_absolute else 1  # 0=절대, 1=상대
        req.blend_type = 0
        req.sync_type = 1  # 1=동기 (완료까지 대기)

        future = self._cli_move_line.call_async(req)
        return future

    def move_home(self):
        """홈 위치로 이동 (모든 관절 0도 - 싱귤러리티 상태)"""
        if not self._cli_move_home.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().error('move_home 서비스 사용 불가')
            return None

        req = MoveHome.Request()
        req.target = 0
        future = self._cli_move_home.call_async(req)
        return future

    def move_to_joint_position(self, joints: List[float],
                                velocity: float = 30.0,
                                acceleration: float = 30.0):
        """관절 각도로 이동 (MoveJoint 서비스)

        Args:
            joints: 6개 관절 각도 [j1, j2, j3, j4, j5, j6] (deg)
            velocity: 관절 속도 (deg/s)
            acceleration: 관절 가속도 (deg/s²)
        Returns:
            Future 객체
        """
        if not self._cli_move_joint.wait_for_service(timeout_sec=2.0):
            self._node.get_logger().error('move_joint 서비스 사용 불가')
            return None

        req = MoveJoint.Request()
        req.pos = [float(j) for j in joints]
        req.vel = float(velocity)
        req.acc = float(acceleration)
        req.time = 0.0
        req.radius = 0.0
        req.mode = 0  # 절대 각도
        req.blend_type = 0
        req.sync_type = 1  # 동기 (완료까지 대기)

        future = self._cli_move_joint.call_async(req)
        return future

    def move_to_ready_position(self, velocity: float = 30.0, acceleration: float = 30.0):
        """작업 준비 자세로 이동 (싱귤러리티 회피)

        E0509 로봇의 작업하기 좋은 자세:
        - J1=0°, J2=-20°, J3=90°, J4=0°, J5=70°, J6=0°
        - End-Effector가 전방 아래를 향하는 자세
        """
        ready_joints = [0.0, -20.0, 90.0, 0.0, 70.0, 0.0]
        return self.move_to_joint_position(ready_joints, velocity, acceleration)

    def stop_movement(self):
        """로봇 이동 즉시 정지 (현재 위치에서 멈춤)"""
        self._node.get_logger().warn('정지 요청 - 현재 위치에서 멈춤')

        if not self._cli_move_stop.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().error('move_stop 서비스 사용 불가')
            return None

        req = MoveStop.Request()
        req.stop_mode = 0  # 0: 즉시 정지 (STOP_TYPE_QUICK)

        future = self._cli_move_stop.call_async(req)
        return future

    def check_reachability(self, x: float, y: float, z: float,
                           rx: float = 0.0, ry: float = 180.0, rz: float = 0.0,
                           sol_space: int = 0):
        """역기구학(Ikin)으로 좌표 도달 가능 여부 확인

        Args:
            x, y, z: 목표 위치 (mm)
            rx, ry, rz: 목표 자세 (deg)
            sol_space: 솔루션 공간 (0~7)
        Returns:
            Future 객체 - result.success로 도달 가능 여부 확인
        """
        if not self._cli_ikin.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().warn('ikin 서비스 사용 불가')
            return None

        req = Ikin.Request()
        req.pos = [float(x), float(y), float(z), float(rx), float(ry), float(rz)]
        req.sol_space = sol_space
        req.ref = 0  # 베이스 좌표계

        future = self._cli_ikin.call_async(req)
        return future
