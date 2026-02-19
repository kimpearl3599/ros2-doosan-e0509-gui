import sys
from datetime import datetime
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QListWidget,
    QRadioButton, QButtonGroup, QTextEdit, QApplication, QMessageBox,
    QSplitter, QTabWidget, QFrame, QToolButton, QToolTip
)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot, QPoint
from PyQt5.QtGui import QFont, QColor, QCursor


class MainWindow(QMainWindow):
    """두산 E0509 로봇 제어 GUI (PyQt5 코드 전용)"""

    # 외부에서 연결할 시그널
    request_move = pyqtSignal(list, float, float, bool)  # 좌표목록, 속도, 가속도, 절대좌표여부
    request_stop = pyqtSignal()
    request_validate = pyqtSignal(float, float, float, float, float, float)  # x, y, z, rx, ry, rz
    request_ready_pose = pyqtSignal()  # 작업 자세로 전환

    def __init__(self):
        super().__init__()
        self._coord_list = []  # [(x, y, z, rx, ry, rz), ...]
        self._pending_coord = None  # 검증 대기 중인 좌표
        self._is_ready_pose = False  # 작업 자세 상태 추적
        self._init_ui()

    def _init_ui(self):
        self.setWindowTitle('Doosan E0509 Robot Control')
        self.setMinimumSize(1000, 750)

        central = QWidget()
        self.setCentralWidget(central)

        splitter = QSplitter(Qt.Horizontal)
        main_layout = QHBoxLayout(central)
        main_layout.addWidget(splitter)

        # ── 왼쪽 열: 제어 패널 (입력 + 옵션 + 실행) ──
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.addWidget(self._create_help_button())
        left_layout.addWidget(self._create_preset_group())
        left_layout.addWidget(self._create_coord_input_group())
        left_layout.addWidget(self._create_coord_list_group())
        left_layout.addWidget(self._create_option_group())
        left_layout.addWidget(self._create_button_group())
        left_layout.addStretch()

        # ── 오른쪽 열: 상태 모니터 (연결/동작/관절/위치/로그) ──
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.addWidget(self._create_connection_group())
        right_layout.addWidget(self._create_status_group())
        right_layout.addWidget(self._create_joint_group())
        right_layout.addWidget(self._create_ee_group())
        right_layout.addWidget(self._create_log_group())

        splitter.addWidget(left_widget)
        splitter.addWidget(right_widget)
        splitter.setSizes([480, 470])

    # ━━━━━━━━━━━━━━ 왼쪽 패널: 제어 (입력/옵션/실행) ━━━━━━━━━━━━━━

    def _create_help_button(self) -> QWidget:
        """? 아이콘 도움말 버튼 생성 (호버 시 툴팁 표시)"""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(5, 5, 5, 5)

        # 로봇 이름 라벨
        title_label = QLabel('<b>Doosan E0509 Robot Control</b>')
        title_label.setStyleSheet('font-size: 13px;')
        layout.addWidget(title_label)

        layout.addStretch()

        # ? 도움말 버튼
        help_btn = QToolButton()
        help_btn.setText('?')
        help_btn.setStyleSheet('''
            QToolButton {
                background-color: #2196F3;
                color: white;
                border-radius: 12px;
                font-weight: bold;
                font-size: 14px;
                min-width: 24px;
                max-width: 24px;
                min-height: 24px;
                max-height: 24px;
            }
            QToolButton:hover {
                background-color: #1976D2;
            }
        ''')

        # 도움말 툴팁 내용
        help_tooltip = '''
<div style="padding: 10px; max-width: 350px;">
<h3 style="color: #1976D2; margin-bottom: 10px;">Doosan E0509 로봇</h3>
<ul style="margin-left: 15px;">
<li>6축 협동 로봇 (Cobot)</li>
<li>최대 도달 거리: 509mm</li>
<li>가반 하중: 5kg</li>
</ul>

<h3 style="color: #1976D2; margin: 10px 0;">좌표계 설명</h3>
<ul style="margin-left: 15px;">
<li><b>X</b>: 로봇 전방(+) / 후방(-)</li>
<li><b>Y</b>: 로봇 좌측(+) / 우측(-)</li>
<li><b>Z</b>: 위(+) / 아래(-), 바닥 기준</li>
</ul>

<h3 style="color: #1976D2; margin: 10px 0;">회전 각도 (End-Effector 자세)</h3>
<ul style="margin-left: 15px;">
<li><b>RX</b>: X축 회전 (Roll)</li>
<li><b>RY</b>: Y축 회전 (Pitch) → 180°=아래, 0°=위</li>
<li><b>RZ</b>: Z축 회전 (Yaw)</li>
</ul>

<h3 style="color: #1976D2; margin: 10px 0;">사용 방법</h3>
<ol style="margin-left: 15px;">
<li>로봇 연결 후 "작업 자세로 전환" 클릭</li>
<li>프리셋 또는 좌표 직접 입력</li>
<li>"좌표 추가" 클릭</li>
<li>"실행" 버튼으로 이동 시작</li>
</ol>
</div>
'''
        help_btn.setToolTip(help_tooltip)
        layout.addWidget(help_btn)

        return widget

    def _create_preset_group(self) -> QGroupBox:
        group = QGroupBox('프리셋 위치')
        layout = QGridLayout()

        # 프리셋 버튼들
        btn_home = QPushButton('홈 위치')
        btn_home.setToolTip('모든 관절 0도 (초기 자세)')
        btn_home.clicked.connect(lambda: self._apply_preset(350, 0, 500, 0, 180, 0))
        layout.addWidget(btn_home, 0, 0)

        btn_front = QPushButton('전방')
        btn_front.setToolTip('로봇 정면 위치')
        btn_front.clicked.connect(lambda: self._apply_preset(400, 0, 300, 0, 180, 0))
        layout.addWidget(btn_front, 0, 1)

        btn_left = QPushButton('좌측')
        btn_left.setToolTip('로봇 좌측 위치 (안전)')
        btn_left.clicked.connect(lambda: self._apply_preset(200, 300, 300, 0, 180, 0))
        layout.addWidget(btn_left, 1, 0)

        btn_right = QPushButton('우측')
        btn_right.setToolTip('로봇 우측 위치 (안전)')
        btn_right.clicked.connect(lambda: self._apply_preset(200, -300, 300, 0, 180, 0))
        layout.addWidget(btn_right, 1, 1)

        btn_high = QPushButton('높은 위치')
        btn_high.setToolTip('Z축 높은 위치')
        btn_high.clicked.connect(lambda: self._apply_preset(300, 0, 500, 0, 180, 0))
        layout.addWidget(btn_high, 2, 0)

        btn_low = QPushButton('낮은 위치')
        btn_low.setToolTip('Z축 낮은 위치')
        btn_low.clicked.connect(lambda: self._apply_preset(350, 0, 200, 0, 180, 0))
        layout.addWidget(btn_low, 2, 1)

        group.setLayout(layout)
        return group

    def _apply_preset(self, x, y, z, rx, ry, rz):
        """프리셋 값을 입력 필드에 적용하고 목록에 추가 (검증된 안전 좌표)"""
        # 입력 필드 업데이트
        self.input_x.setText(str(x))
        self.input_y.setText(str(y))
        self.input_z.setText(str(z))
        self.input_rx.setText(str(rx))
        self.input_ry.setText(str(ry))
        self.input_rz.setText(str(rz))

        # 프리셋은 이미 검증된 안전한 좌표이므로 직접 추가
        self.add_coord_directly(x, y, z, rx, ry, rz)
        self.append_log(f'프리셋 추가: pos({x}, {y}, {z}) rot({rx}, {ry}, {rz})')

    def _create_coord_input_group(self) -> QGroupBox:
        group = QGroupBox('목표 좌표 입력 (E0509 작업 범위: ~509mm)')
        layout = QGridLayout()

        # 현재 위치 표시
        current_pos_label = QLabel('현재 위치:')
        current_pos_label.setStyleSheet('font-weight: bold;')
        layout.addWidget(current_pos_label, 0, 0)

        self.label_current_pos = QLabel('X: ---, Y: ---, Z: ---')
        self.label_current_pos.setStyleSheet('color: blue; font-family: Courier;')
        layout.addWidget(self.label_current_pos, 0, 1)

        # 작업 범위 안내
        range_label = QLabel('권장: 현재 위치에서 ±200mm 이내 | X=0 근처 주의(싱귤러리티)')
        range_label.setStyleSheet('color: gray; font-size: 10px;')
        layout.addWidget(range_label, 1, 0, 1, 2)

        layout.addWidget(QLabel('X (mm):'), 2, 0)
        self.input_x = QLineEdit('350.0')
        self.input_x.setPlaceholderText('-500 ~ 500')
        layout.addWidget(self.input_x, 2, 1)

        layout.addWidget(QLabel('Y (mm):'), 3, 0)
        self.input_y = QLineEdit('0.0')
        self.input_y.setPlaceholderText('-500 ~ 500')
        layout.addWidget(self.input_y, 3, 1)

        layout.addWidget(QLabel('Z (mm):'), 4, 0)
        self.input_z = QLineEdit('400.0')
        self.input_z.setPlaceholderText('200 ~ 600')
        layout.addWidget(self.input_z, 4, 1)

        # 회전 각도 입력
        rotation_label = QLabel('회전 각도 (deg): RX, RY, RZ')
        rotation_label.setStyleSheet('color: gray; font-size: 10px;')
        layout.addWidget(rotation_label, 5, 0, 1, 2)

        layout.addWidget(QLabel('RX (deg):'), 6, 0)
        self.input_rx = QLineEdit('0.0')
        layout.addWidget(self.input_rx, 6, 1)

        layout.addWidget(QLabel('RY (deg):'), 7, 0)
        self.input_ry = QLineEdit('180.0')
        layout.addWidget(self.input_ry, 7, 1)

        layout.addWidget(QLabel('RZ (deg):'), 8, 0)
        self.input_rz = QLineEdit('0.0')
        layout.addWidget(self.input_rz, 8, 1)

        # 경고 라벨
        self.label_warning = QLabel('')
        self.label_warning.setStyleSheet('color: orange; font-size: 10px;')
        layout.addWidget(self.label_warning, 9, 0, 1, 2)

        self.btn_add_coord = QPushButton('좌표 추가')
        self.btn_add_coord.clicked.connect(self._on_add_coord)
        layout.addWidget(self.btn_add_coord, 10, 0, 1, 2)

        group.setLayout(layout)
        return group

    def _create_coord_list_group(self) -> QGroupBox:
        group = QGroupBox('좌표 목록')
        layout = QVBoxLayout()

        self.coord_list_widget = QListWidget()
        layout.addWidget(self.coord_list_widget)

        self.btn_remove_coord = QPushButton('선택 항목 삭제')
        self.btn_remove_coord.clicked.connect(self._on_remove_coord)
        layout.addWidget(self.btn_remove_coord)

        group.setLayout(layout)
        return group

    def _create_option_group(self) -> QGroupBox:
        group = QGroupBox('옵션 설정')
        layout = QGridLayout()

        # 절대/상대 좌표
        self.radio_absolute = QRadioButton('절대 좌표')
        self.radio_absolute.setToolTip('로봇 베이스 기준 절대 위치로 이동')
        self.radio_relative = QRadioButton('상대 좌표')
        self.radio_relative.setToolTip('현재 위치에서 상대적으로 이동')
        self.radio_absolute.setChecked(True)
        coord_group = QButtonGroup(self)
        coord_group.addButton(self.radio_absolute)
        coord_group.addButton(self.radio_relative)
        layout.addWidget(self.radio_absolute, 0, 0)
        layout.addWidget(self.radio_relative, 0, 1)

        # 좌표계 설명
        coord_desc = QLabel('절대: 베이스 기준 | 상대: 현재 위치 기준')
        coord_desc.setStyleSheet('color: gray; font-size: 10px;')
        layout.addWidget(coord_desc, 1, 0, 1, 2)

        # 속도
        layout.addWidget(QLabel('속도 (mm/s):'), 2, 0)
        self.input_velocity = QLineEdit('100.0')
        self.input_velocity.setToolTip('End-Effector 이동 속도 (권장: 50~200)')
        layout.addWidget(self.input_velocity, 2, 1)

        # 가속도
        layout.addWidget(QLabel('가속도 (mm/s²):'), 3, 0)
        self.input_accel = QLineEdit('100.0')
        self.input_accel.setToolTip('End-Effector 가속도 (권장: 50~200)')
        layout.addWidget(self.input_accel, 3, 1)

        group.setLayout(layout)
        return group

    def _create_button_group(self) -> QGroupBox:
        group = QGroupBox('실행')
        layout = QVBoxLayout()

        # 실행/정지 버튼
        btn_row = QHBoxLayout()

        self.btn_execute = QPushButton('▶ 실행')
        self.btn_execute.setStyleSheet('background-color: #4CAF50; color: white; font-weight: bold; padding: 12px; font-size: 14px;')
        self.btn_execute.setToolTip('목록의 모든 좌표로 순차 이동 시작')
        self.btn_execute.clicked.connect(self._on_execute)
        btn_row.addWidget(self.btn_execute)

        self.btn_stop = QPushButton('■ 정지')
        self.btn_stop.setStyleSheet('background-color: #F44336; color: white; font-weight: bold; padding: 12px; font-size: 14px;')
        self.btn_stop.setToolTip('이동 즉시 정지 (현재 위치에서 멈춤)')
        self.btn_stop.setEnabled(False)
        self.btn_stop.clicked.connect(self._on_stop)
        btn_row.addWidget(self.btn_stop)

        layout.addLayout(btn_row)

        # 초기화 버튼
        self.btn_clear = QPushButton('목록 초기화')
        self.btn_clear.setToolTip('좌표 목록 전체 삭제')
        self.btn_clear.clicked.connect(self._on_clear_list)
        layout.addWidget(self.btn_clear)

        group.setLayout(layout)
        return group

    # ━━━━━━━━━━━━━━ 오른쪽 패널: 상태 모니터 ━━━━━━━━━━━━━━

    def _create_connection_group(self) -> QGroupBox:
        group = QGroupBox('연결 상태 및 자세 제어')
        layout = QVBoxLayout()

        # 연결 상태
        self.label_robot_status = QLabel('로봇: ● 연결 대기')
        self.label_robot_status.setStyleSheet('color: gray; font-weight: bold;')
        layout.addWidget(self.label_robot_status)

        # 연결 끊김 시 안내 (기본 숨김)
        self.label_reconnect_info = QLabel(
            '⚠ 에뮬레이터 연결을 확인하세요:\n'
            '  docker run -d --name dsr01_emulator --network host doosanrobot/dsr_emulator:3.0.1'
        )
        self.label_reconnect_info.setStyleSheet(
            'color: #C62828; font-size: 10px; padding: 8px; '
            'background-color: #FFEBEE; border-radius: 3px; font-family: Courier;'
        )
        self.label_reconnect_info.setWordWrap(True)
        self.label_reconnect_info.hide()  # 기본 숨김
        layout.addWidget(self.label_reconnect_info)

        # 작업 자세로 전환 버튼
        self.btn_ready_pose = QPushButton('▶ 작업 자세로 전환')
        self.btn_ready_pose.setStyleSheet('background-color: #2196F3; color: white; font-weight: bold; padding: 10px;')
        self.btn_ready_pose.setToolTip('싱귤러리티를 회피하고 작업 가능한 자세로 전환')
        self.btn_ready_pose.clicked.connect(self._on_ready_pose)
        layout.addWidget(self.btn_ready_pose)

        # 자세 상태 안내
        self.label_pose_info = QLabel('💡 로봇 연결 후 "작업 자세로 전환"을 클릭하세요')
        self.label_pose_info.setStyleSheet('color: #1976D2; font-size: 11px; padding: 5px; background-color: #E3F2FD; border-radius: 3px;')
        layout.addWidget(self.label_pose_info)

        group.setLayout(layout)
        return group

    def _on_ready_pose(self):
        """작업 자세로 전환 버튼 클릭"""
        self.btn_ready_pose.setEnabled(False)
        self.label_pose_info.setText('🔄 작업 자세로 전환 중...')
        self.label_pose_info.setStyleSheet('color: #F57C00; font-size: 11px; padding: 5px; background-color: #FFF3E0; border-radius: 3px;')
        self.append_log('작업 자세로 전환 시작')
        self.request_ready_pose.emit()

    @pyqtSlot(bool, str)
    def on_pose_change_result(self, success: bool, pose_type: str):
        """자세 전환 결과 처리"""
        self.btn_ready_pose.setEnabled(True)

        if success:
            self._is_ready_pose = True
            self.label_pose_info.setText('✓ 작업 준비 완료! 좌표 이동 가능')
            self.label_pose_info.setStyleSheet('color: #2E7D32; font-size: 11px; padding: 5px; background-color: #E8F5E9; border-radius: 3px;')
            self.append_log('작업 자세 전환 완료 - 좌표 이동 가능')
            # 실행 버튼 활성화
            self.btn_execute.setEnabled(True)
        else:
            self._is_ready_pose = False
            self.label_pose_info.setText('✗ 자세 전환 실패 - 다시 시도하세요')
            self.label_pose_info.setStyleSheet('color: #C62828; font-size: 11px; padding: 5px; background-color: #FFEBEE; border-radius: 3px;')
            self.append_log('자세 전환 실패')

    def _create_status_group(self) -> QGroupBox:
        group = QGroupBox('동작 상태')
        layout = QVBoxLayout()

        self.label_motion_status = QLabel('상태: 대기')
        self.label_motion_status.setFont(QFont('', 11, QFont.Bold))
        layout.addWidget(self.label_motion_status)

        group.setLayout(layout)
        return group

    def _create_joint_group(self) -> QGroupBox:
        group = QGroupBox('현재 관절 각도 (deg)')
        layout = QGridLayout()

        self.joint_labels = []
        for i in range(6):
            row, col = divmod(i, 3)
            name_label = QLabel(f'J{i+1}:')
            value_label = QLabel('0.00')
            value_label.setFont(QFont('Courier', 10))
            layout.addWidget(name_label, row, col * 2)
            layout.addWidget(value_label, row, col * 2 + 1)
            self.joint_labels.append(value_label)

        group.setLayout(layout)
        return group

    def _create_ee_group(self) -> QGroupBox:
        group = QGroupBox('End-Effector 위치 (Base 기준 절대좌표)')
        layout = QGridLayout()

        self.ee_labels = {}
        for i, axis in enumerate(['X', 'Y', 'Z']):
            layout.addWidget(QLabel(f'{axis} (mm):'), i, 0)
            lbl = QLabel('0.000')
            lbl.setFont(QFont('Courier', 10))
            layout.addWidget(lbl, i, 1)
            self.ee_labels[axis] = lbl

        group.setLayout(layout)
        return group

    def _create_log_group(self) -> QGroupBox:
        group = QGroupBox('실시간 로그')
        layout = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setFont(QFont('Courier', 9))
        layout.addWidget(self.log_text)

        group.setLayout(layout)
        return group

    # ━━━━━━━━━━━━━━ 이벤트 핸들러 ━━━━━━━━━━━━━━

    def _on_add_coord(self):
        try:
            x = float(self.input_x.text())
            y = float(self.input_y.text())
            z = float(self.input_z.text())
            rx = float(self.input_rx.text())
            ry = float(self.input_ry.text())
            rz = float(self.input_rz.text())
        except ValueError:
            self.append_log('좌표 입력값이 유효하지 않습니다.')
            return

        # 싱귤러리티 좌표 차단 (X ≈ 0)
        if abs(x) < 50:
            self.label_warning.setText('⛔ X ≈ 0: 싱귤러리티 영역 - 추가 불가')
            self.label_warning.setStyleSheet('color: red; font-size: 11px; font-weight: bold;')
            self.append_log(f'⛔ 좌표 추가 거부: X={x:.1f}mm는 싱귤러리티 영역입니다 (|X| < 50mm)')
            return

        # 기본 경고 체크 (거리 기반)
        warnings = self._validate_coordinate(x, y, z)
        if warnings:
            self.label_warning.setText(' | '.join(warnings))
            self.label_warning.setStyleSheet('color: orange; font-size: 10px;')

        # 로봇 역기구학(Ikin) 검증 요청
        self._pending_coord = (x, y, z, rx, ry, rz)
        self.btn_add_coord.setEnabled(False)
        self.btn_add_coord.setText('검증 중...')
        self.append_log(f'좌표 검증 중: pos({x:.1f}, {y:.1f}, {z:.1f}) rot({rx:.0f}, {ry:.0f}, {rz:.0f})')
        self.request_validate.emit(x, y, z, rx, ry, rz)

    @pyqtSlot(bool, str)
    def on_validation_result(self, reachable: bool, message: str):
        """Ikin 검증 결과 처리"""
        self.btn_add_coord.setEnabled(True)
        self.btn_add_coord.setText('좌표 추가')

        if self._pending_coord is None:
            return

        x, y, z, rx, ry, rz = self._pending_coord

        if reachable:
            # 도달 가능 - 좌표 추가
            self._coord_list.append((x, y, z, rx, ry, rz))
            idx = len(self._coord_list)
            self.coord_list_widget.addItem(f'{idx}. pos({x:.1f}, {y:.1f}, {z:.1f}) rot({rx:.0f}, {ry:.0f}, {rz:.0f})')
            self.append_log(f'✓ 좌표 추가 완료: pos({x:.1f}, {y:.1f}, {z:.1f})')
            self.label_warning.setText('')
        else:
            # 도달 불가능 - 경고
            self.label_warning.setText(f'⚠ 도달 불가: {message}')
            self.label_warning.setStyleSheet('color: red; font-size: 11px; font-weight: bold;')
            self.append_log(f'✗ 도달 불가능한 좌표: {message}')

        self._pending_coord = None

    def add_coord_directly(self, x, y, z, rx, ry, rz):
        """검증 없이 직접 좌표 추가 (프리셋용)"""
        self._coord_list.append((x, y, z, rx, ry, rz))
        idx = len(self._coord_list)
        self.coord_list_widget.addItem(f'{idx}. pos({x:.1f}, {y:.1f}, {z:.1f}) rot({rx:.0f}, {ry:.0f}, {rz:.0f})')

    def _validate_coordinate(self, x, y, z):
        """좌표 검증 - 경고 메시지 리스트 반환 (싱귤러리티는 _on_add_coord에서 차단)"""
        warnings = []

        # 작업 범위 초과 경고
        reach = (x**2 + y**2 + z**2) ** 0.5
        if reach > 500:
            warnings.append(f'거리 {reach:.0f}mm: 범위 초과 가능')

        # Z가 너무 낮음
        if z < 100:
            warnings.append('Z<100: 충돌 위험')

        # Z가 너무 높음
        if z > 600:
            warnings.append('Z>600: 범위 초과 가능')

        return warnings

    def _on_remove_coord(self):
        row = self.coord_list_widget.currentRow()
        if row >= 0:
            self._coord_list.pop(row)
            self.coord_list_widget.takeItem(row)
            # 번호 재정렬
            for i in range(self.coord_list_widget.count()):
                c = self._coord_list[i]
                self.coord_list_widget.item(i).setText(f'{i+1}. pos({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}) rot({c[3]:.0f}, {c[4]:.0f}, {c[5]:.0f})')
            self.append_log('선택 좌표 삭제됨')

    def _on_execute(self):
        if not self._coord_list:
            self.append_log('좌표 목록이 비어 있습니다.')
            return

        if not self._is_ready_pose:
            self.append_log('⚠ 먼저 "작업 자세로 전환"을 실행하세요.')
            self.label_pose_info.setText('⚠ 작업 자세로 전환이 필요합니다')
            self.label_pose_info.setStyleSheet('color: #E65100; font-size: 11px; padding: 5px; background-color: #FFF3E0; border-radius: 3px;')
            return

        try:
            vel = float(self.input_velocity.text())
            acc = float(self.input_accel.text())
        except ValueError:
            self.append_log('속도/가속도 값이 유효하지 않습니다.')
            return

        is_absolute = self.radio_absolute.isChecked()
        self.request_move.emit(list(self._coord_list), vel, acc, is_absolute)

    def _on_stop(self):
        self.request_stop.emit()

    def _on_clear_list(self):
        self._coord_list.clear()
        self.coord_list_widget.clear()
        self.append_log('좌표 목록 초기화')

    # ━━━━━━━━━━━━━━ 외부에서 호출하는 업데이트 슬롯 ━━━━━━━━━━━━━━

    @pyqtSlot(str)
    def append_log(self, msg: str):
        timestamp = datetime.now().strftime('%H:%M:%S')
        self.log_text.append(f'[{timestamp}] {msg}')
        self.log_text.verticalScrollBar().setValue(
            self.log_text.verticalScrollBar().maximum()
        )

    @pyqtSlot(list)
    def update_joints(self, joints: list):
        for i, val in enumerate(joints[:6]):
            self.joint_labels[i].setText(f'{val:.2f}')

    @pyqtSlot(float, float, float)
    def update_ee_position(self, x: float, y: float, z: float):
        self.ee_labels['X'].setText(f'{x:.3f}')
        self.ee_labels['Y'].setText(f'{y:.3f}')
        self.ee_labels['Z'].setText(f'{z:.3f}')
        # 입력 필드 근처의 현재 위치도 업데이트
        self.label_current_pos.setText(f'X: {x:.1f}, Y: {y:.1f}, Z: {z:.1f}')

    @pyqtSlot(bool)
    def update_connection_status(self, connected: bool):
        # 이전 상태와 비교해서 변경 시에만 로그 출력
        was_connected = getattr(self, '_was_connected', None)

        if connected:
            self.label_robot_status.setText('로봇: ● 연결됨')
            self.label_robot_status.setStyleSheet('color: green; font-weight: bold;')
            self.label_reconnect_info.hide()
            self.btn_ready_pose.setEnabled(True)
            if was_connected == False:
                self.append_log('로봇 연결 복구됨')
        else:
            self.label_robot_status.setText('로봇: ● 연결 끊김')
            self.label_robot_status.setStyleSheet('color: red; font-weight: bold;')
            self.label_reconnect_info.show()
            self.btn_ready_pose.setEnabled(False)
            self.btn_execute.setEnabled(False)
            self._is_ready_pose = False
            if was_connected == True:
                self.append_log('⚠ 로봇 연결 끊김! 에뮬레이터를 확인하세요.')
                self.label_pose_info.setText('⚠ 연결 끊김 - 에뮬레이터 확인 필요')
                self.label_pose_info.setStyleSheet('color: #C62828; font-size: 11px; padding: 5px; background-color: #FFEBEE; border-radius: 3px;')

        self._was_connected = connected

    @pyqtSlot(str)
    def update_motion_status(self, status: str):
        self.label_motion_status.setText(f'상태: {status}')
        color_map = {
            '대기': 'black',
            '이동 중': 'blue',
            '완료': 'green',
            '정지': 'orange',
            '오류': 'red',
        }
        color = color_map.get(status, 'black')
        self.label_motion_status.setStyleSheet(f'color: {color};')

    def set_moving_state(self, moving: bool):
        """이동 중 UI 상태 전환"""
        self.btn_execute.setEnabled(not moving)
        self.btn_stop.setEnabled(moving)
        self.btn_add_coord.setEnabled(not moving)
        self.btn_clear.setEnabled(not moving)


# GUI 단독 테스트용
if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.append_log('GUI 단독 테스트 모드')
    win.update_connection_status(False)
    win.show()
    sys.exit(app.exec_())
