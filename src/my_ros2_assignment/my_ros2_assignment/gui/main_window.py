import sys
import json
from datetime import datetime
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QHBoxLayout, QVBoxLayout, QGridLayout,
    QGroupBox, QLabel, QLineEdit, QPushButton, QListWidget,
    QRadioButton, QButtonGroup, QTextEdit, QApplication, QMessageBox,
    QSplitter, QFrame, QToolButton, QFileDialog, QGraphicsDropShadowEffect,
    QSizePolicy
)
from PyQt5.QtCore import Qt, pyqtSignal, pyqtSlot
from PyQt5.QtGui import QFont, QColor


class MainWindow(QMainWindow):
    """두산 E0509 로봇 제어 GUI (PyQt5 코드 전용 - 라이트 테마)"""

    # 외부에서 연결할 시그널
    request_move = pyqtSignal(list, float, float, bool)
    request_stop = pyqtSignal()
    request_validate = pyqtSignal(float, float, float, float, float, float)
    request_ready_pose = pyqtSignal()
    request_return_previous = pyqtSignal()  # 이전 위치 복귀
    request_home_return = pyqtSignal()  # 홈 위치 복귀

    # 색상 상수 (라이트 테마)
    COLORS = {
        'bg_main': '#f5f7fa',
        'bg_card': '#ffffff',
        'bg_status_bar': '#2c3e50',
        'border': '#e0e6ed',
        'text_primary': '#2c3e50',
        'text_secondary': '#7f8c8d',
        'success': '#27ae60',
        'warning': '#f39c12',
        'danger': '#e74c3c',
        'info': '#3498db',
        'disabled': '#bdc3c7',
        'log_bg': '#2c3e50',
        'log_text': '#2ecc71',
    }

    def __init__(self):
        super().__init__()
        self._coord_list = []
        self._pending_coord = None
        self._is_ready_pose = False
        self._position_history = []  # 이전 위치 히스토리
        self._init_ui()
        self._apply_styles()

    def _init_ui(self):
        self.setWindowTitle('Doosan E0509 Robot Control')
        self.setMinimumSize(1100, 800)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setSpacing(12)
        main_layout.setContentsMargins(16, 16, 16, 16)

        # ── 상단: 상태 바 ──
        main_layout.addWidget(self._create_status_bar())

        # ── 하단: 좌우 분할 ──
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(self._create_left_panel())
        splitter.addWidget(self._create_right_panel())
        splitter.setSizes([480, 520])
        main_layout.addWidget(splitter, 1)

    def _apply_styles(self):
        """전역 스타일 적용"""
        self.setStyleSheet(f'''
            QMainWindow {{
                background-color: {self.COLORS['bg_main']};
            }}
            QGroupBox {{
                background-color: {self.COLORS['bg_card']};
                border: 1px solid {self.COLORS['border']};
                border-radius: 12px;
                margin-top: 14px;
                padding: 16px;
                font-weight: bold;
                font-size: 12px;
                color: {self.COLORS['text_primary']};
            }}
            QGroupBox::title {{
                subcontrol-origin: margin;
                left: 16px;
                padding: 0 8px;
                color: {self.COLORS['info']};
            }}
            QLabel {{
                color: {self.COLORS['text_primary']};
            }}
            QLineEdit {{
                background-color: {self.COLORS['bg_card']};
                border: 2px solid {self.COLORS['border']};
                border-radius: 6px;
                padding: 8px 10px;
                font-size: 13px;
                color: {self.COLORS['text_primary']};
            }}
            QLineEdit:focus {{
                border: 2px solid {self.COLORS['info']};
            }}
            QRadioButton {{
                color: {self.COLORS['text_primary']};
                spacing: 8px;
                font-size: 13px;
            }}
            QRadioButton::indicator {{
                width: 18px;
                height: 18px;
                border-radius: 9px;
                border: 2px solid {self.COLORS['border']};
                background-color: white;
            }}
            QRadioButton::indicator:checked {{
                background-color: {self.COLORS['info']};
                border-color: {self.COLORS['info']};
            }}
            QListWidget {{
                background-color: {self.COLORS['bg_card']};
                border: 2px solid {self.COLORS['border']};
                border-radius: 8px;
                padding: 6px;
                font-size: 12px;
                color: {self.COLORS['text_primary']};
            }}
            QListWidget::item {{
                padding: 10px;
                border-radius: 6px;
                margin: 2px 0;
            }}
            QListWidget::item:selected {{
                background-color: {self.COLORS['info']};
                color: white;
            }}
            QListWidget::item:hover {{
                background-color: #ebf5fb;
            }}
        ''')

    def _add_shadow(self, widget):
        """위젯에 그림자 효과 추가"""
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(20)
        shadow.setXOffset(0)
        shadow.setYOffset(4)
        shadow.setColor(QColor(0, 0, 0, 40))
        widget.setGraphicsEffect(shadow)

    # ━━━━━━━━━━━━━━ 상태 바 (상단 고정) ━━━━━━━━━━━━━━

    def _create_status_bar(self) -> QFrame:
        """상단 상태 바 생성"""
        frame = QFrame()
        frame.setObjectName('status_bar')
        frame.setFixedHeight(56)
        frame.setStyleSheet(f'''
            QFrame#status_bar {{
                background-color: {self.COLORS['bg_status_bar']};
                border-radius: 10px;
                padding: 8px 16px;
            }}
            QLabel {{
                color: white;
                font-size: 13px;
            }}
        ''')

        layout = QHBoxLayout(frame)
        layout.setContentsMargins(20, 0, 20, 0)
        layout.setSpacing(24)

        # 연결 상태
        self.status_connection = QLabel('● 연결 대기')
        self.status_connection.setStyleSheet('color: #95a5a6; font-weight: bold;')
        layout.addWidget(self.status_connection)

        # 구분선
        layout.addWidget(self._create_separator())

        # 동작 상태
        self.status_motion = QLabel('상태: 대기')
        self.status_motion.setStyleSheet('font-weight: bold;')
        layout.addWidget(self.status_motion)

        # 구분선
        layout.addWidget(self._create_separator())

        # EE 위치
        self.status_ee = QLabel('EE: ---, ---, ---')
        self.status_ee.setFont(QFont('Consolas', 11))
        layout.addWidget(self.status_ee)

        # 구분선
        layout.addWidget(self._create_separator())

        # 속도
        self.status_velocity = QLabel('v: 100')
        layout.addWidget(self.status_velocity)

        layout.addStretch()

        # 도움말 버튼
        help_btn = QToolButton()
        help_btn.setText('?')
        help_btn.setStyleSheet(f'''
            QToolButton {{
                background-color: {self.COLORS['info']};
                color: white;
                border-radius: 14px;
                font-weight: bold;
                font-size: 14px;
                min-width: 28px;
                max-width: 28px;
                min-height: 28px;
                max-height: 28px;
            }}
            QToolButton:hover {{
                background-color: #5dade2;
            }}
        ''')
        help_btn.setToolTip(self._get_help_tooltip())
        layout.addWidget(help_btn)

        self._add_shadow(frame)
        return frame

    def _create_separator(self) -> QFrame:
        """세로 구분선"""
        sep = QFrame()
        sep.setFixedWidth(1)
        sep.setFixedHeight(24)
        sep.setStyleSheet('background-color: #7f8c8d;')
        return sep

    def _get_help_tooltip(self) -> str:
        return '''
<div style="padding: 10px; max-width: 350px;">
<h3 style="color: #3498db;">Doosan E0509 로봇</h3>
<ul>
<li>6축 협동 로봇 (Cobot)</li>
<li>최대 도달 거리: 509mm</li>
<li>가반 하중: 5kg</li>
</ul>
<h3 style="color: #3498db;">좌표계</h3>
<ul>
<li><b>X</b>: 전방(+) / 후방(-)</li>
<li><b>Y</b>: 좌측(+) / 우측(-)</li>
<li><b>Z</b>: 위(+) / 아래(-)</li>
</ul>
<h3 style="color: #3498db;">사용 방법</h3>
<ol>
<li>로봇 연결 확인</li>
<li>"작업 자세로 전환" 클릭</li>
<li>좌표 입력 또는 프리셋 선택</li>
<li>"실행" 버튼 클릭</li>
</ol>
</div>
'''

    # ━━━━━━━━━━━━━━ 왼쪽 패널 (제어) ━━━━━━━━━━━━━━

    def _create_left_panel(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)
        layout.setContentsMargins(0, 0, 8, 0)

        layout.addWidget(self._create_preset_group())
        layout.addWidget(self._create_coord_input_group())
        layout.addWidget(self._create_coord_list_group())
        layout.addWidget(self._create_option_group())
        layout.addWidget(self._create_button_group())
        layout.addStretch()

        return widget

    def _create_preset_group(self) -> QGroupBox:
        group = QGroupBox('프리셋 위치')
        self._add_shadow(group)
        layout = QGridLayout()
        layout.setSpacing(10)

        presets = [
            ('홈', 350, 0, 500, 0, 180, 0),
            ('전방', 400, 0, 300, 0, 180, 0),
            ('좌측', 200, 300, 300, 0, 180, 0),
            ('우측', 200, -300, 300, 0, 180, 0),
            ('높음', 300, 0, 500, 0, 180, 0),
            ('낮음', 350, 0, 200, 0, 180, 0),
        ]

        for i, (name, x, y, z, rx, ry, rz) in enumerate(presets):
            btn = QPushButton(name)
            btn.setMinimumHeight(44)
            btn.setStyleSheet(f'''
                QPushButton {{
                    background-color: white;
                    border: 2px solid {self.COLORS['info']};
                    border-radius: 8px;
                    color: {self.COLORS['info']};
                    font-weight: bold;
                    font-size: 13px;
                }}
                QPushButton:hover {{
                    background-color: {self.COLORS['info']};
                    color: white;
                }}
                QPushButton:pressed {{
                    background-color: #2980b9;
                }}
            ''')
            btn.clicked.connect(lambda _, x=x, y=y, z=z, rx=rx, ry=ry, rz=rz:
                                self._apply_preset(x, y, z, rx, ry, rz))
            layout.addWidget(btn, i // 3, i % 3)

        group.setLayout(layout)
        return group

    def _apply_preset(self, x, y, z, rx, ry, rz):
        self.input_x.setText(str(x))
        self.input_y.setText(str(y))
        self.input_z.setText(str(z))
        self.input_rx.setText(str(rx))
        self.input_ry.setText(str(ry))
        self.input_rz.setText(str(rz))
        self.add_coord_directly(x, y, z, rx, ry, rz)
        self.append_log(f'프리셋 추가: ({x}, {y}, {z})')

    def _create_coord_input_group(self) -> QGroupBox:
        group = QGroupBox('좌표 입력')
        self._add_shadow(group)
        layout = QGridLayout()
        layout.setSpacing(8)

        # 위치 입력 (한 줄)
        layout.addWidget(QLabel('위치(mm)'), 0, 0)

        self.input_x = QLineEdit('350.0')
        self.input_x.setFixedWidth(70)
        self.input_y = QLineEdit('0.0')
        self.input_y.setFixedWidth(70)
        self.input_z = QLineEdit('400.0')
        self.input_z.setFixedWidth(70)

        pos_layout = QHBoxLayout()
        pos_layout.addWidget(QLabel('X:'))
        pos_layout.addWidget(self.input_x)
        pos_layout.addWidget(QLabel('Y:'))
        pos_layout.addWidget(self.input_y)
        pos_layout.addWidget(QLabel('Z:'))
        pos_layout.addWidget(self.input_z)
        pos_layout.addStretch()

        pos_widget = QWidget()
        pos_widget.setLayout(pos_layout)
        layout.addWidget(pos_widget, 0, 1)

        # 회전 입력 (한 줄)
        layout.addWidget(QLabel('회전(°)'), 1, 0)

        self.input_rx = QLineEdit('0.0')
        self.input_rx.setFixedWidth(60)
        self.input_ry = QLineEdit('180.0')
        self.input_ry.setFixedWidth(60)
        self.input_rz = QLineEdit('0.0')
        self.input_rz.setFixedWidth(60)

        rot_layout = QHBoxLayout()
        rot_layout.addWidget(QLabel('RX:'))
        rot_layout.addWidget(self.input_rx)
        rot_layout.addWidget(QLabel('RY:'))
        rot_layout.addWidget(self.input_ry)
        rot_layout.addWidget(QLabel('RZ:'))
        rot_layout.addWidget(self.input_rz)
        rot_layout.addStretch()

        rot_widget = QWidget()
        rot_widget.setLayout(rot_layout)
        layout.addWidget(rot_widget, 1, 1)

        # 경고 라벨
        self.label_warning = QLabel('')
        self.label_warning.setStyleSheet(f'color: {self.COLORS["warning"]}; font-size: 11px;')
        layout.addWidget(self.label_warning, 2, 0, 1, 2)

        # 좌표 추가 버튼
        self.btn_add_coord = QPushButton('+ 좌표 추가')
        self.btn_add_coord.setMinimumHeight(40)
        self.btn_add_coord.setStyleSheet(f'''
            QPushButton {{
                background-color: {self.COLORS['info']};
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 13px;
            }}
            QPushButton:hover {{
                background-color: #5dade2;
            }}
            QPushButton:disabled {{
                background-color: {self.COLORS['disabled']};
            }}
        ''')
        self.btn_add_coord.clicked.connect(self._on_add_coord)
        layout.addWidget(self.btn_add_coord, 3, 0, 1, 2)

        group.setLayout(layout)
        return group

    def _create_coord_list_group(self) -> QGroupBox:
        group = QGroupBox('좌표 목록')
        self._add_shadow(group)
        layout = QVBoxLayout()
        layout.setSpacing(8)

        self.coord_list_widget = QListWidget()
        self.coord_list_widget.setMinimumHeight(120)
        layout.addWidget(self.coord_list_widget)

        # 버튼 행 1: 삭제
        btn_row1 = QHBoxLayout()
        self.btn_remove_coord = self._create_secondary_button('삭제')
        self.btn_remove_coord.clicked.connect(self._on_remove_coord)
        btn_row1.addWidget(self.btn_remove_coord)
        btn_row1.addStretch()
        layout.addLayout(btn_row1)

        # 버튼 행 2: 저장/불러오기/초기화
        btn_row2 = QHBoxLayout()
        self.btn_save = self._create_secondary_button('저장')
        self.btn_save.clicked.connect(self._on_save_coords)
        btn_row2.addWidget(self.btn_save)

        self.btn_load = self._create_secondary_button('불러오기')
        self.btn_load.clicked.connect(self._on_load_coords)
        btn_row2.addWidget(self.btn_load)

        self.btn_clear = self._create_secondary_button('초기화')
        self.btn_clear.clicked.connect(self._on_clear_list)
        btn_row2.addWidget(self.btn_clear)

        layout.addLayout(btn_row2)

        group.setLayout(layout)
        return group

    def _create_secondary_button(self, text: str) -> QPushButton:
        btn = QPushButton(text)
        btn.setMinimumHeight(36)
        btn.setStyleSheet(f'''
            QPushButton {{
                background-color: #ecf0f1;
                border: none;
                border-radius: 6px;
                color: {self.COLORS['text_primary']};
                padding: 8px 16px;
                font-weight: bold;
                font-size: 12px;
            }}
            QPushButton:hover {{
                background-color: #d5dbdb;
            }}
            QPushButton:pressed {{
                background-color: #bdc3c7;
            }}
        ''')
        return btn

    def _create_option_group(self) -> QGroupBox:
        group = QGroupBox('옵션 설정')
        self._add_shadow(group)
        layout = QVBoxLayout()
        layout.setSpacing(12)

        # 절대/상대 좌표
        coord_layout = QHBoxLayout()
        self.radio_absolute = QRadioButton('절대좌표')
        self.radio_relative = QRadioButton('상대좌표')
        self.radio_absolute.setChecked(True)
        coord_group = QButtonGroup(self)
        coord_group.addButton(self.radio_absolute)
        coord_group.addButton(self.radio_relative)
        coord_layout.addWidget(self.radio_absolute)
        coord_layout.addWidget(self.radio_relative)
        coord_layout.addStretch()
        layout.addLayout(coord_layout)

        # 속도/가속도
        speed_layout = QHBoxLayout()
        speed_layout.addWidget(QLabel('속도(mm/s):'))
        self.input_velocity = QLineEdit('100.0')
        self.input_velocity.setFixedWidth(80)
        self.input_velocity.textChanged.connect(self._update_velocity_display)
        speed_layout.addWidget(self.input_velocity)
        speed_layout.addSpacing(16)
        speed_layout.addWidget(QLabel('가속도(mm/s²):'))
        self.input_accel = QLineEdit('100.0')
        self.input_accel.setFixedWidth(80)
        speed_layout.addWidget(self.input_accel)
        speed_layout.addStretch()
        layout.addLayout(speed_layout)

        group.setLayout(layout)
        return group

    def _update_velocity_display(self):
        try:
            vel = self.input_velocity.text()
            self.status_velocity.setText(f'v: {vel}')
        except:
            pass

    def _create_button_group(self) -> QGroupBox:
        group = QGroupBox('실행')
        self._add_shadow(group)
        layout = QVBoxLayout()
        layout.setSpacing(10)

        # 작업 자세 전환
        self.btn_ready_pose = QPushButton('▶ 작업 자세로 전환')
        self.btn_ready_pose.setMinimumHeight(44)
        self.btn_ready_pose.setStyleSheet(f'''
            QPushButton {{
                background-color: {self.COLORS['info']};
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 13px;
            }}
            QPushButton:hover {{
                background-color: #5dade2;
            }}
            QPushButton:disabled {{
                background-color: {self.COLORS['disabled']};
            }}
        ''')
        self.btn_ready_pose.clicked.connect(self._on_ready_pose)
        layout.addWidget(self.btn_ready_pose)

        # 자세 상태 라벨
        self.label_pose_info = QLabel('로봇 연결 후 "작업 자세로 전환"을 클릭하세요')
        self.label_pose_info.setStyleSheet(f'''
            color: {self.COLORS['info']};
            font-size: 11px;
            padding: 8px;
            background-color: #ebf5fb;
            border-radius: 6px;
        ''')
        self.label_pose_info.setWordWrap(True)
        layout.addWidget(self.label_pose_info)

        # 실행/정지 버튼
        exec_layout = QHBoxLayout()

        self.btn_execute = QPushButton('▶ 실행')
        self.btn_execute.setMinimumHeight(54)
        self.btn_execute.setStyleSheet(f'''
            QPushButton {{
                background-color: {self.COLORS['success']};
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 16px;
            }}
            QPushButton:hover {{
                background-color: #2ecc71;
            }}
            QPushButton:pressed {{
                background-color: #1e8449;
            }}
            QPushButton:disabled {{
                background-color: {self.COLORS['disabled']};
            }}
        ''')
        self.btn_execute.clicked.connect(self._on_execute)
        exec_layout.addWidget(self.btn_execute)

        self.btn_stop = QPushButton('■ 정지')
        self.btn_stop.setMinimumHeight(54)
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(f'''
            QPushButton {{
                background-color: {self.COLORS['danger']};
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 16px;
            }}
            QPushButton:hover {{
                background-color: #ec7063;
            }}
            QPushButton:pressed {{
                background-color: #c0392b;
            }}
            QPushButton:disabled {{
                background-color: {self.COLORS['disabled']};
            }}
        ''')
        self.btn_stop.clicked.connect(self._on_stop)
        exec_layout.addWidget(self.btn_stop)

        layout.addLayout(exec_layout)

        # 복귀/홈 버튼
        return_layout = QHBoxLayout()

        self.btn_return_prev = QPushButton('⟲ 이전 위치')
        self.btn_return_prev.setMinimumHeight(46)
        self.btn_return_prev.setEnabled(False)
        self.btn_return_prev.setStyleSheet(f'''
            QPushButton {{
                background-color: {self.COLORS['warning']};
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
            }}
            QPushButton:hover {{
                background-color: #f5b041;
            }}
            QPushButton:disabled {{
                background-color: {self.COLORS['disabled']};
            }}
        ''')
        self.btn_return_prev.clicked.connect(self._on_return_previous)
        return_layout.addWidget(self.btn_return_prev)

        self.btn_home = QPushButton('⌂ 홈 복귀')
        self.btn_home.setMinimumHeight(46)
        self.btn_home.setStyleSheet(f'''
            QPushButton {{
                background-color: {self.COLORS['info']};
                color: white;
                border: none;
                border-radius: 8px;
                font-weight: bold;
                font-size: 14px;
            }}
            QPushButton:hover {{
                background-color: #5dade2;
            }}
            QPushButton:disabled {{
                background-color: {self.COLORS['disabled']};
            }}
        ''')
        self.btn_home.clicked.connect(self._on_home_return)
        return_layout.addWidget(self.btn_home)

        layout.addLayout(return_layout)

        group.setLayout(layout)
        return group

    # ━━━━━━━━━━━━━━ 오른쪽 패널 (상태 모니터) ━━━━━━━━━━━━━━

    def _create_right_panel(self) -> QWidget:
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.setSpacing(12)
        layout.setContentsMargins(8, 0, 0, 0)

        layout.addWidget(self._create_joint_group())
        layout.addWidget(self._create_ee_group())
        layout.addWidget(self._create_log_group(), 1)

        return widget

    def _create_joint_group(self) -> QGroupBox:
        group = QGroupBox('관절 각도 (deg)')
        self._add_shadow(group)
        layout = QGridLayout()
        layout.setSpacing(8)

        self.joint_labels = []
        for i in range(6):
            row, col = divmod(i, 3)
            name_label = QLabel(f'J{i+1}:')
            name_label.setStyleSheet('font-weight: bold;')
            value_label = QLabel('0.00')
            value_label.setFont(QFont('Consolas', 12))
            value_label.setStyleSheet(f'color: {self.COLORS["info"]};')
            layout.addWidget(name_label, row, col * 2)
            layout.addWidget(value_label, row, col * 2 + 1)
            self.joint_labels.append(value_label)

        group.setLayout(layout)
        return group

    def _create_ee_group(self) -> QGroupBox:
        group = QGroupBox('End-Effector 위치 (mm, Base 기준)')
        self._add_shadow(group)
        layout = QHBoxLayout()
        layout.setSpacing(20)

        self.ee_labels = {}
        for axis in ['X', 'Y', 'Z']:
            axis_layout = QHBoxLayout()
            name_label = QLabel(f'{axis}:')
            name_label.setStyleSheet('font-weight: bold;')
            value_label = QLabel('0.000')
            value_label.setFont(QFont('Consolas', 13))
            value_label.setStyleSheet(f'color: {self.COLORS["info"]};')
            axis_layout.addWidget(name_label)
            axis_layout.addWidget(value_label)
            layout.addLayout(axis_layout)
            self.ee_labels[axis] = value_label

        layout.addStretch()
        group.setLayout(layout)
        return group

    def _create_log_group(self) -> QGroupBox:
        group = QGroupBox('실시간 로그')
        self._add_shadow(group)
        layout = QVBoxLayout()

        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet(f'''
            QTextEdit {{
                background-color: {self.COLORS['log_bg']};
                border: none;
                border-radius: 8px;
                color: {self.COLORS['log_text']};
                font-family: 'Consolas', 'D2Coding', monospace;
                font-size: 11px;
                padding: 12px;
            }}
        ''')
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

        if abs(x) < 50:
            self.label_warning.setText('X ≈ 0: 싱귤러리티 영역 - 추가 불가')
            self.label_warning.setStyleSheet(f'color: {self.COLORS["danger"]}; font-weight: bold;')
            self.append_log(f'좌표 추가 거부: X={x:.1f}mm (싱귤러리티)')
            return

        warnings = self._validate_coordinate(x, y, z)
        if warnings:
            self.label_warning.setText(' | '.join(warnings))

        self._pending_coord = (x, y, z, rx, ry, rz)
        self.btn_add_coord.setEnabled(False)
        self.btn_add_coord.setText('검증 중...')
        self.append_log(f'좌표 검증 중: ({x:.1f}, {y:.1f}, {z:.1f})')
        self.request_validate.emit(x, y, z, rx, ry, rz)

    @pyqtSlot(bool, str)
    def on_validation_result(self, reachable: bool, message: str):
        self.btn_add_coord.setEnabled(True)
        self.btn_add_coord.setText('+ 좌표 추가')

        if self._pending_coord is None:
            return

        x, y, z, rx, ry, rz = self._pending_coord

        if reachable:
            self._coord_list.append((x, y, z, rx, ry, rz))
            idx = len(self._coord_list)
            self.coord_list_widget.addItem(f'{idx}. ({x:.1f}, {y:.1f}, {z:.1f}) rot({rx:.0f}, {ry:.0f}, {rz:.0f})')
            self.append_log(f'좌표 추가 완료: ({x:.1f}, {y:.1f}, {z:.1f})')
            self.label_warning.setText('')
        else:
            self.label_warning.setText(f'도달 불가: {message}')
            self.label_warning.setStyleSheet(f'color: {self.COLORS["danger"]}; font-weight: bold;')
            self.append_log(f'도달 불가능: {message}')

        self._pending_coord = None

    def add_coord_directly(self, x, y, z, rx, ry, rz):
        self._coord_list.append((x, y, z, rx, ry, rz))
        idx = len(self._coord_list)
        self.coord_list_widget.addItem(f'{idx}. ({x:.1f}, {y:.1f}, {z:.1f}) rot({rx:.0f}, {ry:.0f}, {rz:.0f})')

    def _validate_coordinate(self, x, y, z):
        warnings = []
        reach = (x**2 + y**2 + z**2) ** 0.5
        if reach > 500:
            warnings.append(f'거리 {reach:.0f}mm: 범위 초과 가능')
        if z < 100:
            warnings.append('Z<100: 충돌 위험')
        if z > 600:
            warnings.append('Z>600: 범위 초과')
        return warnings

    def _on_remove_coord(self):
        row = self.coord_list_widget.currentRow()
        if row >= 0:
            self._coord_list.pop(row)
            self.coord_list_widget.takeItem(row)
            for i in range(self.coord_list_widget.count()):
                c = self._coord_list[i]
                self.coord_list_widget.item(i).setText(
                    f'{i+1}. ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}) rot({c[3]:.0f}, {c[4]:.0f}, {c[5]:.0f})')
            self.append_log('좌표 삭제됨')

    def _on_clear_list(self):
        self._coord_list.clear()
        self.coord_list_widget.clear()
        self.append_log('좌표 목록 초기화')

    def _on_save_coords(self):
        if not self._coord_list:
            self.append_log('저장할 좌표가 없습니다.')
            return

        file_path, _ = QFileDialog.getSaveFileName(
            self, '좌표 저장', '', 'JSON Files (*.json)')

        if file_path:
            data = {
                'version': '1.0',
                'settings': {
                    'velocity': float(self.input_velocity.text() or 100),
                    'acceleration': float(self.input_accel.text() or 100),
                    'is_absolute': self.radio_absolute.isChecked()
                },
                'coordinates': [
                    {'x': c[0], 'y': c[1], 'z': c[2], 'rx': c[3], 'ry': c[4], 'rz': c[5]}
                    for c in self._coord_list
                ]
            }
            with open(file_path, 'w', encoding='utf-8') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self.append_log(f'좌표 저장: {file_path}')

    def _on_load_coords(self):
        file_path, _ = QFileDialog.getOpenFileName(
            self, '좌표 불러오기', '', 'JSON Files (*.json)')

        if file_path:
            try:
                with open(file_path, 'r', encoding='utf-8') as f:
                    data = json.load(f)

                # 설정 복원
                settings = data.get('settings', {})
                self.input_velocity.setText(str(settings.get('velocity', 100)))
                self.input_accel.setText(str(settings.get('acceleration', 100)))
                if settings.get('is_absolute', True):
                    self.radio_absolute.setChecked(True)
                else:
                    self.radio_relative.setChecked(True)

                # 좌표 복원
                self._coord_list.clear()
                self.coord_list_widget.clear()
                for c in data.get('coordinates', []):
                    self.add_coord_directly(c['x'], c['y'], c['z'], c['rx'], c['ry'], c['rz'])

                self.append_log(f'{len(self._coord_list)}개 좌표 불러옴: {file_path}')
            except Exception as e:
                self.append_log(f'불러오기 실패: {str(e)}')

    def _on_execute(self):
        if not self._coord_list:
            self.append_log('좌표 목록이 비어 있습니다.')
            return

        if not self._is_ready_pose:
            self.append_log('먼저 "작업 자세로 전환"을 실행하세요.')
            self.label_pose_info.setText('작업 자세로 전환이 필요합니다')
            self.label_pose_info.setStyleSheet(f'''
                color: {self.COLORS['warning']};
                font-size: 11px;
                padding: 8px;
                background-color: #fef9e7;
                border-radius: 6px;
            ''')
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

    def _on_ready_pose(self):
        self.btn_ready_pose.setEnabled(False)
        self.label_pose_info.setText('작업 자세로 전환 중...')
        self.label_pose_info.setStyleSheet(f'''
            color: {self.COLORS['warning']};
            font-size: 11px;
            padding: 8px;
            background-color: #fef9e7;
            border-radius: 6px;
        ''')
        self.append_log('작업 자세로 전환 시작')
        self.request_ready_pose.emit()

    def _on_return_previous(self):
        if self._position_history:
            self.append_log('이전 위치로 복귀 요청')
            self.request_return_previous.emit()

    def _on_home_return(self):
        self.append_log('홈 위치로 복귀 요청')
        self.request_home_return.emit()

    @pyqtSlot(bool, str)
    def on_pose_change_result(self, success: bool, pose_type: str):
        self.btn_ready_pose.setEnabled(True)

        if success:
            self._is_ready_pose = True
            self.label_pose_info.setText('작업 준비 완료! 좌표 이동 가능')
            self.label_pose_info.setStyleSheet(f'''
                color: {self.COLORS['success']};
                font-size: 11px;
                padding: 8px;
                background-color: #e8f8f5;
                border-radius: 6px;
            ''')
            self.append_log('작업 자세 전환 완료')
            self.btn_execute.setEnabled(True)
        else:
            self._is_ready_pose = False
            self.label_pose_info.setText('자세 전환 실패 - 다시 시도하세요')
            self.label_pose_info.setStyleSheet(f'''
                color: {self.COLORS['danger']};
                font-size: 11px;
                padding: 8px;
                background-color: #fdedec;
                border-radius: 6px;
            ''')
            self.append_log('자세 전환 실패')

    # ━━━━━━━━━━━━━━ 상태 업데이트 슬롯 ━━━━━━━━━━━━━━

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
        self.status_ee.setText(f'EE: {x:.1f}, {y:.1f}, {z:.1f}')

    @pyqtSlot(bool)
    def update_connection_status(self, connected: bool):
        was_connected = getattr(self, '_was_connected', None)

        if connected:
            self.status_connection.setText('● 연결됨')
            self.status_connection.setStyleSheet(f'color: {self.COLORS["success"]}; font-weight: bold;')
            self.btn_ready_pose.setEnabled(True)
            self.btn_home.setEnabled(True)
            if was_connected == False:
                self.append_log('로봇 연결 복구됨')
        else:
            self.status_connection.setText('● 연결 끊김')
            self.status_connection.setStyleSheet(f'color: {self.COLORS["danger"]}; font-weight: bold;')
            self.btn_ready_pose.setEnabled(False)
            self.btn_execute.setEnabled(False)
            self.btn_home.setEnabled(False)
            self._is_ready_pose = False
            if was_connected == True:
                self.append_log('로봇 연결 끊김!')
                self.label_pose_info.setText('연결 끊김 - 에뮬레이터 확인 필요')
                self.label_pose_info.setStyleSheet(f'''
                    color: {self.COLORS['danger']};
                    font-size: 11px;
                    padding: 8px;
                    background-color: #fdedec;
                    border-radius: 6px;
                ''')

        self._was_connected = connected

    @pyqtSlot(str)
    def update_motion_status(self, status: str):
        self.status_motion.setText(f'상태: {status}')
        color_map = {
            '대기': 'white',
            '이동 중': '#3498db',
            '완료': self.COLORS['success'],
            '정지': self.COLORS['warning'],
            '오류': self.COLORS['danger'],
        }
        color = color_map.get(status, 'white')
        self.status_motion.setStyleSheet(f'color: {color}; font-weight: bold;')

    def set_moving_state(self, moving: bool):
        self.btn_execute.setEnabled(not moving)
        self.btn_stop.setEnabled(moving)
        self.btn_add_coord.setEnabled(not moving)
        self.btn_clear.setEnabled(not moving)
        self.btn_return_prev.setEnabled(not moving and len(self._position_history) > 0)

    def add_position_to_history(self, x: float, y: float, z: float, rx: float, ry: float, rz: float):
        """이동 성공 시 위치 히스토리에 추가"""
        self._position_history.append((x, y, z, rx, ry, rz))
        self.btn_return_prev.setEnabled(True)

    def get_previous_position(self):
        """이전 위치 반환"""
        if self._position_history:
            return self._position_history[-1]
        return None


# GUI 단독 테스트용
if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.append_log('GUI 단독 테스트 모드')
    win.update_connection_status(False)
    win.show()
    sys.exit(app.exec_())
