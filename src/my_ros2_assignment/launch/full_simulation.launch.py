"""
Doosan E0509 로봇 시뮬레이션 Launch 파일

실행 순서:
1. 기존 에뮬레이터 정리 및 새로 시작
2. ros2_control (하드웨어 인터페이스) 시작
3. 에뮬레이터 연결 확인 후 컨트롤러 스폰 (wait_and_spawn.sh)
4. Gazebo 시뮬레이션 시작
5. GUI 노드 시작
"""

from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch 인자
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='e0509',
        description='Robot model'
    )

    # 패키지 경로
    pkg_dir = get_package_share_directory('my_ros2_assignment')
    dsr_bringup_dir = get_package_share_directory('dsr_bringup2')

    # 스크립트 경로
    wait_and_spawn_script = os.path.join(pkg_dir, 'scripts', 'wait_and_spawn.sh')

    # =========================================
    # 1단계: 에뮬레이터 정리 및 시작
    # =========================================
    setup_emulator = ExecuteProcess(
        cmd=['bash', '-c', '''
echo "========================================"
echo " Step 1: Setting up Emulator"
echo "========================================"
docker stop dsr01_emulator 2>/dev/null || true
docker rm dsr01_emulator 2>/dev/null || true
docker run -d --name dsr01_emulator --network host doosanrobot/dsr_emulator:3.0.1
echo "Emulator container started, waiting 5 seconds for initialization..."
sleep 5
echo "Emulator should be ready now"
        '''],
        name='setup_emulator',
        output='screen',
    )

    # =========================================
    # 2단계: 두산 Gazebo Bringup
    # (에뮬레이터 시작은 실패해도 OK - 이미 실행 중)
    # =========================================
    gazebo_launch = os.path.join(
        dsr_bringup_dir, 'launch', 'dsr_bringup2_gazebo.launch.py'
    )

    gazebo_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={'model': LaunchConfiguration('model')}.items(),
    )

    # =========================================
    # 3단계: 연결 확인 후 컨트롤러 재스폰
    # =========================================
    wait_and_spawn = ExecuteProcess(
        cmd=['bash', wait_and_spawn_script],
        name='wait_and_spawn',
        output='screen',
    )

    # =========================================
    # 4단계: GUI 노드
    # =========================================
    gui_node = Node(
        package='my_ros2_assignment',
        executable='my_node',
        name='robot_gui',
        output='screen',
    )

    # =========================================
    # 실행 순서 정의
    # =========================================
    return LaunchDescription([
        model_arg,

        # 0초: 에뮬레이터 시작
        setup_emulator,

        # 8초: Gazebo bringup (에뮬레이터 준비 후)
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg='\n========================================'
                           '\n Step 2: Starting Gazebo Bringup'
                           '\n========================================\n'),
                gazebo_bringup,
            ],
        ),

        # 15초: 연결 확인 및 컨트롤러 재스폰
        TimerAction(
            period=15.0,
            actions=[
                LogInfo(msg='\n========================================'
                           '\n Step 3: Checking connection & Spawning'
                           '\n========================================\n'),
                wait_and_spawn,
            ],
        ),

        # 40초: GUI 시작 (컨트롤러 스폰 완료 예상)
        TimerAction(
            period=40.0,
            actions=[
                LogInfo(msg='\n========================================'
                           '\n Step 4: Starting GUI'
                           '\n========================================\n'),
                gui_node,
            ],
        ),
    ])
