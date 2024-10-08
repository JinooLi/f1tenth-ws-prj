#!bin/bash

# ROS2 setup
. /opt/ros/foxy/setup.bash

if [ -f '/workspaces/test_ws/install/setup.bash' ]; then
    echo "ROS 2 workspace found."
    . /workspaces/test_ws/install/setup.bash

    # ROS2 package commands
    alias run_imu="ros2 launch stella_ahrs stella_ahrs_launch.py"
    alias run_lidar=" ros2 launch urg_node2 urg_node2.launch.py"
    alias run_vesc_driver="ros2 launch vesc_driver vesc_driver_node.launch.py"
    alias run_vesc_ackermann="ros2 launch vesc_ackermann ackermann_to_vesc_node.launch.xml"
    alias run_joy="ros2 run joy joy_node"
    alias run_car_control="ros2 run car_control car_control"
    alias run_simulator="ros2 launch f1tenth_gym_ros gym_bridge_launch.py"

    command_list=(
        'run_imu'
        'run_lidar'
        'run_vesc_driver'
        'run_vesc_ackermann'
        'run_joy'
        )

    execute_commands() {
        for one_command in "${command_list[@]}"; do
            ${one_command} > /dev/null 2>&1 # background로 실행 후 출력을 무시
            sleep .5 # 각 명령어 실행 후 0.5초 대기
        done
    }

    save_commands_job_id() { # job id를 저장하기 위해 job_id_list에 저장
        job_id_list=($(jobs -l | grep one_command | awk '{print $1}'))
    }

    kill_commands() {
        for job_id in "${job_id_list[@]}"; do
            kill %${job_id:1:-2} # job id만을 얻기 위해 job_id의 앞 뒤 두 글자를 제외함
        done
        jobs -l | grep one_command
    }

    alias run_all_devices='execute_commands && save_commands_job_id'

    alias stop_all_devices='kill_commands'
fi