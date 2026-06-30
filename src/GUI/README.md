# SML World Cup 2026 GUI

ROS 2의 `/eai/task` (`sml_messages/msg/Task`)를 구독해 World Cup 2026
경기장 레이아웃과 주문 및 스테이션 자원을 표시한다.

## 빌드

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select sml_messages sml_worldcup_gui
source install/setup.bash
```

## 실행

```bash
ros2 run sml_worldcup_gui worldcup_gui
```

전체 Arena 대신 실제 경기 side만 확대해서 표시할 수 있다.

```bash
ros2 run sml_worldcup_gui worldcup_gui --ros-args -p side:=a
ros2 run sml_worldcup_gui worldcup_gui --ros-args -p side:=b
ros2 run sml_worldcup_gui worldcup_gui --ros-args -p side:=all
```

`a` 또는 `b`를 선택하면 해당 side의 6개 station과 shared storage만 표시한다.

기본적으로 `/home/user/ros2_ws/src/GUI/config/sml_worldcup_2026_layout.json`을 읽으며,
해당 파일이 없는 배포 환경에서는 패키지에 설치된 JSON을 사용한다.

다른 Task 토픽이나 레이아웃 파일을 사용할 수도 있다.

```bash
ros2 run sml_worldcup_gui worldcup_gui --ros-args \
  -p topic_name:=/eai/task \
  -p side:=a \
  -p layout_file:=/home/user/ros2_ws/src/GUI/config/sml_worldcup_2026_layout.json
```

GUI와 기존 `task_listener`는 같은 토픽을 동시에 구독할 수 있다.
