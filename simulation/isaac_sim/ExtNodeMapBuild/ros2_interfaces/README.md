# ExtNodeMapBuild ROS 2 Interfaces

Isaac Sim Extension이 FMS 및 AMR과 통신할 때 필요한 ROS 2 interface package다.
이 폴더만 Isaac Sim PC로 복사해 독립적으로 빌드할 수 있다.

## 포함된 인터페이스

- `interfaces/msg/NodeMapChanged.msg`
- `interfaces/srv/GetNodeMap.srv`
- `interfaces/action/VisualizeRoute.action`

패키지명은 FMS 워크스페이스와 동일한 `interfaces`다. 각 인터페이스 정의도
`ros2_ws/src/interfaces`의 같은 파일과 항상 동일하게 유지해야 한다.

## 빌드

```bash
cd ros2_interfaces
source /opt/ros/jazzy/setup.bash
colcon build --packages-select interfaces
source install/setup.bash
```

## Isaac Sim 실행

같은 터미널에서 ROS 2 환경과 이 패키지를 source한 후 Isaac Sim을 실행한다.

```bash
source /opt/ros/jazzy/setup.bash
source ros2_interfaces/install/setup.bash
export ROS_DOMAIN_ID=135

# 프로젝트에서 사용하는 Isaac Sim 실행 명령
```

Extension의 Python 코드는 빌드 후 생성된 다음 모듈을 사용한다.

```python
from interfaces.action import VisualizeRoute
from interfaces.msg import NodeMapChanged
from interfaces.srv import GetNodeMap
```

원본 `.msg`, `.srv`, `.action` 파일은 Python 모듈이 아니므로 빌드 없이 직접
import할 수 없다.
