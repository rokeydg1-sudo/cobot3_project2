# 개발 환경

## Control / FMS 환경

- OS: Ubuntu 24.04
- Python: 3.12
- ROS 2: Jazzy
- cuOpt: 26.8.0 (CUDA 13)
- RMW: Fast DDS (`rmw_fastrtps_cpp`)
- ROS_DOMAIN_ID 허용 범위: 129~135
- 기본 ROS_DOMAIN_ID: 129

### 최초 환경 구축

저장소 루트에서 실행한다.

```bash
cd "$(git rev-parse --show-toplevel)"

python3 -m venv .venv
source .venv/bin/activate

python -m pip install --upgrade pip wheel
python -m pip install -r requirements/control.txt
