# ExtNodeMapBuild

Isaac Sim GUI에서 현재 열린 Stage의 NodeMap 정보를 읽어 FMS로 전달하기 위한 Extension입니다.

현재 단계에서는 Extension의 기본 골격과 시작·종료 진입점만 제공합니다. 이후 다음 기능을 추가합니다.

- `/World/NodeMap/Nodes`에서 Node 추출
- `/World/NodeMap/Edges`에서 Edge 추출
- NodeMap 데이터 검증
- ROS 2를 통한 FMS 전송

