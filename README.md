# morai_project
autonomous driving using MORAI

```
catkin_ws
└─ src
   └─ morai_project
      ├─ launch #include launch files
      ├─ rviz #include rviz files
      ├─ param #include yaml files for parameter
      └─ scripts #include python files
         └─ lib #include files about mgeo

```

## 실행
```
moraisim 실행
시나리오 로드
I 버튼 클릭하여 초기 위치로 spawn 및 P 버튼으로 Parking mode 전환
$ roslaunch rosbridge_server rosbridge_websocket.launch
morai sim 네트워크 설정
$ roslaunch morai_project control.launch
레이아웃대로 화면 배치
도착 후 차량이 완전히 멈추면 마네킹과의 거리 확인
```
