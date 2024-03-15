# morai_project
autonomous driving using MORAI

```
catkin_ws
└─ src
   └─ morai_project
      ├─ launch #include launch files
      ├─ rviz #include rviz files
      └─ scripts #include python files

```

## 실행
```
moraisim 실행
시나리오 로드
$ roslaunch rosbridge_server rosbridge_websocket.launch
morai sim 네트워크 설정
`$ rosrun morai_project global_path_publisher.py`
`$ rosrun morai_project local_path_pub.py`
`$ rosrun morai_project EgoVehicleStatus2odom.py`
`$ rosrun morai_project pure_pursuit_skel.py`
```