# CPSL Bridge Receiver

A ROS 2 package for receiving a selected list of ROS2 topics messages over TCP/UDP between a center machine and multiple robots.

```text
cpsl_bridge_receiver/
├── config/
│   ├── center_receiver.yaml
│   ├── robot_receiver.yaml
├── launch/
│   ├── center_receiver.launch
│   ├── robot_receiver.launch
├── src/
│   ├── map_receiver.py
│   ├── tf_receiver.py
│   ├── waypoint_receiver.py
│   ├── exec_summary_receiver.py
│   ├── scan_receiver.py
├── package.xml
└── README.md
```

## Configurations
Please make sure the IP addresses, robot_ids and etc. are matched in the yaml files.

## Usage
Launch for the Center Computer
```bash
ros2 ros2_receiver_pkg launch center_receiver.launch
````
Make sure you have the receiver package running on every robot you want to connect before running the center sender.

Launch for the Individual Robot
```bash
ros2 ros2_receiver_pkg launch robot_receiver.launch
````
Make sure you have the receiver package running on the center computer you want to connect before running the robot sender.
