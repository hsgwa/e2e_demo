# e2e_demo

## rqt_graph
Here is a result of 'rqt_graph'

![graph](rosgraph.png)
## Setup

### Install LTTng

See [micro-ROS tutorial](https://micro-ros.github.io/docs/tutorials/advanced/tracing/).

### build forked-foxy

```
mkdir -p ~/ros2_foxy_fork/src
cd ~/ros2_foxy_fork

wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos

cd src/ros-tracing/ros2_tracing
git remote add forked https://gitlab.com/HasegawaAtsushi/ros2_tracing
git pull forked
git checkout -b devel_e2e_measurement forked/devel_e2e_measurement

cd ~/ros2_foxy_fork/src/ros2/rclcpp
git remote add forked https://github.com/hsgwa/rclcpp.git
git pull forked
git checkout -b devel_e2e_measurement forked/devel_e2e_measurement

cd ~/ros2_foxy_fork/
colcon build --symlink-install
```


### build demo program and tracetools_analysis

```
mkdir -p ~/ros2_trace_test/src

cd ~/ros2_trace_test/src
git clone https://gitlab.com/HasegawaAtsushi/tracetools_analysis.git -b devel_e2e_measurement

cd ~/ros2_trace_test/src
git clone https://github.com/hsgwa/e2e_demo.git

cd ~/ros2_trace_test
source ~/ros2_foxy_fork/install/setup.bash
colcon build --symlink-install
```

## Usage

### run demo program
```
source ~/ros2_foxy_fork/install/setup.bash
source ~/ros2_trace_test/install/local_setup.bash

ros2 launch e2e_demo demo.launch.py
# send Ctrl+C after few seconds later
# trace data is recorded at ~/.ros/tracing/e2e_demo
```

### visualize with jupyter
```
cd src/e2e_demo/analysis/
jupyter-lab
```

see these samples below.

- comm_latency.ipynb
- create_architecture_template.ipynb
- e2e_latency.ipynb
- flame_graph.ipynb
- node_latency.ipynb
