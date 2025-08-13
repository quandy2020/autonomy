# 03 如何运行

## 1 启动gazebo

* 设置环境变量

```bash
### ROS2 ###
export GLOG_logtostderr=1
export GLOG_alsologtostderr=0
export GLOG_colorlogtostderr=1
export GLOG_minloglevel=0
```

*   启动autonomy

```bash
ros2 launch autonomy_ros autonomy.launch.py
```
