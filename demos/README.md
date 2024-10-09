这是一个PyTree的简易demo

纯Python（无ROS）请参考`src/python_only`，`pip install py-trees`后直接运行`main.py`就可看到行为树具体运行的过程。其中的`behavior_foo.py`是行为的一个基本模板，来自于官方文档，带有很完整的函数注释。

ROS版本位与`WithROS2`中，需先安装
```
sudo apt install \
    ros-humble-py-trees \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-py-trees-ros \
```
`colcon build`后使用`ros2 run PyTree_Test service main`运行。

`colcon build` `mock_msgs`时如果出现`cannot import name 'generate_py' from 'rosidl_generator_py' (/opt/ros/humble/local/lib/python3.10/dist-packages/rosidl_generator_py/__init__.py)`报错，请安装`empy3.3.4`（和Python3.10匹配）和`lark`