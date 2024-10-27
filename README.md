# tk24_decision
中关村仿生智能大赛上层代码！


## 接口
请见飞书文档（https://gqz316bkyzz.feishu.cn/wiki/LFZTwm88sikbVfkR8RScQovkn7c?from=from_copylink）

## 环境
PyTrees安装：
```
pip install py-trees
```

Pytree ROS安装：
```
$ sudo apt install \
    ros-humble-py-trees \
    ros-humble-py-trees-ros-interfaces \
    ros-humble-py-trees-ros
```

## build
需先build `tinker_vision_msgs`再build此目录。

## 运行
使用`ros2 run mock_services mock_services`运行模拟的service（可通过更改`mock_service_node.py`中的变量设置希望模拟哪些service，且可以通过更改每个callback的具体返回值为行为树输入指定的变量值）

之后个，可使用`ros2 run behavior_tree ptrash`运行现有的捡垃圾任务行为树（可通过更改`Constants.py`中的变量改变连接的service名称），或`ros2 run behavior_tree draw_ptrash`绘制行为树的可视化结构。

另可通过更改`main.py`中`tree.tick_tock(period_ms=500.0,post_tick_handler=print_tree)`中的`period_ms`改变行为树每个tick间隔的时间。