# tk24_decision
中关村仿生智能大赛上层代码


## 接口
各个所需函数提供一个ROS2 Service接口，返回reponse中包含 `int32 status`和`string error_msg` 关键字，成功即为0，其它情况返回error code（请在各自的文档中说明和附加对应的error_msg！）谢谢大家。
麻烦各组统一一下message的类型，谢谢～

### 导航：
`is_at(pos)` 返回机器人是否已经在`pos`的位置

`goto(pos)` 前往`pos`位置，成功到达返回`True`，无法到达（障碍物遮挡、发生碰撞）返回`False`

`goto_facing(pos, distance)` 使机器人到达面向`pos`位置距离其`distance`（米）的位置，成功到达返回`True`，无法到达（障碍物遮挡、发生碰撞）返回`False`


### 视觉
`scan_for(categories)`使用顶置相机寻找`categories`（list[str])类的物品，根据距离远近依次返回质心相对机器人位置和具体的category

`find_obj(category)`使用臂上相机寻找该`category`的物品，返回距离最近的分割mask和整个点云图（透明物体可根据计算对点云图进行修改）


### 抓取
`move_arm_to(arm_pos)`将机械臂移动到`arm_pos`，成功返回`True`，失败（无法规划、发生碰撞）返回`False`

`move_arm_joint(joint_pos)`将机械臂的关节移动到预设的角度（一般用于恢复原样）

`get_grasp_pos(mask, point_cloud) -> arm_pos`根据点云分分割返回抓取位姿

`move_gripper(Open)` 打开/闭合夹爪

`grasped_obj(category)`确认是否成功抓取（物品在夹爪中）


### 语音
`get_grasp_target()`返回要抓取的`category`

`wait_for_start()`得到任务开始指令后返回

`annouce(message)`语音广播`message`，结束/失败后返回

