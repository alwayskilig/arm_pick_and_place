# 快速开始指南

## 一键测试（3个终端）

### 终端1：启动仿真
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch wpr_simulation2 wpb_mani.launch.py
```

### 终端2：启动节点
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch arm_pick_and_place full_system.launch.py
```

### 终端3：发送指令
```bash
cd ~/ros2_ws && source install/setup.bash

# 抓取
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 1}"

# 递送
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 2}"

# 复位
ros2 topic pub --once /task/command arm_pick_and_place/msg/TaskCommand "{command_type: 3}"
```

## 常用调试命令

```bash
# 查看话题
ros2 topic list

# 监控状态
ros2 topic echo /task/status

# 查看节点
ros2 node list

# 查看关节
ros2 topic echo /joint_states
```

## 问题？查看完整文档

详细测试流程请查看 `TESTING_GUIDE.md`