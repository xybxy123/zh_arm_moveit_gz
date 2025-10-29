# zh_arm_moveit_gz

基于 MoveIt! 和 Gazebo 的机械臂控制示例，通过 `move_group_interface` 实现关节空间与空间控制。


## 项目简介
本仓库提供一套完整的机械臂仿真控制方案，包含：
- MoveIt! 路径规划与控制接口配置
- Gazebo 物理仿真环境集成
- 基于 `move_group_interface` 的控制示例（关节状态获取、目标位姿规划等）


## 快速开始

### 1. 克隆仓库
```bash
# 创建并进入工作空间（若已有工作空间，直接进入其 src 目录）
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# 克隆代码
git clone https://github.com/xybxy123/zh_arm_moveit_gz.git
```


### 2. 编译项目
```bash
# 返回工作空间根目录
cd ~/catkin_ws

# 编译（支持 catkin_make 或 catkin build）
catkin build

# 加载环境变量
source devel/setup.bash
```


### 3. 启动仿真与控制
```bash
# 启动 Gazebo 仿真环境和 MoveIt! 控制节点
roslaunch arm_tutorials arm_control.launch
```


## 常见问题
- **编译报错“缺少依赖”**：  
  执行 `rosdep install --from-paths src --ignore-src -r -y` 自动安装缺失依赖。


