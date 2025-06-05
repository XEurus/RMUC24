# RMUC24 FivePines 哨兵机器人 导航和决策程序

## 介绍
- 2023赛季 2024赛季 FivePines战队 哨兵机器人 导航和决策程序
- 基于T265和2D激光雷达，使用T265作为视觉里程计。采用激光雷达配合视觉里程计、imu的方案，使用Cartographer算法建图，Amcl定位，Teb_local_palnner算法导航。几乎每份配置文件都有很详细的注释，快速上手。运动控制等都是基于很基础的move_base框架，使用ROS1和Ubuntu20.04。

## 软件架构
### 仿真模块 `RMUC24_simulation`
- 仿真中没有使用双目摄像头作为里程计。仿真只用于测试参数可行性、运行流畅程度。
- **mowen**** 文件夹是其他比赛官方程序的简易小车，没自己做车模型了。
- **fp_sim** 主要的程序文件。
  - **launch** 主要启动文件。
  - **config** 配置参数。
  - **script** 未完成的全向感知方案以及ipc重定位程序。原本的方案是做一个使用二级控制、多线程、低帧率的感知配合高帧率自瞄，可以在不改动自瞄和下位机的情况下做全向感知，非常适合没人配合但是又想做全向感知的时候。使用ipc重定位程序时注意pcl库版本问题。
  - **world** 可用的是 `20245.world` 文件，需要修改文件内STL路径。

### 现实中使用模块 `RMUC24_real`
- **sensor_interface** 传感器相关驱动，包括雷达和其他传感器程序。
- **nav**
  - **script** 键盘控制程序。
  - **config** 包含各种配置文件。
  - **launch** 请使用 `f3_move_base.launch` 进行导航。
  - **teb** 导航参数配置文件。
  - **msg** 自瞄通信格式文件。

### 决策与串口通信模块 `decision_script`
- 决策比较简单，状态机，看到对面车辆就打，可以按时间点去地图上某个点。
- ~~5.py版本中导航点发布存在问题，请改成使用多进程，将串口数据使用队列发给导航点决策进程。当前版本导航使用了阻塞，无法放到主函数内执行。或者将主函数while内导航点发布函数去除~~multi_decision.py中按照上述思路修正了该问题。
- 由于和下位机约定的是我发送后才会回我裁判系统消息，导致有些混乱。最好的方法是上位机直接读裁判系统之后使用ros发布裁判系统消息。


## 安装教程
```shell
mkdir -p work_space_name/src
cd work_space_name/src
git clone https://gitee.com/E_urus/RMUC24.git
cd ..
catkin build
source devel/setup.bash
```

## 使用说明
启动仿真 目前使用省赛地图 里面也有区域赛地图
```shell

roslaunch fp_sim 2024runc_sim.launch

```

启动仿真导航
```shell

roslaunch fp_sim move2.launch

```

实体哨兵导航程序 请更改对应的激光雷达启动程序
```shell

roslaunch nav f3move_base.launch

```
