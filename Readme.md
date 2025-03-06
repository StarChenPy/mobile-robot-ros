# 第48届世界技能大赛移动机器人项目

本项目为移动机器人的上位机项目

使用 ros2 框架开发

## 先决条件
- ros2 环境
- python3

## 如何部署

1. 首先需要 `Ubuntu 22` 系统环境, 也可以是虚拟机或 `wsl` 或 `docker` .

2. 在 `Ubuntu` 中安装 `ros2 humble` .

3. 使用 `pip install opencv-python ros-humble-cv-bridge onnxruntime numpy==1.26.0` 安装所需环境

4. 使用 `git clone https://github.com/StarChenPy/mobile-robot-ros.git` 获取项目

5. 进入项目目录, 运行 `colcon build` 编译项目, 并在终端中 `source install/setup.bash`.

## 如何运行

在编译完项目后, 运行 `ros2 run mobile_robot {模块名}` 即可

目前有着 `b_moudle` 和 `c_moudle`, 分别代表济南市比赛的 B 模块与 C 模块

## 注意事项

使用wsl时, 注意使用桥接模式网络, 否则无法与机器人通信

如果出现未找到命令, 请将项目的环境变量添加到终端
