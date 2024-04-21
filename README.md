# 基于4轴机械臂的嵌入式开发学习

## 概述
该项目旨在围绕4轴机械臂展开，持续学习和提升硬件设计、嵌入式开发、3D建模和物联网应用等多方面综合开发能力。

## 项目组成
- **机械设计**：使用SolidWorks进行4轴机械臂的建模设计，采用无刷电机作为关节。
- **嵌入式开发**：利用STM32F4 MCU实现无刷电机的Field-Oriented Control（FOC）驱动，支持UART和CAN通信协议。
- **控制系统**：设计主控板用于通信和调试，整合STM32和ESP系列MCU实现物联网功能。
- **Web界面**：使用HTML、CSS和JavaScript创建用户友好的Web界面，实现远程控制和监测。

## 当前进展
- 完成了4轴机械臂建模以及FOC电机驱动板的设计(暂未调试)。
- 进行中：FOC驱动板代码。

## 未来计划

- 进行FOC调试，完善FOC驱动。基于DengFoc
- 完善主控板设计，实现通信和控制功能。
- 开发物联网功能，开发Web界面，优化用户体验。

## 项目状态

由于个人情况，项目进展较为缓慢但仍持续进行。

## 如何贡献

欢迎大家提供想法和意见！可以fork本仓库，进行改进并提交pull request。

## 项目参考
**本项目为学习项目，参考了许多资料与课程，以下为一些主要来源**
- **机械臂建模**：@阿奇设计分享 https://www.bilibili.com/cheese/play/ss865?bsource=link_copy
- **DengFOC**： @灯哥开源 https://www.bilibili.com/video/BV1cj411M7Xu/?share_source=copy_web&vd_source=83c3e0b7c1458f20e0fbeab3fe4c3a5a
- **CSDN** ， **知乎**
