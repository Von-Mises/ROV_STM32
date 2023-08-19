# 基于STM32的玄武号 ROV 下位机程序

---

---

## 写在前面

本代码是玄武号下位机控制程序，主控芯片为STM32H743IITx，具有最高480MHz主频。ROV整体布局采用6个推进器，底盘装有2个履带电机。操控采用网络通信发送手柄数据。（2023.8.19更新）

## 系统框图

![system](https://github.com/Von-Mises/ROV_STM32/blob/main/Doc/system.png)

## 结构布局

![structure](https://github.com/Von-Mises/ROV_STM32/blob/main/Doc/structure.png)

## 供电系统框图

![power_suply](https://github.com/Von-Mises/ROV_STM32/blob/main/Doc/power_suply.png)

## 通信系统框图

![communication](https://github.com/Von-Mises/ROV_STM32/blob/main/Doc/communication.png)

## 软件环境

| Toolchain/IDE       | MDK-ARM V5             |
| ------------------- | ---------------------- |
| STM32H7xx_DFP Packs | 2.8.0                  |
| STM32CubeMx         | 5.3.0                  |
| package version     | STM32Cube FW_H7 V1.5.0 |
| FreeRTOS version    | 10.0.1                 |
| CMSIS-RTOS version  | 1.02                   |

## 编程规范

- 变量和函数命名方式遵循 Unix/Linux 风格
- 不需要精确计时的任务，采用自行实现的软件定时器实现，定时精度受任务调度影响
