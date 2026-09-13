---
project: DMstm32
founder: 超凡
document: 架构说明
---

# 工创赛智能救援车工程架构

本文档位于 `Docs/`。使用者先阅读简洁的 [DEVELOPMENT_GUIDE.md](DEVELOPMENT_GUIDE.md)；AI 编程助手、任务模板、
实时性和安全检查详见 [AI_DEVELOPMENT_GUIDE.md](AI_DEVELOPMENT_GUIDE.md)。

项目创始人：超凡。

工程采用“CubeMX 基础区 + BSP + Drivers + Library + App”的依赖方向。H7 是唯一的任务决策和底盘控制中心；泰山派只提供视觉结果，LD06 与 BMI088 只提供感知数据，松甲板只执行 UART7 下发的运动命令。

```text
Core/                 CubeMX/HAL 生成的启动与外设初始化代码
BSP/CAN/              FDCAN 初始化、过滤器、收发与中断分发
BSP/UART/             UART1 视觉接收、UART7 松甲收发与旧 UART3 通信的板级支持
Drivers/Motor/        松甲四路驱动板协议与编码器反馈
Drivers/IMU/          BMI088 传感器驱动（含陀螺仪零偏校准）
Drivers/Input/        ADC 五向按键底层扫描、消抖与回调驱动
Drivers/Display/      LCD 官方驱动
Drivers/Lidar/        LD06 DMA 接收与协议解析驱动
Drivers/Actuator/     TIM1/TIM2 四路舵机 PWM（1=PA0，2=PA2，3=PE9，4=PE13）
Drivers/LED/          WS2812 状态灯驱动
Drivers/Storage/      Flash 零位读写（magic + 校验）
Library/Algorithm/    通用算法（Mahony、rate_pi）
Library/Control/Pid/  通用 PID
App/System/           救援车运行时：周期调度、系统状态和最终安全输出
App/Services/         输入事件等多任务共享服务
App/Tasks/            任务管理、菜单和具体业务任务；全部实现 Enter / Tick / Exit
App/Comm/             旧 UART3 协议与 UART1 视觉协议
MDK-ARM/              Keil 工程与构建产物
```

## 分层规则

- `App/System` 是唯一的应用运行时入口：负责启动流程、公共周期功能、系统状态和安全输出。
- `Core/Src/main.c` 只拥有唯一的 `while (1)`：每圈依次运行 Runtime、Input 和
  TaskManager。任务仅推进一步后返回，不得拥有自己的无限循环。
- `App/Services` 是应用层公共服务。按键扫描只可在 `InputService_Poll()` 中运行；
  任务通过 `InputEvent_t` 接收事件，不得重绑底层按键回调。
- `App/Tasks` 只实现应用状态机；进入、周期推进、退出分别由 `Enter / Tick / Exit`
  完成。当前菜单示例由左键退出，并调用任务 Exit 与全局安全停机。
- `BSP` 只处理板级外设，不包含控制策略。
- `Drivers` / `Library` 是可复用设备与算法，不包含具体任务逻辑。
- `Core` 只保留 CubeMX 生成代码和最小调度入口；不得堆放控制算法。

## 救援车数据流与启动流程

1. Core 完成时钟、GPIO 和 MCU 外设初始化；
2. BSP 与 Driver 初始化当前配置启用的总线和设备；
3. App/System 启动公共服务，并完成需要在应用开始前执行的设备准备；
4. 系统就绪后，TaskManager 显示菜单或启动指定应用任务；
5. 主循环持续运行 Runtime、InputService 和 TaskManager；
6. 任务退出或设备异常时，由任务 Exit 和全局安全接口恢复安全输出。

运行时按最新有效快照调度：IMU 100 Hz、底盘安全 50 Hz、雷达 10-20 Hz、视觉与任务状态机 20 Hz、LCD 4-10 Hz。中断/DMA 只搬运数据和记录时间戳；任务状态机与安全层在主循环中运行。数据源过期时不得继续复用旧控制指令。

## 当前示例

- Task0 为 LD06 点云诊断，Task1 为 IMU 姿态诊断，Task2 为执行机构与方向标定，Task3 为五向按键诊断，Task4～Task5 保留给定位/视觉/底盘联调，Task6 为 UART7 松甲四轮 MG513P30 台架测试。

救援功能最终遵循 `SEARCH -> APPROACH -> PUSH -> VERIFY`。完整视觉 65B 固定帧和实施顺序以 [智能救援小车最终骨架计划](智能救援小车最终骨架计划.md) 为准；该设计尚未全部实现，不得写成已验证功能。

## 板载硬件调用规则（硬约束）

- **任何板载硬件的初始化与调用流程一律照抄官方例程**，不在底层流程上自创：
  - BMI088（SPI）：`external_refs/gitee/dm-mc02/例程/CtrBoard-H7_IMU`
  - TIM/PWM（M996 舵机）：`例程/CtrBoard-H7_TIM`
  - FDCAN/CAN：`例程/CtrBoard-H7_FDCAN`、`例程/CtrBoard-H7_CAN`
  - UART/RS485：`例程/CtrBoard-H7_UART`、`例程/CtrBoard-H7_RS485`
  - WS2812：`例程/CtrBoard-H7_WS2812`
  - ADC：`例程/CtrBoard-H7_ADC`
- 核心驱动文件保持与官方例程一致（如 BMI088driver.c 逐字节照抄）；业务层只做薄封装，
  封装层必须调用官方驱动的公开接口（如 `BMI088_init()` / `BMI088_read()`）。
- 新外设接入时，先说明"照抄哪个例程、改动点在哪"，再动手。
