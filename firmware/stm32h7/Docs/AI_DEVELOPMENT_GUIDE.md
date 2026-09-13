---
project: DMstm32
founder: 超凡
document: AI 开发规范
---

# DMstm32 AI 开发规范

> 本文件提供给参与工创赛智能救援车 DMstm32 开发的 AI 编程助手阅读。队员先阅读[开发手册](DEVELOPMENT_GUIDE.md)，系统目标和协议以[最终骨架计划](智能救援小车最终骨架计划.md)为准。本规范保留 BSP / Driver / App 分层与协作式调度思想，但工程不再按通用模板开发。
> 项目创始人：超凡。

## 0. AI 执行协议

AI 在修改工程前必须先阅读本文件，并遵守以下原则：

1. 先检查当前代码、工程配置和硬件资源，以代码事实为准，不根据目录名猜测；
2. 用户提出功能时，先确认其属于视觉、雷达、IMU、底盘、安全或救援状态机；不得另建主循环或第二套调度器；
3. 严格保持 `App -> Drivers/Library -> BSP -> Core/HAL` 的依赖方向；
4. 任务逻辑必须实现为 `Enter / Tick / Exit`，`Tick` 每次只推进一步并立即返回；
5. 不擅自改动已经验证的官方底层时序、引脚、DMA、中断和电机方向；
6. 涉及电机、舵机或资源复用时，必须提供限幅、超时、退出停机和冲突检查；
7. 修改后检查 Keil 工程是否包含新文件，并完成与风险相称的编译或测试；
8. 只编译不能声称已经烧录，只下载不能声称已经完成实机验证；
9. 如果代码行为变化导致文档过期，应同步更新相关文档；
10. 不确定硬件方向、零点、接线或设备现象时，明确标记待验证，不能编造结论。

## 1. 开始之前

本工程的目标不只是“代码能跑”，还要求在工创赛智能救援场景中安全、可解释地运行：

- 能说明数据从哪里来、经过哪些模块、最后控制了什么；
- 一个任务的修改不能破坏 IMU、CAN、安全停机等全局功能；
- 设备驱动可以复用，不能为每个任务重新写一套；
- 中断和 DMA 不承担复杂业务逻辑；
- 所有电机动作都必须有明确的退出和故障处理；
- 不把未经硬件确认的方向、零点和控制参数当作事实。

AI 开始开发前应依次读取：

1. 本规范；
2. [PROJECT_ARCHITECTURE.md](PROJECT_ARCHITECTURE.md)；
3. [PROJECT_RULES.md](PROJECT_RULES.md)；
4. 自己准备修改的任务文件和对应驱动头文件。

## 2. 架构总览

岳麓框架把系统划分为 `bsp / module / application` 三层。DMstm32 当前目录名称略有
不同，但概念可以直接对应：

| 岳麓概念 | DMstm32 目录 | 职责 |
|---|---|---|
| BSP | `Core/` + `BSP/` | STM32 外设初始化、板级通信、DMA 和中断分发 |
| Module | `Drivers/` + `Library/` | 外部设备协议、设备能力、通用算法 |
| Application | `App/` | 上电流程、公共服务、任务和应用逻辑 |

依赖方向必须保持为：

```text
App
  ↓
Drivers / Library
  ↓
BSP
  ↓
Core / STM32 HAL
```

禁止反向依赖：

- BSP 不得包含 Task 头文件；
- Driver 不得知道当前运行的是 Task0 还是 Task2；
- Library 不得依赖 HAL、GPIO、CAN、LCD 或具体硬件；
- App 不应复制设备协议解析代码。

### 2.1 Core：CubeMX 生成区

`Core/` 保存 CubeMX/HAL 生成的启动、时钟和外设句柄，例如：

```text
Core/Src/main.c
Core/Src/fdcan.c
Core/Src/spi.c
Core/Src/adc.c
Core/Src/usart.c
Core/Src/stm32h7xx_it.c
```

规则：

- 不在 `Core/` 中编写任务、控制算法或协议解析；
- `main.c` 只负责初始化和唯一主循环；
- 修改 CubeMX 配置后，检查生成代码是否覆盖用户代码；
- 中断函数只清标志、搬数据、调用轻量分发接口或置标志。

### 2.2 BSP：板级支持层

`BSP/` 负责“STM32 的某个外设怎样在这块板上工作”，例如 FDCAN 过滤器、发送、
接收与中断分发。

BSP 关心：

- 使用哪个 `hfdcan`、`hspi`、`huart`、`htim`；
- GPIO、片选、DMA、中断和硬件通道；
- 发送与接收字节/报文；
- 收到数据后调用哪个注册回调。

BSP 不关心：

- CAN 报文是不是 QD4310 速度反馈；
- SPI 寄存器是不是 BMI088 陀螺仪；
- UART 数据是不是 LD06 点云；
- 当前运行的任务编号。

### 2.3 Drivers：设备层

`Drivers/` 负责“这是什么设备，它能提供什么能力”：

```text
Drivers/Motor/       QD4310 电机协议
Drivers/IMU/         BMI088 读取与设备封装
Drivers/Lidar/       LD06 DMA 数据与协议解析
Drivers/Input/       五向按键底层驱动
Drivers/Display/     LCD 驱动
Drivers/LED/         WS2812 驱动
Drivers/Actuator/    舵机 PWM
Drivers/Storage/     Flash 标定数据
```

驱动的公开接口应该描述设备能力，例如：

```c
QD4310_SetSpeed(...);
LD06_Process();
LD06_GetData();
ServoPwm_SetAngle(...);
```

新驱动推荐采用“配置 + 实例”的写法：

```c
DeviceConfig_t config = {
    .bus = ...,
    .device_id = ...,
};

Device_t *device = Device_Register(&config);
Device_Start(device);
```

这样同一种设备有多个实例时，不依赖散落的全局变量。

现有官方移植驱动可能仍直接依赖 HAL。未经硬件回归测试，不要为了形式整齐大规模重写
已经验证的官方底层流程；新代码则应优先通过 BSP 接口访问总线。

### 2.4 Library：纯软件算法层

`Library/` 保存与具体硬件无关的算法：

```text
Library/Algorithm/      Mahony、rate_pi 等
Library/Control/Pid/    PID
```

判断一个文件是否应放入 Library 的方法：如果在普通电脑上提供输入数据，它理论上也能
完成计算，那么它通常属于 Library。

Library 中禁止出现：

```c
#include "main.h"
HAL_GetTick();
HAL_SPI_Transmit(...);
QD4310_SetSpeed(...);
LCD_ShowString(...);
```

### 2.5 App：应用层

`App/` 分为四部分：

| 目录 | 内容 |
|---|---|
| `App/System/` | 上电流程、全局周期更新、状态灯、安全停机、启动 UI |
| `App/Services/` | 多任务共享服务，目前主要是统一输入事件 |
| `App/Comm/` | 旧 UART3 任务协议与新 UART1 视觉协议 |
| `App/Tasks/` | 菜单、诊断任务和后续救援状态机 |

## 3. 程序怎样运行

本工程当前不使用 FreeRTOS。系统采用裸机协作式调度，只有 `main.c` 拥有无限循环：

```c
while (1)
{
    uint32_t now = HAL_GetTick();

    AppRuntime_Tick(now);
    InputService_Poll();
    TaskManager_Tick(now);
}
```

每个函数完成一小步后立即返回：

```text
AppRuntime_Tick
  ├─ 更新 IMU
  ├─ 维护电机使能
  ├─ 更新系统状态灯
  └─ 检查启动就绪状态

InputService_Poll
  ├─ 获取 ADC 按键值
  ├─ 消抖
  └─ 生成方向事件

TaskManager_Tick
  ├─ 菜单状态：选择或进入任务
  └─ 运行状态：调用当前任务 Tick
```

任务必须主动、快速返回，协作式调度才能继续运行。任何任务卡住，整个主循环都会卡住。

## 4. 任务生命周期

所有任务遵守 [task_contract.h](../App/Tasks/task_contract.h) 中的接口：

```c
typedef struct
{
    void (*Enter)(uint32_t now_ms);
    void (*Tick)(uint32_t now_ms, InputEvent_t event);
    void (*Exit)(void);
} AppTask_t;
```

### 4.1 Enter：进入任务时执行一次

适合执行：

- 重置任务状态机；
- 保存任务开始时间；
- 初始化或启动本任务独占的设备；
- 设置执行机构安全初值；
- 绘制完整初始页面。

必须重置所有需要重新开始的 `static` 状态，否则第二次进入任务会继承上次运行状态。

### 4.2 Tick：每圈执行一次

适合执行：

- 处理 `InputEvent_t`；
- 读取驱动提供的最新数据；
- 推进任务状态机；
- 按周期发送控制命令；
- 按较低频率更新 LCD。

禁止：

```c
while (1) { }
HAL_Delay(1000);
for (;;) { }
```

周期逻辑使用无阻塞时间差：

```c
if ((uint32_t)(now_ms - sLastTick) >= PERIOD_MS)
{
    sLastTick = now_ms;
    /* 执行一次周期动作 */
}
```

这种写法同时能正确处理 `HAL_GetTick()` 的无符号回绕。

### 4.3 Exit：离开任务时执行一次

必须处理本任务拥有的资源：

- 轮电机速度、电流命令归零；
- 停止雷达或其他持续工作设备；
- 停止不再使用的 DMA、PWM 或通信接收；
- 解除本任务产生的临时状态；
- 将执行机构恢复到明确的安全状态。

左键退出由 `TaskManager` 统一处理：先调用任务 `Exit()`，再执行全局安全停机并返回菜单。
任务一般不会收到 `INPUT_EVENT_LEFT`。

## 5. 开发或修改一个任务

当前任务是救援车诊断与联调入口：

| 任务 | 当前用途 |
|---|---|
| Task0 | LD06 雷达点云 |
| Task1 | BMI088 姿态显示 |
| Task2 | 舵机与轮电机方向标定 |
| Task3 | 五向按键 ADC 与事件诊断 |
| Task4～Task5 | 定位、视觉或底盘联调预留 |
| Task6 | UART7 松甲四轮 MG513P30 台架测试 |

诊断功能可放入对应 Task；长期运行的视觉消费、定位、安全门控和救援状态机应接入 `App/System`/`App/Comm`/`App` 的统一调度，不要复制整个工程或另建第二套主循环。

### 5.1 修改步骤

1. 明确数据输入、输出、周期、过期条件和使用的硬件资源；
2. 选择正确层级：串口协议放 `App/Comm`，硬件收发放 BSP，设备协议放 Driver，任务决策放 App；
3. 在 `Enter()` 中重置状态、启动设备和绘制页面；
4. 在 `Tick()` 中编写非阻塞状态机；
5. 在 `Exit()` 中停止设备并恢复安全；
6. 修改 `task_manager.c` 中的菜单名称；
7. 编译，确保 `0 Error(s)`；
8. 断开负载或限制输出后完成低风险硬件测试；
9. 验证退出后能正常回菜单，再增加动作幅度或控制输出。

### 5.2 任务模板

```c
#include "task3.h"

#include "task_ui.h"

#define TASK3_UPDATE_MS 100U

static uint32_t sUpdateTick;
static uint8_t sState;

static void Task3_Enter(uint32_t now_ms)
{
    sUpdateTick = now_ms;
    sState = 0U;

    TaskUi_Show(3U, "MY TASK", "READY", "CENTER: START", CYAN);
}

static void Task3_Tick(uint32_t now_ms, InputEvent_t event)
{
    if (event == INPUT_EVENT_CENTER)
        sState = 1U;

    if ((uint32_t)(now_ms - sUpdateTick) < TASK3_UPDATE_MS)
        return;

    sUpdateTick = now_ms;

    if (sState != 0U)
    {
        /* 推进一次任务逻辑，然后立即返回。 */
    }
}

static void Task3_Exit(void)
{
    /* 停止本任务使用的执行机构或外设。 */
}

const AppTask_t Task3_Definition =
{
    Task3_Enter,
    Task3_Tick,
    Task3_Exit
};
```

## 6. 救援车通讯与按键使用规则

新增通讯实现必须遵守：UART1 仅接收泰山派固定 65B 视觉帧；UART7 仅服务松甲；UART5 的 LD06 协议不重写；UART3 旧帧完全保留。中断只搬运字节和时间戳，完整帧解析在主循环增量执行。视觉、雷达和 IMU 只能向任务状态机提供快照，禁止直接控制电机。

任务不得直接扫描底层按键。统一使用传入的 `InputEvent_t`：

```c
if (event == INPUT_EVENT_UP)     { }
if (event == INPUT_EVENT_DOWN)   { }
if (event == INPUT_EVENT_RIGHT)  { }
if (event == INPUT_EVENT_CENTER) { }
```

当前约定：

| 物理动作 | QDcan 驱动对象 | 菜单功能 | 任务内功能 |
|---|---|---|---|
| 上 | `key_sw6` | 上一个任务 | 任务自行定义 |
| 下 | `key_sw5` | 下一个任务 | 任务自行定义 |
| 左 | `key_sw3` | 无 | TaskManager 统一拦截并退出 |
| 右 | `key_sw4` | 进入任务 | 任务自行定义 |
| 中键 | `key_sw2` | 无 | 任务自行定义 |

这是 QDcan 的实机验证关系。不得根据 `adc_modlue.c` 中变量名称猜测物理方向，也不得
把某两个方向单独交换；如硬件版本发生变化，应先记录五个 ADC 实测值再整体更新本表。

DMstm32 当前实测值为：静止 `3588`、下 `736`、上 `1470`、左 `2188`、右 `2880`、
中键 `1`。旧驱动的上键和左键阈值没有覆盖实测中心值，因此由 InputService 在调用
`key_process()` 前应用带死区的校准区间；不得删除该校准后直接恢复旧阈值。

禁止在任务中调用：

```c
key_driver_init();
key_attach(...);
get_key_adc();
key_process();
```

## 7. 状态机编写规则

超过两个阶段的流程应使用枚举状态机，不使用长串阻塞延时：

```c
typedef enum
{
    TASK_IDLE = 0,
    TASK_PREPARE,
    TASK_RUNNING,
    TASK_FINISHED,
    TASK_FAULT
} TaskState_t;
```

状态跳转必须同时记录进入时间：

```c
sState = TASK_RUNNING;
sStateTick = now_ms;
```

不要在一个状态中通过 `HAL_Delay()` 等待数秒，应根据：

```c
uint32_t elapsed = (uint32_t)(now_ms - sStateTick);
```

判断何时进入下一状态。

## 8. 驱动接入规则

接入新设备前必须明确：

1. 设备型号和资料来源；
2. 使用 SPI、CAN、UART、I²C、ADC、PWM 中的哪一种；
3. 对应 MCU 外设、引脚、DMA 和中断；
4. 初始化流程来自哪个官方例程；
5. 数据接收是否可能阻塞主循环；
6. 设备的启动、周期处理、停止和故障接口；
7. 与现有设备是否存在定时器、DMA、引脚或总线冲突。

建议调用链：

```text
Task
  ↓ 调用设备能力
Device Driver
  ↓ 请求总线收发
BSP SPI/CAN/UART/PWM
  ↓
HAL
```

设备驱动不得直接调用任务 UI，也不得根据 Task 编号改变行为。

## 9. 中断、DMA 与实时性

中断中只允许：

- 清除中断标志；
- 读取必要寄存器；
- 记录 DMA 写位置；
- 复制少量固定长度数据；
- 置事件标志；
- 调用已经确认轻量、无阻塞的回调。

中断中禁止：

- LCD 绘图；
- 浮点姿态解算；
- 大量协议解析；
- `HAL_Delay()`；
- `printf` 大量输出；
- 电机控制状态机。

DMA 只负责搬运数据，不代表协议已经解析。LD06 这类高速设备应由 DMA 接收，主循环
增量解析，并限制每圈处理量。

LCD 速度远低于控制环：

- 高频采集或控制通常可能达到 500 Hz～1 kHz；
- 普通文字可在 10～20 Hz 更新；
- 大面积图形应进一步降低刷新率；
- 不因画图长期停止 IMU 和电机控制。

## 10. 输出设备与硬件安全

任何操作电机、舵机、继电器等输出设备的任务都必须考虑：

- 电机是否已经使能；
- 反馈是否在线、数据是否新鲜；
- 命令方向、单位和范围是否经过硬件确认；
- 输出是否有限幅；
- 姿态超限时是否立即归零；
- CAN 反馈超时时是否停机；
- 用户退出任务时是否停机；
- 任务异常状态是否能回到安全状态。

首次测试的顺序：

1. 断开负载、限制动力或采用安全测试工装；
2. 使用零命令确认通信；
3. 使用很小的速度、电流或角度变化确认方向；
4. 确认退出键和故障停机；
5. 再逐步增加输出。

未经确认不得假设：

- 执行机构正方向；
- 电流、速度或位置命令的正负含义；
- IMU 与目标设备之间的坐标变换；
- 应用所需的参考姿态或传感器零点；
- 执行机构真实零点；
- 最终 PID/LQR 参数。

## 11. 当前重要资源冲突

`PA0 / TIM2_CH1` 当前同时涉及：

- 舵机 1 PWM；
- LD06 PWM。

两者不能在同一时间占用。任务切换时必须停止并重新配置相关资源。若未来要求雷达与四路
舵机同时工作，必须先重新分配引脚或定时器，不能只靠软件同时初始化。

## 12. 通信开发规则

`App/Comm/` 保存应用层通信协议，例如“请求运行哪个任务”“返回当前状态”。

通信代码应拆分为：

```text
BSP UART/CAN：收发字节和报文
Driver/Module：解析某种设备协议
App/Comm：解释应用层命令
```

通信接收推荐使用：

```text
DMA/中断接收
  ↓
保存字节或完整报文
  ↓
主循环取出消息
  ↓
校验长度、帧头、校验码、序号
  ↓
交给任务管理或对应服务
```

不得在 UART/CAN 中断中直接进入任务或执行长时间动作。

## 13. 编译与烧录

在 VS Code 中按 `Ctrl+Shift+P`，选择 `Tasks: Run Task`：

| 任务 | 用途 |
|---|---|
| `Keil: Build` | 只编译 |
| `Keil: Flash Current AXF` | 烧录已有 AXF，可能不是最新代码 |
| `Keil: Build + Flash` | 编译成功后烧录，推荐 |

命令行也可以运行：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\keil.ps1 -Action BuildFlash
```

脚本只有在 `build.log` 出现 `0 Error(s)` 后才允许继续烧录；只有在 `flash.log` 出现
`Verify OK` 后才报告烧录成功。

不要把以下情况描述为“已经成功”：

- 只生成了旧 AXF；
- 编译报错但 Keil 进程退出；
- 未检测到 ST-Link；
- Program 完成但 Verify 失败；
- 烧录的是错误工程或错误 Target。

## 14. 提交代码前检查

### 通用检查

- [ ] 文件放在正确层级；
- [ ] 没有底层包含上层头文件；
- [ ] 没有新增无限循环和长阻塞延时；
- [ ] 所有周期判断使用无符号时间差；
- [ ] 中断中没有复杂逻辑；
- [ ] 编译为 `0 Error(s)`；
- [ ] 没有把临时调试代码留在正式路径。

### 任务检查

- [ ] `Enter()` 重置全部任务状态；
- [ ] `Tick()` 每次都能快速返回；
- [ ] LCD 刷新已经限频；
- [ ] `Exit()` 停止任务占用的执行机构和设备；
- [ ] 左键可以退出并返回菜单；
- [ ] 第二次进入任务仍能正常运行。

### 输出设备任务检查

- [ ] 输出方向、单位和范围经过硬件确认；
- [ ] 命令有限幅；
- [ ] 反馈超时会停机；
- [ ] 姿态异常会停机；
- [ ] 已完成架空、小输出测试；
- [ ] 未经确认的参数已明确标为待验证。

## 15. 常见错误

### 错误：在 Task 中初始化和扫描按键

结果：任务切换后回调丢失、左键不能退出、多个模块争用按键状态。

正确做法：只使用传入的 `InputEvent_t`。

### 错误：用 HAL_Delay 编写动作流程

结果：IMU、按键、安全检测和其他周期功能同时停止。

正确做法：使用状态机和 `now_ms - state_tick`。

### 错误：在 Driver 中画任务界面

结果：设备驱动无法在其他任务或其他项目复用。

正确做法：Driver 只提供设备数据，Task 决定怎样显示。

### 错误：任务退出时只 return

结果：电机、DMA、PWM 或雷达继续运行。

正确做法：先在 `Exit()` 中释放资源并恢复安全，再返回菜单。

### 错误：为了“架构漂亮”重写已验证官方驱动

结果：目录更整齐，但底层时序、寄存器或 DMA 行为被破坏。

正确做法：先建立稳定接口和回归测试，再渐进封装。

## 16. 参考资料

- [湖南大学岳麓战队 basic_framework](https://github.com/HNUYueLuRM/basic_framework)
- [basic_framework 架构介绍与开发指南](https://github.com/HNUYueLuRM/basic_framework/blob/master/.Doc/%E6%9E%B6%E6%9E%84%E4%BB%8B%E7%BB%8D%E4%B8%8E%E5%BC%80%E5%8F%91%E6%8C%87%E5%8D%97.md)
- [basic_framework BSP 目录](https://github.com/HNUYueLuRM/basic_framework/tree/master/bsp)
- [basic_framework Modules 目录](https://github.com/HNUYueLuRM/basic_framework/tree/master/modules)

本手册借鉴其分层、实例、回调和应用解耦思想；DMstm32 当前仍采用裸机协作式调度，未照搬
其 FreeRTOS 任务和完整消息中心。
