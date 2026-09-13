# 工创小车差速方案（项目归档）

这是工创赛智能救援小车项目停止开发后的公开归档。仓库保留目前可用的 STM32 主控固件和整车 Fusion 360 装配体，希望为后续参赛者、嵌入式开发者和机器人爱好者提供参考。

> 项目状态：已停止维护。现有设计和代码可能尚未完成全部实车验证，使用前请自行检查电气连接、执行机构限位和失控保护。

## 仓库内容

| 目录 | 内容 |
| --- | --- |
| `firmware/stm32h7/` | 基于 STM32H723 的智能救援车主控固件、Keil 工程和开发文档 |
| `mechanical/工创小车差速方案.f3z` | Autodesk Fusion 360 整车装配体归档 |

## 固件快速入口

- Keil 工程：`firmware/stm32h7/MDK-ARM/CtrBoard-H7_FDCAN.uvprojx`
- CubeMX 配置：`firmware/stm32h7/CtrBoard-H7_FDCAN.ioc`
- 开发说明：`firmware/stm32h7/Docs/DEVELOPMENT_GUIDE.md`
- 项目架构：`firmware/stm32h7/Docs/PROJECT_ARCHITECTURE.md`
- 项目规则：`firmware/stm32h7/Docs/PROJECT_RULES.md`

固件面向达妙 STM32H723 控制板，采用裸机协作式调度。项目包含 CAN/FDCAN、UART、IMU、激光雷达、底盘电机、舵机、LCD、按键和任务状态机等模块。具体完成度、硬件约束和安全要求请以 `firmware/stm32h7/Docs/` 中的文档为准。

## 使用整车装配体

`.f3z` 是 Fusion 360 归档格式。可在 Autodesk Fusion 中通过“文件 → 打开/上传”导入。模型约 50 MB，本仓库使用 Git LFS 管理该文件；通过 Git 克隆仓库时请确保已安装 Git LFS。

```bash
git lfs install
git clone https://github.com/Nsea261168/gongchuang-car-archive.git
```

## 重要说明

- 本项目属于未完成工程归档，不保证可以直接参赛、量产或安全运行。
- 上电及实车测试前，应先架空车轮，确认急停、通信超时、方向和速度限制有效。
- 仓库公开不自动改变各文件原有的许可状态。固件目录保留了原仓库中的 `LICENSE` 以及第三方组件许可证；历史文档对许可证的描述存在不一致，实际复用前请自行核对权利归属。
- 整车装配体目前没有单独声明许可；未经权利人另行授权，不应默认获得商业使用权。

## 致后来者

这个项目没有走到最后，但留下的每一行代码、每一次结构调整和每一个没有验证完的方案，都可能成为后来者少走的一段弯路。欢迎阅读、研究并在明确许可范围内继续完善它。
