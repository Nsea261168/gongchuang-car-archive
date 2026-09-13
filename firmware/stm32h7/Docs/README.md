---
project: DMstm32
founder: 超凡
document: 文档入口
---

# DMstm32 - 工创赛智能救援车控制工程

> 面向 2027 中国大学生工程实践与创新能力大赛智能救援赛道的达妙 STM32H723 主控工程。

## 项目信息

| 字段 | 内容 |
|---|---|
| 项目名称 | DMstm32 |
| 项目类型 | 工创赛智能救援车主控固件 |
| 目标平台 | 达妙 STM32H723 控制板 |
| 创始人（Founder） | 超凡 |
| 调度方式 | 裸机协作式调度 |
| 开发工具 | STM32CubeMX / Keil MDK / VS Code |
| 开源许可证 | 尚未指定 |

“创始人”用于记录项目发起者；贡献者名单由 [AUTHORS.md](AUTHORS.md) 维护。开源许可证
决定代码可以怎样被使用、修改和分发，是另一项法律信息，在创始人明确选择前不擅自添加。

## 文档导航

- [最终骨架计划](智能救援小车最终骨架计划.md)：视觉协议、调度、状态机、实施顺序与验收条件；
- [任务调度与多总线实时性规则](任务调度与多总线实时性规则.md)：LCD、UART1/5/7、SPI 的频率、优先级、超时和安全边界；
- [使用手册](DEVELOPMENT_GUIDE.md)：给智能救援车队员安排功能开发与实机验证；
- [AI 开发规范](AI_DEVELOPMENT_GUIDE.md)：实现、验证和安全规则；
- [架构说明](PROJECT_ARCHITECTURE.md)：本车的数据流、目录与硬件资源边界；
- [项目规则](PROJECT_RULES.md)：本项目长期有效的工程硬规则；
- [松甲 / MG513P30 测试](SONGJIA_MG512_TEST.md)：UART7、Task6、四轮台架与安全验证；
- [创始人与贡献者](AUTHORS.md)：项目署名记录。

## 标准署名方式

文档使用 YAML Front Matter 保存机器可读元数据：

```yaml
---
project: DMstm32
founder: 超凡
document: 文档名称
---
```

新增正式文档时应沿用该格式。源码文件若以后需要版权头，应在许可证和版权归属确定后统一
添加，不能将“Founder”自动等同于“Copyright Holder”。
