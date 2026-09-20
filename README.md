# GridTalk

无主机多对多半双工串行总线协议库。适用于 STM32F411 双板系统（控制器 / 遥控器）
共用的 485 总线，也支持任意数量的节点接入同一共享总线。

2026-09-20 从双板固件仓库（20260323 第三版程序）抽出单独立项，自此与固件仓库
**互不依赖**：本项目是协议库的唯一源头，固件仓库里的副本不再与本项目同步。

## 目录结构

```
src/    GridTalk.c / GridTalk.h    协议库本体（唯一副本，平台无关）
sim/                                PC 总线仿真器（编译真实 src/GridTalk.c）
docs/    GridTalk协议设计.md       分层设计说明（L1物理/L2链路/L3应用）
docs/history/                      历史缺陷调查记录（环形级联随机冻结 = DWT 回绕缺陷）
```

## 接入方法

库全部状态封装在 `gridTalk_t` 结构体内，多实例、无全局单例。平台通过钩子接入：

```c
gridTalk_t gt;
gridTalk_init(&gt, USART2, myID);      /* uart=NULL 可选: 仅当不直写寄存器时 */
gt.nowUs   = dwt_us;                   /* µs时钟(如DWT自由计数器); NULL则用usTick累加 */
gt.txByte  = my_send_byte;             /* 发送输出;  NULL则直接写 uart->DR(中断驱动) */
gt.readData    = on_frame;             /* 收到完整帧(CRC已过) */
gt.sendResult  = on_send_result;       /* needAck 发送的最终结果(OK/放弃) */
gridTalk_seed(&gt, entropy);           /* 每次上电注入不同熵, 重定序号基值(必做) */

/* 运行期三个入口, 任一事件到达即驱动状态机: */
gridTalk_rxByte(&gt, byte);            /* UART RX中断, 逐字节喂入 */
gridTalk_txIrq(&gt);                   /* UART TXE/TC中断入口(寄存器直写模式) */
gridTalk_usTick(&gt, us);              /* 无DWT时周期µs心跳 */

/* 发送: */
gridTalk_send(&gt, target, cmd, data, len);      /* 需应答: 超时重发直到OK/放弃 */
gridTalk_sendNoAck(&gt, target, cmd, data, len); /* 无应答 */
gridTalk_sendReply / sendReplyNoAck;             /* 应答走reply通道 */
```

帧格式与参数（定界 `FF 00/FF FE`、目标/源 ID、2B 递增序号、Fletcher-16、
`FF FC`+目标ID 应答、载荷上限 502B）见 `docs/GridTalk协议设计.md`。

## 仿真器

```powershell
cd sim
.\build_run.ps1                       # 默认: 120s, 每字节误码0.4%, 6台设备, 9600bps
.\build_run.ps1 -Duration 60 -Loss 0.4 -Devices 10 -Debug
```

需要 Windows + VS Build Tools（MSVC）。仿真模型（字节级碰撞、误码、打包突发、
接入抖动）说明见 `sim/README.md`。`sim/` 下的测试入口：

| 文件 | 用途 |
| --- | --- |
| `sim.c` | 主仿真器：轮询/中继/回声三类流量 + 统计报告 |
| `ring_sim.c` | 环形级联专项（21 台成环，圈耗时/单跳延迟） |
| `test_dualbus.c` | 双总线实例（g_gt / g_gt485 双通道） |
| `test_multich.c` | 多通道实例 |
| `sim_focus.c` | 聚焦单个行为的精简仿真 |
| `main.h` | PC 假 HAL 头（替代 STM32 的 main.h，只提供库实际用到的原语） |
