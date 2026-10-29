# GridTalk

无主机多对多半双工串行总线协议库。适用于 STM32F411 双板系统（控制器 / 遥控器）
共用的 485 总线，也支持任意数量的节点接入同一共享总线。

2026-09-20 从双板固件仓库（20260323 第三版程序）抽出单独立项，自此与固件仓库
**互不依赖**：本项目是协议库的唯一源头，固件仓库里的副本不再与本项目同步。

## 设计目标

1. **跨平台兼容** —— 协议核心平台无关，不绑定特定 MCU 的寄存器与中断模型；
2. **集成简单** —— 一路定时器 + UART 收发中断即可接入，不依赖操作系统；
3. **对通信链路无要求** —— 485 直连或无线串口透传（攒包、延时、误码）都能跑，
   字节速率自适应，不假设链路质量；
4. **可用性优先** —— 先保证命令可达且恰好执行一次，在此前提下再压缩
   接入/应答/重发时延（实时性服从可用性）；
5. **无主机** —— 节点平等、即插即用，无中心协调与单点故障；
6. **应用层封装丰富** —— 链路层之上提供常用通信模式的开箱即用封装。

### 经典适用场景

- **单片机 + 485**：多节点共享半双工总线；
- **单片机 + 无线串口透传**：DTU / 串口电台等攒包突发上线的模块。

## 目录结构

```
src/    GridTalk.c / GridTalk.h    协议库本体(唯一副本, 平台无关, 不含任何芯片头文件)
port/   gridtalk_port_cmsis.h      ARM(CMSIS)端口: PRIMASK临界区(按编译器自动接入)
        gridtalk_port_stm32.h      STM32 USART绑定: 寄存器直写钩子 + 一行bindUart
app/    gridtalk_pipe.*            应用组件: 可靠有序字节管道(分片传输/fprintf)
        gridtalk_pipe_stdio.h      管道stdio适配(fopencookie/fdevopen自动探测)
sim/                                PC 总线仿真器（编译真实 src/GridTalk.c）
docs/    GridTalk协议设计.md       分层设计说明（L1物理/L2链路/L3应用）
docs/history/                      历史缺陷调查记录（环形级联随机冻结 = DWT 回绕缺陷）
```

## 接入方法

库全部状态封装在 `gridTalk_t` 结构体内，多实例、无全局单例。核心不含任何
芯片头文件：临界区经 `GT_CRIT_ENTER/EXIT` 宏注入（ARM/CMSIS 编译器自动接
入随库端口，其余平台默认空操作或经 `GT_PORT_HEADER` 指定），硬件发送经
`txByte` 与 `txKick/txKickDone/txAbort` 钩子注入。

```c
gridTalk_t gt;
gridTalk_init(&gt, USART2, myID);     /* hw=平台句柄(如USART2); 纯钩子接入传NULL */
gridTalk_setBaud(&gt, 9600);          /* 可选: 已知波特率直接定标byteUs(推荐) */
gt.nowTick = dwt_count;                /* 原始自由计数器(如DWT->CYCCNT): 推荐, */
gt.tickPerUs = 100;                    /*   库内余数精确换算µs, 免回绕失真;
                                          STM32一行: gridTalk_portBindDwt(&gt) */
/* gt.nowUs = dwt_us;                  /* 备选µs时钟 -- 须满足"差值≡真实流逝µs
                                          (mod 2^32)"; "先除后换算"(CYCCNT/100)
                                          不满足, 直接接会在计数回绕后假超时! */
gt.txByte  = my_send_byte;             /* 发送出口(逐字节) */
gt.readData    = on_frame;             /* 收到完整帧(CRC已过) */
gt.sendResult  = on_send_result;       /* needAck 发送的最终结果(OK/放弃) */
gridTalk_seed(&gt, entropy);           /* 每次上电注入不同熵, 重定序号基值(必做) */

/* 运行期入口, 任一事件到达即驱动状态机: */
gridTalk_rxByte(&gt, byte);            /* UART RX中断, 逐字节喂入 */
gridTalk_usTick(&gt, us);              /* 周期µs心跳(轮询模式下兼发送节流, 兼服务nowTick换算累加器) */

/* 中断驱动(推荐): 配置发送中断控制钩子后, 字节节奏由硬件TXE决定 */
gridTalk_txIrq(&gt);                   /* TXE中断: 数据寄存器空, 可交付1字节 */
gridTalk_txCompleteIrq(&gt);           /* TC中断: 末字节移位完成, 帧收尾 */
```

两种发送驱动（`GT_TX_IRQ`，默认 1）：

- **中断驱动（推荐）**：提供 `txKick`（帧字节已备好，开 TXE 中断）、
  `txKickDone`（末字节已交付，转等 TC）、`txAbort`（卡死兜底）三个钩子。
  STM32 一行绑定：`gridTalk_portBindUart(&gt);`（`port/gridtalk_port_stm32.h`，
  寄存器直写），中断服务程序统一入口 `gridTalk_portUsartIsr(&gt)`。
- **轮询**：三个钩子保持 NULL，`usTick/rxByte` 入口按实测 `byteUs` 节流
  逐字节发送。适合无 TX 中断或不想接中断的场合。

接缝清单：**必选** 时间源三选一 `nowTick`（推荐，自由计数器 + `tickPerUs`，
库内余数精确换算）/`nowUs`（须差值回绕一致）/全空（usTick 累加）+ `txByte`
（+临界区宏，自动）；**二选一** 中断三钩子或全空（轮询）；**可选**
`txFrameStart/End`（485 换向）、`onRxRaw`（调试）、`hw`。

发送：

```c
gridTalk_send(&gt, target, cmd, data, len);      /* 需应答: 超时重发直到OK/放弃 */
gridTalk_sendNoAck(&gt, target, cmd, data, len); /* 无应答 */
gridTalk_sendReply / sendReplyNoAck;             /* 应答走reply通道 */
```

## 应用组件:可靠有序字节管道(app/)

在任意总线实例上定义若干互不影响的管道(管道号+对端ID),每条管道是停等式
可靠有序字节流——复用 L2 的重发与序号去重,天然"有序 + 恰好执行一次":

```c
gtpipe_hub_t hub;                        /* 每总线实例一个 */
gtpipe_t    p1, p2;                      /* 每管道一个(静态分配) */

gtpipe_attach(&hub, &gt);                /* 挂接实例: 链式接管回调, 业务流量照旧 */
gtpipe_register(&p1, &hub, 1, peerID);   /* 管道1 -> 对端 */
gtpipe_register(&p2, &hub, 2, peerID);   /* 管道2 -> 对端(互不影响) */

/* 大数据分片传输(自动按~500B分段, 停等+进度回调, 零拷贝): */
char buf[8192];
gtpipe_writeAll(&p1, buf, sizeof(buf), timeoutUs, on_progress, pump, NULL);
gtpipe_readAll(&p1, buf, n, timeoutUs, NULL, pump, NULL);

/* 文本会话: */
gtpipe_printf(&p1, "temp=%d.%02u\n", 25, 5);
/* 有stdio后端时(编译期自动探测), 直接用 fprintf/fscanf: */
/* FILE *f = gtpipe_fopen(&p1, "w"); fprintf(f, "v=%d\n", v); fflush(f); */
```

大数据与小写入共用同一套可靠性;`gtpipe_write/read` 提供非阻塞小读写,
`gtpipe_status` 暴露溢出/失败诊断。语义边界与参数见 `app/gridtalk_pipe.h`。

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
| `test_pipe.c` | 应用层管道组件(分片传输/printf/共存/丢帧/错误路径) |
| `sim_focus.c` | 聚焦单个行为的精简仿真 |

PC 仿真即第二个平台实例：直接以钩子接入仿真总线模型（`txByte` 交付字节、
`txKick/txKickDone` 模拟 TXE/TC 中断），无需伪造寄存器。
