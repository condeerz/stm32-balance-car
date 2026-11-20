# STM32 两轮自平衡车（MPU6050 DMP + 串级 PID）

STM32F103C8T6 主控的两轮自平衡车：MPU6050 经 DMP 直接解算出姿态角（200 Hz），
双正交编码器 M 法测速，TB6612 驱动 520 减速电机，每 5 ms 执行一次三环串级 PID。

## 控制结构

```text
TIM3 更新中断 200 Hz（5 ms）
  └─ Control()
       ├─ 读编码器：TIM2 → Encoder_Left，TIM4 → Encoder_Right（取反）
       ├─ mpu_dmp_get_data() → pitch/roll/yaw，MPU_Get_Gyroscope() → gyrox/gyroy/gyroz
       ├─ 外环速度 PI  → Velocity_out（目标 Target_Speed 来自蓝牙遥控）
       ├─ 内环直立 PD  → Vertical_out（输入：Velocity_out + 机械中值 3°，评价 pitch 与 gyroy）
       ├─ 转向 PD      → Turn_out
       └─ MOTO1 = Vertical_out - Turn_out
          MOTO2 = Vertical_out + Turn_out  → Limit(±7200) → Load() → TIM1 PWM
```

## 关键参数

| 环 | 参数 | 值 |
|---|---|---|
| 直立环 PD | Kp / Kd | 180 / 2 |
| 直立环 | 机械中值 `Med_Angle` | 3° |
| 速度环 PI | Kp / Ki | 0.6 / 0.003（`Ki = Kp / 200`，随 Kp 自动换算） |
| 速度环 | 低通系数 a / 积分限幅 | 0.7 / ±20000 |
| 转向环 PD | Kp / Kd | 10 / 0.6（无左右指令时启用转向约束，有指令时 Kd 置 0） |
| 输出 | PWM 限幅 | ±7200（TIM1 ARR=7199） |
| 设定值限幅 | 前进速度 / 转向 | ±30 / ±150 |

## 外设与时基

| 外设 | 配置 | 用途 |
|---|---|---|
| TIM1 | PSC=0，ARR=7199 → 10 kHz | CH4→MOTO1、CH1→MOTO2 两路 PWM（与电机交叉映射） |
| TIM3 | PSC=71，ARR=4999 → 5 ms | **控制节拍**，更新中断里调用 `Control()` |
| TIM2 / TIM4 | 编码器模式 | 左 / 右轮 M 法测速 |
| I2C1 | 硬件 I2C | MPU6050、OLED |
| USART3 | 115200，中断收 1 字节 | 蓝牙遥控 + VOFA+ 波形 |
| GPIOB 12~15 | 推挽输出 | TB6612 两路方向控制 |

## 目录

| 路径 | 说明 |
|---|---|
| `Core/Src/main.c` | 外设初始化、OLED 实时显示、启动 TIM3 |
| `Core/Src/pid.c` | 三环 PID 与 `Control()`，5 ms 调度入口 |
| `Core/Src/encoder.c` | M 法测速 |
| `Core/Src/motor.c` | 方向引脚 + PWM 输出、限幅 |
| `Core/Src/tim.c` | TIM1 PWM / TIM2、TIM4 编码器 / TIM3 控制节拍 |
| `Core/Src/usart.c` | 蓝牙接收、`printf` 重定向、VOFA+ JustFloat 发送 |
| `Core/Src/oled.c` | OLED 驱动（I2C） |
| `Core/Src/sr04.c` | HC-SR04 超声（已从控制链路移除，仅保留代码） |
| `Core/Src/stm32f1xx_it.c` | 中断入口 |
| `MyCode/` | MPU6050 与 DMP 驱动：`inv_mpu.c`、`inv_mpu_dmp_motion_driver.c`、`mpu6050.c`、`IIC.c` |
| `MDK-ARM/OLED.uvprojx` | Keil MDK 工程文件 |
| `OLED.ioc` | STM32CubeMX 工程配置 |

## 编译与烧录

用 Keil MDK（ARM Compiler 5）打开 `MDK-ARM/OLED.uvprojx`，F7 编译、F8 下载。
仓库不含编译产物（`MDK-ARM/OLED/` 已被忽略），首次编译会重新生成。

## VOFA+ 波形

`Control()` 每 4 个控制周期发一帧 → 50 Hz，JustFloat 协议（float 小端 + 帧尾 `00 00 80 7F`），
共 6 通道：

```
pitch | gyroy | Vertical_out | Velocity_out | Target_Speed | Encoder_Left + Encoder_Right
```

用 `HAL_UART_Transmit_IT` 非阻塞发送，`gState` 非空闲时直接丢帧，避免在中断里等待。
