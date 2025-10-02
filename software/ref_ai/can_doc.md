4 通信协议及示例

4.1 CAN Simple 协议

默认通信接口是 CAN，最大通信速率 1Mbps（可通过 odrv0.can.config.baud_rate 读取和设
置），出⼚默认速率 500Kbps。请注意：早期硬件版本（小于等于 3.7）中 USB 与 CAN不兼容，
请参见 3.1.8 中第 3)条，如何从 USB 切换到 CAN。

驱动器默认的协议即为 CAN Simple，如果用户切换到 CANOpen 协议，则需要通过下述指
令切回 CAN Simple：

odrv0.can.config.protocol = 1

请注意，在版本大于等于 3.10 的硬件版本上，通信接口同时支持 RS485，脉冲⽅向和 PWM。

通过下述指令确保通信接口是 CAN：

odrv0.config.comm_intf_mux = 0

在电机精灵中，也可在通信参数中，切换物理接口以及 CAN 协议版本。

4.1.1 协议帧格式

CAN 通信采用标准帧格式，数据帧，11 位 ID，8 字节数据，如下表所示（左为 MSB，右为
LSB）：

数据域 CAN ID（11bits） Data（8 bytes）

分段 Bit10 ~ Bit5 Bit4 ~ Bit0 Byte0 ~ Byte7

描述 node_id cmd_id 通信数据

- node_id：代表这个电机在总线上的唯一 ID，可在 odrivetool 中用
odrv0.axis0.config.can.node_id 来读取和设置。
- cmd_id：指令编码，表示协议的消息类型，请参见本节余下内容。
- 通信数据：8 个字节，每一个消息中携带的参数会被编码成整数或浮点数，字节序为
小端（little endian），其中浮点数是按照 IEEE 754 标准进行编码。

以 4.1.2 中描述的 Set_Input_Pos 消息为例，假设其三个参数分别为：Input_Pos=3.14，
Vel_FF=1000（表示 1rev/s），Torque_FF=5000（表示 5Nm），而 Set_Input_Pos 消息的 CMD
ID=0x00C，假设驱动器的节点（node_id）被设置成 0x05，则：

- 11 位 CAN ID=(0x05<<5)+0x0C=0xAC
- 根据4.1.2 中对于 Set_Input_Pos 的描述可知，Input_Pos 在第 0个字节开始的 4 个字节，
编码为 C3 F5 48 40（浮点数 3.14 用 IEEE 754 标准编码为 32 位数 0x4048f5c3），Vel_FF
在第 4 个字节开始的 2 个字节，编码为 E8 03（1000=0x03E8），Torque_FF 在第 6 个
字节开始的 2 个字节，编码为 88 13（5000=0x1388），则 8 个字节的通信数据为：

Byte0 Byte1 Byte2 Byte3 Byte4 Byte5 Byte6 Byte7  
C3 F5 48 40 E8 03 88 13

4.1.2 帧消息

下表列出了所有的可用消息：

CMD ID | 名称 | ⽅向 | 参数
---|---|---|---
0x001 | Heartbeat | 电机→主机 | Axis_Error, Axis_State, Motor_Flag, Encoder_Flag, Controller_Flag, Traj_Done, Life
0x002 | Estop | 主机→电机 |
0x003 | Get_Error | 电机→主机 | Error_Type
0x004 | RxSdo | 电机→主机 |
0x005 | TxSdo | 电机→主机 |
0x006 | Set_Axis_Node_ID | 主机→电机 | Axis_Node_ID
0x007 | Set_Axis_State | 主机→电机 | Axis_Requested_State
0x008 | Mit_Control | 主机→电机 |
0x009 | Get_Encoder_Estimates | 电机→主机 | Pos_Estimate, Vel_Estimate
0x00A | Get_Encoder_Count | 电机→主机 | Shadow_Count, Count_In_Cpr
0x00B | Set_Controller_Mode | 主机→电机 | Control_Mode, Input_Mode
0x00C | Set_Input_Pos | 主机→电机 | Input_Pos, Vel_FF, Torque_FF
0x00D | Set_Input_Vel | 主机→电机 | Input_Vel, Torque_FF
0x00E | Set_Input_Torque | 主机→电机 | Input_Torque
0x00F | Set_Limits | 主机→电机 | Velocity_Limit, Current_Limit
0x010 | Start_Anticogging | 主机→电机 |
0x011 | Set_Traj_Vel_Limit | 主机→电机 | Traj_Vel_Limit
0x012 | Set_Traj_Accel_Limits | 主机→电机 | Traj_Accel_Limit, Traj_Decel_Limit
0x013 | Set_Traj_Inertia | 主机→电机 | Traj_Inertia
0x014 | Get_Iq | 电机→主机 | Iq_Setpoint, Iq_Measured
0x016 | Reboot | 主机→电机 |
0x017 | Get_Bus_Voltage_Current | 电机→主机 | Bus_Voltage, Bus_Current
0x018 | Clear_Errors | 主机→电机 |
0x019 | Set_Linear_Count | 主机→电机 | Linear_Count
0x01A | Set_Pos_Gain | 主机→电机 | Pos_Gain
0x01B | Set_Vel_Gains | 主机→电机 | Vel_Gain, Vel_Integrator_Gain
0x01C | Get_Torques | 电机→主机 | Torque_Setpoint

4.1.3 实战案例

4.1.3.1 校准

CAN ID | 帧类型 | 帧数据 | 说明
---|---|---|---
0x007 | 数据帧 | 04 00 00 00 00 00 00 00 | 消息：Set_Axis_State，参数：4，对电机进行校准
0x007 | 数据帧 | 07 00 00 00 00 00 00 00 | 消息：Set_Axis_State，参数：7，对编码器进行校准

4.1.3.2 速度控制

CAN ID | 帧类型 | 帧数据 | 说明
---|---|---|---
0x00B | 数据帧 | 02 00 00 00 02 00 00 00 | 消息：Set_Controller_Mode，参数：2/2，设置控制模式为速度控制，输入模式为速度斜坡
0x007 | 数据帧 | 08 00 00 00 00 00 00 00 | 消息：Set_Axis_State，参数：8，进入闭环控制状态
0x00D | 数据帧 | 00 00 20 41 00 00 00 00 | 消息：Set_Input_Vel，参数：10/0，设置目标速度和力矩前馈（目标速度为10，力矩前馈为0）

4.1.3.3 位置控制

CAN ID | 帧类型 | 帧数据 | 说明
---|---|---|---
0x00B | 数据帧 | 03 00 00 00 03 00 00 00 | 消息：Set_Controller_Mode，参数：3/3，设置控制模式为位置控制，输入模式为位置滤波
0x007 | 数据帧 | 08 00 00 00 00 00 00 00 | 消息：Set_Axis_State，参数：8，进入闭环控制状态
0x0C | 数据帧 | CD CC 0C 40 00 00 00 00 | 消息：Set_Input_Pos，参数：2.2/0/0，设置目标位置=2.2，速度前馈=0，力矩前馈=0

4.1.5 周期消息

用户可配置电机向上位机周期性发送消息，而不用上位机向电机发送请求消息。可通过
odrv0.axis0.config.can 下的一系列配置来打开/关闭周期消息（值为 0 表示关闭，为其他值表示
周期时间，单位为 ms），如下表所示：

消息 | odrivetool 配置 | 默认值
---|---|---
Heartbeat | odrv0.axis0.config.can.heartbeat_rate_ms | 100
Get_Encoder_Estimates | odrv0.axis0.config.can.encoder_rate_ms | 10
Get_Motor_Error | odrv0.axis0.config.can.motor_error_rate_ms | 0
Get_Encoder_Error | odrv0.axis0.config.can.encoder_error_rate_ms | 0
Get_Controller_Error | odrv0.axis0.config.can.controller_error_rate_ms | 0
Get_Sensorless_Error | odrv0.axis0.config.can.sensorless_error_rate_ms | 0
Get_Encoder_Count | odrv0.axis0.config.can.encoder_count_rate_ms | 0
Get_Iq | odrv0.axis0.config.can.iq_rate_ms | 0
Get_Sensorless_Estimates | odrv0.axis0.config.can.sensorless_rate_ms | 0
Get_Bus_Voltage_Current | odrv0.axis0.config.can.bus_vi_rate_ms | 0

默认情况下，前两种周期消息在出⼚时打开，所以当用户监控 CAN 总线时，会看到两种消息以设定周期进行广播。用户可通过下述指令关闭它们：

odrv0.axis0.config.can.heartbeat_rate_ms = 0  
odrv0.axis0.config.can.encoder_rate_ms = 0

---

 运动控制（MIT Control）

运动控制模式通过综合控制位置，速度和力矩来控制电机运动到目标位置，可以下述公式来

其中�������是目标力矩，�����是位置误差，�����是速度误差，��是位置控制增益，��是速度控

制增益（或叫阻尼系数），���是前馈力矩。

运动控制模式使能如下：

odrv0.axis0.controller.config.control_mode = 3

odrv0.axis0.controller.config.input_mode = 9

调整增益：

odrv0.axis0.controller.input_pos = 5 #单位 turns

odrv0.axis0.controller.input_vel = 30 #单位 turn/s

odrv0.axis0.controller.input_torque = 2 #单位 Nm
odrv0.axis0.controller.input_mit_kp = <float> #位置增益，单位 Nm/turn

odrv0.axis0.controller.input_mit_kd = <float> #阻尼系数，单位 Nm/turn/s

然后通过输⼊ input_pos，input_vel，input_torque 来进行运动控制：

请注意，在USB 控制时所输入的位置、速度和扭矩，均是指转子侧，而用 CAN进行MIT

控制时，协议中的位置、速度和扭矩均是指输出轴侧，这是为了与MIT 开源协议保持一致！

--- 

 Get_Encoder_Estimates

CMD ID: 0x009（电机主机）

起始字节 名称 类型 单位 说明

0 Pos_Estimate float32 rev odrv0.axis0.encoder.pos_estimate

当前电机转子的位置

4 Vel_Estimate float32 rev/s odrv0.axis0.encoder.vel_estimate

当前电机转子的转速


 Get_Encoder_Count

CMD ID: 0x00A（电机主机）

起始字节 名称 类型 说明

0 Shadow_Count int32 odrv0.axis0.encoder.shadow_count

编码器多圈计数

4 Count_In_Cpr int32 odrv0.axis0.encoder.count_in_cpr

编码器单圈计数

---

 Set_Linear_Count

CMD ID: 0x019（主机电机）

设置编码器绝对位置。

起始字节 名称 类型 说明

0 Linear_Count int32 odrv0.axis0.encoder.set_linear_count()

设置编码器绝对位置（计数值）。
