# Applications/arm — 机械臂模块

机械臂为"旋转基座 + 同步带伸缩 + 升降"三轴圆柱坐标结构，另带 1 路舵机夹爪、
1 个总线舵机驱动的直线物料盘（夹爪到物料盘的取放点固定，换格由物料盘移动完成），
以及 1 个翻转相机的舵机（视觉标定时翻到标定角并保持，颜色识别也用该视角；只有整臂/伸展轴回零前才恢复默认角）。
本目录按"动作 → 坐标联动 → 单轴电机 → 驱动"分层，每层只有一个入口文件，避免同一含义在多处重复定义。

升降轴**机械回零在顶部顶点**（`ARM_LIFT_HOME_DIR=0`，向上顶顶部限位），但对外坐标 `z_mm` / `lift_mm`
仍然表示**距底部的抬升高度**（0=最底部，向上增大）：`arm_ctrl.c` 用
`ARM_LIFT_TOP_HOME_OFFSET_MM`（=顶部回零点在"从底部计"坐标里的高度，默认 `ARM_LIFT_MAX_MM`）
把顶部机械零位换算回底部坐标，所以原来按"底部抬升高度"填写的位姿参数无需修改。

## 1. 分层与调用方向

```
mission 任务流程 (Applications/mission/mission.c)
   │  Mission_Arm*                任务动作编排: 任务码 -> 颜色/格位 -> 逐物料循环
   ▼
mission_arm.c / mission_arm.h     任务动作编排层(从 mission.c 剥离)
   │  Arm_Action_*                动作流程: 抓料上盘 / 放料入位
   ▼
arm_action.c / arm_action.h       动作序列层(目标位姿 + 夹爪/物料盘配合 + 标定预备/恢复)
   │  Arm_MoveToAndWait / Arm_GripperSetAngle / Arm_PadSetSlot / Arm_FlipSetAngle
   ▼
arm_motion.c / arm_motion.h       坐标联动层: 笛卡尔 / 极坐标 + 到位等待 + 位置回读(三种数据)
   │  Arm_Kin_* / Arm_BaseRotateAbs / Arm_ExtendTo / Arm_LiftTo / Arm_GetPos
   ├──────────────► utils/arm_kinematics.c   正/逆运动学(纯数学, 不碰硬件)
   ▼
utils/arm_ctrl.c / arm_ctrl.h     执行器层: 电机命令/回零/换算/到位 + 夹爪、物料盘、相机翻转舵机
   │  Motor_Send_Cmd / Servo_Port_SetAngle
   ▼
zdt_v5 步进电机驱动(Drivers/User/STEP) / servo 舵机驱动(Drivers/User/SERVO)
```

- 上层只调用下一层的公共接口，不跨层访问驱动；夹爪/物料盘/相机翻转舵机属于执行器、无坐标解算需求，
  动作层直接调用 arm_ctrl 的 `Arm_GripperSetAngle` / `Arm_PadSetSlot` / `Arm_FlipSetAngle`。
- 全模块只有 `arm_ctrl.c` 里的 `s_axis[]` 轴表保存"电机 ID / 轴名 / 回零方向 / 减速比"，
  其它文件不再重复维护这些映射。

## 2. 文件职责

| 文件 | 职责 |
| --- | --- |
| `arm_config.h` | 机械臂的电机/换算/限位/回零/速度/到位/夹爪/物料盘/相机翻转舵机/动作目标位姿参数（全模块参数唯一来源） |
| `arm_motion.c/.h` | 应用运动接口：笛卡尔/极坐标移动、等待到位、当前位置读取（三种数据）、模块初始化 |
| `arm_action.c/.h` | 任务动作：原料区抓取上物料盘、物料盘取料放到暂存区/粗加工区（含物料盘换格与夹爪配合）、视觉标定预备/收尾（三轴预备位姿 + 相机翻转）；工位目标位姿在 `arm_config.h`，动作内部偏移参数在 `.c` 顶部 |
| `utils/arm_ctrl.c/.h` | 执行器控制：电机命令封装/固件内置回零/角度长度换算/位置读取/到位判定 + 夹爪/物料盘/相机翻转舵机输出 |
| `utils/arm_kinematics.c/.h` | 运动学：逆解(笛卡尔/半径 → 基座角度 + 同步带伸长量, 含行程检查) 与正解(回读用) |
| `tools/arm_tools.c` | shell 调试命令 `arm init/home/en/zero/ctrl/pol/pos/gri/pad/flip/preset/presets/act/yaw/key`（`home` 指定单轴固件回零，`en` 只做电机使能/失能，`zero` 以当前位置为零位，`flip` 控制相机翻转舵机角度，`preset` 走到预设工位位姿且 `storage/process` 支持格位/层参数，`act` 跑完整任务动作：原料抓取上盘 / 物料盘取放到粗加工区、暂存区 / 反向 `pick-process`、`pick-storage` 把场地物料收回到物料盘，`act pose` 只走到 yaw 补偿后的场地放置位，`yaw` 设置/清除视觉 yaw 误差的基座补偿角，`key` 键盘点动控制 W/S 伸展、A/D 旋转、Q/E 升降、Z/X 夹爪开合） |

> 全模块的参数只有两处：`arm_config.h`（电机/限位/回零/速度/夹爪/物料盘/相机翻转舵机/动作目标位姿与格位预设）与 `arm_action.c` 顶部"动作内部偏移参数"（抓取抬升/盘面下降）。

## 3. 公共 API

命名约定（与工程其它模块一致）：对外接口 `Arm_<对象/层><动词>`（CamelCase），
模块内部函数 `static arm_*`（snake_case），宏统一 `ARM_` 前缀并带单位后缀（`_MS`/`_MM`/`_DEG`/`_RPM`/`_RAW` 等）。

### arm_motion.h（推荐给应用层使用）

| 接口 | 说明 |
| --- | --- |
| `void Arm_Init(void)` | 初始化机械臂并发起异步整臂回零（非阻塞；重复调用会重新发起一轮回零） |
| `bool Arm_MoveTo(x_mm, y_mm, z_mm)` | 笛卡尔移动（异步下发，不等待到位），`z_mm` 为抬升高度（0=最底部） |
| `bool Arm_MoveToAndWait(x_mm, y_mm, z_mm, timeout_ms)` | 笛卡尔移动并等待到位（下降动作先旋转+伸展再下降；上升动作先上升再旋转+伸展），夹爪动作前使用 |
| `bool Arm_MovePolarTo(extend_mm, base_angle_deg, lift_mm)` | 极坐标移动（不走逆解，直接给基座角度，异步）；`extend_mm` 为**同步带伸展长度**（与 `Arm_ExtendTo`/`arm pos` 的 extend 一致），`lift_mm` 为抬升高度 |
| `bool Arm_MovePolarToAndWait(extend_mm, base_angle_deg, lift_mm, timeout_ms)` | 极坐标移动并等待到位（同样的方向分步：下降先旋转+伸展再下降，上升先上升再旋转+伸展） |
| `void Arm_SetMotionProfile(&profile)` / `const ArmMotionProfile_t *Arm_GetMotionProfile(void)` | 设置/读取当前动作使用的三轴速度/加速度 profile；动作层按动作切换，实现"每个动作速度/加速度解耦"（传 NULL 恢复默认） |
| `bool Arm_GetPos(ArmPos_t *pos)` | 读取当前位置的三种数据（电机角度 / 笛卡尔坐标 / 极坐标），各分量是否有效见 `*_valid` |
| `bool Arm_GetCartesianPose(ArmCartesianPose_t *pose)` | 读取当前笛卡尔坐标 `x/y/z`（mm，`z`=抬升高度） |
| `bool Arm_GetPolarPose(ArmPolarPose_t *pose)` | 读取当前极坐标（伸展长度 / 旋转角度 / 抬升高度） |

位置回读结构（`arm_motion.h` 定义，`arm pos` 与任务层共用）：

```
ArmPos_t
 ├─ motor_angle_deg[] / motor_angle_valid[]      1) 各电机当前角度(°, 顺序: 基座/升降/伸展)
 ├─ cartesian { x_mm, y_mm, z_mm } + cartesian_valid   2) 笛卡尔坐标(mm), z=抬升高度
 └─ polar { extend_mm, base_angle_deg, lift_mm } + polar_valid  3) 极坐标: 伸展长度/旋转角度/抬升高度
```

### arm_ctrl.h（坐标联动层与调试工具使用）

| 接口 | 说明 |
| --- | --- |
| `void Arm_CtrlInit(void)` | 创建回零任务并发起异步整臂回零（一般由 `Arm_Init` 调用） |
| `bool Arm_HomeWaitDone(timeout_ms)` | 等待整臂回零完成 |
| `bool Arm_HomeAxis(axis)` | 指定单轴执行固件回零（阻塞到完成/失败；整臂回零进行中会被拒绝，其它轴不动） |
| `bool Arm_MotorsEnable(enable)` | 只使能（锁轴）/失能（松轴）三轴电机，不发回零/运动命令；失能后机械臂可能因重力下坠 |
| `bool Arm_SetCurrentPosAsZero(timeout_ms)` | 以当前位置为三轴零位（只下发位置清零命令，先等三轴静止；不做回零/运动） |
| `bool Arm_BaseRotateAbs(angle_deg, vel, acc)` | 基座旋转到绝对角度（带软限位） |
| `bool Arm_ExtendTo(length_mm, vel, acc)` | 同步带伸长到绝对长度（0=完全收回） |
| `bool Arm_LiftTo(lift_mm, vel, acc)` | 升降抬升到距底部指定高度（0=最底部，向上增大） |
| `bool Arm_MotorGetAngle(motor_id, &angle_deg)` | 读取指定电机当前输出角度（°；直线轴的物理长度用下面两个接口） |
| `bool Arm_ExtendGetLength(&length_mm)` | 读取同步带当前伸展长度（mm） |
| `bool Arm_LiftGetHeight(&height_mm)` | 读取升降当前抬升高度（mm，0=最底部） |
| `bool Arm_WaitSettled(timeout_ms)` | 等待三轴到位（位置连续稳定 + 已下发目标的轴到达目标） |
| `bool Arm_WaitSettledHorizontal(timeout_ms)` | 只等水平两轴（基座旋转+伸缩）到位（分步运动用） |
| `bool Arm_WaitSettledLift(timeout_ms)` | 只等升降轴到位（上升动作"先升到位再旋转+伸展"用） |
| `bool Arm_GripperSetAngle(angle_deg)` | 夹爪角度；阻塞到角度回读确认到位（到达目标角或夹稳/被挡停，容差/超时见 `arm_config.h`） |
| `bool Arm_PadSetSlot(slot)` | 物料盘换格：把第 `slot` 格移到夹爪固定的取放位置（= `Start` + `WaitArrive`，阻塞到角度回读确认到位） |
| `bool Arm_PadSetSlotStart(slot)` | 仅下发物料盘换格命令（非阻塞），与 `Arm_PadWaitArrive` 配对实现“边转盘边动臂” |
| `bool Arm_PadWaitArrive(timeout_ms)` | 校验物料盘是否已到最近一次 `Arm_PadSetSlotStart` 的格位（角度回读，阻塞到到位/超时） |
| `void Arm_FlipSetAngle(angle_deg)` | 相机翻转舵机角度（标定链路 + `arm flip` 调试命令共用；内部等待 `ARM_FLIP_SETTLE_MS`，`ARM_FLIP_ENABLE=0` 时空操作） |
| `Arm_AxisCount` / `Arm_AxisMotorId` / `Arm_AxisName` | 遍历轴信息（调试/日志用） |

### arm_action.h（mission 任务流程使用）

| 接口 | 说明 |
| --- | --- |
| `bool Arm_Action_LoadPad(color, pad_slot)` | 原料区抓取指定颜色物料（超时保护），放到物料盘第 `pad_slot` 格；**抓料前先启动物料盘换格**，换格与抓取并行 |
| `bool Arm_Action_UnloadToProcessing(pad_slot, process_slot, next_pad_slot)` | 物料盘第 `pad_slot` 格取料 → 粗加工区第 `process_slot` 格；取到料后立即预启动 `next_pad_slot` 换格（0=无下一格），与移臂/放料并行 |
| `bool Arm_Action_UnloadToStorage(pad_slot, storage_slot, layer, next_pad_slot)` | 物料盘第 `pad_slot` 格取料 → 暂存区第 `storage_slot` 格，`layer` 0=第一层 1=叠放层；取到料后立即预启动 `next_pad_slot` 换格 |
| `bool Arm_Action_LoadPadFromProcessing(process_slot, pad_slot)` | 粗加工区第 `process_slot` 格取料 → 物料盘第 `pad_slot` 格（`UnloadToProcessing` 的反向） |
| `bool Arm_Action_LoadPadFromStorage(storage_slot, layer, pad_slot)` | 暂存区第 `storage_slot` 格/`layer` 层取料 → 物料盘第 `pad_slot` 格（`UnloadToStorage` 的反向） |
| `bool Arm_Action_CalcPlacePose(area, slot, layer, &pose)` | 查放置位预设表（粗加工区每格、暂存区每格每层一个逐点标定位姿），与放置/取料动作及调试命令共用同一份数据 |
| `bool Arm_Action_MoveToInitPose(void)` | 回到机械臂初始状态位（行驶/待机位：臂收短+抬升，位姿见 `ARM_POSE_INIT_*`）；整臂回零完成后自动走到该位，每次小车移动前也先回到该位 |
| `bool Arm_Action_CalibrationReady(pose)` | 视觉标定预备：三轴（旋转/伸展/抬升）移动到指定预备位姿并等待到位 + 基座读回校验，张开夹爪到 `ARM_CAL_GRIPPER_OPEN_DEG`，再把相机翻到标定角度 |
| `void Arm_Action_CalibrationFinish(void)` | 视觉标定结束收尾：保持相机翻转角（不再自动恢复默认角；默认角只在整臂/伸展轴回零前或 `arm flip restore` 时恢复），成功/失败/中止后都要调用 |
| `void Arm_Action_SetYawCorrection(yaw_err_deg)` | 设置/清除视觉标定 yaw 误差的基座补偿角：抓取/放置时场地固定工位基座角度 += 该角；0=不补偿 |

标定预备位姿结构（`ArmCalPose_t`，mission_config.h 按目标区域配置后传入）：

```
ArmCalPose_t
 ├─ base_angle_deg   基座目标角度(°)
 ├─ extend_mm        伸展长度(mm), 0=完全收回, 范围 0~ARM_EXTEND_MAX_MM(125)
 └─ lift_mm          抬升高度(mm), 0=最底部, 范围 0~ARM_LIFT_MAX_MM(170)
```

> **统一口径**：整个机械臂模块**所有伸展/位姿参数只填伸展长度数值**（`extend_mm`，即 `arm pos`
> 极坐标段的 `extend` 读数、`Arm_ExtendTo` 的数值），不需要再写 "值 + `ARM_BASE_TO_EXTEND_MM`"，
> 也不需要手工换算水平半径。位姿宏 `ARM_POSE_*_INIT` 可直接抄 `arm pos` 的 extend/rotate/lift 三个读数。
> `ARM_BASE_TO_EXTEND_MM` 只是内部运动学偏移（水平半径 = 偏移 + 伸展长度），仅用于笛卡尔接口
> `Arm_MoveTo` 的逆解和 `arm pos` 的笛卡尔 x/y 正解，**与位姿/伸展配置无关**。
> 伸展长度超出 `0~ARM_EXTEND_MAX_MM(125)` 软限位时会被拒绝：`Arm_Action_CalibrationReady`
> 会在下发第一根轴（伸展）时直接拒绝、**三轴都不动**，任务层判标定失败（默认跳过该点标定继续任务，
> 见 `mission_config.h` 的 `MISSION_CAL_ARM_PREP_SOFT_FAIL=1`）。

> 返回值：`true`=全部步骤确认到位；`false`=参数非法/运动未到位/颜色识别超时/换格失败。
> 动作中途失败时机械臂可能停在中间位姿，任务层应把本轮判失败，不要继续执行后续动作。

## 4. 坐标与单位约定

- `x_mm` / `y_mm`：水平面目标坐标（mm），原点在基座回转中心，`x` 为基座零位方向。
- `base_angle_deg`：基座目标角度（°），0 = 机械零位，正/负为两侧。
- `z_mm` / `lift_mm`：升降轴**距底部的抬升高度**（mm），0 = 最底部，
  数值越大越靠上，上限 `ARM_LIFT_MAX_MM`。（机械回零点在顶部顶点，逻辑坐标仍从底部计，
  见 `ARM_LIFT_TOP_HOME_OFFSET_MM`；不是"距顶部的下伸距离"，调试时不要再按 down 理解。）
- 位置命令单位与读取位置单位不同，不可混用：
  - 位置命令：脉冲/圈 = `ARM_MOTOR_PULSE_PER_REV` × 减速比；
  - 读取位置：65536/圈（Emm 固件，按电机轴）× 减速比；
  - 两者相差 20.48 倍，若把读取位置直接当脉冲下发，电机会失控。
- 软限位：基座 `ARM_BASE_MIN_DEG~ARM_BASE_MAX_DEG`、伸展 `0~ARM_EXTEND_MAX_MM`(125)、
  升降（抬升高度）`0~ARM_LIFT_MAX_MM`(170)，超限时对应接口直接返回 `false` 且不下发命令。
- 伸展轴方向：`0` = 完全收回端（机械零位，回零于此），`Arm_ExtendTo` **数值增大 = 实际伸出**，
  且实际伸出时 `arm pos` 的**电机角度和伸展长度都为正值**。实车"命令方向"和"回读符号"都与初始假设不一致：
  `arm_ctrl.c` 的 `s_axis[]` 把两者分开配置——伸展轴 `linear_sign = -1`（命令：dir=0 收回、dir=1 伸出）、
  `read_sign = -1`（回读：命令取反后固件坐标在伸出时为负，回读再取反得到正读数），
  到位判定用的"期望回读位置"按两者关系换算。换线/换电机后只改这两个符号，
  不要通过改零位/回零方向去"修正"。
- 运动学（仅笛卡尔接口相关）：水平半径 = `ARM_BASE_TO_EXTEND_MM` + 伸展长度；逆解 `base_angle_deg = atan2(y, x)`，
  正解 `x = r·cosθ`、`y = r·sinθ`（位置回读用，不做行程检查）。
  `ARM_BASE_TO_EXTEND_MM` **不是 0**：它是基座回转中心到同步带固定段的固定水平偏移（当前 133mm，
  按暂存区实测点反推的初值），只在 `Arm_MoveTo`/`arm ctrl` 的笛卡尔逆解和 `arm pos` 的笛卡尔 x/y 里使用；
  该值少算时笛卡尔读数会被扭曲、外格还会算出超伸展行程。精确标定：回零后伸到最短，用卷尺量
  回转中心→夹爪中心的水平距离。
  **位姿/极坐标参数不经过这个偏移换算**：`ArmPose_t.extend_mm`、`Arm_MovePolarTo`、`arm pol`
  与 `arm pos` 的 extend 读数完全一致，直接填数值即可（不需要再手动 `+133`）。
- 放置位姿预设（暂存区/粗加工区）：已不再用"中心位姿 + 格距位移换算"，改为**逐点预设表**——
  每个格位（暂存区每层）单独一个 `ARM_POSE_*_SLOTn_*_INIT`，`Arm_Action_CalcPlacePose` 直接查表，
  任务动作 / `arm preset process|storage` / `arm act pose` 共用同一份数据。
  标定方法：用 `arm preset process <slot>`（或 `storage <slot> <layer>`）走到目标位，按 `arm pos` 微调后
  把 `arm pos` 极坐标段的 **extend / rotate / lift 三个数值原样填回** `arm_config.h` 对应宏
  （`extend_mm` / `base_angle_deg` / `lift_mm`，不要再加 `ARM_BASE_TO_EXTEND_MM`）；
  `arm presets` 可列出全部预设值的当前配置。
  这样格位实际不共线/间距不均时也能逐点对准，不再受位移偏移误差影响。
- 物料盘：由总线舵机驱动做直线运动，夹爪到物料盘的取放位置固定（`s_pose_pad` 上方安全高度 + `PAD_REACH_MM` 下降量），
  换格由 `Arm_PadSetSlot` 把目标盘位移动到夹爪下方；第 k 个抓取的物料放第 k 格，取料按同样顺序。
  盘位角度**三个盘位分开填**：`ARM_PAD_SLOT1/2/3_ANGLE_DEG`（实测各盘位角度并不等距，不再用"基准角+固定格距"推算）；
  标定方法：把物料盘摆到目标盘位，用 `servo` 调试命令读多圈角度后填回对应宏（也可先用占位值 `arm pad <slot>` 走位再微调）。

## 5. 标定 / 常改参数索引

| 要改什么 | 改哪里 |
| --- | --- |
| 电机总线 ID、电机数量、夹爪舵机 ID | `arm_config.h` → "电机与轴定义" |
| 各轴减速比、电机每转脉冲、读取位置单位 | `arm_config.h` → "换算参数" |
| 各轴默认速度/加速度（想更快：先加 ACC 档位再逐步加 VEL；同步带伸展轴最敏感） | `arm_config.h` → "各轴默认速度" 的 `ARM_*_VEL`(RPM) / `ARM_*_ACC`(Emm 档位 0~255，不能填 0) |
| 各动作单独速度/加速度（已解耦，互不影响） | `arm_config.h` → "各动作速度/加速度(解耦)" 的 `ARM_PROFILE_xxx_INIT`（原料抓取上盘/盘→粗加工/盘→暂存/粗加工→盘/暂存→盘/回初始位/标定预备），初值=各轴默认值 |
| 各轴回零方向（升降回零到顶部顶点，逻辑高度仍从底部计） | `arm_config.h` → "回零" 的 `ARM_LIFT_HOME_DIR`（升降=0，朝顶部限位） |
| 顶部回零点在"从底部计"坐标里的高度（顶部回零换算用） | `arm_config.h` → "行程软限位" 后的 `ARM_LIFT_TOP_HOME_OFFSET_MM`（默认 `ARM_LIFT_MAX_MM`；实测行程不同时改这里，**不要改位姿参数**） |
| 行程软限位（升降为抬升高度 `ARM_LIFT_MIN_MM` / `ARM_LIFT_MAX_MM`，自底部量起）| `arm_config.h` → "行程软限位"；**目标超出限位时自动钳到限位值继续执行**（想限制某方向可达范围，把 MIN/MAX 改成实测机械极限即可） |
| 运动学几何偏移（基座回转中心到同步带固定段） | `arm_config.h` → "运动学几何参数" 的 `ARM_BASE_TO_EXTEND_MM`（当前 133mm 为实测反推初值，用卷尺量回转中心→夹爪中心校准；不要填 0。**仅内部笛卡尔换算用**，位姿/伸展参数不要再加它） |
| 回零模式/方向/速度/超时 | `arm_config.h` → "回零" |
| 到位判定时间/容差/超时、分步运动"已在目标上"的段跳过容差 | `arm_config.h` → "到位等待"（`ARM_MOVE_SKIP_HORIZONTAL_MM/BASE_DEG/LIFT_MM`：目标与当前差值在容差内时跳过该段，省一次到位等待） |
| 夹爪张开/闭合角度、开合到位判定（容差/轮询/超时/夹稳判定/回读不可用兜底） | `arm_config.h` → "夹爪参数"（常规张开 `ARM_GRIPPER_OPEN_DEG`=-55；标定/原料区抓取大张开 `ARM_CAL_GRIPPER_OPEN_DEG`=-110） |
| 物料盘舵机 ID、盘位数量、**各盘位角度（三个盘位分别实测填写，不用固定格距推算）**、换格到位判定（容差/轮询/超时/回读不可用兜底） | `arm_config.h` → "物料盘参数" 的 `ARM_PAD_SLOTn_ANGLE_DEG` |
| 视觉 yaw 误差的基座补偿（开关/限幅/方向） | `arm_config.h` → "视觉 yaw 误差的机械臂补偿"（`ARM_YAW_CORRECTION_ENABLE/MAX_DEG/SIGN`） |
| 键盘点动步长/连续按键最小间隔 | `arm_config.h` → "键盘点动(arm key)"（`ARM_KEY_EXTEND_STEP_MM/ANGLE_STEP_DEG/LIFT_STEP_MM/REPEAT_MS`） |
| 相机翻转舵机 ID、标定翻转角、复位角、翻转等待时间、启用开关 | `arm_config.h` → "相机翻转舵机"（标定流程内翻到标定角并保持；整臂/伸展轴回零前翻回默认角；`ARM_FLIP_ENABLE=0` 可整体关闭；调试用 `arm flip <deg>` / `arm flip cal\|restore` 直接转角度） |
| 标定前机械臂预备位姿（原料区/粗加工区/暂存区的旋转、伸展、抬升） | `mission_config.h` → "标定前机械臂预备位姿（按目标区域）" 的 `MISSION_CAL_ARM_*`（受 `ARM_CAL_PREP_ENABLE` 控制） |
| 视觉标定/原料区抓取时的夹爪大张开角度（避免挡相机视野/夹空撞料） | `arm_config.h` → "树莓派视觉标定预备动作" 的 `ARM_CAL_GRIPPER_OPEN_DEG`（默认 -110°，标定失败只告警） |
| 上电是否自动回零 | `arm_config.h` → `ARM_AUTO_HOME_ON_INIT`（0=由 `arm init` 手动触发） |
| 各工位目标位姿（机械臂初始状态/原料区抓取/物料盘/暂存区/粗加工区） | `arm_config.h` → "动作目标位姿(待标定)" 的 `ARM_POSE_*_INIT`（**当前均为占位值，必须实测标定**；调试用 `arm preset init\|grab\|pad\|process [slot]\|storage [slot] [layer]` 直接走位核对） |
| 盘面下降（动作内部垂直偏移） | `arm_action.c` 顶部 "动作内部偏移参数" 的 `PAD_REACH_MM`（**占位值，必须实测标定**；原料区抓取后不再单独抬升，抬升由后续移动的"先升到盘安全高度再水平"一次完成） |
| 粗加工区/暂存区各格位、各层的放置位姿（逐点预设） | `arm_config.h` → "动作目标位姿(待标定)" 的 `ARM_POSE_PROCESS_SLOTn_INIT` / `ARM_POSE_STORAGE_SLOTn_Lm_INIT`（**逐点标定**，用 `arm preset ...` 对位后填回） |

## 6. shell 调试命令

```
arm init                       # 后台发起整臂回零(约数十秒), 用 arm pos 观察
arm home [base|lift|extend | motor_id]  # 指定单轴固件回零(阻塞到完成; 无参数列出轴名/电机ID)
arm pos                        # 打印三种数据: 电机角度 / 笛卡尔坐标 xyz / 极坐标(伸展/旋转/抬升)
arm en on|off                  # 只让三轴电机使能(锁轴)/失能(松轴); 不回零、不运动(失能后臂可能下坠)
arm zero                       # 把机械臂当前位置设为三轴零位(只清零, 不回零/不运动; 需机械臂静止; 升降轴须在顶部零位执行)
arm ctrl <x_mm> <y_mm> <z_mm>              # 笛卡尔移动(异步); z=抬升高度(0=最底部)
arm pol  <extend_mm> <angle_deg> <lift_mm> # 极坐标移动(异步); extend=伸展长度(同 arm pos 的 extend), lift=抬升高度(0=最底部)
arm gri  open|close            # 夹爪开合
arm pad  <slot>                # 物料盘换到第 slot 盘位(1~ARM_PAD_SLOT_NUM, 走到该盘位的实测角度, 标定用)
arm flip                       # 相机翻转舵机: 打印配置(ID/标定角/复位角/等待时间/使能)
arm flip <deg>                 # 相机翻转舵机转到指定角度(0~180, 标定翻转角用)
arm flip cal|restore           # 转到 ARM_FLIP_CAL_ANGLE_DEG(标定角) / ARM_FLIP_RESTORE_ANGLE_DEG(复位角)
arm preset init|grab|pad                   # 固定预设位姿(初始状态/原料区抓取位/物料盘位; 阻塞等到位并打印读回坐标)
arm preset process [slot]                  # 粗加工区放置位(slot 1~ARM_SLOT_NUM, 默认第 1 格)
arm preset storage [slot] [layer]          # 暂存区放置位(默认第 1 格/第 0 层; layer 0/1)
arm presets                                # 打印全部预设位姿(固定工位 + 各格位/层预设, 核对标定值用)
arm act grab <color> [pad_slot]            # 完整动作: 原料区抓指定颜色物料 -> 物料盘格位(默认第1格; 等 RPI 颜色识别)
arm act process [pad_slot] [slot]          # 完整动作: 物料盘取料 -> 粗加工区格位(默认 1/1; 含夹爪/抬升/放料)
arm act storage [pad_slot] [slot] [layer]  # 完整动作: 物料盘取料 -> 暂存区格位/层(默认 1/1/0; layer 0/1)
arm act pick-process [slot] [pad_slot]     # 完整动作(反向): 粗加工区格位取料 -> 物料盘格位(默认 1/1)
arm act pick-storage [slot] [layer] [pad_slot]  # 完整动作(反向): 暂存区格位/层取料 -> 物料盘格位(默认 1/0/1)
arm act pose process|storage [slot] [layer]  # 只走到(叠加视觉 yaw 补偿后的)场地放置位, 打印目标/实际基座角
arm yaw                                    # 查看当前视觉 yaw 误差的机械臂补偿角(开关/方向/限幅)
arm yaw <deg>                              # 模拟 rpi cal 的 yaw 误差: 之后 arm act 的场地工位基座旋转叠加该角
arm yaw off                                # 清除 yaw 补偿
arm key [extend_step_mm] [angle_step_deg] [lift_step_mm]  # 键盘点动: W/S=伸展伸长/收回, A/D=基座左转/右转, Q/E=升降/下降, Z/X=夹爪张开/闭合, ^C=退出
arm act                                    # 打印 act 用法与各参数范围/前置条件
servo angle <id> <deg>         # 标定相机翻转角度用: id 填 ARM_FLIP_SERVO_ID, deg 填候选角度
```

`arm pos` 输出三段（任一分量读取失败/电机离线时对应段显示 `unavailable`）：

```
[arm] pos 1/3 motor angles:          # 各电机当前角度(线性轴按逻辑零点折算: 升降从底部、伸展从收回端)
[arm] pos 2/3 cartesian:             # 笛卡尔坐标 x/y/z (mm), z=抬升高度
[arm] pos 3/3 polar:                 # 极坐标: 伸展长度/旋转角度/抬升高度
```

## 7. 与外部模块的关系

- `Applications/mission/mission.c` + `mission_arm.c`：任务流程在导航段之间调用 `mission_arm.c` 的
  `Mission_ArmLoadPad` / `Mission_ArmUnloadToProcessing` / `Mission_ArmLoadPadFromProcessing` /
  `Mission_ArmUnloadToStorage` 四个编排函数，由它们调用 `Arm_Action_*`；
  每轮流程为"原料抓取上盘 → 粗加工区放置 → **从粗加工区取回上盘** → 暂存区放置"，
  取回步骤不能省（否则去暂存区时物料盘是空的，物料会留在粗加工区）；
  物料盘格位号按每轮抓取顺序传 `id + 1`，第 k 个抓到的物料放第 k 格，后续按同样顺序取第 k 格。
  另在每段导航（小车移动）出发前做机械臂准备，分 `Mission_ArmPreMoveStart` / `Mission_ArmPreMoveWait`
  两步（原 `Mission_ArmPreMove` 保留为两步合体的封装）：先把机械臂收回初始状态
  （`Arm_Action_MoveToInitPose`，位姿见 `ARM_POSE_INIT_*`），再把物料盘换到 2 号基准格
  （`ARM_PAD_SLOT_BASE`，`MISSION_PAD_PREPOSE_ENABLE` 可关闭），避免带长臂/低姿态行驶，
  也避免物料盘在行驶时伸出车体刮碰，并给后续换格一个已知初始位置。
  Start 只启动物料盘换格 + 执行收臂动作，Wait 再等物料盘到位；任务层在两步之间把
  "路线首段的原地旋转"提前启动（`MISSION_PAD_PREPOSE_ROTATE_ENABLE`），
  让物料盘换格与车体原地旋转并行。
  机械臂动作是否参与流程由 `mission_config.h` 的 `MISSION_ARM_ENABLE` 控制
  （默认随 `MISSION_DEBUG_MODE`：调试模式关闭、比赛模式开启）。
- `Applications/mission/mission_config.h`：标定前机械臂预备位姿按目标区域配置
  （`MISSION_CAL_ARM_GRAB/PROCESSING/STORAGE_*`），由 `mission_nav.c` 的标定流程
  按目标点名（MATL_GRAB/MATL_PROCESSING/MATL_STORAGE）映射后传给 `Arm_Action_CalibrationReady`。
- `Core/Src/freertos.c`：`ARM_AUTO_HOME_ON_INIT == 1` 时在系统初始化阶段调用 `Arm_Init()`；
  为 0 时不自动回零，需要 shell `arm init`（或任务开始前手动触发），否则绝对位姿不准。
- `Drivers/User/STEP/zdt_v5_cfg.h`：用 `ARM_MOTOR_NUM` 计算总线电机总数；
  `zdt_v5_port.c` 按 ID 1..`ZDT_STEP_NUM` 注册全部电机，其中机械臂电机为 ID 5~7。
- `.eide/eide.yml`：维护本目录参与编译的文件清单；**新增/重命名文件必须同步修改**。
  若工程正在 IDE(Trae/EIDE) 中打开，外部修改该文件可能被 IDE 内存中的旧工程模型回写覆盖，
  改完请在 IDE 中重新加载工程或重新打开项目。

## 8. 相对旧版的重命名对照

| 旧名 | 新名 | 说明 |
| --- | --- | --- |
| `arm_port.c/.h` | `arm_motion.c/.h` | `port` 在本工程专指驱动适配层，原文件实为应用级运动接口 |
| `utils/arm_ctrl_math.c/.h` | `utils/arm_kinematics.c/.h` | 内容为运动学；同时去掉对 CMSIS `arm_math.h` 的依赖与重复的 `PI` 定义 |
| `Arm_Port_Ctrl` / `Arm_Port_Ctrl_Wait` | `Arm_MoveTo` / `Arm_MoveToAndWait` | — |
| `Arm_Port_Ctrl_Polar` | `Arm_MovePolarTo` | — |
| `ArmPose_t.radius_mm` / `Arm_MovePolarTo(radius_mm, ...)` / `arm pol <radius_mm>` / `arm presets` 的 `radius=` | `ArmPose_t.extend_mm` / `Arm_MovePolarTo(extend_mm, ...)` / `arm pol <extend_mm>` / `extend=` | 伸展逻辑统一为"只填伸展长度"：位姿宏/极坐标接口/`arm pos` 读数口径一致，不再写 `值 + ARM_BASE_TO_EXTEND_MM`；水平半径只在内部笛卡尔换算使用 |
| `Arm_Port_Gripper` | `Arm_GripperSetAngle` | 归入 arm_ctrl 层，作为夹爪唯一接口，不再重复暴露 |
| `arm_action_grab_and_storage` | `Arm_Action_LoadPad` | 名称体现"从原料区抓取并放到物料盘" |
| `arm_action_place_at_storage` | `Arm_Action_UnloadToStorage` | — |
| `arm_action_place_at_process_1/2` | `Arm_Action_UnloadToProcess(slot, layer)` | 两个近似函数的差异(叠层)改为参数 |
| `arm_action_init` | （删除） | 死代码，原调用方并不存在 |
| `arm_ctrl_init` | `Arm_CtrlInit` / `Arm_Init` | 模块初始化统一走 `Arm_Init()` |
| `ARM_USING` | `ARM_AUTO_HOME_ON_INIT` | 原宏实为"上电是否自动回零"，并非"是否使用机械臂" |
| `BASE_VEL` 等裸宏 | `ARM_BASE_VEL` 等 | 统一模块前缀，避免泄漏到包含电机驱动的其它编译单元 |
| `arm_home_wait_done` / `arm_wait_settled` / `arm_base_rotate_abs` / `arm_extend_to` / `arm_lift_to` / `arm_motor_get_angle` / `arm_gripper_set_angle` | `Arm_HomeWaitDone` / `Arm_WaitSettled` / `Arm_BaseRotateAbs` / `Arm_ExtendTo` / `Arm_LiftTo` / `Arm_MotorGetAngle` / `Arm_GripperSetAngle` | 公共 API 统一为 `Arm_<对象/层><动词>` 命名 |
| `arm_home_status` / `arm_all_stop` / `arm_motor_*` / `arm_extend_by` / `arm_lift_by` / `arm_extend_rotate_*` / `arm_base_rotate_rel` | （删除） | 全仓无调用的死接口；单轴实现改为 arm_ctrl.c 内部 static |
| `down_mm` / `target_z` 语义 | `z_mm` / `lift_mm` 语义 | 升降坐标由"距顶部下伸距离"改为"距底部抬升高度"；旧下伸值 `d` 等价新抬升值 `ARM_LIFT_MAX_MM - d` |
| `ARM_LIFT_MIN_DOWN_MM` / `ARM_LIFT_MAX_DOWN_MM` | `ARM_LIFT_MIN_MM` / `ARM_LIFT_MAX_MM` | 抬升高度软限位（0=最底部，仍从底部计）；机械回零点后续又调整为顶部顶点，换算见 `ARM_LIFT_TOP_HOME_OFFSET_MM` |
| `arm_action_offset_down` | `arm_action_offset_z` | z 偏移量正方向由"向下"改为"向上" |
| `Arm_LiftTo(down_mm)` | `Arm_LiftTo(lift_mm)` | 参数含义由下伸距离改为抬升高度 |
| `arm pos` 仅打印电机角度 | `arm pos` 打印三种数据；`Arm_GetPos` / `Arm_GetCartesianPose` / `Arm_GetPolarPose` | 新增笛卡尔/极坐标位置回读与结构体 `ArmPos_t` / `ArmCartesianPose_t` / `ArmPolarPose_t` |

## 9. 维护约定与已知限制

- **回零**：整臂回零用 `arm init`（后台异步，三轴按 **升降 → 伸展 → 基座旋转** 顺序回零：先抬离台面、再收短臂、最后旋转扫掠最小）；回零成功后自动移动到**机械臂初始状态位**（行驶/待机位：臂收短+抬升，`ARM_POSE_INIT_*`，需实测标定）。单轴重新找零用 `arm home base|lift|extend`
  或 `arm home <电机ID>`（阻塞到该轴完成/失败，其它轴不动）。两者互斥——整臂回零进行中单轴回零会被拒绝；
  单轴回零后该轴位置基准变化、旧目标记录失效（由 `Arm_HomeAxis` 内部清掉），整臂"是否已回零"状态不变。
- 新增轴时：先把 `ARM_MOTOR_NUM` 加 1，再在 `arm_config.h` 增加该轴的 ID/回零方向/减速比，
  最后在 `arm_ctrl.c` 的 `s_axis[]` 增加一项（`s_axis[]` 是定长数组，顺序不能超限）。
- 动作目标位姿（`arm_config.h` 的 `ARM_POSE_*_INIT`）与动作内部偏移参数（`arm_action.c` 顶部）目前全部是占位值，
  集成前必须实测标定；旧"距顶部下伸 down"坐标下的占位值 `d`
  在新坐标中对应 `z = ARM_LIFT_MAX_MM - d`（如旧 1.0mm 下伸 ≈ z=114.0mm），不可直接沿用旧数值。
- 结构体数据（`static const`）不放在头文件：头文件会被多个编译单元包含，每个单元都会各生成一份副本，
  未使用的单元还会触发未引用告警；因此工位位姿采用"`arm_config.h` 给初始化宏 + `arm_action.c` 内
  `static const ArmPose_t` 绑定"的写法，新增位姿时两处各加一行即可。
- `arm preset` 是调试/标定对位命令：只把三轴移动到目标位姿并阻塞等待到位，到位后打印读回坐标
  （对比目标值与实际值），不做夹爪/物料盘动作、也不做任务到点后的业务校验；也可用 `arm pos` 读三种数据核对。
  `init|grab|pad` 走 `arm_config.h` 的固定预设位姿；`process [slot]` / `storage [slot] [layer]` 通过
  `Arm_Action_CalcPlacePose` **查逐点预设表**（`ARM_POSE_PROCESS_SLOTn_INIT` / `ARM_POSE_STORAGE_SLOTn_Lm_INIT`），
  与任务动作共用同一份数据，改配置后调试与生产同时生效；`arm presets` 可列出全部预设值。
- `arm en on|off` 只下发三轴电机使能/失能命令（`tool en <id> <0/1>` 的三轴批量版），不触发回零或运动；
  失能后电机松轴，机械臂可能因重力下坠，调试前先确保臂在安全位置或有人扶；重新使能后仍需回零才可用绝对位置。
- `arm zero` 只下发三轴"当前位置清零"命令（`tool zero <id>` 的三轴批量版），**不回零、不运动**，但会先等三轴静止
  （运动/离线时拒绝执行）；清零后当前位姿成为各轴 0 点，后续绝对目标与软限位都相对新零点，
  重新 `arm init` 回零会按限位开关重新定义零点。零点由电机固件维护，是否掉电保持以固件手册为准。
  **注意升降轴**：逻辑高度始终按"顶部回零偏移"换算（不随清零改变），所以升降轴应在**顶部机械零位**处执行
  `arm zero`（清零后 `arm pos` 的 lift 仍是 `ARM_LIFT_TOP_HOME_OFFSET_MM`，等效于顶部回零）；
  若在中间位置清零，逻辑高度会整体偏掉一个偏移量、下降/上升方向也会对不上，不要这样用。
- 抓取抬升、盘面下降由"父位姿 + 垂直偏移"派生（`arm_action.c` 的 `arm_action_offset_z`，**正=向上抬升**），
  改这些偏移时不会破坏 XY 对齐。
- 叠层放置的 z 目标只做上限限幅（超过 `ARM_LIFT_MAX_MM` 时压到最大高度=最顶部）；
  下限由运动接口按 `ARM_LIFT_MIN_MM` 的软限位检查兜底（超限时命令返回失败，故叠层参数仍须保证 z ≥ 0）。
- **视觉 yaw 误差用机械臂补偿**（车身旋转修正精度差时）：`rpi cal` 的圆环标定会给出 yaw 误差，
  任务层（`mission_config.h` 的 `MISSION_CAL_YAW_FIX_WITH_ARM=1`，默认）不再让车体为它旋转，
  而是在标定成功后调用 `Arm_Action_SetYawCorrection(误差)`，随后该工位的抓取/放置动作把误差叠加到
  **基座旋转角**上；只作用于场地固定工位（原料区/粗加工区/暂存区），物料盘随车体转动不补偿。
  补偿方向 `ARM_YAW_CORRECTION_SIGN`（实车方向反了改 `-1.0f`）、限幅 `ARM_YAW_CORRECTION_MAX_DEG`（20°）、
  开关 `ARM_YAW_CORRECTION_ENABLE` 均在 `arm_config.h`；圆盘标定(d)无角度字段，误差恒为 0。
  **可脱离任务流程用 shell 验证补偿方向/幅度**（回零后）：
  ```
  arm yaw 5                     # 模拟视觉标定给出 +5° yaw 误差(内部按 SIGN 换算并限幅)
  arm act pose storage 1 0      # 只走到暂存区第1格放置位并打印目标/实际基座角(应比 preset 多约 5°)
  arm yaw off                   # 清除补偿
  ```
  也可用 `arm preset storage 1 0`（不做补偿）与 `arm act pose storage 1 0` 对比基座角差；
  `arm act ...` 运行时若补偿生效会先打印 `[arm] yaw correction active: X deg`。
- **键盘点动**：`arm key [伸展步长mm] [旋转步长°] [升降步长mm]` 进入键盘控制
  （W/S=伸展伸长/收回，A/D=基座左转/右转，Q/E=升降/下降，Z/X=夹爪张开/闭合，^C 退出），每次按键下发一次
  "当前目标 ± 步长"的单轴绝对位置命令，速度/加速度沿用 `ARM_*_VEL/ACC`，软限位超限只提示不运动；
  Z/X 直接调 `Arm_GripperSetAngle`（等角度回读确认到位，不参与三轴目标累计），打印 `gripper open/close: OK/FAILED`。
  步长默认值与连续按键最小间隔在 `arm_config.h` 的"键盘点动(arm key)"区配置。
  注意：点动基于当前实测坐标（回零后使用；未回零时绝对坐标不可信），退出后机械臂保持当前位置。
- 升降轴回零在**顶部顶点**（`ARM_LIFT_HOME_DIR=0`，向上顶顶部限位），但对外坐标始终"从底部计"：
  `ARM_LIFT_TOP_HOME_OFFSET_MM`（默认 = `ARM_LIFT_MAX_MM`）把顶部机械零位换算回底部坐标，
  `arm pos` 的 lift / 位姿里的 `lift_mm` 仍是 0=最底部、向上增大，**原有位姿参数不用改**；
  只有实测"底部端 → 顶部回零触发点"的总行程不是 110mm 时, 才改这个偏移宏。
  若实际回零方向与 `ARM_LIFT_HOME_DIR` 不符会顶错方向，首次联调务必低速确认。
  手动在顶部对齐后也可以用 `arm zero` 代替回零：清零只改电机零点，逻辑高度仍按该偏移换算，
  所以顶部清零后 `arm pos` 的 lift 应显示 110（从底部计），之后下发 `arm pol ... 10` 会正确往下走。
- 伸展轴方向已按实车校正：`0` = 完全收回端（回零于限位开关），命令数值增大 = 实际伸出，
  且 `arm pos` 的电机角度/伸展长度在伸出时都为正。实车"命令方向"与"回读符号"都与初始假设不一致，
  由 `s_axis[]` 中伸展轴的 `linear_sign = -1`（命令）与 `read_sign = -1`（回读）分别承担；
  期望回读位置按二者换算，到位判定不受影响。
  不要改 `ARM_EXTEND_HOME_DIR`（回零仍朝完全收回端）或用 `arm zero` 去"修正"；
  换向/换电机后先用 `arm ctrl` 小行程验证方向，再用 `arm pos` 核对读数。
- 运动顺序（`Arm_MoveToAndWait` / `Arm_MovePolarToAndWait`）：先读当前抬升高度判断升降方向，
  下降动作→先基座旋转+伸展到位、再下降；上升动作→先升到目标高度、再旋转+伸展；
  等高（差在 `ARM_MOVE_LIFT_DIR_TOL_MM` 内）按下降处理。即水平动作始终放在较高高度上做，
  避免带高度斜向进入工位；两段各用传入的 `timeout_ms`，总耗时上限约 2 倍。
  抬升高度读取失败时告警并退回"先水平后升降"。异步接口 `Arm_MoveTo` / `Arm_MovePolarTo` 仍是三轴同时下发(不等待)。
  动作层每次 `arm_action_move` 都走带等待的极坐标接口，所以取放料动作自动按该规则走。
- **速度怎么调（已按动作解耦，可逐动作单独配）**：
  1) 每个对外动作的三轴速度/加速度在 `arm_config.h` → "各动作速度/加速度(解耦)" 的 `ARM_PROFILE_xxx_INIT`，
     动作开始时 `arm_action.c` 调用 `Arm_SetMotionProfile()` 选用，动作内部所有移动都用该 profile；
     **初值全部等于各轴默认速度**（行为与解耦前一致），要单独调某个动作直接改对应宏里那一项即可。
     目前覆盖：原料抓取上盘、盘→粗加工、盘→暂存、粗加工→盘、暂存→盘、回初始位、标定预备；
     调试命令（`arm pol/preset/ctrl/act pose`）用默认 profile，`arm act` 跑真实动作时各动作用各自 profile。
  2) `ARM_*_VEL`(RPM) / `ARM_*_ACC`(Emm 档位 0~255) 仍是"默认值/基准"，改它会影响仍引用它的 profile 初值；
     想把某动作彻底独立出来，就把该动作宏里的值改成具体数值。
  3) 调快顺序：先加大 ACC（加速更跟得上指令、动作更连贯），再按 10%~20% 步进加大 VEL；
     出现过冲/回弹/丢步/异响就回调小（同步带伸展轴最敏感）。
     Emm 加速度档位换算：实际加速度 ≈ `20000/(256-档位)` RPM/s（220≈555、180≈263）；**0 档 = 无加减速，禁止使用**。
  4) 回零速度单独看 `ARM_HOME_VEL_RPM`（默认 30，顶限位要慢）；到位判定 `ARM_SETTLE_STABLE_MS`
     （当前 100ms，需大于电机状态更新周期）等参数也计入动作节拍，可配合调整。
  > Emm 速度模式的"加速/减速"共用同一个档位，所以每个动作解耦的是"速度 + 加减速档位"，没有单独的减速度参数。
- **动作连贯/提速（已做）**：
  - 分步运动里"目标与当前差值在 `ARM_MOVE_SKIP_*` 容差内"的段直接跳过（不发命令、不等到位等待），
    只升降/只水平的动作（盘面下降/抬起、抓取后抬升、回初始位等）不再空等一次到位判定；
  - 夹爪张合改为可并行：`Arm_GripperStartAngle` 只下发命令、与移臂/翻相机并行，需要时再 `Arm_GripperWaitArrive` 确认
    （原料区张爪、场地取料张爪、物料盘取料张爪、标定张爪均已并行化）；
  - 每段导航前的准备把"物料盘换格"与"机械臂回初始位"并行（先 `Arm_PadSetSlotStart` 再收臂，最后确认盘到位）；
  - **物料盘归位期间车体可以原地旋转**：路线首段是原地旋转段（`MISSION_NAV_MODE_ROTATE`）时，
    该旋转提前到物料盘换格期间并行执行（`Mission_ArmPreMoveStart` → 预旋转 → `Mission_ArmPreMoveWait`，
    机械臂回初始位的收臂动作执行完才允许车体转），归位完成时旋转已基本转好、路线首段不再重复旋转；
    只对路线首段的 ROTATE 生效、不改变平移路径，预旋转失败自动回退按原路线执行；
    开关与安全注意见 `mission_config.h` 的 `MISSION_PAD_PREPOSE_ROTATE_ENABLE`；
  - 抓取后不再单独抬升一次，抬升由后续移动的"先升到安全高度再水平"一次完成。
  > 这些是动作层面的重叠，不改变各轴速度/加速度；要整体更快仍按上面的 VEL/ACC 调。
- **超限/未到位处理（不再中断任务）**：
  - 目标超出软限位（`ARM_BASE_MIN/MAX_DEG`、`ARM_EXTEND_MIN/MAX_MM`、`ARM_LIFT_MIN/MAX_MM`）时
    **自动钳到限位值继续执行**并告警（位姿层与执行器层都会钳，不会因为超限直接失败）；
    想限制某个方向的可达范围，就把对应 MIN/MAX 改成实测机械极限，之后超出的目标会自动按该值走；
  - 分步运动某段重试用尽仍未到位时，只打印"target/actual"告警并**按实际可达位置继续后续段/动作**，
    不再返回错误、不中断整轮任务（best effort）；`arm_action_move` 也只会因电机离线而失败；
  - 标定预备动作仍严格（三轴没到位就不发 RPI 标定命令），避免用错误臂位算标定偏移。
- **偶发不能一次性到达预设位置**的排查顺序（本次已做 1)~3)）：
  1) 各轴加速度不能填 0：Emm 固件加速度是"档位(0-255)"，0 = 无加减速（直接全速启停），
     同步带/丝杆冲击大、易过冲/回弹/丢步；当前速度/加速度值见 `arm_config.h` 的"各轴默认速度"；
  2) 电机心跳保护时间 = 状态轮询周期 `step_update_time` + `MOTOR_HEARTBEAT_MARGIN_MS`（现 500ms）：
     余量过小（原 100ms，即保护仅 200ms）时，系统繁忙把低优先级的状态轮询任务拖过保护时间，
     电机会被心跳保护中途停转，表现为"走一半停住、再发一次命令又能走"；
  3) 分步运动每段超时后按 `ARM_MOVE_RETRY`（默认 1 次）自动重发同一绝对目标，
     失败日志会打印目标与实际位姿（base/extend/lift），便于判断差多少；
  4) 若仍偶发：查供电电压/电流、同步带张紧与机械干涉；可开 `MOTOR_STATUS_MOTOR_FLAGS`
     轮询堵转/限位标志确认是否保护停转，或继续降速/加大 `ARM_SETTLE_STABLE_MS`。
- 物料盘格位号从 1 开始，顺序与每轮抓取顺序一致；换格前夹爪必须停在物料盘上方安全高度，
  `Arm_PadSetSlot` 内部用舵机**多圈角度回读**确认到位（连续两次在 `ARM_PAD_SETTLE_TOL_DEG` 内）
  后才允许下降取放，超时 `ARM_PAD_SETTLE_TIMEOUT_MS` 判换格失败（不再是固定延时）；
  若舵机固件不支持角度回读，会提示并回退到 `ARM_PAD_SETTLE_MS` 固定等待。
  相邻格位角度差 270°、换格约需 1s+，所以原固定 300ms 等待容易"没到位就下降"，改为回读判定。
- **物料盘与机械臂并行**：物料盘转动慢，放料/取料动作按“先 `Arm_PadSetSlotStart` 启动换格 →
  机械臂此时移到物料盘上方安全高度 → 下降取放前 `Arm_PadWaitArrive` 校验到位”的顺序执行，
  把物料盘转动时间与移臂时间重叠，缩短整段动作节拍；只有下降取放前才阻塞等待物料盘到位。
  进一步提前启动（节约时间）：
  - `Arm_Action_LoadPad` / `Arm_Action_LoadPadFromProcessing` / `Arm_Action_LoadPadFromStorage`
    在**动作一开始**就启动物料盘换格，换格与原料区抓取/场地取料并行，抓完回到盘上时盘已到位；
  - `Arm_Action_UnloadToProcessing` / `Arm_Action_UnloadToStorage` 取到料后**立即**预启动
    下一盘位（`next_pad_slot`）换格，换格与“移臂到场地、放料”并行，下一次取料时盘已到位。
  > 任务层 `mission_arm.c` 按物料顺序传 `next_pad_slot = 下一盘位(0=最后一格不预启动)`。
- 夹爪张开角度分两档，避免"抓空/撞料"：
  - **原料区抓取**（`arm_action_pick_matl`）使用标定过的大张开角 `ARM_CAL_GRIPPER_OPEN_DEG`（默认 -110°），
    流程是"**下降前先张到 -110°** → 一次移动到抓取位（先水平到位、**单次**下降；不再先降一档安全高度再降一档）"，
    避免下降途中手指开度不够撞到物料/平台；来料位置/姿态不完全确定时张开量更大，不让常规 -55 开度限制手指；
  - **物料盘取料前**（`arm_action_pad_pick`）先张到常规张开角 `ARM_GRIPPER_OPEN_DEG`（-55°）；
    物料盘放料张开、场地格位取料前张开同样使用该常规角；视觉标定也用 -110°（避免挡相机视野）。
- 视觉标定预备位姿按**目标区域**配置（`mission_config.h` 的 `MISSION_CAL_ARM_GRAB/PROCESSING/STORAGE_*`）：
  标定前 `Arm_Action_CalibrationReady` 会先把旋转/伸展两轴移动到该区域位姿并等到位，**再**升降到位
 （顺序固定为"先旋转/伸展、后升降"，不会一边旋转一边升降），并等待稳定，
  再读回校验基座角度（伸展/升降由到位判定兜底），确认到位后才翻相机；
  三个区域的预备位姿不同，**不能**再共用一个基座角度配置；占位值 0 必须实测标定后再用。
  预备位姿到位后还会把**夹爪张开到 `ARM_CAL_GRIPPER_OPEN_DEG`（默认 -110°）**，避免闭合夹爪/夹着的物料
  挡住相机视野；夹爪动作失败只告警、不阻断标定，标定结束不强制恢复夹爪角度（后续抓取动作会自己开合）。
- 相机翻转舵机已接入视觉标定链路：`Arm_Action_CalibrationReady` 在三轴预备位姿确认到位后把相机翻到
  `ARM_FLIP_CAL_ANGLE_DEG`，标定结束 `Arm_Action_CalibrationFinish` **保持该翻转角不再翻回**
  （后续抓取的颜色识别也用该视角），其余标定/动作流程直接进标定姿态，不再来回翻转。
  **恢复默认角 `ARM_FLIP_RESTORE_ANGLE_DEG` 只在机械臂伸缩校准/回零时执行**：
  整臂回零 `arm init`、伸展轴单轴回零 `arm home extend` 开始前会自动翻回默认角，
  避免翻转机构与回零/伸缩动作干涉；调试也可 `arm flip restore` 手动恢复。`ARM_FLIP_ENABLE=0` 可整体关闭。
  翻转 ID/角度/`ARM_FLIP_SETTLE_MS` 均为占位值，必须实测标定后再比赛使用；
  标定角度可直接用 `arm flip <deg>` 试转、`arm flip cal` / `arm flip restore` 验证配置角，
  角度确认后填回 `arm_config.h`（与标定链路同一个 `Arm_FlipSetAngle` 接口，行为一致）。
  另注意 `Servo_Init`（`servo_driver.c`）中硬编码的对 ID 1 的 Stop/Reset/SetOrigin 初始化命令
  是物料盘旧 ID 的遗留，若翻转舵机实际 ID 为 1 需同步核对。
- `arm pos` 的坐标由 S_CPOS 读回位置换算，其方向约定须与位置命令(0xFD)一致；
  首次联调建议用 `arm ctrl` 下发已知位置后用 `arm pos` 交叉验证各轴方向，方向不符时先查电机/驱动方向配置，
  不要用改零位的方式"修正"读数。
- `arm pos` 的笛卡尔/极坐标依赖三轴均在线且已完成回零（回零前读到的是上电/相对位置，不代表机械绝对位姿）；
  任一轴离线时对应 `*_valid` 为 false，shell 输出 `unavailable`。
- 任务码约定（沿用历史实现）：**抓取行**每 3 位是颜色码 1~6（对应 `RPI_Color_t`）；
  **放置行**每 3 位是目标格位号 1~`MISSION_MATERIAL_NUM`（`mission_arm.c` 会先校验，0/越界直接判失败）。
- 动作函数已改为返回 `bool`：任一步运动未到位/颜色识别超时都会返回 `false`，由 mission 把本轮判失败；
  原料抓取等待颜色的超时由 `ARM_GRAB_COLOR_TIMEOUT_MS` 控制（不再无限等待）。
  树莓派颜色检测现在只回传**一个颜色**（应答帧 `0xEF, color, 0xFE`，见 `Drivers/User/RPI/README.md`），
  抓取逻辑按该颜色做匹配，连续两次检测到目标颜色才认为稳定。
- `arm act` 跑的就是 mission 用的同一套 `Arm_Action_*` 完整动作（含夹爪/物料盘/颜色识别/到位等待），
  用来整段联调：
  - 动作前会先检查三轴电机是否在线：日志出现 `arm: motor IDx (name) offline` 表示电机没上电/总线没接，
    位置读不到（`arm pos` 对应轴显示 `unavailable`），不是动作逻辑问题；
  - 本次上电还没回零时会提示先 `arm init`（手动 `arm zero` 的流程可忽略该提示）。
  - `arm act grab <color> [pad_slot]`：原料区抓指定颜色 → 物料盘（颜色支持 1~6 或
    red/yellow/blue/green/black/lightblue；**依赖树莓派颜色识别在线**，否则会等到 `ARM_GRAB_COLOR_TIMEOUT_MS` 超时失败）；
  - `arm act process [pad_slot] [slot]`：物料盘取料 → 粗加工区格位（slot 1~`ARM_SLOT_NUM`）；
  - `arm act storage [pad_slot] [slot] [layer]`：物料盘取料 → 暂存区格位/叠层（layer 0/1）；
  - `arm act pick-process [slot] [pad_slot]`：粗加工区格位取料 → 物料盘（`process` 的反向）；
  - `arm act pick-storage [slot] [layer] [pad_slot]`：暂存区格位/叠层取料 → 物料盘（`storage` 的反向）；
    反向动作与放置动作共用同一份**逐点预设表**（`Arm_Action_CalcPlacePose`），取料点=放料点，
    取到料后不再单独抬升一次：直接移臂到物料盘（移动的"先升到盘安全高度再水平"一次完成抬升），
    最后按 `arm_action_pad_release` 放回物料盘；
  - 执行前需已回零（`arm init`）且 `ARM_POSE_*_INIT` / 各格位预设 `ARM_POSE_*_SLOTn_*_INIT` 已按实车**逐点**标定，
    否则会按错误位姿取放料（可用 `arm presets` 核对、`arm preset process|storage ...` 对位标定）。
- 放料完成后会先垂直抬升 `ARM_PLACE_LIFT_MM` 再移臂；夹爪开合改为**单圈角度回读确认到位**
  （到达 `ARM_GRIPPER_OPEN_DEG`/`ARM_GRIPPER_CLOSE_DEG`，或检测到"已开始动作且角度不再变化"即夹稳/被挡停），
  超时 `ARM_GRIPPER_SETTLE_TIMEOUT_MS` 判失败；不再是固定 `ARM_GRIPPER_SETTLE_MS` 延时，
  该宏仅作"舵机不支持角度回读"时的兜底等待。这三个参数与放置格位数量 `ARM_SLOT_NUM` 都在
  `arm_config.h` 的"任务动作公共参数"区。
- 驱动配置 `zdt_v5_cfg.h` 仍引用应用层 `arm_config.h`（仅取 `ARM_MOTOR_NUM`），
  这是当前工程既有的驱动→应用反向依赖，若后续统一到驱动层配置需同步修改底盘/机械臂/`eide.yml`。
