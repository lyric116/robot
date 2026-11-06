G1 29 自由度扔苹果 V10 身体控制部署包 — model_9999

这是最新完成 10000 次 PPO 更新的投掷策略，针对 250 g 苹果。
包含 ONNX（内置归一化）、准确关节映射、身体控制源码、两种 CPU 的
SDK/ONNX Runtime 依赖、离线回放自检和释放事件接口。
因时灵巧手由用户单独部署，本包不发布任何手指命令。

已完成：仿真、ONNX 导出一致性、x86_64 C++ 离线数值自检，以及
lo 回环网卡 / DDS 域 77 的启动、准备、投掷、Hold、单次释放和只读模式测试。
未完成：本包在机器人上的运行验收、ARM64 实机时间测试、因时手的
负载惯性/释放延迟校准。此包是待联调部署版本，不代表真机验收通过。
最终模型仿真向前距离约 46.17 m；不能当作真机投掷距离。
它仍是约 0.74 m 的低位出手，带噪测试最大自碰撞穿透约 2.4 mm，
未通过高位投掷和严格 2 mm 碰撞筛选。完整结果见 validation/。

一、解压与编译（在机器人开发计算机上执行）

  unzip g1_apple_throw_v10_model9999.zip
  cd g1_apple_throw_v10_model9999
  chmod +x build.sh start_controller.sh
  bash build.sh

build.sh 只本地编译并运行不连接 DDS 的自检，不安装系统服务、不控制机器人。
已附 x86_64 自检/控制程序；ARM64 机器执行 build.sh 生成本机程序。
如缺少编译依赖，Ubuntu 对应软件包是：
  build-essential cmake libyaml-cpp-dev libeigen3-dev
Unitree SDK2 和 ONNX Runtime 1.22.0 已放在 thirdparty，无需另行下载。
不需要安装训练用 PyTorch、MuJoCo 或 mjlab。Python 仅用于校验和事件示例。

二、先离线自检

  bash start_controller.sh --self-test

输出 passed=true。自检会按真实仿真轨迹比较 126 维观测、30 维输出、
29 个关节目标及力矩，并验证只触发一次释放。此命令不连接机器人。

三、按已有官方调试模式流程取得控制权

本程序不会主动释放原厂运动模式，也不会停止其他控制器。
按之前的操作流程支撑机器人、停止 locomotion 控制器，再使用原部署的
g1_debug_mode --release 工具切换模式。启动本包前应无其他 rt/lowcmd 发布者。
原工具路径（此前的机器人安装）为：
  ~/g1_precision_9999/bin/g1_debug_mode --release

本包使用 Unitree SDK2、G1 HG 消息、PR 模式 mode_pr=0、mode_machine=5。
状态操作沿用官方 Passive / FixStand / 按键启用 policy 的顺序，Throw/Hold
是为该模型新增的适配，不是可以直接替换原 Velocity ONNX 的官方原版控制器。

四、启动和投掷

先只读查看（不会发布 lowcmd）：
  bash start_controller.sh --monitor

退出只读程序后启动控制：
  bash start_controller.sh --control

默认 DDS 网卡 eth0；Wi-Fi SSH 登录不改变机器人内部 DDS 网卡。
需要其他网卡时可设置 G1_NETWORK，或传 --network 网卡名。

  1. 启动后为 Passive 阻尼状态。
  2. 支撑机器人，L2 + ↑ 进入 FixStand，用 2 秒过渡至该模型准备姿势。
  3. 准备姿势完成后让双脚落地；右手按你们独立的因时手程序握住苹果。
     初始训练假设苹果已在右手上，右手腕坐标中的握持中心约为
     [0.11, -0.045, 0] m；真实手的安装和握持偏差需要联调。
  4. 松开按键，再按 R2 + A，开始一次投掷。
  5. 约 4.1 秒后进入 Hold，保持最后一个受限关节目标，不自动归零、
     不重新开始推理，也不会自动抓握或重复投掷。
  6. 再次投掷：L2 + ↑ 回到准备姿势，重新放好苹果，再按 R2 + A。

L2 + B 返回 Passive。Ctrl+C 退出前也会短时发送 Passive 阻尼命令；
退出/阻尼不是站立保持，按原操作流程先可靠支撑机器人。
Hold 使用最后关节目标做 PD 保持，不等于持续运行站立平衡 policy。
4.1 秒来自本模型名义仿真回合终点；真实恢复和平衡仍需验收。

五、身体控制与训练的一致性

  策略 50 Hz；身体伺服发送 500 Hz。
  输入 126、输出 30；第 30 维为释放动作。
  输入中没有苹果位置、世界位置或参考动作。
  观测包含 IMU、关节、上一动作、内部时钟/释放锁存、上次限幅后的目标。
  ONNX 已内置训练归一化，禁止外部重复标准化。
  29 关节映射、默认角、增益、动作缩放均来自训练环境，见 policy/deploy.yaml。
  策略目标按同一 90% 角度范围和速度限幅；计算 PD 后按同一力矩-速度包络截断。
  Throw/Hold/FixStand 下发 tau，机内 kp=kd=0，避免重复叠加 PD。
  Passive 使用官方式 kd=3 阻尼。准备增益沿用官方 FixStand，输出力矩受限。
  实测角度/速度越过训练中的硬阈值、非有限数据或 lowstate 超时会回 Passive；
  没有旧控制器的“单步目标变化 >0.6 rad”额外保护。
  真机端没有外部定位和仿真碰撞传感器，不能在线复现仿真的自碰撞检查。

六、对接独立的因时手程序

policy 输出索引 29 第一次 >0.5 时发送一次本机 Unix datagram 事件：
  /tmp/g1_throw_release.sock
示例格式：
  {"event":"release","hand":"right","pid":1234,"trial_id":1,
   "policy_step":21,"policy_time_s":0.42,"monotonic_ns":123456789,
   "action":0.8,"release_commanded":true}

你们的手部进程在投掷开始前绑定此 socket，收到后执行自己的右手松开接口。
按 (pid, trial_id) 去重；不要用 stdout 日志的刷新时间来决定松手时刻。
事件是“请求松手”，不是物理苹果已离手的反馈。时钟字段供测延迟使用。
发送是非阻塞且每回合仅一次；没有接收端时不会阻止身体动作。
status.json 的 release_event_delivered 表示 datagram 发送成功，不表示手已打开。
release_commanded 会锁存到下次投掷，release_events.jsonl 保留事件日志。
不重复发事件，不补发已经错过的释放时间。

只查看事件、不给手发送命令的示例：
  python3 scripts/watch_release.py

不要同时运行示例接收器和正式因时手接收器，它们使用同一个 socket。
若手部程序运行在另一台计算机，需要你们提供转发桥接并测量端到端延迟；
本包的 socket 是机器人本机接口，不是网络广播。

七、文件

  policy/policy.onnx                    推理模型（含归一化）
  policy/deploy.yaml                    参数和映射
  policy/hardware_limits.json           训练时 90% 限制来源
  policy/observation_contract.json      输入字段定义
  include/throw_policy.hpp              观测/推理/限幅/力矩核心
  src/controller.cpp                   SDK 身体控制、按键状态和事件发送
  validation/golden_rollout.json        原始仿真逐步对照数据
  validation/export_parity.json        ONNX 导出误差
  validation/cpp_parity.json            C++ 回放误差（x86_64）
  validation/local_dds_test.json        回环通信状态流程与单次释放测试
  validation/simulation_evaluation.json 最终模型 64 次带噪评估
  manifest.json / SHA256SUMS.txt        来源、限制和文件校验

本包没有 SSH 密码、Wi-Fi 密码、机器人地址或训练环境虚拟环境。

参考来源与版本：
  官方部署状态流程 / FixStand 增益：
  https://github.com/unitreerobotics/unitree_rl_mjlab
  本地基线 commit: 1425b15f73bd4095f0df53709d7c389c3eb9e790
  Unitree SDK2 接口及其 BSD 3-Clause 许可证：
  https://github.com/unitreerobotics/unitree_sdk2
  本包使用本机已有 SDK 构建，原始许可证在 thirdparty/unitree_sdk2/。
  ONNX Runtime 1.22.0：
  https://github.com/microsoft/onnxruntime
  许可证和第三方声明随两种 CPU 架构的运行库一起提供。
