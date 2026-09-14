# 天轶 2.0 二维导航准入核实

## 当前交付边界

2026-09-08，从官方 `main` 的 `01bf5f6b526a80173919f0bace64c860f0ca3fa4`
创建独立分支 `feat/tianyi-planar-navigation`。本阶段交付只读探针、SDK 核实记录
以及默认关闭的 RGB 自描述帧插件。2026-09-14 增加二维扫描/轮询里程计插件，
**不是已完成的导航 Driver，不可作为物理执行通过凭证**。
未修改 ActuCore、其他工作树、真机运行配置或机器人服务；未下发运动、取消、
心跳注册、自检或定位配置命令。

目标接口如下；RGB、二维扫描和里程计已实现 opt-in 注册；
`loco`、独立停车守护及 execution_status 尚未实现，不具备物理执行能力：

| 能力 | 工具 | 默认输出与 QoS |
|---|---|---|
| 二维雷达 | `navigation_lidar_2d` | `/ubuntu/navigation/scan`；LaserScan，Reliable/KeepLast(2)/Volatile |
| 连续局部里程计 | `navigation_odom` | `/ubuntu/navigation/odom`；Odometry，Reliable/KeepLast(5)/Volatile |
| 速度执行 | `loco` | 输入 `/ubuntu/navigation/nav2/velocity_proposal`；String，Reliable/KeepLast(1)/Volatile |
| 执行反馈 | 随 `loco` 输出，无独立运动卡片 | `/ubuntu/navigation/execution_status`；String，Reliable/KeepLast(1)/Volatile |
| RGB 帧 | `camera_rgb_frame`（默认关闭） | `/ubuntu/camera/rgb_frame`；UInt8MultiArray/PSE1，Reliable/KeepLast(1)/Volatile |

现有三维 `navigation_lidar` 和 IMU 不重命名。天轶当前只有 `nav`、
`chassis_raw` 等底盘入口，没有现成的 `loco`；后续复用底盘连接与实际速度
接口，提供同名能力，不把旧工具改名，也不新增 `navigation_motion`。

## 已核实与尚未核实

| 接口/条件 | 当前证据 | 尚缺什么 |
|---|---|---|
| 原始扫描 | 真机原生 SDK 30 帧可读；官方 ROS2 转换按微秒处理原始时间 | 实测结束时间早于开始时间；时钟域、扫描时长和处理语义仍不可信 |
| 局部里程计 | `getOdoPose()` 存在，HTTP `odopose` 实测可读；官方将其定义为上电后累计里程计位姿 | SDK 返回 `core::Pose`，没有采样时间或序号；须取得真实更新标记 |
| 速度 | `getSpeed()` 存在，返回 `MotionRequest`，含 vx/vy/omega；坐标约定 X 前、Y 左 | 该结构无时间；实际反馈而非指令值的语义、刷新机制、与里程计配对均待确认 |
| 速度执行 | SDK 有 `velocityControl()` 和 `setVelocity(vx,vy,omega)` | 默认受厂商定位质量监控；不得未经确认切换监控模式或替代安全机制 |
| 控制权/看门狗 | SDK 有 start/refresh/stopHeartBeat；现场 `teleop_dispatch`、`chassis_control` 均在运行 | 心跳不等于排他锁；须确认所有控制源仲裁、进程退出停车以及独占获取/失效反馈 |
| 雷达 TF | 尚无本机已验证安装外参 | 型号、扫描平面、`base_link → laser` 实测外参；不得填 identity |
| RGB/动态 TF | 已实现 opt-in Image/CameraInfo → PSE1；本地真实 ROS 跨域链测试通过 | 真机源时钟、实际关节状态与曝光时刻 TF；不是本轮已验收能力 |

官方 SDK `Pose` 与 `MotionRequest` 没有采样时间，是当前准入缺口。
不能将单独的系统时间查询、HTTP 接收时间或扫描时间冒充其源时间。
新合同允许显式 `poll_estimate`：请求区间中点仅作为里程计/TF 估计时间，
真实源年龄仍未知，不能借此生成停止确认。
存在 `TimedPose` 类型也不代表 `getOdoPose()` 返回带时间的位姿。
SDK 接口声明存在不等于目标固件已经正确实现；端点轮询成功不等于新反馈。

## 二维传感器本地实现（2026-09-14）

### 入口与配置

`main.py` 按 config.yaml 的 `plugins.navigation_lidar_2d`、
`plugins.navigation_odom` 注册，均默认关闭。每张卡片 start/stop/info 独立；
start 返回 starting 不代表已有数据，必须等 info.ready。stop 终止并回收本卡片
worker 及 SDK 子进程，重启重新建立 ROS publisher/TF，不停止既有业务。
进程停止只是采集停止，**不是底盘停车**。

- 每个插件在新解释器中加载 loopback DDS profile，再创建 domain 42 ROS 节点；
  不复用主进程的厂商 profile，不经 Dashboard JSON 桥传算法输入。
- 默认 topic 随实际 namespace 生成，上表 ubuntu 是示例；可配置 `topic`。
- 同步提供 `<topic>/status`，String、Reliable/KeepLast(1)/Volatile。
  info/status 包含 ready、reason、发布数、重复数、错误/超时数、请求起止/耗时、
  输出年龄；查询次数不称为真实设备帧率。
- scan: 填写 SDK reader 的绝对路径、主机/端口、经确认的 `range_min_m`、
  `range_max_m`、`angle_increment_rad`，确认 `source_clock_domain=ros_system_time`
  和 `axes_verified=true` 后启用。默认角度区间 [-π,π]，输入须为 ROS X前Y左的弧度。
  `base_to_laser` 必须包含 `translation`（米）、`rotation_xyzw`（单位四元数）和
  `source`（实测标定依据）；表示 laser 坐标到 base_link，不能用示例值上机。
- SDK reader 使用下方编译命令生成的 `navigation_sdk_probe`，由插件传入
  `--stream-scan`；该模式只轮询 getRawLaserScan，不读取或调用运动 Action。
  SDK/二进制未打入镜像；完成厂商许可确认后由操作者提供，缺失时明确不就绪。
- odom: 使用现有 `slamtec.base_url` 的 GET odopose/speed，显式选择
  `odom_time_mode=poll_estimate`；源时间默认模式拒绝启动，因为接口无源时间。
  确认 `local_odom_verified`、`actual_speed_verified` 后，填写有依据的
  `covariance_source`、六项 `pose_variance` 和 `twist_variance`（x/y/z/roll/pitch/yaw
  对角方差，必须正数）；这些配置是人工核实记录，不是自动检测或执行授权。
- 轮询默认 20 Hz；两个请求分别计时，默认 100 ms 超时丢弃。Odometry header/TF
  使用位姿请求区间中点，速度中点单独记录；不声称同步采样或实时反馈。
  请求超时/通信失败后锁存、时间倒退/跳变或位姿归零/大跳变后锁存，停止输出
  可用里程计，需排查后 stop/start 并重新定位。无法识别原点附近所有服务重启。

### 数据边界

扫描 start 微秒×1000，并按 start 去重；倒退锁存到显式重启。
不规则角度按配置增量分箱，同箱保留最近有效回波，空箱用 +Inf。
拒绝单点、非法角度/距离及不足两个有效角度箱的帧；不插值制造回波。
分箱后不假设采集顺序，time_increment=0；end 的估计语义未完成硬件验证，
scan_time=0，同时保留 start/end 和 end_before_start 诊断，不用负差值。
开始时间超龄、未来时间均拒绝，不硬编码历史时钟偏移。

传感器状态与停车状态严格分离：没有新增 `stop_confirmed=true` 回执，
没有用 getSpeed 缓存值恢复运动权限。下一步执行部分仍须明确厂商独立停车
接口、反馈更新证明、控制权仲裁与底盘断通信看门狗后实现；不提供空执行器。

### 本地复现与证据边界

从 Driver 仓库根目录执行（ROS Humble、Python，仓库根目录加入 PYTHONPATH）：

```bash
export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/x-humanoid/tianyi2.0/dds-local.xml"
TIANYI_ROS_TEST=1 python3 -m unittest discover \
  -s x-humanoid/tianyi2.0/tests -p test_navigation_sensors.py -v
python3 -m unittest discover \
  -s x-humanoid/tianyi2.0/tests -p test_navigation_probe.py -v
```

测试只启动 loopback HTTP fixture、合成 SDK reader 和独立 domain 93/94 的 ROS
收发，不连接机器人。覆盖标准消息非空、晚加入静态 TF、QoS、扫描去重、
时间/数据拒绝、轮询超时、stop/start 与进程回收。不设置 TIANYI_ROS_TEST 时
明确跳过 ROS 集成检查，不能宣称相同验收等级。

本轮结果：上述导航传感器 13 项、只读探针 8 项全部通过；另在本地
ROS Humble perception 容器执行 `test_camera_rgb_frame.py`，2 项通过（共 23 项）。
Python 语法检查、`git diff --check` 通过。第一次本机 HTTP 测试因沙箱禁止
监听端口失败，改到 `--network none` 本地容器后通过；初次 ROS 测试的配置
路径和 Humble QoS API 不匹配已修正并复跑，未跳过失败断言。

2026-09-14 已核对官方 SDK 下载 SHA-256 与下方记录一致，并在本地 ARM64
ROS Humble 容器编译新版采集器成功；实际调用缺参数/非法端口均返回 2。
编译不代表真机扫描通过。镜像只新增 Python 文件 COPY，无新增 apt/pip 依赖。
尚未验证完整 Driver 镜像、Canvas、现场扫描/里程计/TF 和运动，未提交或部署。
回滚本切片：关闭两个插件并重新启动 Driver（由现场操作者执行）；旧能力未改名。

## SDK 来源与许可边界

- 来源：[Slamtec 官方下载页](https://www.slamtec.com/cn/support)，
  ARM64 GCC 11 C++ SDK；包内 `version.txt` 为 `SDK VERSION : 5.1.1-rtm`。
- 已取得的官方归档：
  `https://bucket-download.slamtec.com/d6e4f7cc8cd7b9e8838111cf2f79c27baf316f5b/slamware_sdk_linux-aarch64-gcc11.tar.gz`。
- SHA-256：`93aa0ca27b7e32522319d7fddece7abd497cfbce89b697b935bc02f2f11918d7`。
  这是本次下载的内容摘要，不是厂商签名。下载要求官网 Referer；缺失时本次
  返回 HTTP 403、`x-error-info: EmptyReferer`，补官网来源后返回 200。
- SDK 仅置于本地临时目录核实，没有纳入仓库或 Driver 镜像。
  包内厂商头文件有版权声明，示例 Makefile 有 GPL 声明；没有复制示例源码。
  分发或打包 SDK 前仍须确认厂商授权及其依赖许可证，不能视为本仓 Apache 授权。
- [官方 API 说明](https://developer.slamtec.com/docs/slamware/cpp-sdk/5.1.1_rtm/slamware_core_platform/)
  用于解释 `getOdoPose()`，具体签名另外与实际下载的头文件交叉核对。
- 另核对官方 ARM64 ROS2 SDK：
  `https://bucket-download.slamtec.com/e9a4ac935187c21ffdd650af91ae87897ae42a5e/slamware_ros2_sdk_linux-aarch64-gcc11.tar.gz`，
  SHA-256 `3b6ffbfa3b8dcf2b21a65dcd126d1374b9854cbd6486a02d6813f72bfdadd6b4`。
  ROS wrapper 的 LICENSE 为 BSD-2-Clause；不代表附带二进制 SDK 的分发许可。
  只读解包，未复制进仓库、未在机器人启动该节点。

### 为什么不能原样运行官方 ROS2 节点

已核对 `src/slamware_ros_sdk/src/server/` 的源码，而非仅按 topic 名称判断：

- `server_params.cpp` 默认 `pub_accumulate_odometry=false`、`raw_ladar_data=false`。
- `server_workers.cpp`：里程计按配置或接口可用性选择 `getOdoPose()`，否则使用
  厂商全局 `getPose()`；Odometry 使用 `rclcpp::Clock().now()`，不是反馈采样时间。
- 同文件 raw scan 路径将原始时间乘 1000（即按微秒转纳秒），按 start 去重，
  但直接用 end-start 作时长，未拒绝实测的负时长；另按厂商扫描位姿发布
  `map → laser`。非 raw 路径使用轮询前后时间作为扫描时间。
- `slamware_ros_sdk_server.cpp` 的 cmd_vel 路径没有我们的 nav_id/TTL/停止确认
  契约，另有 `NotMonitored` 控制路径，不能拿它绕过控制权与安全验证。

因此更换订阅名称不足以满足标准导航输入：源时间、局部里程计、TF 和控制权
仍须逐项修正/确认。不能启动整套厂商节点来制造“已有 scan/odom”的假闭环。

## 五分钟只读 REST 复测

从 Driver 根目录运行；`SLAMTEC_BASE_URL` 由操作者填入现有底盘地址，不使用
运动端点。只依赖 Python 标准库，不启动 ROS、相机或厂商导航节点。

```bash
python3 x-humanoid/tianyi2.0/navigation_probe.py \
  --base-url "$SLAMTEC_BASE_URL" --samples 10
```

探针只对扫描、odopose、speed 三个固定端点 GET；拒绝重定向、含凭据 URL，
不使用环境代理；每个响应上限 2 MiB，超时默认 2 秒，最多 100 轮。
退出码 0 只代表读取完成，2 代表输入/读取错误，不是导航验收成功。
输出仅包含采样结构、数量、载荷摘要和少量位姿/速度值，不保存图像或完整地图。
请求/接收时间有独立名字，不会输出假的 source_stamp 或 stop_confirmed。
不同载荷数不当作不同源帧数；相同载荷也不能区分静止与缓存。

本次现场结果：30/30 次读取成功；10 次扫描得到 10 种不同载荷，1725–1741 点。
扫描仅含 `laser_points/pose`，点字段为 `angle/distance/valid`；odopose 仅有
六维位姿，speed 仅有 vx/vy/omega；三者均无采样时间。odopose 和零速载荷各
只有一种，**未据此认定新鲜或确认停止**。因此源帧率、源年龄均记为未知。

## SDK 原始扫描探针

`navigation_sdk_probe.cpp` 独立使用官方 SDK：只连接、读取原始扫描/局部位姿/
速度、断开。每个调用分别记录请求与接收时间，扫描保留原始起止整数时间，
不猜测单位或归一化；里程计、速度的源时间明确为 null。输出 NDJSON，扫描
逐点列为 `[angle,distance,valid]`，位姿列为 `[x,y,z,yaw,pitch,roll]`。
这些是离线核实数据，不是 ROS 算法输入；文件含环境几何，采样输出勿直接提交。

在获准的 Linux ARM64/GCC11 开发环境编译；`SDK_ROOT` 指向解压后的
`linux-aarch64-release`。本次已在本地隔离 ARM64 Linux 容器完成链接，并通过
临时只读 SSH 转发读取现场 SDK；没有向机器人复制或运行探针程序：

```bash
c++ -std=c++14 -I "$SDK_ROOT/include" \
  x-humanoid/tianyi2.0/navigation_sdk_probe.cpp -L "$SDK_ROOT/lib" \
  -Wl,--start-group -lrpos_robotplatforms_rpslamware -lrpos_framework \
  -lboost_atomic -lboost_chrono -lboost_date_time -lboost_regex -lboost_system \
  -lboost_thread -lboost_filesystem -lboost_random -lbase64 -ljsoncpp -lrlelib \
  -lcurl -lssl -lcrypto -lcares -Wl,--end-group -pthread -ldl -lrt -lz \
  -o navigation_sdk_probe
timeout 20s ./navigation_sdk_probe "$SLAMTEC_HOST" "$SLAMTEC_SDK_PORT" 10
```

外层 timeout 限制整个只读采样进程，单次 connect 超时 3 秒；其他 SDK 读取
不能假设有可配置超时。截止时退出码 124；各次读取失败记 error 并返回 2。
该程序不发运动，结束它不需要也不能调用停车或取消他人任务。

本次原生读取：30/30 raw scan、30/30 odometry、30/30 speed 成功；扫描每帧
2955–2968 点，30 个不同 start，**30/30 的 end < start**。另一组短时轮询
100 次观察到 80 个不同 start，约 10.3 个不同帧/秒，但 99 次 end < start。
速度和里程计各出现两种载荷，仍没有反馈时间/序号，不能用于确认零速。
原始扫描时间与本机时间也未证明同域；记录原值，不自动归一化或修正负时长。
临时 SSH 转发已取消，未停止既有遥操/SBUS/底盘进程。

## 可选 RGB 自描述帧

已有 `camera_head`、RGB legacy topic 和相机服务行为不变。新插件只订阅已存在的
`/ob_camera_head/color/image_raw`（Image）和 `/ob_camera_head/color/camera_info`
（CameraInfo），两者 Reliable/KeepLast(1)/Volatile，不启动/重启 Orbbec 服务。
输入发布者必须提供兼容 QoS；没有源数据则保持 `awaiting_image`，不伪造图像。

启用前由操作者在 Driver 配置中设置：

```yaml
plugins:
  camera_rgb_frame:
    enabled: true
    source_clock_domain: unverified
    image_topic: /ob_camera_head/color/image_raw
    camera_info_topic: /ob_camera_head/color/camera_info
    max_age_sec: 0.5
```

默认输出 `/<namespace>/camera/rgb_frame`，可通过 `topic` 配置；ROS 名称不允许
连字符，因此不使用此前计划中的 `rgb-frame`。旧 topic 不重命名。
工具动作是 `start/stop/info`；格式 `application/vnd.phanthy.sensor-envelope.v1`，
schema `phanthy.sensor.camera_rgb_frame.v1`，ROS 类型 UInt8MultiArray。
PSE1 为 little-endian `<4sII>` magic/JSON 长度/JPEG 长度，后接 UTF-8 JSON 与 JPEG。

- 保留上游 header 的时间和 frame，另记录 Driver 回调接收时间。默认时钟域为
  `unverified`；只有确认上游确为曝光/采集时间且与 ROS 系统时钟同域后，才设置
  `ros_system_time`。正整数 stamp 本身不证明同域。不用发布时间补缺失源时间。
- 零/无效时间发布 `stamp_ns=null`、`timing.available=false`；未知时钟同样不就绪。
  同一 start 周期内拒绝重复/倒序源时间；设备时间重置后需明确 stop/start，
  不自行将新时钟假定有效。接收超龄及已验证时钟下源超龄/未来帧会被拒绝。
- 每帧携带实际 CameraInfo 的 K/D/R/P、尺寸、frame 和内容摘要 calibration_id。
  缺失、不匹配或无效内参标为 unavailable，不填默认内参冒充标定。
  此 ID 表示标定内容版本，不是设备序列号；同一标定重连保持同 ID，变化后更新。
- **尚不提供头部动态外参**：`base_to_camera=null`。只有原始图像/内参可记录，
  不能据此声称 LiDAR 投影或全机坐标转换已可用。
- 单个编码线程只留最新待处理帧；Unix socket 发送超时 0.5 秒，新桥输出使用
  Reliable/KeepLast(1)/Volatile，旧客户端 QoS 不变。该输出需要既有 socket bridge。
  `submitted` 仅表示本地发送成功，不代表下游 DDS 已接收；测试另做真实接收验证。
- stop 丢弃待处理帧、等待编码线程退出并断开 socket；超时明确报
  `camera_frame_stop_pending`，不报停成功。保留惰性订阅以避免已有快速重订阅问题，
  最终 Driver 关闭才销毁节点；已提交传输中的帧可能在下游稍晚到达。
- 持续编码/发送错误只打印首次和每 100 次，info 保留计数。`ready` 仅表示本帧
  时间/内参输入满足配置要求，不代表动态 TF、导航或物理执行已就绪。

没有新增 apt/pip 依赖；Dockerfile 只增加此 Python 模块 COPY。新增能力默认关闭，
当前未部署；以后部署需独立授权并保留旧镜像/配置，回退时禁用此插件或恢复旧镜像。

## 验证与继续条件

```bash
python3 -m unittest discover -s x-humanoid/tianyi2.0/tests -p 'test_navigation_probe.py' -v
clang++ -std=c++14 -fsyntax-only -Wno-deprecated-declarations \
  -I "$SDK_ROOT/include" x-humanoid/tianyi2.0/navigation_sdk_probe.cpp
git diff --check
```

- REST 探针 8 项本地 HTTP/单元测试通过，包括重复载荷、未知时间字段、错误
  响应、非有限数、超时、大小限制、拒绝重定向和 CLI 退出码。
- 初次本地 HTTP 测试被沙箱禁止监听回环端口；获准后实测通过，没有放宽断言。
- SDK 探针使用本次下载的真实头文件，在 macOS Clang 上语法检查通过；
  后续本地 ARM64 链接及 SDK 真机只读采样通过，发现上述数据准入问题。
- `python3 x-humanoid/tianyi2.0/tests/test_camera_rgb_frame.py -v` 在本地 ROS Humble、
  cv2/numpy 容器（`--network none`）运行；2 项用例覆盖合成 Image→JPEG/PSE1、
  CameraInfo、缺失/非法时间与标定、行填充、Domain 0→42 真正 DDS 接收、QoS、
  stop/start、重复/倒序、latest-only 合并、超龄拒绝、bridge 断连恢复与最终清理。
  同一隔离容器复测 REST 探针 8 项通过；既有 `test_camera_lifecycle.py` 的 9 项
  回归通过。`git diff --check` 通过。没有构建/部署完整天轶 Driver 镜像。
  **未做天轶相机真机发布验收、跨传感器同步、底盘运动或完整导航验收**。

继续实施前需要厂商/现场确认：带更新标记的实际里程计和速度接口；扫描
时钟与真实雷达安装外参；现有 SBUS/遥控/原生导航的控制权仲裁和看门狗。
这些条件未满足时，不注册假就绪工具，不开放 `loco` 提案执行。获取条件后
再完成真实 producer、latest-only 执行、停止确认和连续任务测试，代码与
相关文档同提交。当前无需部署或回滚运行镜像；探针不在启动链中。
