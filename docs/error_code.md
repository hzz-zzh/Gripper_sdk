## 1. 错误码详细说明

### 1.1 `GRIPPER_OK = 0`

表示接口执行成功。

---

### 1.2 `GRIPPER_ERR_INVALID_ARGUMENT = -1`

表示输入参数无效。

#### 常见场景

- 句柄为空
- 输出指针为空
- 最大速度参数非法
- 最大电流参数非法
- 设备地址超出允许范围
- 波特率不在支持列表中

#### 典型错误字符串

- `invalid handle`
- `invalid max_speed_rpm`
- `invalid max_current_amp`
- `invalid device address: expected 1~254`
- `invalid RS485 baudrate: expected one of ...`

#### 处理建议

检查接口入参是否合法，确认调用方传入的数值和指针正确。

---

### 1.3 `GRIPPER_ERR_NOT_CONNECTED = -2`

表示设备尚未连接，或当前连接已失效。

#### 常见场景

- 未调用 `gripper_connect()`
- 设备已断开
- 串口被拔出或失效

#### 典型错误字符串

- `device not connected`

#### 处理建议

先执行连接流程，必要时重新连接设备。

---

### 1.4 `GRIPPER_ERR_NOT_INITIALIZED = -3`

表示设备尚未完成初始化 / 回零，不允许执行依赖零点的操作。

#### 常见场景

- 未执行 `gripper_initialize()`
- 初始化失败后直接发位置控制命令

#### 典型错误字符串

- `gripper not initialized`

#### 处理建议

先完成初始化流程，再执行开合或位置控制。

---

### 1.5 `GRIPPER_ERR_TIMEOUT = -10`

表示通信或业务流程超时。

#### 常见场景

- 串口读超时
- 初始化过程超时
- 设备未返回应答
- 写入命令后等待时间超限

#### 典型错误字符串

- `read timeout`
- `initialize timeout`

#### 处理建议

检查：

- 串口号是否正确
- 波特率是否匹配
- 设备地址是否正确
- 接线是否正常
- 从机是否正在重启或切换配置

---

### 1.6 `GRIPPER_ERR_TRANSPORT = -11`

表示底层传输层错误，通常发生在串口打开、读写阶段。

#### 常见场景

- 串口打开失败
- 串口未准备好
- 底层写失败
- 底层读失败

#### 典型错误字符串

- `failed to open transport`
- `failed to open rs485 port`
- `transport not ready`
- `transport write failed`
- `transport read failed`

#### 处理建议

检查：

- 设备文件是否存在，如 `/dev/ttyACM0`
- 当前用户是否有串口访问权限
- 串口是否被其他程序占用

---

### 1.7 `GRIPPER_ERR_PROTOCOL = -12`

表示收到了格式不符合协议要求的数据帧，或载荷解析失败。

#### 常见场景

- CRC 校验失败
- 数据包长度不对
- 返回帧结构异常
- 协议版本不一致

#### 典型错误字符串

- `invalid version payload length`
- `invalid clear-fault payload length`
- `invalid realtime payload length`
- `skipped malformed frame: crc mismatch`
- `skipped malformed frame: frame too short`
- `skipped malformed frame: frame size mismatch`
- `skipped frame with invalid payload length`

#### 处理建议

检查：

- 主从协议版本是否一致
- 波特率是否正确
- 总线是否存在干扰
- 是否读到了其他设备发出的异常数据

---

### 1.8 `GRIPPER_ERR_BAD_RESPONSE = -13`

表示收到了应答，但该应答不是当前请求所期待的内容。

#### 常见场景

- 包序号不匹配
- 命令码不匹配
- 设备地址不匹配
- 缓冲区中残留旧应答

#### 典型错误字符串

- `skipped unexpected response sequence`
- `skipped unexpected response command`
- `skipped unexpected response device`
- `skipped non-slave header byte`

#### 处理建议

检查当前总线环境是否干净，确认：

- 总线上不存在地址冲突
- 不同从机不会同时应答
- 请求与应答能一一对应

---

### 1.9 `GRIPPER_ERR_OUT_OF_RANGE = -20`

表示目标值超出设备允许范围。

#### 常见场景

- 目标 `opening` 超出机械结构有效范围
- 百分比或位置映射结果越界

#### 典型错误字符串

- `target opening_mm is outside the valid geometry range`

#### 处理建议

检查目标开口、位置、百分比是否落在设备允许的有效区间内。

---

### 1.10 `GRIPPER_ERR_INVALID_CONFIG = -21`

表示初始化配置或控制配置本身不合法。

#### 常见场景

- 初始化方向不是 `+1` 或 `-1`
- 搜索速度小于等于 `0`
- 轮询周期小于等于 `0`
- 超时时间小于等于 `0`
- 连续检测次数小于等于 `0`
- 回退距离为负数

#### 典型错误字符串

- `invalid initialize config: search_direction must be +1 or -1`
- `invalid initialize config: search_speed_mm_s must be > 0`
- `invalid initialize config: poll_interval_ms must be > 0`
- `invalid initialize config: timeout_ms must be > 0`
- `invalid initialize config: detect_consecutive_samples must be > 0`
- `invalid initialize config: backoff_after_zero_mm must be >= 0`

#### 处理建议

检查初始化配置结构体中每一项参数是否处于合理范围。

---

### 1.11 `GRIPPER_ERR_INVALID_STATE = -22`

表示当前设备状态不允许执行该操作。

#### 常见场景

- 在不合适的运行状态下发起某个动作
- 当前设备处于错误状态或流程状态不匹配

#### 处理建议

先读取设备状态，确认电机状态、运行状态、故障状态满足执行条件。

---

### 1.12 `GRIPPER_ERR_DEVICE_FAULT = -30`

表示设备发生了明确的故障。

#### 常见场景

- 初始化过程中出现故障
- 回退过程中出现故障
- 实时状态中的 `fault_code` 非 `0`

#### 典型错误字符串

- `fault occurred during initialize: ...`
- `fault occurred during homing backoff: ...`

#### 处理建议

读取状态中的 `fault_code`，结合 `gripper_fault_code_to_string()` 进一步判断故障原因。

---

### 1.13 `GRIPPER_ERR_OPERATION_FAILED = -31`

表示操作未能成功完成，但不属于前面更具体的错误分类。

#### 常见场景

- 某些业务流程失败，但没有归入超时、协议错、参数错、设备故障等类型

#### 处理建议

结合 `gripper_get_last_error()` 查看详细原因。

---

### 1.14 `GRIPPER_ERR_COMM_CONFIG_APPLIED = 1`

这是一个特殊返回值。

它表示：通信参数很可能已经写入成功，但当前连接上下文失效，因此未能按旧通信参数收到应答。

#### 常见场景

调用：

- `gripper_set_communication_config()`

并修改了：

- 设备地址
- RS485 波特率

设备可能在参数写入后立即切换到新地址或新波特率，导致 SDK 按旧通信参数等待应答时发生超时。

#### 典型说明字符串

- `communication config may have been applied; reconnect with new address/baudrate`

#### 处理建议

这不是普通失败。应按以下方式处理：

1. 断开当前连接
2. 使用新的设备地址和新的波特率重新连接
3. 重新读取设备参数进行验证