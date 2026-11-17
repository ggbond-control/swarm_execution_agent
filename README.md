# Swarm 本机执行代理

节点 `swarm_execution_agent_node` 将已提交计划转换为 TaskHub YAML，执行本机启动／暂停／取消，映射累计导航证据，并持久重发结果。没有云端 Server 依赖，也不导入原执行 Agent。

默认连接真实 TaskHub（空 `task_hub_prefix`）。整套控制栈仅由 Bringup 的 `cooperation.mode: swarm` 启动，没有 `allow_motion` 二次开关；本机启动仍须满足已提交决策与执行许可。模拟测试显式指定 `/swarm/mock` 前缀。启动先取消旧路线；检测到本 ROS 域的旧控制节点或重复服务提供者时禁止控制。

启动调用前持久保存执行围栏。每个许可绑定设备会话、epoch、revision 和完整路线名，经多数派读屏障确认，以本机请求时刻计算有效期。许可过期请求暂停，迟到启动成功补偿取消，不自动恢复旧路线。

使用 `/task_hub/navigation_results` 累计 JSON 作为证据；状态索引不能充当到达记录。同位置点通过生成 YAML 时的映射对应原始任务，等待路线仍须真实执行。停止响应后核对累计快照，缺失则保留 UNKNOWN。挂载操作是否成功不决定导航到达数。

`task_hub_call_timeout` 默认 15 秒。控制调用超时后报告未知并等待旧调用排空，随后补偿取消；不会并发发起替代控制调用。永久不返回的服务不能被软件宣称为物理停止。

附带 `swarm_mock_taskhub`，仅提供隔离模拟服务、位姿和结果，不控制导航。完整用法见 [协调器 README](../swarm_mission_coordinator/README.md)，验证见 [TEST_LOG](TEST_LOG.md)。
