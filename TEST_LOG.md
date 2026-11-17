# swarm_execution_agent 测试记录

## 2026-11-16

本包的单元测试位于 `test/`，跨包流程和故障验证统一归档于
[Swarm TEST_LOG](../swarm_mission_coordinator/TEST_LOG.md)，避免维护多份相互冲突的计数。

本地隔离验证覆盖累计导航证据、一次补执行、暂停取消、旧实例／旧 revision 拒绝、调用超时补偿及 outbox 恢复。真实 C++ TaskHub 接模拟导航的三场景均通过。

统一配置后删除 `allow_motion` 参数，默认使用真实 TaskHub；模拟测试显式传 `/swarm/mock` 前缀。CYVET-A／B 各部署一套 swarm，在真实定位、真实 TaskHub 与导航下各执行一个近距离点，两份累计导航结果均封存 COMPLETED，整轮 2/2 完成。两板最终 Agent 状态 finalized，没有遗留执行路线。本轮未在实机注入掉线、重启或导航失败；这些场景的结果仍仅来自隔离测试。
