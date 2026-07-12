# 故障矩阵

以下矩阵是必须在真实硬件上填写的验收模板。仓库自动测试不能替代 Loiter、AUTO、GUIDED 的飞控行为验证；未填写项均阻塞飞行。

| 故障注入 | 软件预期 | Loiter 实测 | AUTO 实测 | GUIDED 实测 | 飞行门禁 |
| --- | --- | --- | --- | --- | --- |
| 停 Livox 点云 | cloud WARN 后虚拟墙，sender 随上游降级 | 待 HIL | 待 HIL | 待 HIL | 阻塞 |
| 杀 cloud | sender 超时后虚拟墙，systemd 5 秒内恢复 cloud | 待 HIL | 待 HIL | 待 HIL | 阻塞 |
| 杀 sender | supervisor 发现并重启；期间无法发送虚拟墙 | 待 HIL | 待 HIL | 待 HIL | 阻塞 |
| 拔 UART/USB | sender 关闭旧 FD 并退避重连；飞控侧处理 proximity timeout | 待 HIL | 待 HIL | 待 HIL | 阻塞 |
| 断 TF | cloud 明确 TF 降级并发送虚拟墙 | 待 HIL | 待 HIL | 待 HIL | 阻塞 |
| Jetson 掉电 | 软件完全失效；依赖飞控 failsafe、模式降级和人工接管 | 待 HIL | 待 HIL | 待 HIL | 阻塞 |

记录时必须附飞控固件版本、参数备份、时间戳、链路、进入/退出状态所需时间、恢复滞回和遥控接管结果。
