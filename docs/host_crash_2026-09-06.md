# 2026-09-06 主机异常重启记录

用户报告电脑崩溃后，读取 Windows System 日志确认：

- 最近启动时间：2026-09-06 16:38:48（本机时间）；
- 事件 41/6008 记录非正常重启；
- 事件 1001 报告 Bug Check `0x000000d1`；
- 转储：`C:\Windows\Minidump\090626-31484-01.dmp`。

[微软对 0xD1 的说明](https://learn.microsoft.com/en-us/windows-hardware/drivers/debugger/bug-check-0xd1--driver-irql-not-less-or-equal)
表明它是驱动在不合适的中断级别访问内存。当前未分析转储调用栈，不能确定具体驱动，
不能据此判定为 NVIDIA、WSL、项目代码或内存耗尽。

恢复后检查：项目修改和此前通过的 Foxglove 感知验收记录保留；WSL 可用内存约 7.1 GiB，
swap 未使用。后续仿真验收串行执行；定位测试禁用 GUI、降低 CPU 优先级、限制算法及
BLAS 线程。没有更改 Windows 驱动、系统安全设置或 WSL 全局内存配置。
