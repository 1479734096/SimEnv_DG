# 第三方源码说明

本目录保留 KISS-ICP 上游 C++ 源码分发，不迁入其 Python 包、ROS wrapper、评测 notebook 或 CI 配置。上游 C++ 目录自带 `metrics` 静态库源码；父包以 `EXCLUDE_FROM_ALL` 引入，只链接 `kiss_icp::pipeline`，因此 `metrics` 不进入默认构建、节点链接、安装或正式启动链。

- 上游：KISS-ICP
- 固定提交：`460c6f2cc3616c122c42006898036df9c20b636f`
- 上游描述：`v0.3.0`
- 许可证：见 `kiss_icp/cpp/kiss_icp/LICENSE` 及其 `3rdparty/*/LICENSE`
- 迁移原则：第三方文件保持原样；参赛方构建边界和适配只位于父包 `CMakeLists.txt`、`include/` 与 `src/`
