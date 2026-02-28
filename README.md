# Lidar 组件

## 项目简介

Lidar 组件提供统一的激光雷达抽象层，屏蔽不同厂商、不同型号激光雷达的硬件差异，为上层应用提供一致的点云数据接口。解决多品牌激光雷达适配繁琐、接口不统一的问题。

## 功能特性

### 支持的功能

- 统一的点云数据结构（FLU 坐标系）
- 多种连接方式：UART 串口、以太网、仿真模式
- 支持的激光雷达驱动：
  - `drv_uart_ydlidar` - YDLIDAR 系列
  - `drv_uart_rplidar` - SLAMTEC RPLIDAR 系列
- 可配置的扫描参数（转速、角度范围、距离范围）
- 支持坐标变换（4x4 齐次变换矩阵）
- 回调式数据推送机制
- 多返回模式支持（strongest/last/dual）

### 暂不支持

- 以太网激光雷达驱动（接口已预留）
- 多雷达同步采集
- 点云滤波与降采样

## 快速开始

### 环境准备

- CMake >= 3.10
- C99 / C++11 编译器


### 构建编译

脱离 SDK 单独构建：

```bash
cd peripherals-lidar
mkdir build && cd build
cmake .. -DLIDAR_ENABLED_DRIVERS="drv_uart_ydlidar;drv_uart_rplidar"
make -j$(nproc)
```


### 运行示例

```bash
# 运行 UART 激光雷达测试程序
./test_lidar_uart /dev/ttyUSB0 115200 YDLIDAR
```

示例代码：

```c
#include "lidar.h"

void on_frame(struct lidar_dev *dev, const struct lidar_frame *frame, void *ctx) {
    printf("收到 %u 个点\n", frame->point_count);
}

int main() {
    struct lidar_dev *dev = lidar_alloc_uart("my_lidar", "/dev/ttyUSB0", 115200, "YDLIDAR", NULL);

    struct lidar_config cfg = {
        .rpm = 600,
        .angle_min_deg = -180.0f,
        .angle_max_deg = 180.0f,
        .range_min_m = 0.1f,
        .range_max_m = 12.0f,
    };

    lidar_init(dev, &cfg);
    lidar_set_callback(dev, on_frame, NULL);
    lidar_start(dev);

    // ... 运行一段时间 ...

    lidar_stop(dev);
    lidar_free(dev);
    return 0;
}
```

## 详细使用

> 详细 API 说明与高级用法请参考官方文档（待补充）。

## 常见问题

### Q: 串口打开失败，提示权限不足

将当前用户加入 `dialout` 组：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

### Q: 点云数据为空或点数很少

1. 检查激光雷达电源是否正常
2. 确认波特率设置与雷达实际配置一致
3. 检查 `range_min_m` / `range_max_m` 范围是否合理

### Q: 编译时找不到 ydlidar_sdk / rplidar_sdk

独立构建时，YDLIDAR/RPLIDAR 的第三方 SDK 会在 CMake 配置阶段自动拉取到 `~/.cache/thirdparty/`。若配置失败，请检查网络连接。

### Q: 报错 `Could not find components_ydlidar_sdk` 或类似

通常因 `~/.cache/thirdparty/ydlidar` 中存在**过期的旧缓存**。**解决方法：清空缓存后重新配置构建**：

```bash
rm -rf ~/.cache/thirdparty/ydlidar
# 或在容器中（如 root 用户）：
rm -rf /root/.cache/thirdparty/ydlidar

# 重新执行 cmake 与 make
cd build && rm -rf * && cmake .. -DLIDAR_ENABLED_DRIVERS="drv_uart_ydlidar;drv_uart_rplidar" && make -j$(nproc)
```

## 版本与发布

| 版本 | 日期 | 说明 |
|------|------|------|
| 0.1.0 | 2026-02-28 | 初始版本，支持 YDLIDAR、RPLIDAR |

## 贡献方式

1. Fork 本仓库
2. 创建特性分支 (`git checkout -b feature/xxx`)
3. 提交更改 (`git commit -m 'Add xxx'`)
4. 推送分支 (`git push origin feature/xxx`)
5. 创建 Pull Request


## License

本组件源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
