# Lidar Component

## Introduction

The Lidar component provides a unified LiDAR abstraction layer that hides hardware differences between various manufacturers and models, offering a consistent point cloud data interface for upper-layer applications. It solves the problem of tedious multi-brand LiDAR adaptation and inconsistent interfaces.

## Features

### Supported Features

- Unified point cloud data structure (FLU coordinate system)
- Multiple connection methods: UART serial, Ethernet, simulation mode
- Supported LiDAR drivers:
  - `drv_uart_ydlidar` - YDLIDAR series
  - `drv_uart_rplidar` - SLAMTEC RPLIDAR series
- Configurable scan parameters (RPM, angle range, distance range)
- Coordinate transformation support (4x4 homogeneous transformation matrix)
- Callback-based data push mechanism
- Multiple return mode support (strongest/last/dual)

### Not Yet Supported

- Ethernet LiDAR drivers (interface reserved)
- Multi-LiDAR synchronized acquisition
- Point cloud filtering and downsampling

## Quick Start

### Prerequisites

- CMake >= 3.10
- C99 / C++11 compiler


### Build

Standalone build (without SDK):

```bash
cd peripherals-lidar
mkdir build && cd build
cmake .. -DLIDAR_ENABLED_DRIVERS="drv_uart_ydlidar;drv_uart_rplidar"
make -j$(nproc)
```


### Run Example

```bash
# Run UART LiDAR test program
./test_lidar_uart /dev/ttyUSB0 115200 YDLIDAR
```

Example code:

```c
#include "lidar.h"

void on_frame(struct lidar_dev *dev, const struct lidar_frame *frame, void *ctx) {
    printf("Received %u points\n", frame->point_count);
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

    // ... run for a while ...

    lidar_stop(dev);
    lidar_free(dev);
    return 0;
}
```

## Detailed Usage

> For detailed API documentation and advanced usage, please refer to the official documentation (to be added).

## FAQ

### Q: Failed to open serial port due to permission denied

Add current user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
# Re-login to take effect
```

### Q: Point cloud data is empty or has very few points

1. Check if the LiDAR power supply is normal
2. Confirm the baud rate matches the actual LiDAR configuration
3. Check if `range_min_m` / `range_max_m` range is reasonable

### Q: Cannot find ydlidar_sdk / rplidar_sdk during compilation

In standalone build, YDLIDAR/RPLIDAR third-party SDKs are auto-fetched to `~/.cache/thirdparty/` during CMake configure. If configure fails, check your network connection.

### Q: `Could not find components_ydlidar_sdk` or similar

Usually caused by **stale cache** in `~/.cache/thirdparty/ydlidar`. **Fix: clear cache and reconfigure**:

```bash
rm -rf ~/.cache/thirdparty/ydlidar
# In container (e.g. root user):
rm -rf /root/.cache/thirdparty/ydlidar

# Re-run cmake and make
cd build && rm -rf * && cmake .. -DLIDAR_ENABLED_DRIVERS="drv_uart_ydlidar;drv_uart_rplidar" && make -j$(nproc)
```

## Version & Release

| Version | Date | Description |
|---------|------|-------------|
| 1.0.0 | 2026-02 | Initial release, supports YDLIDAR, RPLIDAR |

## Contributing

1. Fork this repository
2. Create a feature branch (`git checkout -b feature/xxx`)
3. Commit your changes (`git commit -m 'Add xxx'`)
4. Push to the branch (`git push origin feature/xxx`)
5. Create a Pull Request


## License

The source files in this component are declared as Apache-2.0 in their headers. The `LICENSE` file in this directory shall prevail.
