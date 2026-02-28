/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LIDAR_CORE_H
#define LIDAR_CORE_H

/*
 * Private header for Lidar component (motor-like minimal style).
 */

#include "../include/lidar.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 1. 参数适配包：用于将 alloc 参数打包成 void*（同时携带 instance 名称） */
struct lidar_args_uart {
    const char *instance;
    const char *dev_path;
    uint32_t baud;
    void *ex_args;
};

struct lidar_args_ethernet {
    const char *instance;
    const char *ip;
    uint16_t port;
    void *ex_args;
};

/* 2. 驱动类型枚举 */
enum lidar_driver_type {
    LIDAR_DRV_ETHERNET = 0,
    LIDAR_DRV_UART,
    LIDAR_DRV_SIM,
};

/* 3. 虚函数表（驱动实现） */
struct lidar_ops {
    int (*init)(struct lidar_dev *dev);
    int (*start)(struct lidar_dev *dev);
    int (*stop)(struct lidar_dev *dev);
    int (*parse_packet)(struct lidar_dev *dev,
                const uint8_t *raw_data,
                uint32_t raw_len,
                struct lidar_frame *frame);
    void (*free)(struct lidar_dev *dev);
};

/* 4. 设备对象（私有实现） */
struct lidar_dev {
    const char *name; /* instance name */
    struct lidar_config config;
    const struct lidar_ops *ops;
    void *priv_data;
    lidar_callback_t cb;
    void *cb_ctx;
    struct lidar_frame buffers[2];
    int active_buf_idx;
    bool running;
};

/* 5. 通用工厂函数类型 */
typedef struct lidar_dev *(*lidar_factory_t)(void *args);

/* 6. 注册节点结构 */
struct driver_info {
    const char *name;              /* driver name */
    enum lidar_driver_type type;   /* bus type */
    lidar_factory_t factory;
    struct driver_info *next;
};

void lidar_driver_register(struct driver_info *info);

#define REGISTER_LIDAR_DRIVER(_name, _type, _factory) \
    static struct driver_info __drv_info_##_factory = { \
        .name = _name, \
        .type = _type, \
        .factory = _factory, \
        .next = 0 \
    }; \
    __attribute__((constructor)) \
    static void __auto_reg_##_factory(void) { \
        lidar_driver_register(&__drv_info_##_factory); \
    }

/* Internal helpers */
struct lidar_dev *lidar_dev_alloc(const char *instance, size_t priv_size);

#ifdef __cplusplus
}
#endif

#endif  /* LIDAR_CORE_H */

