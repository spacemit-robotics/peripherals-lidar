/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * RPLidar UART driver adapter for spacemit_robotis lidar framework.
 *
 * This driver uses the Slamtec LIDAR SDK (sl_lidar_driver.h) to:
 * - configure and start scanning
 * - pull scan frames in a worker thread
 * - convert to struct lidar_frame (point cloud) and fire dev->cb
 *
 * Enable by adding "drv_uart_rplidar" to
 * SROBOTIS_PERIPHERALS_LIDAR_ENABLED_DRIVERS and create device with:
 *   lidar_alloc_uart("rplidar", "/dev/ttyUSB0", 460800, "RPLIDAR", NULL);
 */

#include <pthread.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include "lidar_core.h"
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

struct rplidar_priv {
    char dev_path[128];
    uint32_t baud;

    sl::IChannel *channel;
    sl::ILidarDriver *driver;

    pthread_t th;
    bool th_started;
    volatile bool stop_req;
};

static uint64_t now_realtime_ns(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
        return 0;
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull +
            static_cast<uint64_t>(ts.tv_nsec);
}

static float clampf(float v, float lo, float hi) {
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

static void *rplidar_worker(void *arg) {
    struct lidar_dev *dev = static_cast<struct lidar_dev *>(arg);
    struct rplidar_priv *priv =
        dev ? static_cast<struct rplidar_priv *>(dev->priv_data) : nullptr;

    if (!dev || !priv || !priv->driver)
        return nullptr;

    sl_lidar_response_measurement_node_hq_t nodes[8192];

    while (dev->running && !priv->stop_req) {
        size_t count = sizeof(nodes) / sizeof(nodes[0]);
        sl_result res = priv->driver->grabScanDataHq(nodes, count);

        if (SL_IS_OK(res)) {
            /* Sort by angle */
            priv->driver->ascendScanData(nodes, count);

            struct lidar_frame *frame = &dev->buffers[dev->active_buf_idx];
            uint32_t n = static_cast<uint32_t>(count);

            if (n > frame->capacity)
                n = frame->capacity;

            frame->system_stamp_ns = now_realtime_ns();
            frame->sensor_stamp_ns = 0;  /* SDK doesn't provide timestamp */
            frame->point_count = n;

            for (uint32_t i = 0; i < n; i++) {
                const sl_lidar_response_measurement_node_hq_t *node = &nodes[i];
                struct lidar_point *p = &frame->points[i];

                /* angle in degrees (Q8.8 format) -> radians */
                float angle_deg =
                    static_cast<float>(node->angle_z_q14) * 90.0f / 16384.0f;
                float angle_rad =
                    angle_deg * static_cast<float>(M_PI) / 180.0f;

                /* distance in mm -> meters */
                float range_m =
                    static_cast<float>(node->dist_mm_q2) / 4.0f / 1000.0f;

                p->x = range_m * std::cos(angle_rad);
                p->y = range_m * std::sin(angle_rad);
                p->z = 0.0f;
                p->intensity =
                    clampf(static_cast<float>(node->quality >> 2), 0.0f, 255.0f);
                p->t_offset_ns = 0;
                p->ring = 0;
                p->flags = 0;
                p->_padding = 0;
            }

            /* swap buffer index for next frame */
            dev->active_buf_idx ^= 1;

            if (dev->cb) {
                dev->cb(dev, frame, dev->cb_ctx);
            }
        } else {
            /* Avoid busy spin on errors */
            usleep(10 * 1000);
        }
    }

    return nullptr;
}

static int rplidar_init(struct lidar_dev *dev) {
    struct rplidar_priv *priv =
        static_cast<struct rplidar_priv *>(dev->priv_data);

    if (!priv)
        return -EINVAL;

    /* Create serial channel */
    sl::Result<sl::IChannel *> channelResult =
        sl::createSerialPortChannel(priv->dev_path, priv->baud);
    if (!channelResult) {
        printf("[RPLIDAR] Failed to create serial channel\n");
        return -EIO;
    }
    priv->channel = *channelResult;

    /* Create driver */
    sl::Result<sl::ILidarDriver *> driverResult = sl::createLidarDriver();
    if (!driverResult) {
        printf("[RPLIDAR] Failed to create lidar driver\n");
        delete priv->channel;
        priv->channel = nullptr;
        return -ENOMEM;
    }
    priv->driver = *driverResult;

    /* Connect */
    sl_result res = priv->driver->connect(priv->channel);
    if (!SL_IS_OK(res)) {
        printf("[RPLIDAR] Failed to connect: 0x%08x\n", res);
        delete priv->driver;
        delete priv->channel;
        priv->driver = nullptr;
        priv->channel = nullptr;
        return -EIO;
    }

    /* Get device info */
    sl_lidar_response_device_info_t devinfo;
    res = priv->driver->getDeviceInfo(devinfo);
    if (SL_IS_OK(res)) {
        printf("[RPLIDAR] Model: %d, Firmware: %d.%02d, Hardware: %d\n",
                devinfo.model, devinfo.firmware_version >> 8,
                devinfo.firmware_version & 0xff, devinfo.hardware_version);
        printf("[RPLIDAR] Serial: ");
        for (int i = 0; i < 16; i++) {
            printf("%02X", devinfo.serialnum[i]);
        }
        printf("\n");
    }

    /* Check health */
    sl_lidar_response_device_health_t health;
    res = priv->driver->getHealth(health);
    if (SL_IS_OK(res)) {
        if (health.status == SL_LIDAR_STATUS_ERROR) {
            printf("[RPLIDAR] Health status: Error! Code: %d\n",
                    health.error_code);
            priv->driver->reset();
        } else if (health.status == SL_LIDAR_STATUS_WARNING) {
            printf("[RPLIDAR] Health status: Warning! Code: %d\n",
                    health.error_code);
        } else {
            printf("[RPLIDAR] Health status: OK\n");
        }
    }

    printf("[RPLIDAR] init ok: %s dev=%s baud=%u\n",
            dev->name ? dev->name : "(null)", priv->dev_path, priv->baud);
    return 0;
}

static int rplidar_start(struct lidar_dev *dev) {
    struct rplidar_priv *priv =
        static_cast<struct rplidar_priv *>(dev->priv_data);
    int rc;

    if (!priv || !priv->driver)
        return -EINVAL;

    priv->stop_req = false;

    /* Start motor and scanning */
    priv->driver->setMotorSpeed();
    sl_result res = priv->driver->startScan(false, true);
    if (!SL_IS_OK(res)) {
        printf("[RPLIDAR] startScan failed: 0x%08x\n", res);
        return -EIO;
    }

    rc = pthread_create(&priv->th, NULL, rplidar_worker, dev);
    if (rc != 0) {
        printf("[RPLIDAR] pthread_create failed: %s\n", strerror(rc));
        priv->driver->stop();
        return -rc;
    }
    priv->th_started = true;

    printf("[RPLIDAR] started scanning\n");
    return 0;
}

static int rplidar_stop(struct lidar_dev *dev) {
    struct rplidar_priv *priv =
        dev ? static_cast<struct rplidar_priv *>(dev->priv_data) : nullptr;
    if (!priv)
        return -EINVAL;

    priv->stop_req = true;

    if (priv->th_started) {
        (void)pthread_join(priv->th, NULL);
        priv->th_started = false;
    }

    if (priv->driver) {
        priv->driver->stop();
        priv->driver->setMotorSpeed(0);
    }

    printf("[RPLIDAR] stopped\n");
    return 0;
}

static int rplidar_parse_packet(struct lidar_dev *dev, const uint8_t *raw_data,
                                uint32_t raw_len, struct lidar_frame *frame) {
    /* Not used: SDK handles IO internally. */
    (void)dev;
    (void)raw_data;
    (void)raw_len;
    (void)frame;
    return -ENOSYS;
}

static void rplidar_free(struct lidar_dev *dev) {
    if (!dev)
        return;

    (void)rplidar_stop(dev);

    struct rplidar_priv *priv =
        static_cast<struct rplidar_priv *>(dev->priv_data);
    if (priv) {
        if (priv->driver) {
            priv->driver->disconnect();
            delete priv->driver;
            priv->driver = nullptr;
        }
        if (priv->channel) {
            delete priv->channel;
            priv->channel = nullptr;
        }
    }

    if (dev->buffers[0].points)
        free(dev->buffers[0].points);
    if (dev->buffers[1].points)
        free(dev->buffers[1].points);
    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free(const_cast<char *>(dev->name));
    free(dev);
}

static const struct lidar_ops rplidar_ops = {
    .init = rplidar_init,
    .start = rplidar_start,
    .stop = rplidar_stop,
    .parse_packet = rplidar_parse_packet,
    .free = rplidar_free,
};

static struct lidar_dev *rplidar_create(void *args) {
    struct lidar_args_uart *a = static_cast<struct lidar_args_uart *>(args);
    struct lidar_dev *dev;
    struct rplidar_priv *priv;

    if (!a || !a->instance || !a->dev_path)
        return nullptr;

    dev = lidar_dev_alloc(a->instance, sizeof(*priv));
    if (!dev)
        return nullptr;

    priv = static_cast<struct rplidar_priv *>(dev->priv_data);
    dev->ops = &rplidar_ops;

    strncpy(priv->dev_path, a->dev_path, sizeof(priv->dev_path) - 1);
    priv->dev_path[sizeof(priv->dev_path) - 1] = '\0';
    priv->baud = a->baud;
    priv->channel = nullptr;
    priv->driver = nullptr;
    priv->th_started = false;
    priv->stop_req = false;

    printf("[RPLIDAR] create ok: %s dev=%s baud=%u\n",
            dev->name ? dev->name : "(null)", priv->dev_path, priv->baud);
    return dev;
}

/* Register using model name "RPLIDAR" */
REGISTER_LIDAR_DRIVER("RPLIDAR", LIDAR_DRV_UART, rplidar_create);
