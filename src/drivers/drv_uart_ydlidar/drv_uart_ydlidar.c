/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * YDLidar UART driver adapter for spacemit_robotis lidar framework.
 *
 * This driver uses the vendored YDLidar-SDK C API (ydlidar_sdk.h) to:
 * - configure and start scanning
 * - pull scan frames in a worker thread
 * - convert to struct lidar_frame (point cloud) and fire dev->cb
 *
 * Enable by adding "drv_uart_ydlidar" to SROBOTIS_PERIPHERALS_LIDAR_ENABLED_DRIVERS
 * and create device with:
 *   lidar_alloc_uart("ydlidar", "/dev/ttyUSB0", 230400, "YDLIDAR", NULL);
 */

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <unistd.h>

#include "lidar_core.h"
#include "ydlidar_sdk.h"

struct ydlidar_opts {
    int lidar_type;
    int sample_rate;
    int abnormal_check_count;
    bool single_channel;
    bool intensity;
    bool motor_dtr;
    bool hb;
    /* remember which values were explicitly forced by env */
    bool forced_single_channel;
    bool forced_lidar_type;
    bool forced_sample_rate;
    bool forced_motor_dtr;
    bool forced_abnormal_check_count;
};

struct ydlidar_priv {
    char dev_path[128];
    uint32_t baud;

    YDLidar *laser;
    LaserFan scan;
    bool scan_inited;

    pthread_t th;
    bool th_started;
    volatile bool stop_req;

    struct ydlidar_opts opts;
};

static uint64_t now_realtime_ns(void)
{
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0)
        return 0;
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool env_get_bool(const char *name, bool *out)
{
    const char *v = getenv(name);
    if (!v || !*v)
        return false;

    /* accept 0/1, true/false, yes/no (case-insensitive) */
    while (isspace((unsigned char)*v))
        v++;
    if (*v == '1') {
        *out = true;
        return true;
    }
    if (*v == '0') {
        *out = false;
        return true;
    }

    if (!strcasecmp(v, "true") || !strcasecmp(v, "yes") || !strcasecmp(v, "on")) {
        *out = true;
        return true;
    }
    if (!strcasecmp(v, "false") || !strcasecmp(v, "no") || !strcasecmp(v, "off")) {
        *out = false;
        return true;
    }
    return false;
}

static void ydlidar_opts_load_env(struct ydlidar_priv *priv)
{
    /* Only override what we expose; defaults are set in ydlidar_create(). */
    bool b;
    if (env_get_bool("YDLIDAR_SINGLE_CHANNEL", &b)) {
        priv->opts.single_channel = b;
        priv->opts.forced_single_channel = true;
    }
    if (env_get_bool("YDLIDAR_INTENSITY", &b))
        priv->opts.intensity = b;
    if (env_get_bool("YDLIDAR_MOTOR_DTR", &b))
    {
        priv->opts.motor_dtr = b;
        priv->opts.forced_motor_dtr = true;
    }
    if (env_get_bool("YDLIDAR_HEARTBEAT", &b))
        priv->opts.hb = b;

    {
        const char *v = getenv("YDLIDAR_LIDAR_TYPE");
        if (v && *v) {
            priv->opts.lidar_type = (int)strtol(v, NULL, 10);
            priv->opts.forced_lidar_type = true;
        }
    }
    {
        const char *v = getenv("YDLIDAR_SAMPLE_RATE");
        if (v && *v) {
            priv->opts.sample_rate = (int)strtol(v, NULL, 10);
            priv->opts.forced_sample_rate = true;
        }
    }
    {
        const char *v = getenv("YDLIDAR_ABNORMAL_CHECK_COUNT");
        if (v && *v) {
            int n = (int)strtol(v, NULL, 10);
            if (n < 2) n = 2;
            if (n > 50) n = 50;
            priv->opts.abnormal_check_count = n;
            priv->opts.forced_abnormal_check_count = true;
        }
    }
}

static void ydlidar_apply_common_opts(struct lidar_dev *dev, struct ydlidar_priv *priv)
{
    /* string properties */
    setlidaropt(priv->laser, LidarPropSerialPort, priv->dev_path, (int)sizeof(priv->dev_path));
    {
        char empty[1] = {0};
        setlidaropt(priv->laser, LidarPropIgnoreArray, empty, (int)sizeof(empty));
    }

    /* int properties */
    {
        int baud = (int)priv->baud;
        (void)setlidaropt(priv->laser, LidarPropSerialBaudrate, &baud, (int)sizeof(baud));
    }
    {
        int lidar_type = priv->opts.lidar_type;
        (void)setlidaropt(priv->laser, LidarPropLidarType, &lidar_type, (int)sizeof(lidar_type));
    }
    {
        int dev_type = YDLIDAR_TYPE_SERIAL;
        (void)setlidaropt(priv->laser, LidarPropDeviceType, &dev_type, (int)sizeof(dev_type));
    }
    {
        int sample_rate = priv->opts.sample_rate;
        (void)setlidaropt(priv->laser, LidarPropSampleRate, &sample_rate, (int)sizeof(sample_rate));
    }
    {
        int abnormal = priv->opts.abnormal_check_count;
        (void)setlidaropt(priv->laser, LidarPropAbnormalCheckCount, &abnormal, (int)sizeof(abnormal));
    }

    /* bool properties */
    {
        bool fixed = false;
        (void)setlidaropt(priv->laser, LidarPropFixedResolution, &fixed, (int)sizeof(fixed));
    }
    {
        bool reversion = false;
        bool inverted = false;
        (void)setlidaropt(priv->laser, LidarPropReversion, &reversion, (int)sizeof(reversion));
        (void)setlidaropt(priv->laser, LidarPropInverted, &inverted, (int)sizeof(inverted));
    }
    {
        bool auto_reconnect = true;
        (void)setlidaropt(priv->laser, LidarPropAutoReconnect, &auto_reconnect, (int)sizeof(auto_reconnect));
    }
    {
        bool single = priv->opts.single_channel;
        (void)setlidaropt(priv->laser, LidarPropSingleChannel, &single, (int)sizeof(single));
    }
    {
        bool intensity = priv->opts.intensity;
        (void)setlidaropt(priv->laser, LidarPropIntenstiy, &intensity, (int)sizeof(intensity));
    }
    {
        bool motor_dtr = priv->opts.motor_dtr;
        (void)setlidaropt(priv->laser, LidarPropSupportMotorDtrCtrl, &motor_dtr, (int)sizeof(motor_dtr));
    }
    {
        bool hb = priv->opts.hb;
        (void)setlidaropt(priv->laser, LidarPropSupportHeartBeat, &hb, (int)sizeof(hb));
    }

    /* float properties */
    {
        float max_angle = dev->config.angle_max_deg;
        float min_angle = dev->config.angle_min_deg;
        (void)setlidaropt(priv->laser, LidarPropMaxAngle, &max_angle, (int)sizeof(max_angle));
        (void)setlidaropt(priv->laser, LidarPropMinAngle, &min_angle, (int)sizeof(min_angle));
    }
    {
        float max_range = dev->config.range_max_m;
        float min_range = dev->config.range_min_m;
        (void)setlidaropt(priv->laser, LidarPropMaxRange, &max_range, (int)sizeof(max_range));
        (void)setlidaropt(priv->laser, LidarPropMinRange, &min_range, (int)sizeof(min_range));
    }
    {
        /* lidar_config uses rpm; SDK wants Hz */
        float hz = 0.0f;
        if (dev->config.rpm > 0)
            hz = (float)dev->config.rpm / 60.0f;
        if (hz <= 0.0f)
            hz = 10.0f;
        (void)setlidaropt(priv->laser, LidarPropScanFrequency, &hz, (int)sizeof(hz));
    }
}

static void ydlidar_destroy_laser(struct ydlidar_priv *priv)
{
    if (!priv || !priv->laser)
        return;
    /* best-effort cleanup */
    (void)turnOff(priv->laser);
    disconnecting(priv->laser);
    lidarDestroy(&priv->laser);
    priv->laser = NULL;
}

static int ydlidar_setup_and_initialize(struct lidar_dev *dev, struct ydlidar_priv *priv)
{
    if (!dev || !priv)
        return -EINVAL;

    priv->laser = lidarCreate();
    if (!priv->laser) {
        printf("[YDLIDAR] lidarCreate failed\n");
        return -ENOMEM;
    }

    if (!priv->scan_inited) {
        LaserFanInit(&priv->scan);
        priv->scan_inited = true;
    }

    ydlidar_apply_common_opts(dev, priv);

    if (!initialize(priv->laser)) {
        printf("[YDLIDAR] initialize failed: %s\n", DescribeError(priv->laser));
        ydlidar_destroy_laser(priv);
        return -EIO;
    }

    return 0;
}

static void *ydlidar_worker(void *arg)
{
    struct lidar_dev *dev = (struct lidar_dev *)arg;
    struct ydlidar_priv *priv = dev ? (struct ydlidar_priv *)dev->priv_data : NULL;

    if (!dev || !priv)
        return NULL;

    while (dev->running && !priv->stop_req) {
        if (doProcessSimple(priv->laser, &priv->scan)) {
            struct lidar_frame *frame = &dev->buffers[dev->active_buf_idx];
            uint32_t n = priv->scan.npoints;

            if (n > frame->capacity)
                n = frame->capacity;

            frame->system_stamp_ns = now_realtime_ns();
            frame->sensor_stamp_ns = priv->scan.stamp;
            frame->point_count = n;

            for (uint32_t i = 0; i < n; i++) {
                const LaserPoint *lp = &priv->scan.points[i];
                struct lidar_point *p = &frame->points[i];
                const float r = lp->range;
                const float a = lp->angle; /* rad */

                p->x = r * cosf(a);
                p->y = r * sinf(a);
                p->z = 0.0f;
                p->intensity = clampf(lp->intensity, 0.0f, 255.0f);
                p->t_offset_ns = (uint32_t)((double)i * (double)priv->scan.config.time_increment * 1e9);
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

    return NULL;
}

static int ydlidar_init(struct lidar_dev *dev)
{
    struct ydlidar_priv *priv = dev->priv_data;

    if (!priv)
        return -EINVAL;

    /* SDK installs signal handlers; safe to call multiple times. */
    os_init();

    ydlidar_destroy_laser(priv);
    ydlidar_opts_load_env(priv);
    int r = ydlidar_setup_and_initialize(dev, priv);
    if (r < 0)
        return r;

    printf("[YDLIDAR] init ok: %s dev=%s baud=%u rpm=%d\n",
            dev->name ? dev->name : "(null)",
            priv->dev_path, priv->baud, dev->config.rpm);
    printf("[YDLIDAR] opts: lidar_type=%d sample_rate=%d single_channel=%d intensity=%d motor_dtr=%d\n",
            priv->opts.lidar_type, priv->opts.sample_rate,
            (int)priv->opts.single_channel, (int)priv->opts.intensity, (int)priv->opts.motor_dtr);
    return 0;
}

static int ydlidar_start(struct lidar_dev *dev)
{
    struct ydlidar_priv *priv = dev->priv_data;
    int rc;

    if (!priv || !priv->laser)
        return -EINVAL;

    priv->stop_req = false;

    if (!turnOn(priv->laser)) {
        printf("[YDLIDAR] turnOn failed: %s\n", DescribeError(priv->laser));

        /* Minimal retry matrix for common mis-configurations. */
        struct ydlidar_opts saved = priv->opts;
        struct {
            const char *name;
            bool try_single_toggle;
            bool try_motor_dtr_toggle;
            bool try_tof_profile;
        } tries[] = {
            { "toggle_single_channel", true,  false, false },
            { "toggle_motor_dtr",      false, true,  false },
            { "tof_profile",           false, false, true  },
        };

        for (size_t i = 0; i < sizeof(tries)/sizeof(tries[0]); i++) {
            priv->opts = saved;

            if (tries[i].try_single_toggle) {
                if (priv->opts.forced_single_channel)
                    continue;
                priv->opts.single_channel = !priv->opts.single_channel;
            }
            if (tries[i].try_motor_dtr_toggle) {
                if (priv->opts.forced_motor_dtr)
                    continue;
                priv->opts.motor_dtr = !priv->opts.motor_dtr;
            }
            if (tries[i].try_tof_profile) {
                if (priv->opts.forced_lidar_type || priv->opts.forced_sample_rate)
                    continue;
                priv->opts.lidar_type = TYPE_TOF;
                priv->opts.sample_rate = 20;
            }

            printf("[YDLIDAR] retry(%s): lidar_type=%d sample_rate=%d single_channel=%d motor_dtr=%d\n",
                    tries[i].name,
                    priv->opts.lidar_type, priv->opts.sample_rate,
                    (int)priv->opts.single_channel, (int)priv->opts.motor_dtr);

            ydlidar_destroy_laser(priv);
            if (ydlidar_setup_and_initialize(dev, priv) != 0)
                continue;
            if (turnOn(priv->laser)) {
                printf("[YDLIDAR] retry(%s) succeeded\n", tries[i].name);
                goto start_worker;
            }
            printf("[YDLIDAR] retry(%s) turnOn failed: %s\n",
                    tries[i].name, DescribeError(priv->laser));
        }

        priv->opts = saved;
        return -EIO;
    }

start_worker:
    rc = pthread_create(&priv->th, NULL, ydlidar_worker, dev);
    if (rc != 0) {
        printf("[YDLIDAR] pthread_create failed: %s\n", strerror(rc));
        (void)turnOff(priv->laser);
        return -rc;
    }
    priv->th_started = true;
    return 0;
}

static int ydlidar_stop(struct lidar_dev *dev)
{
    struct ydlidar_priv *priv = dev ? (struct ydlidar_priv *)dev->priv_data : NULL;
    if (!priv)
        return -EINVAL;

    priv->stop_req = true;

    if (priv->th_started) {
        (void)pthread_join(priv->th, NULL);
        priv->th_started = false;
    }

    if (priv->laser) {
        (void)turnOff(priv->laser);
        disconnecting(priv->laser);
    }

    return 0;
}

static int ydlidar_parse_packet(struct lidar_dev *dev,
                const uint8_t *raw_data,
                uint32_t raw_len,
                struct lidar_frame *frame)
{
    /* Not used: YDLidar-SDK handles IO internally. */
    (void)dev;
    (void)raw_data;
    (void)raw_len;
    (void)frame;
    return -ENOSYS;
}

static void ydlidar_free(struct lidar_dev *dev)
{
    if (!dev)
        return;

    (void)ydlidar_stop(dev);

    struct ydlidar_priv *priv = dev->priv_data;
    if (priv) {
        if (priv->scan_inited) {
            LaserFanDestroy(&priv->scan);
            priv->scan_inited = false;
        }
        if (priv->laser) {
            ydlidar_destroy_laser(priv);
            priv->laser = NULL;
        }
    }

    if (dev->buffers[0].points) free(dev->buffers[0].points);
    if (dev->buffers[1].points) free(dev->buffers[1].points);
    if (dev->priv_data) free(dev->priv_data);
    if (dev->name) free((void *)dev->name);
    free(dev);
}

static const struct lidar_ops ydlidar_ops = {
    .init = ydlidar_init,
    .start = ydlidar_start,
    .stop = ydlidar_stop,
    .parse_packet = ydlidar_parse_packet,
    .free = ydlidar_free,
};

static struct lidar_dev *ydlidar_create(void *args)
{
    struct lidar_args_uart *a = (struct lidar_args_uart *)args;
    struct lidar_dev *dev;
    struct ydlidar_priv *priv;

    if (!a || !a->instance || !a->dev_path)
        return NULL;

    dev = lidar_dev_alloc(a->instance, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = dev->priv_data;
    dev->ops = &ydlidar_ops;

    strncpy(priv->dev_path, a->dev_path, sizeof(priv->dev_path) - 1);
    priv->dev_path[sizeof(priv->dev_path) - 1] = '\0';
    priv->baud = a->baud;
    priv->laser = NULL;
    priv->scan_inited = false;
    priv->th_started = false;
    priv->stop_req = false;
    memset(&priv->scan, 0, sizeof(priv->scan));

    /* defaults (may be overridden by env on init/start) */
    priv->opts.lidar_type = TYPE_TRIANGLE;
    priv->opts.sample_rate = 5;
    priv->opts.abnormal_check_count = 8;
    /*
     * Default to single-channel mode.
     *
     * Many YDLidar models (e.g. S2PRO) require single_channel=true; if set wrong,
     * SDK init/turnOn may fail and then succeed after toggling. We keep env overrides
     * so other models can still force the opposite behavior.
     */
    priv->opts.single_channel = true;
    priv->opts.intensity = false;
    priv->opts.motor_dtr = true;
    priv->opts.hb = false;
    priv->opts.forced_single_channel = false;
    priv->opts.forced_lidar_type = false;
    priv->opts.forced_sample_rate = false;
    priv->opts.forced_motor_dtr = false;
    priv->opts.forced_abnormal_check_count = false;
    printf("[YDLIDAR] create ok: %s dev=%s baud=%u\n",
            dev->name ? dev->name : "(null)",
            priv->dev_path, priv->baud);
    return dev;
}

/* Register using model name "YDLIDAR" */
REGISTER_LIDAR_DRIVER("YDLIDAR", LIDAR_DRV_UART, ydlidar_create);



