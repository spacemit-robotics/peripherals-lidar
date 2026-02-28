/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lidar_core.h"
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <stdio.h>

int lidar_init(struct lidar_dev *dev, const struct lidar_config *config)
{
    if (!dev || !dev->ops || !dev->ops->init)
        return -EINVAL;

    if (config) {
        dev->config = *config;
    } else {
        /* default config */
        memset(&dev->config, 0, sizeof(struct lidar_config));
        dev->config.rpm = 600;
        dev->config.angle_min_deg = -180.0f;
        dev->config.angle_max_deg = 180.0f;
        dev->config.range_min_m = 0.1f;
        dev->config.range_max_m = 100.0f;
        dev->config.return_mode = 0;
        dev->config.enable_transform = false;
    }

    return dev->ops->init(dev);
}

void lidar_set_callback(struct lidar_dev *dev, lidar_callback_t cb, void *ctx)
{
    if (dev) {
        dev->cb = cb;
        dev->cb_ctx = ctx;
    }
}

int lidar_start(struct lidar_dev *dev)
{
    if (!dev || !dev->ops || !dev->ops->start)
        return -EINVAL;

    dev->running = true;
    return dev->ops->start(dev);
}

int lidar_stop(struct lidar_dev *dev)
{
    if (!dev || !dev->ops || !dev->ops->stop)
        return -EINVAL;

    dev->running = false;
    return dev->ops->stop(dev);
}

void lidar_free(struct lidar_dev *dev)
{
    if (!dev)
        return;

    if (dev->ops && dev->ops->free) {
        dev->ops->free(dev);
        return;
    }

    /* fallback */
    if (dev->buffers[0].points) free(dev->buffers[0].points);
    if (dev->buffers[1].points) free(dev->buffers[1].points);
    if (dev->priv_data) free(dev->priv_data);
    if (dev->name) free((void *)dev->name);
    free(dev);
}

bool lidar_is_connected(struct lidar_dev *dev)
{
    if (!dev)
        return false;
    return dev->running;
}

int lidar_get_rpm(struct lidar_dev *dev)
{
    if (!dev)
        return -EINVAL;
    return dev->config.rpm;
}

struct lidar_dev *lidar_dev_alloc(const char *name, size_t priv_size)
{
    struct lidar_dev *dev;
    void *priv = NULL;
    char *name_copy = NULL;

    dev = calloc(1, sizeof(*dev));
    if (!dev)
        return NULL;

    if (priv_size) {
        priv = calloc(1, priv_size);
        if (!priv) {
            free(dev);
            return NULL;
        }
        dev->priv_data = priv;
    }

    if (name) {
        size_t n = strlen(name);
        name_copy = calloc(1, n + 1);
        if (!name_copy) {
            free(priv);
            free(dev);
            return NULL;
        }
        memcpy(name_copy, name, n);
        name_copy[n] = '\0';
        dev->name = name_copy;
    }

    /* initialize frame buffers */
    dev->buffers[0].capacity = 10000;
    dev->buffers[0].points = calloc(dev->buffers[0].capacity, sizeof(struct lidar_point));
    dev->buffers[1].capacity = 10000;
    dev->buffers[1].points = calloc(dev->buffers[1].capacity, sizeof(struct lidar_point));
    dev->active_buf_idx = 0;
    dev->running = false;

    return dev;
}

/* --- driver registry (minimal, motor-like) --- */

static struct driver_info *g_driver_list = NULL;

void lidar_driver_register(struct driver_info *info)
{
    if (!info)
        return;
    info->next = g_driver_list;
    g_driver_list = info;
}

static struct driver_info *find_driver(const char *name, enum lidar_driver_type type)
{
    struct driver_info *curr = g_driver_list;
    while (curr) {
        if (curr->name && name && strcmp(curr->name, name) == 0) {
            if (curr->type == type)
                return curr;
            printf("[LIDAR] driver '%s' type mismatch (expected %d got %d)\n",
                    name, (int)type, (int)curr->type);
            return NULL;
        }
        curr = curr->next;
    }
    printf("[LIDAR] driver '%s' not found\n", name ? name : "(null)");
    return NULL;
}

static int split_driver_instance(const char *name,
        char *driver, size_t driver_sz,
        const char **instance)
{
    const char *sep;
    size_t len;

    if (!name || !driver || !driver_sz || !instance)
        return -EINVAL;

    sep = strchr(name, ':');
    if (!sep)
        return 0;

    len = (size_t)(sep - name);
    if (len == 0 || len + 1 > driver_sz || !*(sep + 1))
        return -EINVAL;

    memcpy(driver, name, len);
    driver[len] = '\0';
    *instance = sep + 1;
    return 1;
}

/* --- factory functions (public API) --- */

struct lidar_dev *lidar_alloc_uart(const char *name, const char *dev_path,
        uint32_t baud, const char *model, void *ex_args)
{
    struct driver_info *drv;
    struct lidar_args_uart args;
    char driver[64];
    const char *instance = NULL;
    int r;

    if (!name || !dev_path)
        return NULL;

    /* Prefer model as driver name (backward compatible) */
    if (model && *model) {
        strncpy(driver, model, sizeof(driver) - 1);
        driver[sizeof(driver) - 1] = '\0';
        instance = name;
    } else {
        r = split_driver_instance(name, driver, sizeof(driver), &instance);
        if (r < 0)
            return NULL;
        if (r == 0) {
            strncpy(driver, name, sizeof(driver) - 1);
            driver[sizeof(driver) - 1] = '\0';
            instance = name;
        }
    }

    drv = find_driver(driver, LIDAR_DRV_UART);
    if (!drv || !drv->factory)
        return NULL;

    args.instance = instance;
    args.dev_path = dev_path;
    args.baud = baud;
    args.ex_args = ex_args;
    return drv->factory(&args);
}

struct lidar_dev *lidar_alloc_ethernet(const char *name, const char *ip,
        uint16_t port, const char *model, void *ex_args)
{
    struct driver_info *drv;
    struct lidar_args_ethernet args;
    char driver[64];
    const char *instance = NULL;
    int r;

    if (!name || !ip)
        return NULL;

    if (model && *model) {
        strncpy(driver, model, sizeof(driver) - 1);
        driver[sizeof(driver) - 1] = '\0';
        instance = name;
    } else {
        r = split_driver_instance(name, driver, sizeof(driver), &instance);
        if (r < 0)
            return NULL;
        if (r == 0) {
            strncpy(driver, name, sizeof(driver) - 1);
            driver[sizeof(driver) - 1] = '\0';
            instance = name;
        }
    }

    drv = find_driver(driver, LIDAR_DRV_ETHERNET);
    if (!drv || !drv->factory)
        return NULL;

    args.instance = instance;
    args.ip = ip;
    args.port = port;
    args.ex_args = ex_args;
    return drv->factory(&args);
}

struct lidar_dev *lidar_alloc_sim(const char *name, void *ex_args)
{
    struct lidar_dev *dev;
    if (!name)
        return NULL;
    (void)ex_args;
    dev = lidar_dev_alloc(name, 0);
    if (!dev)
        return NULL;
    /* sim driver not implemented; leave ops NULL */
    return dev;
}
