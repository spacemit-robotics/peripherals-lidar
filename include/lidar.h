/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LIDAR_H
#define LIDAR_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * struct lidar_point - single point (32 bytes, SIMD aligned)
 * Coordinate system: FLU (Front-Left-Up)
 */
struct lidar_point {
    float x;               /* meters */
    float y;               /* meters */
    float z;               /* meters, 0 for 2D lidar */
    float intensity;       /* 0.0 ~ 255.0 */
    uint32_t t_offset_ns;  /* time offset from frame start (ns) */
    uint16_t ring;         /* beam id (0~127), 0 for 2D lidar */
    uint16_t flags;        /* 0=normal, 1=dual return second */
    uint64_t _padding;
};

/*
 * struct lidar_frame - point cloud frame
 */
struct lidar_frame {
    uint64_t system_stamp_ns;  /* host receive time */
    uint64_t sensor_stamp_ns;  /* sensor hardware time */
    uint32_t point_count;
    uint32_t capacity;
    struct lidar_point *points;
};

/*
 * struct lidar_config - runtime configuration
 */
struct lidar_config {
    int   rpm;              /* target rotation speed */
    float angle_min_deg;    /* FOV min angle (degrees) */
    float angle_max_deg;    /* FOV max angle (degrees) */
    float range_min_m;      /* minimum range (meters) */
    float range_max_m;      /* maximum range (meters) */
    int   return_mode;      /* 0=strongest, 1=last, 2=dual */
    bool  enable_transform;
    float transform_matrix[16];  /* 4x4 homogeneous matrix */
};

/* opaque handle */
struct lidar_dev;

/* error codes */
#define LIDAR_OK            0
#define LIDAR_ERR_ALLOC    -1
#define LIDAR_ERR_CONNECT  -2
#define LIDAR_ERR_TIMEOUT  -3
#define LIDAR_ERR_CONFIG   -4

typedef void (*lidar_callback_t)(struct lidar_dev *dev,
        const struct lidar_frame *frame, void *ctx);

/* --- factory functions --- */

struct lidar_dev *lidar_alloc_ethernet(const char *name, const char *ip,
        uint16_t port, const char *model, void *ex_args);
struct lidar_dev *lidar_alloc_uart(const char *name, const char *dev_path,
        uint32_t baud, const char *model, void *ex_args);
struct lidar_dev *lidar_alloc_sim(const char *name, void *ex_args);

/* --- lifecycle --- */

int lidar_init(struct lidar_dev *dev, const struct lidar_config *config);
void lidar_set_callback(struct lidar_dev *dev, lidar_callback_t cb, void *ctx);
int lidar_start(struct lidar_dev *dev);
int lidar_stop(struct lidar_dev *dev);
void lidar_free(struct lidar_dev *dev);

/* --- utilities --- */

bool lidar_is_connected(struct lidar_dev *dev);
int lidar_get_rpm(struct lidar_dev *dev);

#ifdef __cplusplus
}
#endif

#endif  /* LIDAR_H */
