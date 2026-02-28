/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../include/lidar.h"

static volatile bool running = true;
static uint32_t frame_count = 0;

static void signal_handler(int sig)
{
    (void)sig;
    running = false;
}

static void lidar_frame_callback(struct lidar_dev *dev,
        const struct lidar_frame *frame, void *ctx)
{
    uint32_t i;
    float min_x = 0.0f, max_x = 0.0f;
    float min_y = 0.0f, max_y = 0.0f;
    float min_z = 0.0f, max_z = 0.0f;
    bool first = true;

    (void)dev;
    (void)ctx;

    frame_count++;

    /* Calculate bounding box */
    for (i = 0; i < frame->point_count; i++) {
        const struct lidar_point *p = &frame->points[i];

        if (first) {
            min_x = max_x = p->x;
            min_y = max_y = p->y;
            min_z = max_z = p->z;
            first = false;
        } else {
            if (p->x < min_x)
                min_x = p->x;
            if (p->x > max_x)
                max_x = p->x;
            if (p->y < min_y)
                min_y = p->y;
            if (p->y > max_y)
                max_y = p->y;
            if (p->z < min_z)
                min_z = p->z;
            if (p->z > max_z)
                max_z = p->z;
        }
    }

    printf("[Frame %u] Points: %u, BBox: [%.2f, %.2f] x [%.2f, %.2f] x [%.2f, %.2f]\n",
            frame_count, frame->point_count,
            min_x, max_x, min_y, max_y, min_z, max_z);
}

static void usage(const char *prog)
{
    printf("Usage:\n");
    printf("  %s [MODEL [DEV_PATH [BAUD]]]\n", prog);
    printf("\n");
    printf("Examples:\n");
    printf("  %s VLP16 /dev/ttyUSB0 115200\n", prog);
    printf("  %s YDLIDAR /dev/ttyUSB0 230400\n", prog);
    printf("\n");
    printf("Notes:\n");
    printf("  - MODEL must match the registered driver name (e.g. VLP16, YDLIDAR).\n");
}

int main(int argc, char **argv)
{
    struct lidar_dev *lidar;
    struct lidar_config config;
    int ret;
    const char *model = "YDLIDAR";
    const char *dev_path = "/dev/ttyUSB0";
    uint32_t baud = 115200;
    const char *instance = "lidar_uart";

    printf("=== LIDAR UART Test ===\n\n");

    if (argc > 1 && (!strcmp(argv[1], "-h") || !strcmp(argv[1], "--help"))) {
        usage(argv[0]);
        return 0;
    }
    if (argc > 1 && argv[1] && *argv[1]) model = argv[1];
    if (argc > 2 && argv[2] && *argv[2]) dev_path = argv[2];
    if (argc > 3 && argv[3] && *argv[3]) baud = (uint32_t)strtoul(argv[3], NULL, 10);
    if (argc > 4) {
        fprintf(stderr, "Too many arguments.\n\n");
        usage(argv[0]);
        return -1;
    }

    /* Setup signal handler for graceful shutdown */
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    /* 1. Create UART LIDAR device */
    printf("[1] Creating UART LIDAR device...\n");
    printf("   MODEL=%s DEV=%s BAUD=%u\n", model, dev_path, baud);
    lidar = lidar_alloc_uart(instance, dev_path, baud, model, NULL);
    if (!lidar) {
        fprintf(stderr, "Failed to create LIDAR device\n");
        return -1;
    }
    printf("   LIDAR device created\n\n");

    /* 2. Configure LIDAR */
    printf("[2] Configuring LIDAR...\n");
    config = (struct lidar_config){
        .rpm = 600,
        .angle_min_deg = -180.0f,
        .angle_max_deg = 180.0f,
        .range_min_m = 0.1f,
        .range_max_m = 100.0f,
        .return_mode = 0,   /* strongest return */
        .enable_transform = false,
    };

    ret = lidar_init(lidar, &config);
    if (ret < 0) {
        fprintf(stderr, "Failed to init LIDAR: %d\n", ret);
        goto cleanup;
    }
    printf("   LIDAR configured: %d RPM, FOV: [%.1f, %.1f] deg, Range: [%.1f, %.1f] m\n\n",
            config.rpm, config.angle_min_deg, config.angle_max_deg,
            config.range_min_m, config.range_max_m);

    /* 3. Register callback */
    printf("[3] Registering frame callback...\n");
    lidar_set_callback(lidar, lidar_frame_callback, NULL);
    printf("   Callback registered\n\n");

    /* 4. Start LIDAR */
    printf("[4] Starting LIDAR...\n");
    ret = lidar_start(lidar);
    if (ret < 0) {
        fprintf(stderr, "Failed to start LIDAR: %d\n", ret);
        goto cleanup;
    }
    printf("   LIDAR started\n\n");

    /* 5. Check connection status */
    printf("[5] Checking connection status...\n");
    if (lidar_is_connected(lidar)) {
        printf("   LIDAR is connected\n");
    } else {
        printf("   LIDAR is not connected\n");
    }
    printf("   Current RPM: %d\n\n", lidar_get_rpm(lidar));

    /* 6. Main loop - wait for frames (demo: run for 3 seconds) */
    printf("[6] Waiting for frames (demo: 3 seconds)...\n\n");
    for (int sec = 0; running && sec < 10; sec++) {
        sleep(1);
        if (frame_count > 0 && frame_count % 10 == 0) {
            printf("[Status] Received %u frames\n", frame_count);
        }
    }

    printf("\n[7] Stopping LIDAR...\n");
    ret = lidar_stop(lidar);
    if (ret < 0) {
        fprintf(stderr, "Failed to stop LIDAR: %d\n", ret);
    } else {
        printf("   LIDAR stopped\n");
    }

    printf("\n=== Test Summary ===\n");
    printf("Total frames received: %u\n", frame_count);
    printf("Test completed\n");

cleanup:
    lidar_free(lidar);
    return (ret < 0) ? -1 : 0;
}
