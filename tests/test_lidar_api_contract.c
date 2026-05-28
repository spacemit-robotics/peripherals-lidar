/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../src/lidar_core.h"

struct fake_priv {
    char dev_path[128];
    uint32_t baud;
    int init_calls;
    int start_calls;
    int stop_calls;
};

struct callback_state {
    int calls;
    bool failed;
};

static void report_expectation_failure(const char *expr, const char *file,
        int line, const char *message)
{
    fprintf(stderr, "[FAIL] %s:%d: %s", file, line, expr);
    if (message && *message)
        fprintf(stderr, " (%s)", message);
    fputc('\n', stderr);
}

#define EXPECT_TRUE(expr, message) \
    do { \
        if (!(expr)) { \
            report_expectation_failure(#expr, __FILE__, __LINE__, message); \
            return 1; \
        } \
    } while (0)

static void callback_expect(struct callback_state *state, bool expr,
        const char *message)
{
    if (!expr) {
        report_expectation_failure("callback", __FILE__, __LINE__, message);
        state->failed = true;
    }
}

static int float_close(float a, float b)
{
    return fabsf(a - b) < 1e-5f;
}

static int fake_init(struct lidar_dev *dev)
{
    struct fake_priv *priv;

    if (!dev || !dev->priv_data)
        return -EINVAL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->init_calls++;

    if (strcmp(priv->dev_path, "/dev/fail-init") == 0)
        return -EIO;

    return 0;
}

static int fake_start(struct lidar_dev *dev)
{
    struct fake_priv *priv;
    struct lidar_frame *frame;

    if (!dev || !dev->priv_data)
        return -EINVAL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->start_calls++;

    frame = &dev->buffers[dev->active_buf_idx];
    if (!frame->points || frame->capacity < 2)
        return -ENOMEM;

    frame->system_stamp_ns = 123456789ull;
    frame->sensor_stamp_ns = 987654321ull;
    frame->point_count = 2;

    frame->points[0] = (struct lidar_point) {
        .x = 1.0f,
        .y = 0.0f,
        .z = 0.0f,
        .intensity = 42.0f,
        .t_offset_ns = 100,
        .ring = 0,
        .flags = 0,
    };
    frame->points[1] = (struct lidar_point) {
        .x = 0.0f,
        .y = 2.0f,
        .z = 0.0f,
        .intensity = 7.0f,
        .t_offset_ns = 200,
        .ring = 0,
        .flags = 0,
    };

    dev->active_buf_idx ^= 1;

    if (dev->cb)
        dev->cb(dev, frame, dev->cb_ctx);

    return 0;
}

static int fake_stop(struct lidar_dev *dev)
{
    struct fake_priv *priv;

    if (!dev || !dev->priv_data)
        return -EINVAL;

    priv = (struct fake_priv *)dev->priv_data;
    priv->stop_calls++;
    return 0;
}

static int fake_parse_packet(struct lidar_dev *dev, const uint8_t *raw_data,
        uint32_t raw_len, struct lidar_frame *frame)
{
    (void)dev;
    (void)raw_data;
    (void)raw_len;
    (void)frame;
    return -ENOSYS;
}

static void fake_free(struct lidar_dev *dev)
{
    if (!dev)
        return;

    if (dev->buffers[0].points)
        free(dev->buffers[0].points);
    if (dev->buffers[1].points)
        free(dev->buffers[1].points);
    if (dev->priv_data)
        free(dev->priv_data);
    if (dev->name)
        free((void *)dev->name);
    free(dev);
}

static const struct lidar_ops fake_ops = {
    .init = fake_init,
    .start = fake_start,
    .stop = fake_stop,
    .parse_packet = fake_parse_packet,
    .free = fake_free,
};

static struct lidar_dev *fake_uart_create(void *args)
{
    struct lidar_args_uart *uart_args = (struct lidar_args_uart *)args;
    struct lidar_dev *dev;
    struct fake_priv *priv;

    if (!uart_args || !uart_args->instance || !uart_args->dev_path)
        return NULL;

    dev = lidar_dev_alloc(uart_args->instance, sizeof(*priv));
    if (!dev)
        return NULL;

    priv = (struct fake_priv *)dev->priv_data;
    dev->ops = &fake_ops;

    strncpy(priv->dev_path, uart_args->dev_path, sizeof(priv->dev_path) - 1);
    priv->dev_path[sizeof(priv->dev_path) - 1] = '\0';
    priv->baud = uart_args->baud;
    return dev;
}

REGISTER_LIDAR_DRIVER("FAKEUART", LIDAR_DRV_UART, fake_uart_create);

static void frame_callback(struct lidar_dev *dev,
        const struct lidar_frame *frame, void *ctx)
{
    struct callback_state *state = (struct callback_state *)ctx;

    callback_expect(state, dev != NULL, "device should not be NULL");
    callback_expect(state, frame != NULL, "frame should not be NULL");
    if (!frame)
        return;

    state->calls++;
    callback_expect(state, frame->system_stamp_ns == 123456789ull,
            "system timestamp mismatch");
    callback_expect(state, frame->sensor_stamp_ns == 987654321ull,
            "sensor timestamp mismatch");
    callback_expect(state, frame->point_count == 2,
            "point count should be 2");
    callback_expect(state, frame->points != NULL,
            "points buffer should not be NULL");
    if (!frame->points || frame->point_count < 2)
        return;

    callback_expect(state, float_close(frame->points[0].x, 1.0f),
            "first point x mismatch");
    callback_expect(state, float_close(frame->points[0].y, 0.0f),
            "first point y mismatch");
    callback_expect(state, float_close(frame->points[1].x, 0.0f),
            "second point x mismatch");
    callback_expect(state, float_close(frame->points[1].y, 2.0f),
            "second point y mismatch");
}

static int test_default_config(void)
{
    struct lidar_dev *dev = lidar_alloc_uart("fake-default", "/dev/fake0",
            115200, "FAKEUART", NULL);

    EXPECT_TRUE(dev != NULL, "fake driver allocation should succeed");
    EXPECT_TRUE(!lidar_is_connected(dev), "new device should not be connected");
    EXPECT_TRUE(lidar_init(dev, NULL) == 0,
            "init with default config should succeed");
    EXPECT_TRUE(lidar_get_rpm(dev) == 600, "default rpm should be 600");
    EXPECT_TRUE(float_close(dev->config.angle_min_deg, -180.0f),
            "default min angle mismatch");
    EXPECT_TRUE(float_close(dev->config.angle_max_deg, 180.0f),
            "default max angle mismatch");
    EXPECT_TRUE(float_close(dev->config.range_min_m, 0.1f),
            "default min range mismatch");
    EXPECT_TRUE(float_close(dev->config.range_max_m, 100.0f),
            "default max range mismatch");

    lidar_free(dev);
    printf("[PASS] default-config\n");
    return 0;
}

static int test_functional_lifecycle(void)
{
    struct lidar_dev *dev;
    struct fake_priv *priv;
    struct callback_state state = {0};
    struct lidar_config config = {
        .rpm = 720,
        .angle_min_deg = -90.0f,
        .angle_max_deg = 90.0f,
        .range_min_m = 0.2f,
        .range_max_m = 12.5f,
        .return_mode = 1,
        .enable_transform = true,
        .transform_matrix = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 1.0f, 0.0f,
            0.1f, 0.2f, 0.3f, 1.0f,
        },
    };

    dev = lidar_alloc_uart("fake-functional", "/dev/fake1", 230400,
            "FAKEUART", NULL);
    EXPECT_TRUE(dev != NULL, "functional fake device allocation should succeed");

    lidar_set_callback(dev, frame_callback, &state);
    EXPECT_TRUE(lidar_init(dev, &config) == 0,
            "init with custom config should succeed");
    EXPECT_TRUE(lidar_get_rpm(dev) == 720, "custom rpm should be observable");
    EXPECT_TRUE(float_close(dev->config.range_max_m, 12.5f),
            "custom range max mismatch");
    EXPECT_TRUE(dev->config.enable_transform,
            "transform flag should be preserved");

    priv = (struct fake_priv *)dev->priv_data;
    EXPECT_TRUE(priv != NULL, "private state should exist");
    EXPECT_TRUE(strcmp(priv->dev_path, "/dev/fake1") == 0,
            "fake driver path mismatch");
    EXPECT_TRUE(priv->baud == 230400, "fake driver baud mismatch");

    EXPECT_TRUE(lidar_start(dev) == 0, "start should succeed");
    EXPECT_TRUE(lidar_is_connected(dev), "device should be connected after start");
    EXPECT_TRUE(state.calls == 1, "callback should fire once per start");
    EXPECT_TRUE(!state.failed, "callback validation should succeed");

    EXPECT_TRUE(lidar_stop(dev) == 0, "first stop should succeed");
    EXPECT_TRUE(!lidar_is_connected(dev), "device should disconnect after stop");

    EXPECT_TRUE(lidar_start(dev) == 0, "second start should succeed");
    EXPECT_TRUE(state.calls == 2, "callback should fire again on restart");
    EXPECT_TRUE(lidar_stop(dev) == 0, "second stop should succeed");
    EXPECT_TRUE(priv->init_calls == 1, "init should run once");
    EXPECT_TRUE(priv->start_calls == 2, "start should run twice");
    EXPECT_TRUE(priv->stop_calls == 2, "stop should run twice");

    lidar_free(dev);
    printf("[PASS] functional-lifecycle\n");
    return 0;
}

static int run_functional_tests(void)
{
    if (test_default_config() != 0)
        return 1;
    if (test_functional_lifecycle() != 0)
        return 1;

    printf("ALL TESTS PASSED: functional\n");
    return 0;
}

static int test_invalid_inputs(void)
{
    EXPECT_TRUE(lidar_alloc_uart(NULL, "/dev/fake0", 115200,
                "FAKEUART", NULL) == NULL,
            "NULL name should be rejected");
    EXPECT_TRUE(lidar_alloc_uart("bad", NULL, 115200,
                "FAKEUART", NULL) == NULL,
            "NULL path should be rejected");
    EXPECT_TRUE(lidar_alloc_uart("bad", "/dev/fake0", 115200,
                "UNKNOWN", NULL) == NULL,
            "unknown model should be rejected");
    EXPECT_TRUE(lidar_alloc_ethernet("bad", "127.0.0.1", 2368,
                "FAKEUART", NULL) == NULL,
            "UART driver should not be allocatable as ethernet");
    EXPECT_TRUE(lidar_init(NULL, NULL) == -EINVAL,
            "NULL init should return -EINVAL");
    EXPECT_TRUE(lidar_start(NULL) == -EINVAL,
            "NULL start should return -EINVAL");
    EXPECT_TRUE(lidar_stop(NULL) == -EINVAL,
            "NULL stop should return -EINVAL");
    EXPECT_TRUE(!lidar_is_connected(NULL),
            "NULL device should not appear connected");
    EXPECT_TRUE(lidar_get_rpm(NULL) == -EINVAL,
            "NULL get_rpm should return -EINVAL");

    printf("[PASS] invalid-inputs\n");
    return 0;
}

static int test_missing_sim_ops(void)
{
    struct lidar_dev *dev = lidar_alloc_sim("fake-sim", NULL);

    EXPECT_TRUE(dev != NULL, "sim device allocation should succeed");
    EXPECT_TRUE(lidar_init(dev, NULL) == -EINVAL,
            "sim init should fail without ops");
    EXPECT_TRUE(lidar_start(dev) == -EINVAL,
            "sim start should fail without ops");
    EXPECT_TRUE(lidar_stop(dev) == -EINVAL,
            "sim stop should fail without ops");

    lidar_free(dev);
    printf("[PASS] missing-sim-ops\n");
    return 0;
}

static int test_driver_init_failure(void)
{
    struct lidar_dev *dev = lidar_alloc_uart("fake-fail-init",
            "/dev/fail-init", 115200, "FAKEUART", NULL);

    EXPECT_TRUE(dev != NULL, "device allocation before failing init should succeed");
    EXPECT_TRUE(lidar_init(dev, NULL) == -EIO,
            "driver init failure should propagate -EIO");
    EXPECT_TRUE(!lidar_is_connected(dev),
            "failed init should not mark device connected");

    lidar_free(dev);
    printf("[PASS] driver-init-failure\n");
    return 0;
}

static int run_error_tests(void)
{
    if (test_invalid_inputs() != 0)
        return 1;
    if (test_missing_sim_ops() != 0)
        return 1;
    if (test_driver_init_failure() != 0)
        return 1;

    printf("ALL TESTS PASSED: error-paths\n");
    return 0;
}

static void usage(const char *prog)
{
    fprintf(stderr, "Usage: %s [functional|errors|all]\n", prog);
}

int main(int argc, char **argv)
{
    const char *mode = "all";

    if (argc > 2) {
        usage(argv[0]);
        return 2;
    }

    if (argc == 2)
        mode = argv[1];

    if (strcmp(mode, "functional") == 0)
        return run_functional_tests();
    if (strcmp(mode, "errors") == 0)
        return run_error_tests();
    if (strcmp(mode, "all") == 0) {
        if (run_functional_tests() != 0)
            return 1;
        return run_error_tests();
    }

    usage(argv[0]);
    return 2;
}
