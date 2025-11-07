#pragma once

#include "madelink_master_test.h"
#include "madelink_node.h"
#include "test_properties.h"
#include "test_uart.h"

#include <pthread.h>
#include <stdbool.h>

typedef struct
{
    size_t id;
    mdl_node_ctx_t node_ctx;
    uart_driver_t uart;
    int original_rx_fd; /* This fd must be passed to the node before this one. */
    uint8_t node_data[TEST_PROP_SIZE];
} mdl_test_node_ctx_t;

typedef struct
{
    size_t node_cnt;
    mdl_test_node_ctx_t *node_list;
    pthread_t thread;
    bool running;
} mdl_test_node_group_ctx_t;

bool mdl_test_node_init(mdl_test_node_group_ctx_t *node_test_grp, size_t node_cnt, mdl_test_master_ctx_t *master);

bool mdl_test_node_deinit(mdl_test_node_group_ctx_t *node_test_grp);

void setup_mdl_node_prop_list_handlers(void);