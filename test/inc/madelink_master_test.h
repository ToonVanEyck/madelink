#pragma once

#include "madelink_master.h"
#include "test_uart.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

typedef struct
{
    int original_rx_fd; // Original read end of the pipe;
    uart_driver_t uart;
    mdl_master_ctx_t master_ctx;
    uint8_t *node_data;
    size_t node_cnt;
} mdl_test_master_ctx_t;

void mdl_test_master_init(mdl_test_master_ctx_t *ctx);
void mdl_test_master_deinit(mdl_test_master_ctx_t *ctx);

void setup_mdl_master_prop_list_handlers(void);