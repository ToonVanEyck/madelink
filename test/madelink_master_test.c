#include "madelink_master_test.h"
#include "test_properties.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static bool node_cnt_update(void *userdata, uint16_t node_cnt);
static bool node_exists_and_must_be_written(void *userdata, uint16_t node_idx, uint8_t property, bool *must_be_written);
static bool master_set_handler(void *userdata, uint16_t node_idx, uint8_t *buf, size_t *size);
static bool master_get_handler(void *userdata, uint16_t node_idx, uint8_t *buf, size_t *size);
static void master_node_error_set_handler(void *userdata, uint16_t node_idx, mdl_node_err_t error,
                                          mdl_node_state_t state);

void mdl_test_master_init(mdl_test_master_ctx_t *ctx)
{
    int pipefd[2];
    if (pipe(pipefd) == -1)
    {
        perror("pipe");
        exit(EXIT_FAILURE);
    }

    ctx->uart.rx_fd = -1; // This will be set when connecting masters
    ctx->uart.tx_fd = pipefd[1];
    ctx->original_rx_fd = pipefd[0];
    ctx->node_data = NULL;
    ctx->node_cnt = 0;

    ctx->uart.print_debug = "MASTER";

    mdl_master_uart_cb_cfg_t uart_cb = {
        .read = (uart_read_cb_t)uart_read,
        .write = (uart_write_cb_t)uart_write,
        .read_timeout_set = (uart_read_timeout_set_cb_t)uart_read_timeout_set,
        .flush_rx_buff = (uart_flush_rx_buff_cb_t)uart_flush_rx_buff,
        .wait_tx_done = (uart_wait_tx_done_cb_t)uart_wait_tx_done,
    };

    uart_delay_xth_tx(&ctx->uart, 0, 0); // No delay by default

    mdl_master_cb_cfg_t master_cb = {
        .node_cnt_update = (mdl_master_node_cnt_update_cb_t)node_cnt_update,
        .node_exists_and_must_be_written =
            (mdl_master_node_exists_and_must_be_written_cb_t)node_exists_and_must_be_written,
        .node_error_set = (mdl_master_node_error_set_cb_t)master_node_error_set_handler,
    };

    setup_mdl_master_prop_list_handlers();
    mdl_prop_t *master_properties = calloc(PROPERTY_CNT, sizeof(mdl_prop_t));
    memcpy(master_properties, mdl_prop_list, PROPERTY_CNT * sizeof(mdl_prop_t));

    mdl_master_init(&ctx->master_ctx, &uart_cb, &ctx->uart, &master_cb, master_properties, PROPERTY_CNT, ctx);

    return;
}

void mdl_test_master_deinit(mdl_test_master_ctx_t *ctx)
{
    free(ctx->node_data);
    free(ctx->master_ctx.prop_list);
    return;
}

void setup_mdl_master_prop_list_handlers(void)
{
    mdl_prop_list[PROP_STATIC_RW].handler.get = master_get_handler;
    mdl_prop_list[PROP_STATIC_RW].handler.set = master_set_handler;

    mdl_prop_list[PROP_STATIC_RO].handler.get = NULL;
    mdl_prop_list[PROP_STATIC_RO].handler.set = master_set_handler;

    mdl_prop_list[PROP_STATIC_WO].handler.get = master_get_handler;
    mdl_prop_list[PROP_STATIC_WO].handler.set = NULL;

    mdl_prop_list[PROP_DYNAMIC_RW].handler.get = master_get_handler;
    mdl_prop_list[PROP_DYNAMIC_RW].handler.set = master_set_handler;

    mdl_prop_list[PROP_DYNAMIC_RO].handler.get = NULL;
    mdl_prop_list[PROP_DYNAMIC_RO].handler.set = master_set_handler;

    mdl_prop_list[PROP_DYNAMIC_WO].handler.get = master_get_handler;
    mdl_prop_list[PROP_DYNAMIC_WO].handler.set = NULL;
}

static bool node_cnt_update(void *userdata, uint16_t node_cnt)
{
    mdl_test_master_ctx_t *ctx = (mdl_test_master_ctx_t *)userdata;
    printf("Node count updated to %d (%p)\n", node_cnt, ctx);

    ctx->node_cnt = node_cnt;
    ctx->node_data = realloc(ctx->node_data, node_cnt * TEST_PROP_SIZE);
    if (!ctx->node_data)
    {
        perror("realloc");
        return false;
    }

    return true;
}

static bool node_exists_and_must_be_written(void *userdata, uint16_t node_idx, uint8_t property, bool *must_be_written)
{
    mdl_test_master_ctx_t *ctx = (mdl_test_master_ctx_t *)userdata;

    *must_be_written = node_idx % 2 == 0; // Write to even nodes only
    bool exists = node_idx < ctx->node_cnt;
    // printf("Node %d exists: %d, must be written: %d (%p)\n", node_idx, exists, *must_be_written, ctx);
    return exists;
}

static bool master_set_handler(void *userdata, uint16_t node_idx, uint8_t *buf, size_t *size)
{
    mdl_test_master_ctx_t *ctx = (mdl_test_master_ctx_t *)userdata;
    memcpy(ctx->node_data + node_idx * TEST_PROP_SIZE, buf, *size);
    printf("Dummy set handler called for node %d with size %ld and user data %p:\n", node_idx, *size, userdata);
    for (uint16_t i = 0; i < *size; i++)
    {
        printf("  Data[%d]: %02X\n", i, buf[i]);
    }
    return true;
}

static bool master_get_handler(void *userdata, uint16_t node_idx, uint8_t *buf, size_t *size)
{
    printf("Dummy get handler called for node %d with size %ld and user data %p\n", node_idx, *size, userdata);
    mdl_test_master_ctx_t *ctx = (mdl_test_master_ctx_t *)userdata;
    *size = TEST_PROP_SIZE;
    memcpy(buf, ctx->node_data + node_idx * TEST_PROP_SIZE, *size);
    return true;
}

static void master_node_error_set_handler(void *userdata, uint16_t node_idx, mdl_node_err_t error, mdl_node_state_t state)
{
    printf("Node %d: \"%s\" Error occurred in state \"%s\"\n", node_idx, mdl_node_error_to_str(error),
           mdl_node_state_to_str(state));
}