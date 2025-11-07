#include "madelink_master.h"

#include <assert.h>
#include <string.h>

//======================================================================================================================
//                                                   MACROS a DEFINES
//======================================================================================================================

#define RX_BYTES_TIMEOUT(_byte_cnt) (1000 + (_byte_cnt) * 1) /**< Timeout in ms for receiving bytes. */

#ifndef TAG
#define TAG "madelink_master"
#endif

#if !defined(MDL_LOGE) || !defined(MDL_LOGW) || !defined(MDL_LOGI) || !defined(MDL_LOGD)
#include <stdio.h>
#define MDL_LOGE(tag, format, ...) printf("E: [%s]: " format "\n", tag, ##__VA_ARGS__)
#define MDL_LOGW(tag, format, ...) printf("W: [%s]: " format "\n", tag, ##__VA_ARGS__)
#define MDL_LOGI(tag, format, ...) printf("I: [%s]: " format "\n", tag, ##__VA_ARGS__)
#define MDL_LOGD(tag, format, ...) printf("D: [%s]: " format "\n", tag, ##__VA_ARGS__)
#endif

/**
 * Macro which can be used to check the condition. If the condition is not 'true', it prints the message
 * and returns with the supplied 'err_code'.
 */
#ifndef MDL_RETURN_ON_FALSE
#define MDL_RETURN_ON_FALSE(a, err_code, log_tag, format, ...)                                       \
    do                                                                                               \
    {                                                                                                \
        if (!(a))                                                                                    \
        {                                                                                            \
            MDL_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(, ) __VA_ARGS__); \
            return err_code;                                                                         \
        }                                                                                            \
    } while (0)
#endif

/**
 * Macro which can be used to check the error code. If the code is not MDL_OK, it prints the message and returns.
 */
#ifndef MDL_RETURN_ON_ERROR
#define MDL_RETURN_ON_ERROR(x, log_tag, format, ...)                                                 \
    do                                                                                               \
    {                                                                                                \
        mdl_master_err_t err_rc_ = (x);                                                              \
        if (err_rc_ != MDL_OK)                                                                       \
        {                                                                                            \
            MDL_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(, ) __VA_ARGS__); \
            return err_rc_;                                                                          \
        }                                                                                            \
    } while (0)
#endif

/**
 * Macro which can be used to check the condition. If the condition is not 'true', it prints the message,
 * sets the local variable 'ret' to the supplied 'err_code', and then exits by jumping to 'goto_tag'.
 */
#ifndef MDL_GOTO_ON_FALSE
#define MDL_GOTO_ON_FALSE(a, err_code, goto_tag, log_tag, format, ...)                               \
    do                                                                                               \
    {                                                                                                \
        if (!(a))                                                                                    \
        {                                                                                            \
            MDL_LOGE(log_tag, "%s(%d): " format, __FUNCTION__, __LINE__ __VA_OPT__(, ) __VA_ARGS__); \
            ret = err_code;                                                                          \
            goto goto_tag;                                                                           \
        }                                                                                            \
    } while (0)
#endif

/**
 * Macro which can be used to check the error code. If the code is not MDL_OK, it prints the message,
 * sets the local variable 'ret' to the code, and then exits by jumping to 'goto_tag'.
 */
#ifndef MDL_GOTO_ON_ERROR
#define MDL_GOTO_ON_ERROR(x, goto_tag, log_tag, format, ...) \
    do                                                       \
    {                                                        \
        (void)log_tag;                                       \
        mdl_master_err_t err_rc_ = (x);                      \
        if (err_rc_ != MDL_OK)                               \
        {                                                    \
            ret = err_rc_;                                   \
            goto goto_tag;                                   \
        }                                                    \
    } while (0)
#endif

//======================================================================================================================
//                                                   PUBLIC FUNCTIONS
//======================================================================================================================

void mdl_master_init(mdl_master_ctx_t *ctx, mdl_master_uart_cb_cfg_t *uart_cb_cfg, void *uart_userdata,
                     mdl_master_cb_cfg_t *master_cb_cfg, mdl_prop_t *prop_list, size_t prop_list_size, void *prop_userdata)
{
    assert(ctx != NULL);
    assert(master_cb_cfg != NULL);
    assert(master_cb_cfg->node_cnt_update != NULL);
    assert(master_cb_cfg->node_exists_and_must_be_written != NULL);
    assert(master_cb_cfg->node_error_set != NULL);
    assert(uart_cb_cfg != NULL);
    assert(uart_cb_cfg->read != NULL);
    assert(uart_cb_cfg->write != NULL);
    assert(uart_cb_cfg->read_timeout_set != NULL);
    assert(uart_cb_cfg->flush_rx_buff != NULL);
    assert(uart_cb_cfg->wait_tx_done != NULL);
    assert(prop_list != NULL);
    assert(prop_list_size > 0);

    ctx->prop_list = prop_list;
    ctx->prop_list_size = prop_list_size;
    ctx->prop_userdata = prop_userdata;
    ctx->uart = *uart_cb_cfg;
    ctx->uart_userdata = uart_userdata;
    ctx->master = *master_cb_cfg;

    uart_cb_cfg->read_timeout_set(uart_userdata, MDL_NODE_TIMEOUT_MS);

    ctx->queued.state = MDL_MASTER_QUEUE_STATE_UNDEFINED;
}

//----------------------------------------------------------------------------------------------------------------------

mdl_master_err_t mdl_master_prop_read(mdl_master_ctx_t *ctx, mdl_prop_id_t property_id)
{
    mdl_master_err_t err = MDL_MASTER_OK;
    MDL_RETURN_ON_FALSE(ctx != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "ctx is NULL");

    /* Initiate the message. */
    mdl_msg_header_t tx_header = {0};
    mdl_header_action_set(&tx_header, MDL_ACTION_READ);

    if (property_id == MDL_MASTER_READ_NODE_ERRORS)
    {
        /* Prepare to read node errors instead of property. */
        mdl_header_read_error_bit_set(&tx_header, true);
        MDL_LOGI(TAG, "Reading Node Errors from all Nodes.");
    }
    else
    {
        mdl_header_property_set(&tx_header, property_id);

        /* Check if the property exists. */
        MDL_RETURN_ON_FALSE(property_id < ctx->prop_list_size, MDL_MASTER_ERR_NOT_SUPPORTED, TAG,
                            "Property (%d) does not exist", property_id);

        /* Check if the property handler supports writing. (We read from the nodes and write to the master. ) */
        MDL_RETURN_ON_FALSE(ctx->prop_list[property_id].handler.set != NULL, MDL_MASTER_ERR_NOT_SUPPORTED, TAG,
                            "Property (%d) %s is not readable.", property_id,
                            ctx->prop_list[property_id].attribute.name);

        MDL_LOGI(TAG, "Reading Property (%d) %s from all Nodes.", property_id,
                 ctx->prop_list[property_id].attribute.name);
    }

    /* Update the header parity.*/
    mdl_header_parity_set(&tx_header, true);

    /* Flush uart RX buffer. */
    ctx->uart.flush_rx_buff(ctx->uart_userdata);

    /* Send the header. */
    MDL_RETURN_ON_FALSE(ctx->uart.write(ctx->uart_userdata, tx_header.raw, MDL_ACTION_HEADER_SIZE) ==
                            MDL_ACTION_HEADER_SIZE,
                        MDL_MASTER_ERR_FAIL, TAG, "Failed to send header");

    /* Receive the header. */
    mdl_msg_header_t rx_header = {0};
    MDL_RETURN_ON_FALSE(ctx->uart.read(ctx->uart_userdata, rx_header.raw, MDL_ACTION_HEADER_SIZE) ==
                            MDL_ACTION_HEADER_SIZE,
                        MDL_MASTER_ERR_TIMEOUT, TAG, "Failed to receive header");

    /* Check header integrity. */
    MDL_RETURN_ON_FALSE(mdl_header_parity_check(rx_header), MDL_MASTER_ERR_FAIL, TAG, "Header parity invalid");
    MDL_RETURN_ON_FALSE(mdl_header_action_get(rx_header) == MDL_ACTION_READ, MDL_MASTER_ERR_FAIL, TAG,
                        "Header action corrupted");

    if (property_id == MDL_MASTER_READ_NODE_ERRORS)
    {
        MDL_RETURN_ON_FALSE(mdl_header_read_error_bit_get(rx_header), MDL_MASTER_ERR_FAIL, TAG,
                            "Header read error bit not set");
        MDL_RETURN_ON_FALSE(mdl_header_property_get(rx_header) == 0, MDL_MASTER_ERR_FAIL, TAG,
                            "Header property corrupted");
    }
    else
    {
        MDL_RETURN_ON_FALSE(!mdl_header_read_error_bit_get(rx_header), MDL_MASTER_ERR_FAIL, TAG,
                            "Header read error bit set");
        MDL_RETURN_ON_FALSE(mdl_header_property_get(rx_header) == property_id, MDL_MASTER_ERR_FAIL, TAG,
                            "Header property corrupted");
    }

    /* Update the node count. */
    uint16_t node_cnt = mdl_header_node_cnt_get(rx_header);
    ctx->master.node_cnt_update(ctx->prop_userdata, node_cnt);

    /* Receive the data. */
    for (uint16_t i = 0; i < node_cnt; i++)
    {
        uint8_t property_data[MDL_PAYLOAD_SIZE_MAX] = {0};
        size_t data_size = MDL_PROPERTY_SIZE_MAX;

        /* Read data until the end of the message. */
        size_t read_cnt = 0;
        do
        {
            MDL_RETURN_ON_FALSE(ctx->uart.read(ctx->uart_userdata, property_data + read_cnt, 1) == 1,
                                MDL_MASTER_ERR_TIMEOUT, TAG, "Failed to receive read all data");
            read_cnt++;
        } while (property_data[read_cnt - 1] != 0x00 && read_cnt < MDL_PAYLOAD_SIZE_MAX);

        /* Decode the payload. */
        MDL_RETURN_ON_FALSE(mdl_payload_cobs_decode(property_data, &data_size, property_data, read_cnt),
                            MDL_MASTER_ERR_COBS_DEC, TAG, "Failed to decode COBS payload");

        if (data_size == 0)
        {
            // No data received for this node, skip processing.
            continue;
        }

        /* Verify the checksum. */
        MDL_RETURN_ON_FALSE(mdl_checksum_calculate(property_data, data_size) == MDL_CHECKSUM_OK, MDL_MASTER_ERR_CHECKSUM,
                            TAG, "Payload checksum invalid");
        data_size--; /* Remove checksum from size. */

        /* Handle the data. */
        if (property_id == MDL_MASTER_READ_NODE_ERRORS)
        {
            ctx->master.node_error_set(ctx->prop_userdata, i, (mdl_node_err_t)property_data[0],
                                       (mdl_node_state_t)property_data[1]);
        }
        else
        {
            if (!ctx->prop_list[property_id].handler.set(ctx->prop_userdata, i, property_data, &data_size))
            {
                err = MDL_MASTER_ERR_WRITE_CB;
                MDL_LOGE(TAG, "\"set\" callback failed for node %d", i);
            }
        }
    }

    return err;
}

//----------------------------------------------------------------------------------------------------------------------

mdl_master_err_t mdl_master_prop_write(mdl_master_ctx_t *ctx, mdl_prop_id_t property_id, uint16_t node_cnt,
                                       bool staged_write, bool broadcast)
{
    MDL_RETURN_ON_FALSE(ctx != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "ctx is NULL");

    /* Check if the property exists. */
    MDL_RETURN_ON_FALSE(property_id < ctx->prop_list_size, MDL_MASTER_ERR_NOT_SUPPORTED, TAG,
                        "Property (%d) does not exist", property_id);

    /* Check if the property handler supports reading. (We read from the master and write to the nodes. ) */
    MDL_RETURN_ON_FALSE(ctx->prop_list[property_id].handler.get != NULL, MDL_MASTER_ERR_NOT_SUPPORTED, TAG,
                        "Property (%d) %s is not readable.", property_id, ctx->prop_list[property_id].attribute.name);

    /* Check If the node count is valid. */
    MDL_RETURN_ON_FALSE(broadcast || node_cnt > 0, MDL_MASTER_ERR_INVALID_ARG, TAG,
                        "Node count must be greater than zero in non-broadcast mode");

    MDL_LOGI(TAG, "Writing Property (%d) %s to all Nodes.", property_id, ctx->prop_list[property_id].attribute.name);

    /* Initiate the message. */
    mdl_msg_header_t tx_header = {0};
    mdl_header_action_set(&tx_header, broadcast ? MDL_ACTION_BROADCAST : MDL_ACTION_WRITE);
    mdl_header_staging_bit_set(&tx_header, staged_write);
    mdl_header_property_set(&tx_header, property_id);
    mdl_header_node_cnt_set(&tx_header, broadcast ? 0 : node_cnt);
    mdl_header_parity_set(&tx_header, true);

    /* Flush uart RX buffer. */
    ctx->uart.flush_rx_buff(ctx->uart_userdata);

    /* Send the header. */
    MDL_RETURN_ON_FALSE(ctx->uart.write(ctx->uart_userdata, tx_header.raw, MDL_ACTION_HEADER_SIZE) ==
                            MDL_ACTION_HEADER_SIZE,
                        MDL_MASTER_ERR_FAIL, TAG, "Failed to send header");

    node_cnt = broadcast ? 1 : node_cnt; /* Update node count for broadcast mode */

    size_t property_size = 0;
    uint8_t property_data[MDL_PAYLOAD_SIZE_MAX] = {0};
    /* Write the data.*/
    for (int16_t i = node_cnt - 1; i >= 0; i--)
    {
        size_t unencoded_size = 0;

        /* Get the property data. */
        MDL_RETURN_ON_FALSE(ctx->prop_list[property_id].handler.get(
                                ctx->prop_userdata, i, property_data + MDL_COBS_OVERHEAD_SIZE - 1, &unencoded_size),
                            MDL_MASTER_ERR_READ_CB, TAG, "\"get\" callback failed for node %d", i);

        assert(unencoded_size <= MDL_PROPERTY_SIZE_MAX);

        /* Append checksum. */
        property_data[MDL_COBS_OVERHEAD_SIZE - 1 + unencoded_size] =
            mdl_checksum_calculate(property_data + MDL_COBS_OVERHEAD_SIZE - 1, unencoded_size);

        /* Encode the property data. */
        property_size = sizeof(property_data);
        MDL_RETURN_ON_FALSE(mdl_payload_cobs_encode(property_data, &property_size,
                                                    property_data + MDL_COBS_OVERHEAD_SIZE - 1, unencoded_size + 1),
                            MDL_MASTER_ERR_COBS_ENC, TAG, "Failed to encode COBS payload");

        /* Send the property data. */
        MDL_RETURN_ON_FALSE(ctx->uart.write(ctx->uart_userdata, property_data, property_size) == property_size,
                            MDL_MASTER_ERR_FAIL, TAG, "Failed to send property data");
    }

    /* Receive the header. */
    mdl_msg_header_t rx_header = {0};
    MDL_RETURN_ON_FALSE(ctx->uart.read(ctx->uart_userdata, rx_header.raw, MDL_ACTION_HEADER_SIZE) ==
                            MDL_ACTION_HEADER_SIZE,
                        MDL_MASTER_ERR_TIMEOUT, TAG, "Failed to receive header");

    /* Check header integrity. */
    MDL_RETURN_ON_FALSE(mdl_header_parity_check(rx_header), MDL_MASTER_ERR_FAIL, TAG, "Header parity invalid");
    MDL_RETURN_ON_FALSE(mdl_header_action_get(rx_header) == (broadcast ? MDL_ACTION_BROADCAST : MDL_ACTION_WRITE),
                        MDL_MASTER_ERR_FAIL, TAG, "Header action corrupted");
    MDL_RETURN_ON_FALSE(mdl_header_property_get(rx_header) == property_id, MDL_MASTER_ERR_FAIL, TAG,
                        "Header property corrupted");

    /* Update the node count. */
    uint16_t node_cnt_rx = mdl_header_node_cnt_get(rx_header);

    /* When we are not in broadcast mode, we expect the node count to be zero, If it is more than zero, the number of
     * nodes has dropped. It can't be less than zero because that would crash the node. When not in broadcast mode, the
     * node count shall be the number of nodes in the system. */
    if (!broadcast)
    {
        node_cnt -= node_cnt_rx;
    }
    else
    {
        node_cnt = node_cnt_rx;
    }

    ctx->master.node_cnt_update(ctx->prop_userdata, node_cnt);

    if (broadcast)
    {
        /* Receive the property data */
        uint8_t property_data_rx[MDL_PROPERTY_SIZE_MAX] = {0};
        MDL_RETURN_ON_FALSE(ctx->uart.read(ctx->uart_userdata, property_data_rx, property_size) == property_size,
                            MDL_MASTER_ERR_TIMEOUT, TAG, "Failed to receive property data");

        /* Compare received property with transmitted property data. */
        MDL_RETURN_ON_FALSE(memcmp(property_data, property_data_rx, property_size) == 0,
                            MDL_MASTER_ERR_BROADCAST_CORRUPTED, TAG, "Broadcast data corrupted");
    }

    return MDL_MASTER_OK;
}

//----------------------------------------------------------------------------------------------------------------------

mdl_master_err_t mdl_master_sync_ack(mdl_master_ctx_t *ctx, mdl_sync_error_code_t *node_errors_present)
{
    MDL_RETURN_ON_FALSE(ctx != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "ctx is NULL");
    MDL_RETURN_ON_FALSE(node_errors_present != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "node_errors_present is NULL");

    mdl_msg_header_t tx_header = {0};
    mdl_header_action_set(&tx_header, MDL_ACTION_SYNC);
    mdl_header_sync_type_set(&tx_header, MDL_SYNC_ACK);

    /* Flush uart RX buffer. */
    ctx->uart.flush_rx_buff(ctx->uart_userdata);

    /* Send the header. */
    ctx->uart.wait_tx_done(ctx->uart_userdata);
    MDL_RETURN_ON_FALSE(ctx->uart.write(ctx->uart_userdata, tx_header.raw, MDL_SYNC_HEADER_SIZE) == MDL_SYNC_HEADER_SIZE,
                        MDL_MASTER_ERR_FAIL, TAG, "Failed to send SYNC header");

    /* Receive the header. */
    mdl_msg_header_t rx_header = {0};
    MDL_RETURN_ON_FALSE(ctx->uart.read(ctx->uart_userdata, rx_header.raw, MDL_SYNC_HEADER_SIZE) == MDL_SYNC_HEADER_SIZE,
                        MDL_MASTER_ERR_TIMEOUT, TAG, "Failed to receive SYNC header");

    /* Check for header corruption. */
    MDL_RETURN_ON_FALSE(mdl_header_action_get(rx_header) == MDL_ACTION_SYNC, MDL_MASTER_ERR_FAIL, TAG,
                        "Header action corrupted");
    MDL_RETURN_ON_FALSE(mdl_header_sync_type_get(rx_header) == MDL_SYNC_ACK, MDL_MASTER_ERR_FAIL, TAG,
                        "Header sync type corrupted");

    /* Check header error bits. */
    *node_errors_present = mdl_header_sync_error_get(rx_header);

    return MDL_MASTER_OK;
}

//----------------------------------------------------------------------------------------------------------------------

mdl_master_err_t mdl_master_prop_node_commit_prop(mdl_master_ctx_t *ctx)
{
    MDL_RETURN_ON_FALSE(ctx != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "ctx is NULL");

    mdl_msg_header_t tx_header = {0};
    mdl_header_action_set(&tx_header, MDL_ACTION_SYNC);
    mdl_header_sync_type_set(&tx_header, MDL_SYNC_COMMIT);

    /* Flush uart RX buffer. */
    ctx->uart.flush_rx_buff(ctx->uart_userdata);

    /* Send the header. */
    ctx->uart.wait_tx_done(ctx->uart_userdata);
    MDL_RETURN_ON_FALSE(ctx->uart.write(ctx->uart_userdata, tx_header.raw, MDL_SYNC_HEADER_SIZE) == MDL_SYNC_HEADER_SIZE,
                        MDL_MASTER_ERR_FAIL, TAG, "Failed to send SYNC header");

    /* Receive the header. */
    mdl_msg_header_t rx_header = {0};
    MDL_RETURN_ON_FALSE(ctx->uart.read(ctx->uart_userdata, rx_header.raw, MDL_SYNC_HEADER_SIZE) == MDL_SYNC_HEADER_SIZE,
                        MDL_MASTER_ERR_TIMEOUT, TAG, "Failed to receive SYNC header");

    /* Check for header corruption. */
    MDL_RETURN_ON_FALSE(mdl_header_action_get(rx_header) == MDL_ACTION_SYNC, MDL_MASTER_ERR_FAIL, TAG,
                        "Header action corrupted");
    MDL_RETURN_ON_FALSE(mdl_header_sync_type_get(rx_header) == MDL_SYNC_COMMIT, MDL_MASTER_ERR_FAIL, TAG,
                        "Header sync type corrupted");
    /* Error bits should not be set in commit action. */
    MDL_RETURN_ON_FALSE(mdl_header_sync_error_get(rx_header) == 0, MDL_MASTER_ERR_FAIL, TAG,
                        "Header error bits corrupted");

    return MDL_MASTER_OK;
}

//----------------------------------------------------------------------------------------------------------------------

mdl_master_err_t mdl_master_queue_prop_read(mdl_master_ctx_t *ctx, mdl_prop_id_t property_id)
{
    MDL_RETURN_ON_FALSE(ctx != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "ctx is NULL");

    /* Check if another action is already queued. */
    MDL_RETURN_ON_FALSE(ctx->queued.state == MDL_MASTER_QUEUE_STATE_UNDEFINED, MDL_MASTER_ERR_QUEUE_FULL, TAG,
                        "Another action is already queued");

    /* Queue the read action. */
    ctx->queued.state = MDL_MASTER_QUEUE_STATE_READ;
    ctx->queued.prop_id = property_id;
    ctx->queued.attempt_cnt = 0;

    return MDL_MASTER_OK;
}

//----------------------------------------------------------------------------------------------------------------------

mdl_master_err_t mdl_master_queue_prop_write(mdl_master_ctx_t *ctx, mdl_prop_id_t property_id, uint16_t node_cnt,
                                             bool broadcast)
{
    MDL_RETURN_ON_FALSE(ctx != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "ctx is NULL");

    /* Check if another action is already queued. */
    MDL_RETURN_ON_FALSE(ctx->queued.state == MDL_MASTER_QUEUE_STATE_UNDEFINED, MDL_MASTER_ERR_QUEUE_FULL, TAG,
                        "Another action is already queued");

    /* Queue the write action. */
    ctx->queued.state = MDL_MASTER_QUEUE_STATE_WRITE;
    ctx->queued.prop_id = property_id;
    ctx->queued.node_cnt = node_cnt;
    ctx->queued.broadcast = broadcast;
    ctx->queued.attempt_cnt = 0;

    return MDL_MASTER_OK;
}

//----------------------------------------------------------------------------------------------------------------------

mdl_master_err_t mdl_master_communication_handler(mdl_master_ctx_t *ctx, uint32_t *next_tick_ms)
{
    MDL_RETURN_ON_FALSE(ctx != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "ctx is NULL");
    MDL_RETURN_ON_FALSE(next_tick_ms != NULL, MDL_MASTER_ERR_INVALID_ARG, TAG, "next_tick_ms is NULL");

    mdl_master_err_t err = MDL_MASTER_OK;
    mdl_sync_error_code_t node_errors_present = MDL_SYNC_ERR_NONE;

    switch (ctx->queued.state)
    {
    case MDL_MASTER_QUEUE_STATE_READ:
        err = mdl_master_prop_read(ctx, ctx->queued.prop_id);
        *next_tick_ms = (err == MDL_MASTER_OK) ? 0 : MDL_NODE_RECOVERY_DELAY_MS;
        break;

    case MDL_MASTER_QUEUE_STATE_WRITE:
        err = mdl_master_prop_write(ctx, ctx->queued.prop_id, ctx->queued.node_cnt, false, ctx->queued.broadcast);
        *next_tick_ms = (err == MDL_MASTER_OK) ? 0 : MDL_NODE_RECOVERY_DELAY_MS;
        if (err == MDL_MASTER_OK)
        {
            ctx->queued.state = MDL_MASTER_QUEUE_STATE_SYNC_ACK;
            err = mdl_master_communication_handler(ctx, next_tick_ms); /* Immediately proceed to SYNC ACK */
        }
        break;

    case MDL_MASTER_QUEUE_STATE_SYNC_ACK:
        err = mdl_master_sync_ack(ctx, &node_errors_present);
        *next_tick_ms = (err == MDL_MASTER_OK) ? 0 : MDL_NODE_RECOVERY_DELAY_MS;
        if (err == MDL_MASTER_OK && node_errors_present != MDL_SYNC_ERR_NONE)
        {
            ctx->queued.state = MDL_MASTER_QUEUE_STATE_READ;
            ctx->queued.original_prop_id = ctx->queued.prop_id; /* Store for later. */
            ctx->queued.prop_id = MDL_MASTER_READ_NODE_ERRORS;
            err = mdl_master_communication_handler(ctx, next_tick_ms); /* Immediately proceed to READ NODE ERRORS */
            *next_tick_ms = (err == MDL_MASTER_OK) ? 0 : MDL_NODE_RECOVERY_DELAY_MS;
            if (err == MDL_MASTER_OK)
            {
                /* Restore original property ID for next attempt. */
                ctx->queued.prop_id = ctx->queued.original_prop_id;
                ctx->queued.state = MDL_MASTER_QUEUE_STATE_WRITE;
                err = MDL_MASTER_ERR_FAIL; /* Indicate that an error was present. */
            }
        }
        break;

    case MDL_MASTER_QUEUE_STATE_UNDEFINED:
        err = MDL_MASTER_ERR_QUEUE_EMPTY;
        *next_tick_ms = 0;
        break;

    default:
        err = MDL_MASTER_ERR_INVALID_ARG;
        *next_tick_ms = 0;
        break;
    }

    if (err == MDL_MASTER_OK)
    {
        /* Clear the queued action on success or timeout. */
        ctx->queued.state = MDL_MASTER_QUEUE_STATE_UNDEFINED;
    }

    return err;
}