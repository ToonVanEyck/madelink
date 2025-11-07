#include "madelink_node.h"

#include <assert.h>
#include <string.h>

#ifndef TAG
#define TAG "madelink_node"
#endif

#if !defined(MDL_LOGE) || !defined(MDL_LOGW) || !defined(MDL_LOGI) || !defined(MDL_LOGD)
#include <stdio.h>
#define MDL_LOGE(tag, format, ...) printf("E: [%s]: " format "\n", tag, ##__VA_ARGS__)
#define MDL_LOGW(tag, format, ...) printf("W: [%s]: " format "\n", tag, ##__VA_ARGS__)
#define MDL_LOGI(tag, format, ...) printf("I: [%s]: " format "\n", tag, ##__VA_ARGS__)
#define MDL_LOGD(tag, format, ...) printf("D: [%s]: " format "\n", tag, ##__VA_ARGS__)
#endif

//======================================================================================================================
//                                                   GLOBAL VARIABLES
//======================================================================================================================

/** Maps a node error to a sync error bit.*/
static const mdl_sync_error_code_t sync_err_map[] = {MDL_NODE_ERROR_GENERATOR(GENERATE_2ND_FIELD)};

//======================================================================================================================
//                                                  FUNCTION PROTOTYPES
//======================================================================================================================

/* Madelink state implementations. */
void mdl_node_state_rx_header(mdl_node_ctx_t *ctx);
void mdl_node_state_error(mdl_node_ctx_t *ctx);
void mdl_node_state_dec_node_cnt(mdl_node_ctx_t *ctx);
void mdl_node_state_tx_prop(mdl_node_ctx_t *ctx);
void mdl_node_state_rx_prop(mdl_node_ctx_t *ctx);
void mdl_node_state_commit_prop(mdl_node_ctx_t *ctx);

/* Madelink state change functions. */
void mdl_node_state_change(mdl_node_ctx_t *ctx, mdl_node_state_t state);
void mdl_node_error_and_terminate(mdl_node_ctx_t *ctx, mdl_node_err_t err);
void mdl_node_error_and_continue(mdl_node_ctx_t *ctx, mdl_node_err_t err);

/* Madelink timer functions. */
void mdl_node_timer_start(mdl_node_ctx_t *ctx);
bool mdl_node_timer_elapsed(mdl_node_ctx_t *ctx);

/* Madelink UART functions. */
static size_t mdl_node_uart_read_if_writable(mdl_node_ctx_t *ctx, uint8_t *buf, size_t length);
static bool mdl_node_uart_forward_byte(mdl_node_ctx_t *ctx, uint8_t *byte, bool forward);

//======================================================================================================================
//                                                   PUBLIC FUNCTIONS
//======================================================================================================================

void mdl_node_init(mdl_node_ctx_t *ctx, const mdl_node_uart_cb_cfg_t *uart_cb, void *uart_userdata, mdl_prop_t *prop_list,
                   size_t prop_list_size, void *prop_userdata)
{
    memset(ctx, 0, sizeof(*ctx)); /* Clear context */

    ctx->prop_list = prop_list;
    ctx->prop_list_size = prop_list_size;
    ctx->prop_userdata = prop_userdata;
    ctx->uart = *uart_cb;
    ctx->uart_userdata = uart_userdata;

    ctx->state = MDL_NODE_STATE_RX_HEADER;
    ctx->next_state = MDL_NODE_STATE_UNDEFINED;
    ctx->last_error = MDL_NODE_ERR_NONE;
    ctx->error_state = MDL_NODE_STATE_UNDEFINED;
}

//----------------------------------------------------------------------------------------------------------------------

void mdl_node_tick(mdl_node_ctx_t *ctx, uint32_t tick_ms)
{
    ctx->last_tick_ms = tick_ms;

    if (ctx->next_state != MDL_NODE_STATE_UNDEFINED)
    {
        MDL_LOGD(TAG, "State: %s -> %s", mdl_node_state_to_str(ctx->state), mdl_node_state_to_str(ctx->next_state));
        ctx->state = ctx->next_state;
        ctx->next_state = MDL_NODE_STATE_UNDEFINED;
        ctx->data_cnt = 0;
    }

    if (ctx->uart.is_busy(ctx->uart_userdata))
    {
        mdl_node_timer_start(ctx); /* Restart the timer. */
    }

    switch (ctx->state)
    {
    case MDL_NODE_STATE_RX_HEADER:
        mdl_node_state_rx_header(ctx);
        break;
    case MDL_NODE_STATE_ERROR:
        mdl_node_state_error(ctx);
        break;
    case MDL_NODE_STATE_DEC_NODE_CNT:
        mdl_node_state_dec_node_cnt(ctx);
        break;
    case MDL_NODE_STATE_TX_PROP:
        mdl_node_state_tx_prop(ctx);
        break;
    case MDL_NODE_STATE_RX_PROP:
        mdl_node_state_rx_prop(ctx);
        break;
    case MDL_NODE_STATE_COMMIT_PROP:
        mdl_node_state_commit_prop(ctx);
        break;
    default:
        MDL_LOGD(TAG, "Invalid state");
        break;
    }
}

//----------------------------------------------------------------------------------------------------------------------

bool mdl_node_is_busy(mdl_node_ctx_t *ctx)
{
    return (ctx->state != MDL_NODE_STATE_RX_HEADER) || ctx->uart.is_busy(ctx->uart_userdata);
}

//======================================================================================================================
//                                                         PRIVATE FUNCTIONS
//======================================================================================================================

void mdl_node_state_rx_header(mdl_node_ctx_t *ctx)
{
    uint8_t data;
    /* Try to read a header byte */
    if (mdl_node_uart_read_if_writable(ctx, &data, 1) == 0)
    {
        /* We couldn't read a header byte. */
        if (ctx->data_cnt && mdl_node_timer_elapsed(ctx))
        {
            mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_TIMEOUT); /* Timeout error */
        }
        return; /* No header byte received & No timeout occurred */
    }

    mdl_node_timer_start(ctx); /* Start the timer. We don't want a timeout in this state. */

    /* Handle the data. */
    if (ctx->data_cnt == 0)
    {                              /* Handle first header byte. */
        ctx->header.raw[0] = data; /* Store the first header byte. */
        ctx->header.raw[1] = 0;    /* Clear the second header byte */
        ctx->header.raw[2] = 0;    /* Clear the third header byte */
        ctx->action = mdl_header_action_get(ctx->header);
        ctx->staged_write = mdl_header_staging_bit_get(ctx->header);

        if (ctx->action == MDL_ACTION_SYNC)
        {
            mdl_sync_type_t sync_type = mdl_header_sync_type_get(ctx->header);
            if (sync_type == MDL_SYNC_ACK)
            { /* Append our error code. */
                mdl_header_sync_error_add(&ctx->header, sync_err_map[ctx->last_error]);
                mdl_node_state_change(ctx, MDL_NODE_STATE_RX_HEADER);
            }
            else if (sync_type == MDL_SYNC_COMMIT)
            { /* Commit any staged property changes. */
                mdl_node_state_change(ctx, MDL_NODE_STATE_COMMIT_PROP);
            }
        }
    }
    else if (ctx->data_cnt == 1)
    {                              /* Handle second header byte. */
        ctx->header.raw[1] = data; /* Set the second header byte. */
        ctx->property_id = mdl_header_property_get(ctx->header);
        /* Get the node count, increment it and transmit it. */
        ctx->node_cnt = mdl_header_node_cnt_get(ctx->header);
        ctx->node_cnt += (ctx->action == MDL_ACTION_WRITE) ? (-1) : (1);
        mdl_header_node_cnt_set(&ctx->header, ctx->node_cnt & 0x1fff); /* Mask to 13 bits to avoid assertion. */
    }
    else
    {                                             /* Handle third header byte. */
        mdl_header_node_cnt_set(&ctx->header, 0); /* Clear the node count to get the MSB. */
        ctx->header.raw[2] = data;                /* Set the third header byte. */
        int16_t node_cnt_msb = mdl_header_node_cnt_get(ctx->header);
        ctx->node_cnt += node_cnt_msb;
        /* Decrement the node count before determining the header parity. */
        mdl_header_node_cnt_set(&ctx->header,
                                ((ctx->action == MDL_ACTION_WRITE) ? (ctx->node_cnt + 1) : (ctx->node_cnt - 1)));
        bool header_parity_valid = mdl_header_parity_check(ctx->header);
        mdl_header_node_cnt_set(&ctx->header, (ctx->node_cnt) & 0x1fff); /* Restore cnt. */
        bool header_valid = true;
        if (!header_parity_valid)
        {
            mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_HEADER_PARITY); /* Header parity error */
            header_valid = false;                                          /* Break parity to help next nodes. */
        }
        else
        {
            mdl_node_state_change(ctx, MDL_NODE_STATE_DEC_NODE_CNT); /* Switch to next state. */
            if (ctx->action == MDL_ACTION_WRITE && !++ctx->node_cnt)
            {
                /* Invalid state, node count can't be zero for write action. */
                mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_INVALID_STATE);
                header_valid = false; /* Break parity to help next nodes. */
            }
            /* Ensure node_cnt is 1 for broadcast. */
            ctx->node_cnt = (ctx->action == MDL_ACTION_BROADCAST) ? 1 : ctx->node_cnt;
        }
        mdl_header_parity_set(&ctx->header, header_valid); /* Ensure parity is set correctly. */
    }
    ctx->uart.write(ctx->uart_userdata, ctx->header.raw + ctx->data_cnt, 1);

    /* Increment the data count. */
    ctx->data_cnt++;
}

//----------------------------------------------------------------------------------------------------------------------

void mdl_node_state_error(mdl_node_ctx_t *ctx)
{
    /* While in error mode, read all bytes to flush them. */
    uint8_t data = 0;
    while (mdl_node_uart_read_if_writable(ctx, &data, 1))
        ;
    if (ctx->uart.is_busy(ctx->uart_userdata))
    {
        mdl_node_timer_start(ctx); /* Restart the timer. */
    }
    else if (mdl_node_timer_elapsed(ctx))
    {
        mdl_node_state_change(ctx, MDL_NODE_STATE_RX_HEADER);
    }
}

//----------------------------------------------------------------------------------------------------------------------

void mdl_node_state_dec_node_cnt(mdl_node_ctx_t *ctx)
{
    if (ctx->node_cnt == 0)
    {
        if (!ctx->staged_write)
        {
            mdl_node_state_change(ctx, MDL_NODE_STATE_COMMIT_PROP);
        }
        else
        {
            mdl_node_state_change(ctx, MDL_NODE_STATE_RX_HEADER);
        }
        return;
    }

    /* Decrement the node count. */
    ctx->node_cnt--;

    if (ctx->action == MDL_ACTION_READ && ctx->node_cnt == 0)
    {
        mdl_node_state_change(ctx, MDL_NODE_STATE_TX_PROP);
    }
    else
    {
        /* Write / Broadcast actions always go to RX_PROP state. */
        mdl_node_state_change(ctx, MDL_NODE_STATE_RX_PROP);
    }
}

//----------------------------------------------------------------------------------------------------------------------

void mdl_node_state_tx_prop(mdl_node_ctx_t *ctx)
{
    /* Check for timeout. */
    if (mdl_node_timer_elapsed(ctx))
    {
        mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_TIMEOUT); /* Timeout error */
        return;
    }

    /* Get the property data. */
    if (ctx->uart.cnt_writable(ctx->uart_userdata) >= 1)
    {
        if (ctx->data_cnt == 0)
        {
            size_t unencoded_size = 0;
            bool valid_property = true;

            if (mdl_header_read_error_bit_get(ctx->header))
            {
                /* Read the error instead of the property. */
                unencoded_size = 2;
                ctx->property_data[MDL_COBS_OVERHEAD_SIZE - 1 + 0] = ctx->last_error;
                ctx->property_data[MDL_COBS_OVERHEAD_SIZE - 1 + 1] = ctx->error_state;
                /* Clear the last error and error state. */
                ctx->last_error = MDL_NODE_ERR_NONE;
                ctx->error_state = MDL_NODE_STATE_UNDEFINED;
            }
            else if (ctx->property_id >= ctx->prop_list_size)
            {
                mdl_node_error_and_continue(ctx, MDL_NODE_ERR_PROP_NOT_SUPPORTED); /* Property not supported. */
                valid_property = false;
            }
            else if (ctx->prop_list[ctx->property_id].handler.get == NULL)
            {
                mdl_node_error_and_continue(ctx, MDL_NODE_ERR_READ_NOT_SUPPORTED); /* Can't read active property. */
                valid_property = false;
            }
            else if (!ctx->prop_list[ctx->property_id].handler.get(
                         ctx->prop_userdata, 0, ctx->property_data + MDL_COBS_OVERHEAD_SIZE - 1, &unencoded_size))
            {
                mdl_node_error_and_continue(ctx, MDL_NODE_ERR_READ_CB); /* Property read failed. */
                valid_property = false;
            }

            /* Append checksum. */
            ctx->property_data[MDL_COBS_OVERHEAD_SIZE - 1 + unencoded_size] =
                mdl_checksum_calculate(ctx->property_data + MDL_COBS_OVERHEAD_SIZE - 1, unencoded_size);

            /* Encode the property data. */
            ctx->property_size = sizeof(ctx->property_data);
            if (!mdl_payload_cobs_encode(ctx->property_data, &ctx->property_size,
                                         ctx->property_data + MDL_COBS_OVERHEAD_SIZE - 1, unencoded_size + 1))
            {
                mdl_node_error_and_continue(ctx, MDL_NODE_ERR_COBS_ENC); /* COBS encoding failed. */
                valid_property = false;
            }

            if (!valid_property)
            {
                /* Send a an empty COBS packet. */
                ctx->property_size = 1;
                ctx->property_data[0] = 0;
            }

            /* Transmit the property data. */
            ctx->data_cnt += ctx->uart.write(ctx->uart_userdata, ctx->property_data, ctx->property_size);
        }
        else
        {
            /* Continue transmitting the property data. */
            ctx->data_cnt += ctx->uart.write(ctx->uart_userdata, ctx->property_data + ctx->data_cnt,
                                             ctx->property_size - ctx->data_cnt);
        }

        /* Check if the property data has been fully transmitted. */
        if (ctx->data_cnt == ctx->property_size)
        {
            mdl_node_state_change(ctx, MDL_NODE_STATE_RX_HEADER);
        }
    }
}

//----------------------------------------------------------------------------------------------------------------------

void mdl_node_state_rx_prop(mdl_node_ctx_t *ctx)
{
    /* Check for timeout. */
    if (mdl_node_timer_elapsed(ctx))
    {
        mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_TIMEOUT); /* Timeout error */
        return;
    }

    /* Receive the property data,do not forward it if it is meant for this node only (node_cnt = 0). */
    while (mdl_node_uart_forward_byte(ctx, ctx->property_data + ctx->data_cnt,
                                      ctx->node_cnt || ctx->action == MDL_ACTION_BROADCAST))
    {
        if (ctx->property_data[ctx->data_cnt] == 0)
        {
            ctx->property_size = sizeof(ctx->property_data);

            /* Decode the payload. */
            if (!mdl_payload_cobs_decode(ctx->property_data, &ctx->property_size, ctx->property_data,
                                         ctx->data_cnt + 1))
            {
                mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_COBS_DEC); /* COBS decoding failed. */
                return;
            }

            /* Verify the payload checksum. */
            if (mdl_checksum_calculate(ctx->property_data, ctx->property_size) != MDL_CHECKSUM_OK)
            {
                mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_CHECKSUM); /* Checksum error */
                return;
            }
            ctx->property_size--; /* Remove checksum byte from size. */

            mdl_node_state_change(ctx, MDL_NODE_STATE_DEC_NODE_CNT);
            break;
        }
        ctx->data_cnt++;
    }
}

//----------------------------------------------------------------------------------------------------------------------

void mdl_node_state_commit_prop(mdl_node_ctx_t *ctx)
{
    if (!ctx->uart.tx_buff_empty(ctx->uart_userdata))
    {
        return; /* Don't commit property before all data has been written. */
    }

    if (ctx->property_id >= ctx->prop_list_size)
    {
        mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_PROP_NOT_SUPPORTED); /* Property not supported. */
        return;
    }

    if (ctx->prop_list[ctx->property_id].handler.set == NULL)
    {
        mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_WRITE_NOT_SUPPORTED); /* Can't write active property. */
        return;
    }

    if (!ctx->prop_list[ctx->property_id].handler.set(ctx->prop_userdata, 0, ctx->property_data, &ctx->property_size))
    {
        mdl_node_error_and_terminate(ctx, MDL_NODE_ERR_WRITE_CB); /* Property write failed. */
        return;
    }

    mdl_node_state_change(ctx, MDL_NODE_STATE_RX_HEADER);
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Changes the state of the madelink FSM.
 *
 * This function changes the state of the madelink FSM to the provided state.
 * It also resets the rx and tx counters of the context.
 *
 * @param[inout] ctx Pointer to the mdl_node_ctx_t structure containing the context information.
 * @param[in] state The new state of the madelink FSM.
 */
void mdl_node_state_change(mdl_node_ctx_t *ctx, mdl_node_state_t state)
{
    ctx->next_state = state;
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Store error information and change state to ERROR state.
 *
 * @param[inout] ctx Pointer to the mdl_node_ctx_t structure containing the context information.
 * @param[in] err The error code to store.
 */
void mdl_node_error_and_terminate(mdl_node_ctx_t *ctx, mdl_node_err_t err)
{
    mdl_node_error_and_continue(ctx, err);
    mdl_node_state_change(ctx, MDL_NODE_STATE_ERROR);
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Store error information and continue in active state.
 *
 * @param[inout] ctx Pointer to the mdl_node_ctx_t structure containing the context information.
 * @param[in] err The error code to store.
 */
void mdl_node_error_and_continue(mdl_node_ctx_t *ctx, mdl_node_err_t err)
{
    ctx->last_error = err;
    ctx->error_state = ctx->state;
    MDL_LOGE(TAG, "\"%s\" Error occurred in \"%s\" state!", mdl_node_error_to_str(err),
             mdl_node_state_to_str(ctx->error_state));
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Starts the timeout timer.
 *
 * @param[in] ctx Pointer to the mdl_node_ctx_t structure containing the context information.
 */
void mdl_node_timer_start(mdl_node_ctx_t *ctx)
{
    ctx->timeout_tick_cnt = ctx->last_tick_ms + MDL_NODE_TIMEOUT_MS;
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Checks if the timeout period has elapsed.
 *
 * @param[in] ctx Pointer to the mdl_node_ctx_t structure containing the context information.
 *
 * @return true if the timeout period has elapsed, false otherwise.
 */
bool mdl_node_timer_elapsed(mdl_node_ctx_t *ctx)
{
    return ctx->last_tick_ms > ctx->timeout_tick_cnt;
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Reads data from the UART but only if it can immediately be written to the next module.
 *
 * @param[in] ctx Pointer to the mdl_node_ctx_t structure containing the context information.
 * @param[out] buf Pointer to the buffer where the read data will be stored.
 * @param[in] length The number of bytes to read.
 *
 * @return The number of bytes read.
 */
static size_t mdl_node_uart_read_if_writable(mdl_node_ctx_t *ctx, uint8_t *buf, size_t length)
{
    if (ctx->uart.cnt_writable(ctx->uart_userdata) < length)
    {
        return 0;
    }
    return ctx->uart.read(ctx->uart_userdata, buf, length);
}

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Read and Write a single byte from the input UART to the output UART if possible.
 *
 * @param[in] ctx Pointer to the mdl_node_ctx_t structure containing the context information.
 * @param[out] byte Value of the byte read.
 * @param[in] forward If true, the byte will be forwarded to the output UART. If false, the byte will only be read.
 *
 * @return true if a byte was forwarded, false otherwise.
 */
static bool mdl_node_uart_forward_byte(mdl_node_ctx_t *ctx, uint8_t *byte, bool forward)
{
    return ((!forward || ctx->uart.cnt_writable(ctx->uart_userdata)) && ctx->uart.read(ctx->uart_userdata, byte, 1) &&
            (!forward || ctx->uart.write(ctx->uart_userdata, byte, 1)));
}

//----------------------------------------------------------------------------------------------------------------------
