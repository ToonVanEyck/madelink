#pragma once

#include "madelink_shared.h"

#define MDL_MASTER_READ_NODE_ERRORS (-1) /**< Use as property with the master read function to read node errors. */

typedef bool (*mdl_master_node_cnt_update_cb_t)(void *userdata, uint16_t node_cnt);
typedef bool (*mdl_master_node_exists_and_must_be_written_cb_t)(void *userdata, uint16_t node_idx,
                                                                mdl_prop_id_t property_id, bool *must_be_written);
typedef void (*mdl_master_node_error_set_cb_t)(void *userdata, uint16_t node_idx, mdl_node_err_t error,
                                               mdl_node_state_t state);

typedef enum
{
    MDL_MASTER_OK = 0,
    MDL_MASTER_ERR_TIMEOUT,
    MDL_MASTER_ERR_UART,
    MDL_MASTER_ERR_PAYLOAD,
    MDL_MASTER_ERR_CHECKSUM,
    MDL_MASTER_ERR_INVALID_ARG,
    MDL_MASTER_ERR_NO_MEM,
    MDL_MASTER_ERR_NOT_SUPPORTED,
    MDL_MASTER_ERR_FAIL,
    MDL_MASTER_ERR_WRITE_CB,
    MDL_MASTER_ERR_READ_CB,
    MDL_MASTER_ERR_COBS_ENC,
    MDL_MASTER_ERR_COBS_DEC,
    MDL_MASTER_ERR_BROADCAST_CORRUPTED,
    MDL_MASTER_ERR_QUEUE_EMPTY,
    MDL_MASTER_ERR_QUEUE_FULL,
} mdl_master_err_t;

typedef struct
{
    uart_read_cb_t read;
    uart_write_cb_t write;
    uart_read_timeout_set_cb_t read_timeout_set;
    uart_flush_rx_buff_cb_t flush_rx_buff;
    uart_wait_tx_done_cb_t wait_tx_done;
} mdl_master_uart_cb_cfg_t;

typedef struct
{
    mdl_master_node_cnt_update_cb_t node_cnt_update;
    mdl_master_node_exists_and_must_be_written_cb_t node_exists_and_must_be_written;
    mdl_master_node_error_set_cb_t node_error_set;
} mdl_master_cb_cfg_t;

typedef enum
{
    MDL_MASTER_QUEUE_STATE_UNDEFINED = 0,
    MDL_MASTER_QUEUE_STATE_READ,
    MDL_MASTER_QUEUE_STATE_WRITE,
    MDL_MASTER_QUEUE_STATE_SYNC_ACK,
    MDL_MASTER_QUEUE_STATE_SYNC_COMMIT,
} mdl_master_queue_state_t;

typedef struct
{
    mdl_master_queue_state_t state;
    mdl_prop_id_t prop_id;
    mdl_prop_id_t original_prop_id;
    uint16_t node_cnt;
    bool broadcast;
    uint8_t attempt_cnt;
} mdl_master_queued_action_t;

typedef struct
{
    mdl_prop_t *prop_list;         /**< List of property handlers and attributes. */
    size_t prop_list_size;         /**< Size of prop_list. */
    void *prop_userdata;           /**< User data for the property callback functions. */
    mdl_master_uart_cb_cfg_t uart; /**< Uart driver callback configurations. */
    void *uart_userdata;           /**< Uart user data to be used by callback functions. */

    mdl_master_cb_cfg_t master; /**< Master callback configurations. */

    mdl_master_queued_action_t queued; /**< Currently queued action. */
} mdl_master_ctx_t;

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Initializes the madelink context.
 *
 * This function initializes the madelink context based on the provided UART driver.
 *
 * @param[inout] ctx Pointer to the #mdl_node_ctx_t structure containing the context information.
 * @param[in] uart_cb_cfg Pointer to the UART callback structure.
 * @param[in] uart_userdata Pointer to the UART user data.
 * @param[in] master_cb_cfg Pointer to the master callback structure.
 * @param[in] prop_list Pointer to the list of properties.
 * @param[in] prop_list_size Size of the property list.
 * @param[in] prop_userdata Pointer to user data for the property callback functions.
 */
void mdl_master_init(mdl_master_ctx_t *ctx, mdl_master_uart_cb_cfg_t *uart_cb_cfg, void *uart_userdata,
                     mdl_master_cb_cfg_t *master_cb_cfg, mdl_prop_t *prop_list, size_t prop_list_size,
                     void *prop_userdata);

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Reads a property from all nodes.
 *
 * @param[inout] ctx Pointer to the #mdl_master_ctx_t structure containing the context information.
 * @param[in] property_id The ID of the property to read. Use MDL_MASTER_READ_NODE_ERRORS to read node errors.
 *
 * @return MDL_MASTER_OK on success, or an error code on failure.
 */
mdl_master_err_t mdl_master_prop_read(mdl_master_ctx_t *ctx, mdl_prop_id_t property_id);

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Writes a property from all nodes.
 *
 * @param[inout] ctx Pointer to the #mdl_master_ctx_t structure containing the context information.
 * @param[in] property_id The ID of the property to write.
 * @param[in] node_cnt The number of nodes to write the property to. (Must be 0 for broadcast.)
 * @param[in] staged_write If true, the property will be staged and not applied immediately.
 * @param[in] broadcast If true, the property read from node 1 will be written to all nodes.
 *
 * @return MDL_MASTER_OK on success, or an error code on failure.
 */
mdl_master_err_t mdl_master_prop_write(mdl_master_ctx_t *ctx, mdl_prop_id_t property_id, uint16_t node_cnt,
                                       bool staged_write, bool broadcast);

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Performs a SYNC ACK action with all nodes.
 *
 * @param[inout] ctx Pointer to the #mdl_master_ctx_t structure containing the context information.
 * @param[out] node_errors_present Pointer where node errors will be reported.
 *
 * @return MDL_MASTER_OK on success, or an error code on failure.
 */
mdl_master_err_t mdl_master_sync_ack(mdl_master_ctx_t *ctx, mdl_sync_error_code_t *node_errors_present);

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Queue a read request for a property from all nodes.
 *
 * @param[inout] ctx Pointer to the #mdl_master_ctx_t structure containing the context information.
 * @param[in] property_id The ID of the property to read. Use MDL_MASTER_READ_NODE_ERRORS to read node errors.
 *
 * @return MDL_MASTER_OK on success, MDL_MASTER_ERR_FAIL if there was already a queued action.
 */
mdl_master_err_t mdl_master_queue_prop_read(mdl_master_ctx_t *ctx, mdl_prop_id_t property_id);

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Queue a write request for a property from all nodes.
 *
 * @param[inout] ctx Pointer to the #mdl_master_ctx_t structure containing the context information.
 * @param[in] property_id The ID of the property to write.
 * @param[in] node_cnt The number of nodes to write the property to. (Must be 0 for broadcast.)
 * @param[in] broadcast If true, the property read from node 1 will be written to all nodes.
 *
 * @return MDL_MASTER_OK on success, MDL_MASTER_ERR_FAIL if there was already a queued action.
 */
mdl_master_err_t mdl_master_queue_prop_write(mdl_master_ctx_t *ctx, mdl_prop_id_t property_id, uint16_t node_cnt,
                                             bool broadcast);

//----------------------------------------------------------------------------------------------------------------------

/**
 * @brief Handles the queued madelink requests.
 *
 * This function executes the madelink master FSM based on the provided context. As long as the function
 * returns an error code and a next_tick_ms greater than 0, it should be called again to continue processing.
 *
 * @param[inout] ctx Pointer to the #mdl_master_ctx_t structure containing the context information.
 * @param[out] next_tick_ms If this function does not return MDL_MASTER_ERR_TIMEOUT and this value is not 0, call this
 * function again after the specified milliseconds.
 *
 * @return MDL_MASTER_OK on success, or an error code on failure.
 */
mdl_master_err_t mdl_master_communication_handler(mdl_master_ctx_t *ctx, uint32_t *next_tick_ms);