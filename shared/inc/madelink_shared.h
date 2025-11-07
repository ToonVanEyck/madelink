#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#define ABI_VERSION 2

#define MDL_PROPERTY_SIZE_MAX (254) /**< Maximum length of a property. */

/** Additional overhead available for COBS encoding and checksum and zero terminator. */
#define MDL_COBS_OVERHEAD_SIZE ((MDL_PROPERTY_SIZE_MAX / 0xff) + 3)
#define MDL_PAYLOAD_SIZE_MAX (MDL_PROPERTY_SIZE_MAX + MDL_COBS_OVERHEAD_SIZE) /**< Maximum size of a payload. */

#ifndef MDL_NODE_TIMEOUT_MS
#define MDL_NODE_TIMEOUT_MS (25) /**< Time after which a timeout event occurs. */
#endif

#ifndef MDL_NODE_RECOVERY_DELAY_MS
#define MDL_NODE_RECOVERY_DELAY_MS (MDL_NODE_TIMEOUT_MS + 5) /**< Time to wait to ensure a timeout occurs. */
#endif

#define MDL_ACTION_HEADER_SIZE (3) /**< Size of the action header. */
#define MDL_SYNC_HEADER_SIZE (1)   /**< Size of the sync header. */

#define MDL_CHECKSUM_OK (0x00) /**< Checksum value indicating no error. */

#define GENERATE_1ST_FIELD(a, ...) a,
#define GENERATE_2ND_FIELD(a, b, ...) b,
#define GENERATE_3RD_FIELD(a, b, c, ...) c,

/**
 * @brief Madelink UART set read timeout callback.
 * Set the timeout for the blocking master read function.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 * @param[in] timeout_ms Timeout in milliseconds.
 */
typedef void (*uart_read_timeout_set_cb_t)(void *uart_userdata, uint32_t timeout_ms);

/**
 * @brief Madelink UART read callback.
 *
 * @note The master requires a blocking read with timeout functionality. The node implementation must be non-blocking.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 * @param[in] data Pointer to the data buffer.
 * @param[in] size Size of the data buffer.
 *
 * @return Number of bytes read, or -1 on error.
 */
typedef size_t (*uart_read_cb_t)(void *uart_userdata, uint8_t *data, size_t size);

/**
 * @brief Madelink UART rx buffer space used callback.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 *
 * @return Number of bytes available in the rx buffer.
 */
typedef size_t (*uart_cnt_readable_cb_t)(void *uart_userdata);

/**
 * @brief Madelink UART write callback.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 * @param[in] data Pointer to the data buffer.
 * @param[in] size Size of the data buffer.
 *
 * @return Number of bytes written, or -1 on error.
 */
typedef size_t (*uart_write_cb_t)(void *uart_userdata, const uint8_t *data, size_t size);

/**
 * @brief Madelink UART tx buffer space available callback.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 *
 * @return Number of bytes available in the tx buffer.
 */
typedef size_t (*uart_cnt_writable_cb_t)(void *uart_userdata);

/**
 * @brief Madelink UART tx pending callback.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 *
 * @return false if tx pending, true if tx buffer empty.
 */
typedef bool (*uart_tx_buff_empty_cb_t)(void *uart_userdata);

/**
 * @brief Madelink UART busy callback.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 */
typedef bool (*uart_is_busy_cb_t)(void *uart_userdata);

/**
 * @brief Madelink UART flush rx buffer callback.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 */
typedef void (*uart_flush_rx_buff_cb_t)(void *uart_userdata);

/**
 * @brief Madelink UART wait tx done callback.
 *
 * @param[in] uart_userdata Pointer to uart ctx.
 */
typedef void (*uart_wait_tx_done_cb_t)(void *uart_userdata);

/**
 * @brief Madelink property GET / SET handler callback.
 *
 * @param[inout] userdata Pointer to user data.
 * @param[in] node_idx Index of the node for which the property is requested. Only used by the master.
 * @param[in] buf Pointer to the data buffer.
 * @param[inout] size Pointer to the size of the data buffer.
 */
typedef bool (*mdl_prop_handler_cb_t)(void *userdata, uint16_t node_idx, uint8_t *buf, size_t *size);

/**
 * @brief Madelink property GET / SET alternate handler callback.
 *
 * @param[inout] userdata Pointer to user data.
 * @param[in] node_idx Index of the node for which the property is requested. Only used by the master.
 * @param[inout] data Pointer to a data structure to be interpreted by the handler.
 *
 * @return true on success, false on failure.
 */
typedef bool (*mdl_prop_handler_alt_cb_t)(void *userdata, uint16_t node_idx, void *data);

/**
 * @brief Madelink property GET / SET alternate handler callback.
 *
 * @param[inout] userdata Pointer to user data.
 * @param[in] node_idx Index of the node for which the property is requested. Only used by the master.
 * @param[inout] data Pointer to a data structure to be interpreted by the handler.
 *
 * @return true on success, false on failure.
 */
typedef bool (*mdl_prop_handler_alt_cb_t)(void *userdata, uint16_t node_idx, void *data);

/**
 * @brief Madelink property GET / SET alternate handler callback.
 *
 * @param[inout] userdata_a Pointer to user data A to compare.
 * @param[inout] userdata_b Pointer to user data B to compare.
 *
 * @return true if equal, false otherwise.
 */
typedef bool (*mdl_prop_compare_cb_t)(const void *userdata_a, const void *userdata_b);

typedef enum
{
    MDL_ACTION_READ = 0,      /**< Read property data from all nodes. */
    MDL_ACTION_WRITE = 1,     /**< Write (different) property data to all nodes. */
    MDL_ACTION_BROADCAST = 2, /**< Broadcast / Write (the same) property data to all nodes. */
    MDL_ACTION_SYNC = 3,      /**< Synchronization action used for error checking and committing staged data. */
} mdl_action_t;

typedef enum
{
    MDL_SYNC_ACK = 0,        /**< Single byte command to acknowledge errorless reception of data. */
    MDL_SYNC_COMMIT = 1,     /**< Single byte command to commit previously staged data. */
    MDL_SYNC_RESERVED_1 = 2, /**< Reserved for future use. */
    MDL_SYNC_RESERVED_2 = 3, /**< Reserved for future use. */
} mdl_sync_type_t;

// clang-format off
#define MDL_SYNC_ERROR_GENERATOR(GENERATOR)                                                                       \
    GENERATOR(MDL_SYNC_ERR_NONE          = 0,        "No Error")                                                  \
    GENERATOR(MDL_SYNC_ERR_OTHER         = (1 << 0), "Others")                                                    \
    GENERATOR(MDL_SYNC_ERR_TRANSMISSION  = (1 << 1), "Transmission")                                              \
    GENERATOR(MDL_SYNC_ERR_HANDLER       = (1 << 2), "Property Handler")                                          \
    GENERATOR(MDL_SYNC_ERR_NOT_SUPPORTED = (1 << 3), "Property Not Supported")

#define MDL_NODE_STATE_GENERATOR(GENERATOR)                                                                       \
    GENERATOR(MDL_NODE_STATE_UNDEFINED,    "Undefined")                                                           \
    GENERATOR(MDL_NODE_STATE_RX_HEADER,    "Receive Header")                                                      \
    GENERATOR(MDL_NODE_STATE_ERROR,        "Error")                                                               \
    GENERATOR(MDL_NODE_STATE_DEC_NODE_CNT, "Decrement Node Cnt")                                                  \
    GENERATOR(MDL_NODE_STATE_TX_PROP,      "Transmit Property Data")                                              \
    GENERATOR(MDL_NODE_STATE_RX_PROP,      "Receive Property Data")                                               \
    GENERATOR(MDL_NODE_STATE_COMMIT_PROP,  "Commit Property Data")

#define MDL_NODE_ERROR_GENERATOR(GENERATOR)                                                                       \
    GENERATOR(MDL_NODE_ERR_NONE = 0,            MDL_SYNC_ERR_NONE,          "None")                                \
    GENERATOR(MDL_NODE_ERR_INVALID_STATE,       MDL_SYNC_ERR_OTHER,         "Invalid State")                       \
    GENERATOR(MDL_NODE_ERR_HEADER_PARITY,       MDL_SYNC_ERR_TRANSMISSION,  "Header Parity")                       \
    GENERATOR(MDL_NODE_ERR_COBS_ENC,            MDL_SYNC_ERR_TRANSMISSION,  "COBS Encoding")                       \
    GENERATOR(MDL_NODE_ERR_COBS_DEC,            MDL_SYNC_ERR_TRANSMISSION,  "COBS Decoding")                       \
    GENERATOR(MDL_NODE_ERR_CHECKSUM,            MDL_SYNC_ERR_TRANSMISSION,  "Checksum")                            \
    GENERATOR(MDL_NODE_ERR_TIMEOUT,             MDL_SYNC_ERR_TRANSMISSION,  "Timeout")                             \
    GENERATOR(MDL_NODE_ERR_WRITE_CB,            MDL_SYNC_ERR_HANDLER,       "Write Callback")                      \
    GENERATOR(MDL_NODE_ERR_READ_CB,             MDL_SYNC_ERR_HANDLER,       "Read Callback")                       \
    GENERATOR(MDL_NODE_ERR_READ_NOT_SUPPORTED,  MDL_SYNC_ERR_NOT_SUPPORTED, "Read Not Supported")                  \
    GENERATOR(MDL_NODE_ERR_WRITE_NOT_SUPPORTED, MDL_SYNC_ERR_NOT_SUPPORTED, "Write Not Supported")                 \
    GENERATOR(MDL_NODE_ERR_PROP_NOT_SUPPORTED,  MDL_SYNC_ERR_NOT_SUPPORTED, "Property Not Supported")
// clang-format on

typedef enum
{
    MDL_SYNC_ERROR_GENERATOR(GENERATE_1ST_FIELD)
} mdl_sync_error_code_t;
typedef enum
{
    MDL_NODE_STATE_GENERATOR(GENERATE_1ST_FIELD)
} mdl_node_state_t;
typedef enum
{
    MDL_NODE_ERROR_GENERATOR(GENERATE_1ST_FIELD)
} mdl_node_err_t;

/**
 * @brief Madelink property id type.
 */
typedef int16_t mdl_prop_id_t;

/**
 * @brief Madelink property handler.
 */
typedef struct
{
    mdl_prop_handler_cb_t get; /**< Read the property from the node or buffer into a buffer. */
    mdl_prop_handler_cb_t set; /**< Write the property to the node or buffer from a buffer. */
#ifdef MDL_HANDLER_ALTERNATE_ENABLE
    mdl_prop_handler_alt_cb_t get_alt; /**< Alternate read handler using a data structure. */
    mdl_prop_handler_alt_cb_t set_alt; /**< Alternate write handler using a data structure. */
    mdl_prop_compare_cb_t compare;     /**< Compare two property data structures. */
#endif
} mdl_prop_handler_t;

/**
 * @brief Madelink property attributes.
 */
typedef struct
{
    char *name; /**< Name of the property. */
} mdl_prop_attr_t;

/**
 * @brief Madelink property structure.
 */
typedef struct
{
    mdl_prop_handler_t handler; /**< Property handler functions. */
    mdl_prop_attr_t attribute;  /**< Property attributes. */
} mdl_prop_t;

/**
 * @brief Bit field definitions for the message header.
 *
 * The header can be one of two types: Action header or Sync header.
 * The type is determined by the first two bits (ACTION field).
 * @code
 * Action header (3 bytes):
 * 23                                 16 15            8 7                    0
 * +-------------+----------------------+---------------+---------------------+
 * |            BYTE 0                  |    BYTE 1     |        BYTE 2       |
 * +-------------+----------------------+-----+---------+-------+-------------+
 * | ACTION (2b) | STAGE (1b) | PROPERTY (6b) |  NODE_CNT (13b) | PARITY (1b) |
 * |             | ERROR (1b) | N/A           |                 |             |
 * +-------------+------------+---------------+-----------------+-------------+
 * 23          22 21        21 20           14 13              1 0            0
 *
 * Sync header (1 byte):
 * 7                                                0
 * +------------------------------------------------+
 * |                      BYTE 0                    |
 * +-------------+----------------+-----------------+
 * | ACTION (2b) | SYNC_TYPE (2b) | ERROR_CODE (4b) |
 * +-------------+----------------+-----------------+
 * 7            6 5              4 3                0
 * \endcode
 * @note
 * The node count is split into 7 MSB bits and 6 LSB bits. This is done because we must always transmit the node count
 * LSB before the MSB.
 */
typedef struct
{
    uint8_t raw[MDL_ACTION_HEADER_SIZE]; /* Raw access to the header bytes. */
} mdl_msg_header_t;

/**
 * @brief Encode a payload using COBS encoding.
 *
 * @param[out] dst Pointer to the destination buffer.
 * @param[inout] dst_len Pointer to the length of the destination buffer. Will be updated with the actual length used.
 * @param[in] src Pointer to the source buffer.
 * @param[in] src_len Length of the source buffer.
 *
 * @return true if encoding was successful, false otherwise.
 */
bool mdl_payload_cobs_encode(uint8_t *dst, size_t *dst_len, const uint8_t *src, size_t src_len);

/**
 * @brief Decode a payload using COBS encoding.
 *
 * @param[out] dst Pointer to the destination buffer.
 * @param[inout] dst_len Pointer to the length of the destination buffer. Will be updated with the actual length used.
 * @param[in] src Pointer to the source buffer.
 * @param[in] src_len Length of the source buffer.
 *
 * @return true if decoding was successful, false otherwise.
 */
bool mdl_payload_cobs_decode(uint8_t *dst, size_t *dst_len, const uint8_t *src, size_t src_len);

/**
 * @brief Calculate the checksum of a data buffer.
 * Calculates an 8 bit checksum of an array of data. If the array is terminated with the checksum, the result will be
 * 0xFF.
 *
 * @param[in] data Pointer to the data buffer.
 * @param[in] size Size of the data buffer.
 *
 * @return The calculated checksum.
 */
uint8_t mdl_checksum_calculate(const uint8_t *data, size_t size);

/**
 * @brief Extract the action type from a message header.
 *
 * @param[in] header The message header to extract the action type from.
 *
 * @return The action type.
 */
mdl_action_t mdl_header_action_get(mdl_msg_header_t header);

/**
 * @brief Set the action type in a message header.
 *
 * @param[inout] header Pointer to the message header to set the action type for.
 * @param[in] action The action type to set.
 */
void mdl_header_action_set(mdl_msg_header_t *header, mdl_action_t action);

/**
 * @brief Extract the stage bit from a message header.
 *
 * @param[in] header The message header to extract the stage bit from.
 *
 * @return The stage bit.
 */
bool mdl_header_staging_bit_get(mdl_msg_header_t header);

/**
 * @brief Set the stage bit in a message header.
 *
 * @param[inout] header Pointer to the message header to set the stage bit for.
 * @param[in] stage The stage value to set (true for stage, false for no stage).
 */
void mdl_header_staging_bit_set(mdl_msg_header_t *header, bool stage);

/**
 * @brief Extract the read_error bit from a message header.
 *
 * @param[in] header The message header to extract the read_error bit from.
 *
 * @return The read_error bit.
 */
static inline bool mdl_header_read_error_bit_get(mdl_msg_header_t header)
{
    return mdl_header_staging_bit_get(header);
}

/**
 * @brief Set the read_error bit in a message header.
 *
 * @param[inout] header Pointer to the message header to set the read_error bit for.
 * @param[in] read_error The read_error value to set (true for read_error, false for no read_error).
 */
static inline void mdl_header_read_error_bit_set(mdl_msg_header_t *header, bool read_error)
{
    mdl_header_staging_bit_set(header, read_error);
}

/**
 * @brief Extract the property id from a message header.
 *
 * @param[in] header The message header to extract the property id from.
 *
 * @return The property id.
 */
mdl_prop_id_t mdl_header_property_get(mdl_msg_header_t header);

/**
 * @brief Set the property id in a message header.
 *
 * @param[inout] header Pointer to the message header to set the property id for.
 * @param[in] property The property id to set.
 */
void mdl_header_property_set(mdl_msg_header_t *header, mdl_prop_id_t property);

/**
 * @brief Extract the node count from a message header.
 *
 * @param[in] header The message header to extract the node count from.
 *
 * @return The node count.
 */
uint16_t mdl_header_node_cnt_get(mdl_msg_header_t header);

/**
 * @brief Set the node count in a message header.
 *
 * @param[inout] header Pointer to the message header to set the node count for.
 * @param[in] node_cnt The node count to set.
 */
void mdl_header_node_cnt_set(mdl_msg_header_t *header, uint16_t node_cnt);

/**
 * @brief Verify the parity of a message header.
 *
 * @param[in] header The message header to verify.
 *
 * @return true if the parity is valid, false otherwise.
 */
bool mdl_header_parity_check(mdl_msg_header_t header);

/**
 * @brief Set the parity bit in a message header.
 *
 * @param[inout] header Pointer to the message header to set the parity bit for.
 * @param[in] valid The parity value to set (true for valid, false for invalid).
 */
void mdl_header_parity_set(mdl_msg_header_t *header, bool valid);

/**
 * @brief Extract the sync type from a message header.
 *
 * @param[in] header The message header to extract the sync type from.
 *
 * @return The sync type.
 */
mdl_sync_type_t mdl_header_sync_type_get(mdl_msg_header_t header);

/**
 * @brief Set the sync type in a message header.
 *
 * @param[inout] header Pointer to the message header to set the sync type for.
 * @param[in] sync_type The sync type to set.
 */
void mdl_header_sync_type_set(mdl_msg_header_t *header, mdl_sync_type_t sync_type);

/**
 * @brief Extract the sync error code from a message header.
 *
 * @param[in] header The message header to extract the sync error code from.
 *
 * @return The sync error code.
 */
mdl_sync_error_code_t mdl_header_sync_error_get(mdl_msg_header_t header);

/**
 * @brief Set the sync error code in a message header.
 *
 * @param[inout] header Pointer to the message header to set the sync error code for.
 * @param[in] error_code The sync error code to set.
 */
void mdl_header_sync_error_set(mdl_msg_header_t *header, mdl_sync_error_code_t error_code);

/**
 * @brief Add a sync error code in a message header by logical OR.
 *
 * @param[inout] header Pointer to the message header to set the sync error code for.
 * @param[in] error_code The sync error code to set.
 */
void mdl_header_sync_error_add(mdl_msg_header_t *header, mdl_sync_error_code_t error_code);

/**
 * @brief Convert a node state to a string.
 *
 * @note unsafe if state is out of bounds!
 *
 * @param[in] state The node state to convert.
 *
 * @return A string representation of the node state.
 */
char *mdl_node_state_to_str(mdl_node_state_t state);

/**
 * @brief Convert a node error to a string.
 *
 * @note unsafe if error is out of bounds!
 *
 * @param[in] error The node error to convert.
 *
 * @return A string representation of the node error.
 */
char *mdl_node_error_to_str(mdl_node_err_t error);