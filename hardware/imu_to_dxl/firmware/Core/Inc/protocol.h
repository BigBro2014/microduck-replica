#ifndef IMU_TO_DXL_PROTOCOL_H
#define IMU_TO_DXL_PROTOCOL_H

#include <stdint.h>

/* Project-specific Dynamixel Protocol 2.0 slave, not a ROBOTIS servo. */
#define PROTO_DEVICE_ID             200u
#define PROTO_MODEL_NUMBER          0x4D44u
#define PROTO_FIRMWARE_VERSION      1u
#define PROTO_PACKET_CAPACITY       512u
#define PROTO_MAX_READ              256u
#define PROTO_BYTE_TIMEOUT_US       1500u
#define PROTO_TRANSACTION_TIMEOUT_US 25000u
#define PROTO_REPLY_EXPIRY_US       2000u

enum {
    PROTO_OK = 0,
    PROTO_ERR_RESULT = 1,
    PROTO_ERR_INSTRUCTION = 2,
    PROTO_ERR_CRC = 3,
    PROTO_ERR_RANGE = 4,
    PROTO_ERR_LENGTH = 5,
    PROTO_ERR_LIMIT = 6,
    PROTO_ERR_ACCESS = 7
};

/* Callbacks return a protocol error code. read must fill the complete requested
 * range atomically; write must validate the WHOLE range before changing data.
 * Protocol-owned addresses: 0..9 (identity/configuration) and 68 (return level).
 * Application read callbacks may receive subranges around these addresses.
 * Application write callbacks never receive a range touching owned addresses.
 * send is synchronous: send every byte, wait for UART TC (not TXE), release
 * the driver, re-enable RX, then return. No callbacks run in an ISR. */
typedef uint8_t (*proto_read_fn)(void *user, uint16_t address,
                              uint16_t length, uint8_t *out);
typedef uint8_t (*proto_write_fn)(void *user, uint16_t address,
                               uint16_t length, const uint8_t *data);
typedef void (*proto_send_fn)(void *user, const uint8_t *packet, uint16_t length);

typedef struct {
    uint8_t rx[PROTO_PACKET_CAPACITY];
    uint8_t tx[PROTO_PACKET_CAPACITY];
    proto_read_fn read;
    proto_write_fn write;
    proto_send_fn send;
    void *user;
    uint32_t last_byte_us;
    uint32_t transaction_start_us;
    uint32_t reply_due_us;
    uint32_t rx_packets;
    uint32_t tx_packets;
    uint32_t crc_errors;
    uint32_t malformed_packets;
    uint32_t receive_timeouts;
    uint32_t cancelled_replies;
    uint16_t rx_length;
    uint16_t rx_expected;
    uint16_t tx_length;
    uint8_t pending;             /* 0 none, 1 ready/delay, 2 preceding ID */
    uint8_t preceding_id;
    uint8_t return_delay;        /* Address 9, units of 2 us; RAM only. */
    uint8_t status_return_level; /* Address 68: 0 ping, 1 reads, 2 all. */
} proto_context;

void proto_init(proto_context *ctx, proto_read_fn read_registers,
                proto_write_fn write_registers, proto_send_fn send, void *user);
/* Unsigned 32-bit MICROSECOND timestamps; wraparound is supported. Pass the
 * actual receive timestamp when draining an ISR ring, not the later drain time.
 * Drain RX before proto_poll, and poll frequently (target <100 us).
 * Sync/Bulk Read require observing the immediately preceding listed ID's valid
 * status packet. If it is absent, this slave cancels instead of colliding.
 * Keep ID 200 FIRST in the controller's Sync Read list, as upstream bus.rs does.
 * Broadcast Ping is deliberately silent: only addressed Ping is supported.
 * No Feetech/Protocol 1, Fast Sync Read, Reg Write, reboot, or flash writes. */
void proto_feed(proto_context *ctx, uint8_t byte, uint32_t now_us);
void proto_poll(proto_context *ctx, uint32_t now_us);
/* Discard partial RX/pending TX after UART overrun or a board-level bus fault. */
void proto_reset_receiver(proto_context *ctx);
uint16_t proto_crc16(const uint8_t *bytes, uint16_t length);

#endif
