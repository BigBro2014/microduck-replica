#include "control_table.h"
#include "board.h"
#include "imu.h"
#include <string.h>

static uint8_t user_marker; /* Address 65: RAM marker, there is no LED on this PCB. */
static void put16(uint8_t *p, uint16_t value) { p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8); }
static void put32(uint8_t *p, uint32_t value)
{ for (unsigned i = 0; i < 4u; ++i) p[i] = (uint8_t)(value >> (8u * i)); }

uint8_t control_table_read(void *user, uint16_t address, uint16_t length, uint8_t *out)
{
    proto_context *protocol = (proto_context *)user;
    uint8_t table[256] = {0};
    imu_snapshot_t s;
    if (!out || !length || address >= 256u || length > 256u - address) return PROTO_ERR_RANGE;
    /* Both acquisition and this callback run in main, so this is an untorn snapshot. */
    imu_get_snapshot(&s);
    table[65] = user_marker;
    for (unsigned i = 0; i < 3u; ++i) {
        put16(&table[124u + i * 2u], (uint16_t)s.gyro[i]);
        put16(&table[130u + i * 2u], s.quat_fp16[i]);
        put16(&table[136u + i * 2u], (uint16_t)s.raw_accel[i]);
    }
    /* 136..143 is OUR diagnostic extension, not claimed upstream ABI. */
    table[142] = (uint8_t)s.sample_count;
    table[143] = (s.ready ? 1u : 0u) | (s.configured ? 2u : 0u) | (s.error ? 4u : 0u);
    table[144] = s.who_am_i; table[145] = s.error;
    table[146] = board_diagnostics.clock_status;
    table[147] = BOARD_ENABLE_WATCHDOG;
    put32(&table[148], s.sample_count);
    put32(&table[152], s.gyro_count);
    put32(&table[156], s.accel_count);
    put32(&table[160], board_millis());
    put32(&table[164], board_millis() - s.last_gyro_ms);
    put32(&table[168], board_millis() - s.last_quat_ms);
    put32(&table[172], s.spi_errors);
    put32(&table[176], s.fifo_overflows);
    put32(&table[180], s.invalid_quaternions);
    put32(&table[184], s.recovery_count);
    put32(&table[188], board_diagnostics.rx_overflows);
    put32(&table[192], board_diagnostics.uart_errors);
    put32(&table[196], protocol->crc_errors);
    put32(&table[200], protocol->malformed_packets);
    put32(&table[204], protocol->receive_timeouts);
    put32(&table[208], protocol->rx_packets);
    put32(&table[212], board_diagnostics.tx_packets);
    put32(&table[216], protocol->cancelled_replies);
    put32(&table[220], board_diagnostics.reset_flags);
    put32(&table[224], board_diagnostics.clock_hz);
    put32(&table[228], board_diagnostics.tx_timeouts);
    put32(&table[232], board_diagnostics.log_dropped);
    table[236] = 1u; /* Diagnostic map revision. */
    put32(&table[244], board_diagnostics.tx_busy_drops);
    memcpy(out, &table[address], length);
    return PROTO_OK;
}

uint8_t control_table_write(void *user, uint16_t address, uint16_t length, const uint8_t *data)
{
    (void)user;
    if (!data || length != 1u) return PROTO_ERR_ACCESS;
    if (address == 65u) { user_marker = data[0]; return PROTO_OK; }
    if (address == 240u && data[0] == 0xA5u) { imu_request_reinit(); return PROTO_OK; }
    return PROTO_ERR_ACCESS;
}
