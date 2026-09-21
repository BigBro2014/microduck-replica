#include "protocol.h"
#include <string.h>

/* Wire format reference, accessed 2026-09-15:
 * https://emanual.robotis.com/docs/en/dxl/protocol2/
 * Independent implementation; no vendor protocol source is incorporated. */

#define BROADCAST_ID 254u
#define STATUS_INSTRUCTION 0x55u

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

uint16_t proto_crc16(const uint8_t *bytes, uint16_t length)
{
    uint16_t crc = 0u;
    uint16_t i;
    uint8_t bit;
    for (i = 0u; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)bytes[i] << 8);
        for (bit = 0u; bit < 8u; ++bit) {
            crc = (uint16_t)((crc & 0x8000u) != 0u
                ? ((uint32_t)crc << 1) ^ 0x8005u : (uint32_t)crc << 1);
        }
    }
    return crc;
}

static void cancel_reply(proto_context *ctx)
{
    if (ctx->pending != 0u) {
        ++ctx->cancelled_replies;
    }
    ctx->pending = 0u;
    ctx->tx_length = 0u;
}

void proto_reset_receiver(proto_context *ctx)
{
    ctx->rx_length = 0u;
    ctx->rx_expected = 0u;
    cancel_reply(ctx);
}

void proto_init(proto_context *ctx, proto_read_fn read_registers,
                proto_write_fn write_registers, proto_send_fn send, void *user)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->read = read_registers;
    ctx->write = write_registers;
    ctx->send = send;
    ctx->user = user;
    ctx->status_return_level = 2u;
}

static int owned(uint16_t address)
{
    return address <= 9u || address == 68u;
}

static uint8_t owned_value(const proto_context *ctx, uint16_t address)
{
    switch (address) {
    case 0u: return (uint8_t)(PROTO_MODEL_NUMBER & 0xFFu);
    case 1u: return (uint8_t)(PROTO_MODEL_NUMBER >> 8);
    case 6u: return PROTO_FIRMWARE_VERSION;
    case 7u: return PROTO_DEVICE_ID;
    case 8u: return 3u; /* Dynamixel baud selector 3 = 1,000,000 bit/s. */
    case 9u: return ctx->return_delay;
    case 68u: return ctx->status_return_level;
    default: return 0u; /* Model information 2..5 is reserved for this board. */
    }
}

static uint8_t read_table(proto_context *ctx, uint16_t address,
                          uint16_t length, uint8_t *out)
{
    uint16_t offset = 0u;
    if (length == 0u || length > PROTO_MAX_READ) {
        return PROTO_ERR_LENGTH;
    }
    if ((uint32_t)address + length > 65536u) {
        return PROTO_ERR_ACCESS;
    }
    while (offset < length) {
        uint16_t here = (uint16_t)(address + offset);
        if (owned(here)) {
            out[offset++] = owned_value(ctx, here);
        } else {
            uint16_t run = 1u;
            uint8_t error;
            while ((uint16_t)(offset + run) < length &&
                   !owned((uint16_t)(here + run))) {
                ++run;
            }
            if (ctx->read == 0) {
                return PROTO_ERR_ACCESS;
            }
            error = ctx->read(ctx->user, here, run, out + offset);
            if (error != PROTO_OK) {
                return error;
            }
            offset = (uint16_t)(offset + run);
        }
    }
    return PROTO_OK;
}

static uint8_t write_table(proto_context *ctx, uint16_t address,
                           uint16_t length, const uint8_t *data)
{
    uint16_t i;
    if (length == 0u) {
        return PROTO_ERR_LENGTH;
    }
    if ((uint32_t)address + length > 65536u) {
        return PROTO_ERR_ACCESS;
    }
    /* Restrict protocol settings to single-byte writes. This avoids a partial
     * write when a request crosses a read-only field or an application range. */
    if (address == 9u && length == 1u) {
        if (data[0] > 254u) {
            return PROTO_ERR_RANGE;
        }
        ctx->return_delay = data[0];
        return PROTO_OK;
    }
    if (address == 68u && length == 1u) {
        if (data[0] > 2u) {
            return PROTO_ERR_RANGE;
        }
        ctx->status_return_level = data[0];
        return PROTO_OK;
    }
    for (i = 0u; i < length; ++i) {
        if (owned((uint16_t)(address + i))) {
            return PROTO_ERR_ACCESS;
        }
    }
    return ctx->write != 0
        ? ctx->write(ctx->user, address, length, data) : PROTO_ERR_ACCESS;
}

/* Caller places payload at tx[9]. Stuff in place; no second payload buffer. */
static void make_status(proto_context *ctx, uint8_t error, uint16_t data_length)
{
    uint16_t end = (uint16_t)(9u + data_length);
    uint16_t i;
    uint16_t crc;
    ctx->tx[0] = 0xFFu;
    ctx->tx[1] = 0xFFu;
    ctx->tx[2] = 0xFDu;
    ctx->tx[3] = 0u;
    ctx->tx[4] = PROTO_DEVICE_ID;
    ctx->tx[7] = STATUS_INSTRUCTION;
    ctx->tx[8] = error;
    for (i = 9u; i < end; ++i) {
        if (ctx->tx[i - 2u] == 0xFFu && ctx->tx[i - 1u] == 0xFFu &&
            ctx->tx[i] == 0xFDu) {
            /* PROTO_MAX_READ=256 guarantees <= 355 bytes including stuffing. */
            memmove(ctx->tx + i + 2u, ctx->tx + i + 1u,
                    (size_t)(end - i - 1u));
            ctx->tx[++i] = 0xFDu;
            ++end;
        }
    }
    ctx->tx[5] = (uint8_t)((end - 5u) & 0xFFu);
    ctx->tx[6] = (uint8_t)((end - 5u) >> 8);
    crc = proto_crc16(ctx->tx, end);
    ctx->tx[end++] = (uint8_t)(crc & 0xFFu);
    ctx->tx[end++] = (uint8_t)(crc >> 8);
    ctx->tx_length = end;
}

static void schedule_reply(proto_context *ctx, uint32_t now_us, int preceding)
{
    ctx->transaction_start_us = now_us;
    ctx->reply_due_us = now_us + (uint32_t)ctx->return_delay * 2u;
    ctx->pending = preceding < 0 ? 1u : 2u;
    ctx->preceding_id = (uint8_t)preceding;
}

static void reply_error(proto_context *ctx, uint8_t error, uint32_t now_us)
{
    make_status(ctx, error, 0u);
    schedule_reply(ctx, now_us, -1);
}

static void reply_read(proto_context *ctx, uint16_t address, uint16_t length,
                       uint32_t now_us, int preceding)
{
    uint8_t error = read_table(ctx, address, length, ctx->tx + 9u);
    make_status(ctx, error, error == PROTO_OK ? length : 0u);
    schedule_reply(ctx, now_us, preceding);
}

/* All participating IDs must be unique, real device IDs. No device is allowed
 * to start a response for a malformed list, even when its own entry is valid. */
static int valid_id_list(const uint8_t *parameters, uint16_t length,
                         uint16_t start, uint16_t stride)
{
    uint16_t i;
    uint16_t j;
    if (length <= start || stride == 0u || (length - start) % stride != 0u) {
        return 0;
    }
    for (i = start; i < length; i = (uint16_t)(i + stride)) {
        if (parameters[i] > 252u) {
            return 0;
        }
        for (j = start; j < i; j = (uint16_t)(j + stride)) {
            if (parameters[i] == parameters[j]) {
                return 0;
            }
        }
    }
    return 1;
}

static void execute(proto_context *ctx, uint16_t body_length, uint32_t now_us)
{
    uint8_t id = ctx->rx[4];
    uint8_t instruction = ctx->rx[7];
    uint8_t *p = ctx->rx + 8u;
    uint16_t n = (uint16_t)(body_length - 1u);
    uint16_t i;
    uint16_t length;
    uint8_t error;
    uint8_t previous_level;
    if (instruction == STATUS_INSTRUCTION) {
        if (n >= 1u && id <= 252u && ctx->pending == 2u &&
            id == ctx->preceding_id &&
            (uint32_t)(now_us - ctx->transaction_start_us) <
                PROTO_TRANSACTION_TIMEOUT_US) {
            ctx->pending = 1u;
            ctx->reply_due_us = now_us + (uint32_t)ctx->return_delay * 2u;
        }
        return;
    }
    /* Every new valid controller instruction ends the earlier transaction,
     * including instructions addressed only to another device. */
    cancel_reply(ctx);
    if (id != PROTO_DEVICE_ID && id != BROADCAST_ID) {
        return;
    }
    switch (instruction) {
    case 0x01u: /* PING: addressed only, always responds. */
        if (id == BROADCAST_ID) {
            return;
        }
        if (n != 0u) {
            reply_error(ctx, PROTO_ERR_LENGTH, now_us);
            return;
        }
        ctx->tx[9] = (uint8_t)(PROTO_MODEL_NUMBER & 0xFFu);
        ctx->tx[10] = (uint8_t)(PROTO_MODEL_NUMBER >> 8);
        ctx->tx[11] = PROTO_FIRMWARE_VERSION;
        make_status(ctx, PROTO_OK, 3u);
        schedule_reply(ctx, now_us, -1);
        break;
    case 0x02u: /* READ never executes on broadcast ID. */
        if (id == BROADCAST_ID || ctx->status_return_level < 1u) {
            return;
        }
        if (n != 4u) {
            reply_error(ctx, PROTO_ERR_LENGTH, now_us);
        } else {
            reply_read(ctx, le16(p), le16(p + 2u), now_us, -1);
        }
        break;
    case 0x03u: /* WRITE; broadcast writes execute but never reply. */
        previous_level = ctx->status_return_level;
        error = n >= 3u ? write_table(ctx, le16(p), (uint16_t)(n - 2u), p + 2u)
                         : PROTO_ERR_LENGTH;
        /* A successful write changing level gets the old level's final ACK. */
        if (id != BROADCAST_ID && previous_level == 2u) {
            reply_error(ctx, error, now_us);
        }
        break;
    case 0x82u: /* SYNC READ */
        if (id != BROADCAST_ID || ctx->status_return_level < 1u) {
            return;
        }
        if (n < 5u || !valid_id_list(p, n, 4u, 1u)) {
            ++ctx->malformed_packets;
            return;
        }
        for (i = 4u; i < n; ++i) {
            if (p[i] == PROTO_DEVICE_ID) {
                reply_read(ctx, le16(p), le16(p + 2u), now_us,
                           i == 4u ? -1 : (int)p[i - 1u]);
                return;
            }
        }
        break;
    case 0x83u: /* SYNC WRITE */
        if (id != BROADCAST_ID || n < 5u) {
            return;
        }
        length = le16(p + 2u);
        if (length == 0u || length >= n ||
            !valid_id_list(p, n, 4u, (uint16_t)(length + 1u))) {
            ++ctx->malformed_packets;
            return;
        }
        for (i = 4u; i < n; i = (uint16_t)(i + length + 1u)) {
            if (p[i] == PROTO_DEVICE_ID) {
                (void)write_table(ctx, le16(p), length, p + i + 1u);
                return;
            }
        }
        break;
    case 0x92u: /* BULK READ */
        if (id != BROADCAST_ID || ctx->status_return_level < 1u) {
            return;
        }
        if (!valid_id_list(p, n, 0u, 5u)) {
            ++ctx->malformed_packets;
            return;
        }
        for (i = 0u; i < n; i = (uint16_t)(i + 5u)) {
            if (p[i] == PROTO_DEVICE_ID) {
                reply_read(ctx, le16(p + i + 1u), le16(p + i + 3u), now_us,
                           i == 0u ? -1 : (int)p[i - 5u]);
                return;
            }
        }
        break;
    default:
        if (id == PROTO_DEVICE_ID && ctx->status_return_level == 2u) {
            reply_error(ctx, PROTO_ERR_INSTRUCTION, now_us);
        }
        break;
    }
}

static int complete_packet(proto_context *ctx, uint32_t now_us)
{
    uint16_t end = (uint16_t)(ctx->rx_expected - 2u);
    uint16_t source;
    uint16_t destination = 7u;
    if (proto_crc16(ctx->rx, end) != le16(ctx->rx + end)) {
        ++ctx->crc_errors;
        return 0;
    }
    /* CRC is checked BEFORE removing stuffing. Reject noncanonical, unstuffed
     * FF FF FD payload, even if an attacker recomputes a valid CRC for it. */
    for (source = 7u; source < end; ++source) {
        uint8_t byte = ctx->rx[source];
        int stuff = destination >= 9u && ctx->rx[destination - 2u] == 0xFFu &&
                    ctx->rx[destination - 1u] == 0xFFu && byte == 0xFDu;
        ctx->rx[destination++] = byte;
        if (stuff) {
            if ((uint16_t)(source + 1u) >= end || ctx->rx[source + 1u] != 0xFDu) {
                ++ctx->malformed_packets;
                return 0;
            }
            ++source;
        }
    }
    ++ctx->rx_packets;
    execute(ctx, (uint16_t)(destination - 7u), now_us);
    return 1;
}

void proto_feed(proto_context *ctx, uint8_t byte, uint32_t now_us)
{
    static const uint8_t header[4] = {0xFFu, 0xFFu, 0xFDu, 0u};
    if (ctx->rx_length != 0u &&
        (uint32_t)(now_us - ctx->last_byte_us) > PROTO_BYTE_TIMEOUT_US) {
        ctx->rx_length = 0u;
        ctx->rx_expected = 0u;
        ++ctx->receive_timeouts;
    }
    /* A pending response loses its turn as soon as someone else uses the bus.
     * The application MUST drain RX before calling proto_poll. */
    if (ctx->pending == 1u) {
        cancel_reply(ctx);
    }
    ctx->last_byte_us = now_us;
    if (ctx->rx_length < 4u) {
        if (byte == header[ctx->rx_length]) {
            ctx->rx[ctx->rx_length++] = byte;
        } else if (byte == 0xFFu) {
            ctx->rx[0] = 0xFFu;
            ctx->rx[1] = 0xFFu;
            ctx->rx_length = ctx->rx_length >= 1u ? 2u : 1u;
        } else {
            ctx->rx_length = 0u;
        }
        return;
    }
    ctx->rx[ctx->rx_length++] = byte;
    if (ctx->rx_length == 5u && byte > 252u && byte != BROADCAST_ID) {
        ctx->rx_length = 0u;
        ++ctx->malformed_packets;
        return;
    }
    if (ctx->rx_length == 7u) {
        uint16_t wire_length = le16(ctx->rx + 5u);
        if (wire_length < 3u || wire_length > PROTO_PACKET_CAPACITY - 7u) {
            ctx->rx_length = 0u;
            ctx->rx_expected = 0u;
            ++ctx->malformed_packets;
            return;
        }
        ctx->rx_expected = (uint16_t)(wire_length + 7u);
    }
    /* A canonical stuffed body cannot contain this full unstuffed header.
     * Check only the body; arbitrary CRC bytes must not look like a header. */
    if (ctx->rx_length >= 11u && ctx->rx_length <= ctx->rx_expected - 2u &&
        memcmp(ctx->rx + ctx->rx_length - 4u, header, 4u) == 0) {
        memcpy(ctx->rx, header, 4u);
        ctx->rx_length = 4u;
        ctx->rx_expected = 0u;
        ++ctx->malformed_packets;
        return;
    }
    if (ctx->rx_expected != 0u && ctx->rx_length == ctx->rx_expected) {
        /* A falsely SHORT length may end halfway through the next header.
         * Preserve its longest matching suffix when rejecting the packet. */
        uint16_t suffix = 3u;
        while (suffix != 0u &&
               memcmp(ctx->rx + ctx->rx_length - suffix, header, suffix) != 0) {
            --suffix;
        }
        if (complete_packet(ctx, now_us)) {
            suffix = 0u;
        }
        memcpy(ctx->rx, header, suffix);
        ctx->rx_length = suffix;
        ctx->rx_expected = 0u;
    }
}

void proto_poll(proto_context *ctx, uint32_t now_us)
{
    if (ctx->rx_length != 0u &&
        (uint32_t)(now_us - ctx->last_byte_us) > PROTO_BYTE_TIMEOUT_US) {
        ctx->rx_length = 0u;
        ctx->rx_expected = 0u;
        ++ctx->receive_timeouts;
    }
    if (ctx->pending == 2u &&
        (uint32_t)(now_us - ctx->transaction_start_us) >=
            PROTO_TRANSACTION_TIMEOUT_US) {
        cancel_reply(ctx);
    }
    if (ctx->pending == 1u && (int32_t)(now_us - ctx->reply_due_us) >= 0) {
        uint16_t length = ctx->tx_length;
        if ((uint32_t)(now_us - ctx->reply_due_us) > PROTO_REPLY_EXPIRY_US ||
            ctx->rx_length != 0u) {
            cancel_reply(ctx);
            return;
        }
        ctx->pending = 0u;
        ctx->tx_length = 0u;
        if (ctx->send != 0) {
            ctx->send(ctx->user, ctx->tx, length);
            ++ctx->tx_packets;
        }
    }
}
