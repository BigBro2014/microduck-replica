#include "protocol.h"
#include <stdio.h>
#include <string.h>

static unsigned checks;
static unsigned failures;
#define CHECK(test) do { ++checks; if (!(test)) { ++failures; \
    printf("FAIL line %u: %s\n", (unsigned)__LINE__, #test); } } while (0)

typedef struct {
    proto_context protocol;
    uint8_t registers[320];
    uint8_t output[512];
    uint16_t output_length;
    unsigned sends;
    unsigned writes;
    uint32_t now;
} fixture;

static uint8_t read_regs(void *user, uint16_t address, uint16_t length, uint8_t *out)
{
    fixture *f = (fixture *)user;
    if ((uint32_t)address + length > sizeof(f->registers)) {
        return PROTO_ERR_ACCESS;
    }
    memcpy(out, f->registers + address, length);
    return PROTO_OK;
}

static uint8_t write_regs(void *user, uint16_t address, uint16_t length,
                          const uint8_t *data)
{
    fixture *f = (fixture *)user;
    if ((uint32_t)address + length > sizeof(f->registers)) {
        return PROTO_ERR_ACCESS;
    }
    memcpy(f->registers + address, data, length);
    ++f->writes;
    return PROTO_OK;
}

static void send_bytes(void *user, const uint8_t *packet, uint16_t length)
{
    fixture *f = (fixture *)user;
    CHECK(length <= sizeof(f->output));
    memcpy(f->output, packet, length);
    f->output_length = length;
    ++f->sends;
}

static void init(fixture *f)
{
    unsigned i;
    memset(f, 0, sizeof(*f));
    for (i = 0u; i < sizeof(f->registers); ++i) {
        f->registers[i] = (uint8_t)i;
    }
    proto_init(&f->protocol, read_regs, write_regs, send_bytes, f);
}

/* Test traffic encoder uses the known-vector-tested public CRC helper. The
 * protocol under test does not call this encoder. Status payloads are checked
 * against independently specified expected bytes, not an encoder round trip. */
static uint16_t packet(uint8_t *out, uint8_t id, uint8_t instruction,
                        const uint8_t *params, uint16_t count)
{
    uint16_t n = 7u;
    uint16_t i;
    uint16_t crc;
    out[0] = 0xFFu; out[1] = 0xFFu; out[2] = 0xFDu; out[3] = 0u; out[4] = id;
    for (i = 0u; i <= count; ++i) {
        uint8_t byte = i == 0u ? instruction : params[i - 1u];
        out[n++] = byte;
        if (n >= 10u && out[n-3u] == 0xFFu && out[n-2u] == 0xFFu && byte == 0xFDu) {
            out[n++] = 0xFDu;
        }
    }
    out[5] = (uint8_t)(n - 5u); out[6] = (uint8_t)((n - 5u) >> 8);
    crc = proto_crc16(out, n);
    out[n++] = (uint8_t)crc; out[n++] = (uint8_t)(crc >> 8);
    return n;
}

static void receive(fixture *f, const uint8_t *data, uint16_t length)
{
    uint16_t i;
    for (i = 0u; i < length; ++i) {
        f->now += 10u;
        proto_feed(&f->protocol, data[i], f->now);
    }
}

static void request(fixture *f, uint8_t id, uint8_t instruction,
                     const uint8_t *params, uint16_t count)
{
    uint8_t wire[512];
    uint16_t length = packet(wire, id, instruction, params, count);
    receive(f, wire, length);
}

static void poll(fixture *f, uint32_t elapsed)
{
    f->now += elapsed;
    proto_poll(&f->protocol, f->now);
}

static void test_crc_reference(void)
{
    /* ROBOTIS eManual exact Ping, Read and Sync Read example vectors. */
    static const uint8_t ping[] = {255,255,253,0,1,3,0,1};
    static const uint8_t broadcast_ping[] = {255,255,253,0,254,3,0,1};
    static const uint8_t read[] = {255,255,253,0,1,7,0,2,132,0,4,0};
    static const uint8_t sync[] = {255,255,253,0,254,9,0,130,132,0,4,0,1,2};
    static const uint8_t status[] = {255,255,253,0,1,7,0,85,0,6,4,38};
    CHECK(proto_crc16(ping, sizeof(ping)) == 0x4E19u);
    CHECK(proto_crc16(broadcast_ping, sizeof(broadcast_ping)) == 0x4231u);
    CHECK(proto_crc16(read, sizeof(read)) == 0x151Du);
    CHECK(proto_crc16(sync, sizeof(sync)) == 0xFACEu);
    CHECK(proto_crc16(status, sizeof(status)) == 0x5D65u);
}

static void test_ping_and_read(void)
{
    fixture f;
    const uint8_t read[] = {124,0,12,0};
    unsigned i;
    init(&f);
    request(&f, 200, 1, 0, 0);
    CHECK(f.sends == 0u); /* Feed never sends synchronously or from an ISR. */
    poll(&f, 0);
    CHECK(f.sends == 1u && f.output_length == 14u);
    CHECK(f.output[4] == 200 && f.output[7] == 85 && f.output[8] == 0);
    CHECK(f.output[9] == 0x44 && f.output[10] == 0x4D && f.output[11] == 1);
    CHECK(proto_crc16(f.output, 12u) == (uint16_t)(f.output[12] | f.output[13] << 8));
    request(&f, 200, 2, read, sizeof(read));
    poll(&f, 0);
    CHECK(f.sends == 2u && f.output_length == 23u);
    for (i = 0u; i < 12u; ++i) { CHECK(f.output[9u + i] == 124u + i); }
    request(&f, 199, 1, 0, 0); poll(&f, 0);
    request(&f, 254, 1, 0, 0); poll(&f, 0); /* Documented unsupported discovery. */
    request(&f, 254, 2, read, sizeof(read)); poll(&f, 0);
    CHECK(f.sends == 2u);
}

static void test_stuffing(void)
{
    fixture f;
    const uint8_t data[] = {80,0,255,255,253,253,255,255,255,253,0};
    const uint8_t read[] = {80,0,9,0};
    const uint8_t expected[] = {255,255,253,253,253,255,255,255,253,253,0};
    init(&f);
    request(&f, 200, 3, data, sizeof(data)); poll(&f, 0);
    CHECK(f.writes == 1u);
    CHECK(memcmp(f.registers + 80, data + 2, 9u) == 0);
    request(&f, 200, 2, read, sizeof(read)); poll(&f, 0);
    CHECK(f.output_length == 22u && f.output[5] == 15u);
    CHECK(memcmp(f.output + 9u, expected, sizeof(expected)) == 0);
    CHECK(proto_crc16(f.output, 20u) == (uint16_t)(f.output[20] | f.output[21] << 8));
    /* Unstuffed payload with valid CRC is not accepted. */
    {
        uint8_t bad[] = {255,255,253,0,200,8,0,3,80,0,255,255,253,0,0};
        uint16_t crc = proto_crc16(bad, 13u);
        bad[13] = (uint8_t)crc; bad[14] = (uint8_t)(crc >> 8);
        receive(&f, bad, sizeof(bad)); poll(&f, 0);
        CHECK(f.writes == 1u && f.protocol.malformed_packets != 0u);
    }
}

static void test_error_recovery(void)
{
    fixture f;
    uint8_t wire[512];
    uint16_t n;
    const uint8_t bad_length[] = {255,255,253,0,200,255,255};
    const uint8_t false_start[] = {255,255,253,0,200,60,0,2,99,11,12};
    const uint8_t false_short[] = {255,255,253,0,200,3,0};
    init(&f);
    n = packet(wire, 200, 1, 0, 0);
    wire[n - 1u] ^= 1u;
    receive(&f, wire, n); poll(&f, 0);
    CHECK(f.sends == 0u && f.protocol.crc_errors == 1u);
    receive(&f, bad_length, sizeof(bad_length));
    request(&f, 200, 1, 0, 0); poll(&f, 0);
    CHECK(f.sends == 1u);
    receive(&f, false_start, sizeof(false_start));
    request(&f, 200, 1, 0, 0); poll(&f, 0);
    CHECK(f.sends == 2u); /* Embedded new header resynchronizes corrupt length. */
    n = packet(wire, 200, 1, 0, 0);
    receive(&f, wire, 6u);
    poll(&f, 1501u);
    receive(&f, wire + 6u, (uint16_t)(n - 6u)); poll(&f, 0);
    CHECK(f.sends == 2u && f.protocol.receive_timeouts == 1u);
    request(&f, 200, 1, 0, 0); poll(&f, 0);
    CHECK(f.sends == 3u);
    /* Extra leading FF bytes are tolerated as header search noise. */
    proto_feed(&f.protocol, 255u, ++f.now);
    request(&f, 200, 1, 0, 0); poll(&f, 0);
    CHECK(f.sends == 4u);
    receive(&f, false_short, sizeof(false_short));
    request(&f, 200, 1, 0, 0); poll(&f, 0);
    CHECK(f.sends == 5u); /* A short false length may split the next header. */
}

static void test_register_protection(void)
{
    fixture f;
    const uint8_t id[] = {7,0,5};
    const uint8_t baud[] = {8,0,0};
    const uint8_t mixed[] = {67,0,99,0};
    const uint8_t range[] = {255,255,2,0};
    const uint8_t huge[] = {0,0,1,1};
    const uint8_t zero[] = {124,0,0,0};
    const uint8_t invalid_delay[] = {9,0,255};
    const uint8_t missing[] = {64,1,1,0};
    init(&f);
    request(&f, 200, 3, id, sizeof(id)); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_ACCESS && f.writes == 0u);
    request(&f, 200, 3, baud, sizeof(baud)); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_ACCESS);
    request(&f, 200, 3, mixed, sizeof(mixed)); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_ACCESS && f.registers[67] == 67u);
    request(&f, 200, 2, range, sizeof(range)); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_ACCESS && f.output_length == 11u);
    request(&f, 200, 2, huge, sizeof(huge)); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_LENGTH);
    request(&f, 200, 2, zero, sizeof(zero)); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_LENGTH);
    request(&f, 200, 3, invalid_delay, sizeof(invalid_delay)); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_RANGE && f.protocol.return_delay == 0u);
    request(&f, 200, 2, missing, sizeof(missing)); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_ACCESS);
    request(&f, 200, 0x8Au, 0, 0); poll(&f, 0);
    CHECK(f.output[8] == PROTO_ERR_INSTRUCTION);
}

static void test_return_level_and_delay(void)
{
    fixture f;
    const uint8_t quiet[] = {68,0,0};
    const uint8_t all[] = {68,0,2};
    const uint8_t delay[] = {9,0,100};
    const uint8_t read[] = {7,0,3,0};
    init(&f);
    request(&f, 200, 3, quiet, sizeof(quiet)); poll(&f, 0);
    CHECK(f.sends == 1u && f.protocol.status_return_level == 0u);
    request(&f, 200, 2, read, sizeof(read)); poll(&f, 0);
    CHECK(f.sends == 1u);
    request(&f, 200, 1, 0, 0); poll(&f, 0);
    CHECK(f.sends == 2u);
    request(&f, 200, 3, all, sizeof(all)); poll(&f, 0);
    CHECK(f.sends == 2u && f.protocol.status_return_level == 2u);
    request(&f, 200, 3, delay, sizeof(delay)); poll(&f, 199u);
    CHECK(f.sends == 2u);
    poll(&f, 1u); CHECK(f.sends == 3u);
    request(&f, 200, 2, read, sizeof(read)); poll(&f, 200u);
    CHECK(f.sends == 4u);
    CHECK(f.output[9] == 200u && f.output[10] == 3u && f.output[11] == 100u);
}

static void test_sync_read_order(void)
{
    fixture f;
    const uint8_t ours_first[] = {124,0,12,0,200,1,2};
    const uint8_t ours_second[] = {124,0,12,0,1,200,2};
    const uint8_t absent[] = {124,0,12,0,1,2};
    const uint8_t duplicate[] = {124,0,12,0,200,1,200};
    const uint8_t status[] = {0,1,2,3};
    uint8_t corrupt[512];
    uint16_t n;
    init(&f);
    request(&f, 254, 0x82u, absent, sizeof(absent)); poll(&f, 0);
    request(&f, 200, 0x82u, ours_first, sizeof(ours_first)); poll(&f, 0);
    request(&f, 254, 0x82u, duplicate, sizeof(duplicate)); poll(&f, 0);
    CHECK(f.sends == 0u);
    request(&f, 254, 0x82u, ours_first, sizeof(ours_first)); poll(&f, 0);
    CHECK(f.sends == 1u && f.output_length == 23u);
    request(&f, 254, 0x82u, ours_second, sizeof(ours_second)); poll(&f, 50u);
    CHECK(f.sends == 1u && f.protocol.pending == 2u);
    request(&f, 2, 0x55u, status, sizeof(status)); poll(&f, 0);
    CHECK(f.sends == 1u); /* Unrelated slave cannot release this turn. */
    n = packet(corrupt, 1, 0x55u, status, sizeof(status));
    corrupt[n - 1u] ^= 1u;
    receive(&f, corrupt, n); poll(&f, 0);
    CHECK(f.sends == 1u); /* Corrupt preceding status cannot release it. */
    request(&f, 1, 0x55u, status, sizeof(status)); poll(&f, 0);
    CHECK(f.sends == 2u && f.protocol.pending == 0u);
    request(&f, 254, 0x82u, ours_second, sizeof(ours_second));
    poll(&f, PROTO_TRANSACTION_TIMEOUT_US);
    request(&f, 1, 0x55u, status, sizeof(status)); poll(&f, 0);
    CHECK(f.sends == 2u); /* Late status never revives an expired read. */
    request(&f, 254, 0x82u, ours_second, sizeof(ours_second));
    request(&f, 9, 1, 0, 0); /* New controller instruction cancels waiting. */
    request(&f, 1, 0x55u, status, sizeof(status)); poll(&f, 0);
    CHECK(f.sends == 2u);
    request(&f, 254, 0x82u, ours_first, sizeof(ours_first));
    poll(&f, PROTO_REPLY_EXPIRY_US + 1u);
    CHECK(f.sends == 2u); /* Don't inject a delayed reply into the next cycle. */
}

static void test_sync_write_and_bulk(void)
{
    fixture f;
    const uint8_t write[] = {65,0,1,0,1,0,200,1};
    const uint8_t absent[] = {65,0,1,0,1,1,2,0};
    const uint8_t truncated[] = {65,0,2,0,200,1};
    const uint8_t duplicate[] = {65,0,1,0,200,1,200,0};
    const uint8_t bulk[] = {1,124,0,12,0,200,124,0,12,0};
    const uint8_t bulk_first[] = {200,124,0,12,0,1,144,0,2,0};
    const uint8_t malformed_bulk[] = {200,124,0,12};
    const uint8_t status[] = {0};
    init(&f);
    request(&f, 254, 0x83u, absent, sizeof(absent)); poll(&f, 0);
    request(&f, 254, 0x83u, truncated, sizeof(truncated)); poll(&f, 0);
    request(&f, 254, 0x83u, duplicate, sizeof(duplicate)); poll(&f, 0);
    CHECK(f.writes == 0u && f.sends == 0u);
    request(&f, 254, 0x83u, write, sizeof(write)); poll(&f, 0);
    CHECK(f.writes == 1u && f.sends == 0u && f.registers[65] == 1u);
    request(&f, 254, 0x92u, bulk, sizeof(bulk)); poll(&f, 0);
    CHECK(f.sends == 0u);
    request(&f, 1, 0x55u, status, sizeof(status)); poll(&f, 0);
    CHECK(f.sends == 1u && f.output_length == 23u);
    request(&f, 254, 0x92u, bulk_first, sizeof(bulk_first)); poll(&f, 0);
    CHECK(f.sends == 2u);
    request(&f, 254, 0x92u, malformed_bulk, sizeof(malformed_bulk)); poll(&f, 0);
    CHECK(f.sends == 2u);
}

static void test_wraparound_and_reset(void)
{
    fixture f;
    const uint8_t delay[] = {9,0,100};
    init(&f);
    request(&f, 254, 3, delay, sizeof(delay)); poll(&f, 0);
    f.now = 0xFFFFFF70u;
    request(&f, 200, 1, 0, 0);
    poll(&f, 199u); CHECK(f.sends == 0u);
    poll(&f, 1u); CHECK(f.sends == 1u);
    request(&f, 200, 1, 0, 0);
    proto_reset_receiver(&f.protocol); poll(&f, 200u);
    CHECK(f.sends == 1u);
}

static void test_capacity_and_noise(void)
{
    struct { unsigned before; fixture f; unsigned after; } guarded;
    uint8_t wire[512];
    uint8_t payload[502];
    uint16_t n;
    uint32_t random = 0x16432478u;
    unsigned i;
    const uint8_t maximum[] = {0,0,0,1};
    guarded.before = 0xDEADBEEFu; guarded.after = 0x12345678u;
    init(&guarded.f);
    memset(guarded.f.registers, 255, sizeof(guarded.f.registers));
    for (i = 0; i < 256u; i += 3u) { guarded.f.registers[i + 2u] = 253u; }
    request(&guarded.f, 200, 2, maximum, sizeof(maximum)); poll(&guarded.f, 0);
    CHECK(guarded.f.sends == 1u && guarded.f.output_length < 512u);
    memset(payload, 0, sizeof(payload));
    payload[0] = 124;
    n = packet(wire, 199, 3, payload, sizeof(payload));
    CHECK(n == 512u);
    receive(&guarded.f, wire, n); poll(&guarded.f, 0);
    CHECK(guarded.f.writes == 0u);
    for (i = 0u; i < 200000u; ++i) {
        random ^= random << 13; random ^= random >> 17; random ^= random << 5;
        proto_feed(&guarded.f.protocol, (uint8_t)random, ++guarded.f.now);
        if ((i & 127u) == 0u) { poll(&guarded.f, 0); }
    }
    poll(&guarded.f, 2000u);
    request(&guarded.f, 200, 1, 0, 0); poll(&guarded.f, 0);
    CHECK(guarded.f.sends == 2u);
    CHECK(guarded.before == 0xDEADBEEFu && guarded.after == 0x12345678u);
}

int main(void)
{
    test_crc_reference();
    test_ping_and_read();
    test_stuffing();
    test_error_recovery();
    test_register_protection();
    test_return_level_and_delay();
    test_sync_read_order();
    test_sync_write_and_bulk();
    test_wraparound_and_reset();
    test_capacity_and_noise();
    printf("Protocol tests: %u checks, %u failures. Context: %u bytes.\n",
           checks, failures, (unsigned)sizeof(proto_context));
    return failures == 0u ? 0 : 1;
}
