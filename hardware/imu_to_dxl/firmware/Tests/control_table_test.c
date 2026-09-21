#include "control_table.h"
#include "imu.h"
#include "board.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
board_diagnostics_t board_diagnostics;
static imu_snapshot_t sensor;
static unsigned restart_count;
void imu_get_snapshot(imu_snapshot_t *out) { *out = sensor; }
void imu_request_reinit(void) { ++restart_count; }
uint32_t board_millis(void) { return 123456u; }
int main(void)
{
    proto_context protocol = {0};
    uint8_t out[256] = {0};
    const uint8_t marker = 42, restart = 0xa5;
    const uint8_t expected[] = {0x00,0x80,0xff,0x7f,0xff,0xff,0x00,0x38,0x00,0xb8,0x00,0x80};
    sensor.ready = sensor.configured = 1;
    sensor.gyro[0] = -32768; sensor.gyro[1] = 32767; sensor.gyro[2] = -1;
    sensor.quat_fp16[0] = 0x3800; sensor.quat_fp16[1] = 0xb800; sensor.quat_fp16[2] = 0x8000;
    sensor.sample_count = 0x12345678;
    sensor.who_am_i = 0x70;
    assert(control_table_read(&protocol,124,12,out) == PROTO_OK);
    assert(memcmp(out,expected,sizeof(expected)) == 0);
    assert(control_table_read(&protocol,142,10,out) == PROTO_OK);
    assert(out[0] == 0x78 && out[1] == 3 && out[2] == 0x70);
    assert(out[6] == 0x78 && out[7] == 0x56 && out[8] == 0x34 && out[9] == 0x12);
    assert(control_table_read(&protocol,255,2,out) == PROTO_ERR_RANGE);
    assert(control_table_read(&protocol,0,0,out) == PROTO_ERR_RANGE);
    assert(control_table_read(&protocol,0,256,out) == PROTO_OK);
    assert(control_table_write(&protocol,65,1,&marker) == PROTO_OK);
    assert(control_table_read(&protocol,65,1,out) == PROTO_OK && out[0] == marker);
    assert(control_table_write(&protocol,64,2,out) == PROTO_ERR_ACCESS);
    assert(control_table_write(&protocol,124,1,&marker) == PROTO_ERR_ACCESS);
    assert(control_table_write(&protocol,240,1,&marker) == PROTO_ERR_ACCESS);
    assert(control_table_write(&protocol,240,1,&restart) == PROTO_OK && restart_count == 1);
    puts("control table: 14 checks passed");
    return 0;
}
