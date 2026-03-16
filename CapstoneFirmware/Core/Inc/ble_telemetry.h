#ifndef BLE_TELEMETRY_H
#define BLE_TELEMETRY_H

#include <stdint.h>

typedef struct __attribute__((packed))
{
    float iu;
    float iv;
    float iw;
    float speed_rpm;
    uint8_t mode;
} TelemetryPacket_t;

void BLE_Stack_Init(void);
void BLE_Telemetry_Init(void);
void BLE_StartAdvertising(void);
void BLE_SendTelemetry(const TelemetryPacket_t *pkt);

#endif
