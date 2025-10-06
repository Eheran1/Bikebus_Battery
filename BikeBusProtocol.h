// BikeBusProtocol.h
#ifndef BIKEBUS_PROTOCOL_H
#define BIKEBUS_PROTOCOL_H

#include <Arduino.h>
#include "DalyProtocol.h"  // for rawBlock0/rawBlock1 definitions

// BikeBus tokens we care about:
#define TOKEN_READ_TEMPERATURE 0x12        // SMBus 0x08
#define TOKEN_READ_PACK_VOLTAGE 0x14       // SMBus 0x09
#define TOKEN_READ_PACK_CURRENT 0x16       // SMBus 0x0A
#define TOKEN_READ_AVG_CURRENT 0x18        // SMBus 0x0B
#define TOKEN_READ_SOC 0x1C                // SMBus 0x0D
#define TOKEN_REMAINING_CAPACITY 0x1E      // SMBus 0x0F
#define TOKEN_READ_AVG_TIME_TO_EMPTY 0x26  // SMBus 0x12
#define TOKEN_READ_BATTERY_STATUS 0x2E     // SMBus 0x16

// Cell voltage tokens (10 cells, every other token)
#define TOKEN_CELL1_VOLTAGE 0x7A  // SMBus 0x3C
#define TOKEN_CELL2_VOLTAGE 0x7C
#define TOKEN_CELL3_VOLTAGE 0x7E
#define TOKEN_CELL4_VOLTAGE 0x80
#define TOKEN_CELL5_VOLTAGE 0x82
#define TOKEN_CELL6_VOLTAGE 0x84
#define TOKEN_CELL7_VOLTAGE 0x86
#define TOKEN_CELL8_VOLTAGE 0x88
#define TOKEN_CELL9_VOLTAGE 0x8A
#define TOKEN_CELL10_VOLTAGE 0x8C

class BikeBusProtocol {
public:
  // now nine parameters: two currents, one voltage, one SoC, etc.
  static uint16_t getValueForToken(
    uint8_t token,
    const uint16_t raw0[],
    float batteryCurrent,
    float batteryCurrentKalman,
    float batteryTemperatureC,
    float averageTimeToEmptyMin,
    float batteryVoltageV,
    float batterySOCpercent,
    uint16_t statusBits);
};

#endif  // BIKEBUS_PROTOCOL_H
