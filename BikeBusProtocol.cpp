// BikeBusProtocol.cpp
#include "BikeBusProtocol.h"
#include "DalyProtocol.h"

// helper to build manufacture‐date word:
static uint16_t encodeManufactureDate(uint16_t year, uint8_t month, uint8_t day) {
  // Python does:   year = ((value>>9)&0x7F)+1980
  // so reverse:    y = year-1980; value = (y<<9)|(month<<5)|day
  uint16_t y = year - 1980;
  return (y << 9) | (month << 5) | (day & 0x1F);
}


uint16_t BikeBusProtocol::getValueForToken(uint8_t token,
                                           const uint16_t raw0[],
                                           float Battery_Current,
                                           float Battery_Current_Kalman,
                                           float Battery_Temperature,
                                           float Average_Time_to_Empty,
                                           float Battery_Voltage,
                                           float Battery_SOC,
                                           uint16_t BikeBus_statusBits) {


  switch (token) {
    case TOKEN_READ_TEMPERATURE:
      {
        int16_t v = int16_t(Battery_Temperature * 10.0f);
        return uint16_t(v);
      }

    case TOKEN_READ_PACK_VOLTAGE:
      {
        return uint16_t(Battery_Voltage * 1000.0f);
      }

    case TOKEN_READ_PACK_CURRENT:
      {
        int16_t v = int16_t(Battery_Current * 100.0f);
        return uint16_t(v);
      }

    case TOKEN_READ_AVG_CURRENT:
      {
        int16_t v = int16_t(Battery_Current_Kalman * 100.0f);
        return uint16_t(v);
      }

    case TOKEN_READ_SOC:
      {
        return uint16_t(Battery_SOC);
      }


    case TOKEN_REMAINING_CAPACITY:
      {
        return 0;
      }

    case TOKEN_READ_AVG_TIME_TO_EMPTY:
      {
        int16_t v = int16_t(Average_Time_to_Empty);
        return uint16_t(v);
      }  // in minutes

    case TOKEN_READ_BATTERY_STATUS:
      {
        return BikeBus_statusBits;
      }  // fault status registers


    // cells:
    case TOKEN_CELL1_VOLTAGE: return uint16_t(raw0[0x00] * 1ull);  // mV: raw0[0]×1mV
    case TOKEN_CELL2_VOLTAGE: return uint16_t(raw0[0x01]);
    case TOKEN_CELL3_VOLTAGE: return uint16_t(raw0[0x02]);
    case TOKEN_CELL4_VOLTAGE: return uint16_t(raw0[0x03]);
    case TOKEN_CELL5_VOLTAGE: return uint16_t(raw0[0x04]);
    case TOKEN_CELL6_VOLTAGE: return uint16_t(raw0[0x05]);
    case TOKEN_CELL7_VOLTAGE: return uint16_t(raw0[0x06]);
    case TOKEN_CELL8_VOLTAGE: return uint16_t(raw0[0x07]);
    case TOKEN_CELL9_VOLTAGE: return uint16_t(raw0[0x08]);
    case TOKEN_CELL10_VOLTAGE: return uint16_t(raw0[0x09]);
    default:
      return 0;
  }
}
