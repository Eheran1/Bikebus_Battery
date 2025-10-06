// ——————————————————————————————————————————————
// DalyProtocol.h
// ——————————————————————————————————————————————
#ifndef DALY_PROTOCOL_H
#define DALY_PROTOCOL_H

#include <Arduino.h>

namespace DalyProtocol {
// ——— Fault-register bit labels ———
// Register 0x3A (Fault 1)
static const char *const FAULT1_DESC[16] = {
  "Lvl1 cell V hi", "Lvl2 cell V hi",
  "Lvl1 cell V lo", "Lvl2 cell V lo",
  "Lvl1 tot V hi", "Lvl2 tot V hi",
  "Lvl1 tot V lo", "Lvl2 tot V lo",
  "Lvl1 chg T hi", "Lvl2 chg T hi",
  "Lvl1 chg T lo", "Lvl2 chg T lo",
  "Lvl1 dsg T hi", "Lvl2 dsg T hi",
  "Lvl1 dsg T lo", "Lvl2 dsg T lo"
};
// Register 0x3B (Fault 2)
static const char *const FAULT2_DESC[16] = {
  "Lvl1 chg I hi", "Lvl2 chg I hi",
  "Lvl1 dsg I lo", "Lvl2 dsg I lo",
  "Lvl1 SOC hi", "Lvl2 SOC hi",
  "Lvl1 SOC lo", "Lvl2 SOC lo",
  "Lvl1 ΔV hi", "Lvl2 ΔV hi",
  "Lvl1 ΔT hi", "Lvl2 ΔT hi",
  "Resv", "Resv",
  "Resv", "Resv"
};
// Register 0x3C (Fault 3)
static const char *const FAULT3_DESC[16] = {
  "Chg MOS T warn", "Dsg MOS T warn",
  "Chg MOS T fail", "Dsg MOS T fail",
  "Chg MOS adh fail", "Dsg MOS adh fail",
  "Chg MOS circ fault", "Dsg MOS circ fault",
  "AFE chip fail", "Unit offline",
  "Temp sensor fail", "EEPROM fail",
  "RTC fail", "Precharge fail",
  "Comm fail", "Network mod fail"
};
// Register 0x3D (Fault 4) – always assume none
static const char *const FAULT4_DESC[1] = { "OK" };

// ——— Decode a 16-bit fault word into a human string ———

// Decode any fault‐register bits:
static inline String decodeFaultBits(uint16_t bits,
                                     const char *const desc[],
                                     uint8_t count) {
  String out;
  for (uint8_t i = 0; i < count; i++) {
    if (bits & (1 << i)) {
      if (out.length()) out += ", ";
      out += desc[i];
    }
  }
  if (out.length() == 0) out = "OK";
  return out;
}

// Convenience overloads:
static inline String decodeFault1(uint16_t bits) {
  return decodeFaultBits(bits, FAULT1_DESC, 16);
}
static inline String decodeFault2(uint16_t bits) {
  return decodeFaultBits(bits, FAULT2_DESC, 16);
}
static inline String decodeFault3(uint16_t bits) {
  return decodeFaultBits(bits, FAULT3_DESC, 16);
}
static inline String decodeFault4(uint16_t /*bits*/) {
  return String(FAULT4_DESC[0]);
}

/**
 * Read the three Daly fault registers (0x3A,0x3B,0x3C) and produce
 * the 16-bit SMBus status word for BikeBus token 0x16.
 */
static inline uint16_t getBatteryStatusBits(const uint16_t raw[]) {
  uint16_t f1 = raw[0x3A];  // Fault register #1
  uint16_t f2 = raw[0x3B];  // Fault register #2
  uint16_t f3 = raw[0x3C];  // Fault register #3

  uint16_t sb = 0;

  // Bit 0: HOCD – Hardware Over-Current Discharge
  // Daly has no 'discharge‐overcurrent' alarm so we leave it 0

  // Bit 1: HOCC – Hardware Over-Current Charge
  if (f2 & ((1<<0)|(1<<1))) sb |= (1<<1);

  // Bit 2: HSC – Hardware Short Circuit
  // Not supported by Daly BMS → leave 0

  // Bits 3 & 4 ignored per your request

  // Bit 5: COV – Cell Over Voltage
  if (f1 & ((1<<0)|(1<<1))) sb |= (1<<5);

  // Bit 6: BOT – Board Over-Temperature
  // Map from any MOS-temp alarm in Fault3 Byte0 (bits 0..3):
  if (f3 & ((1<<0)|(1<<1)|(1<<2)|(1<<3))) sb |= (1<<6);

  // Bit 7: CUV – Cell Under Voltage
  if (f1 & ((1<<2)|(1<<3))) sb |= (1<<7);

  // Bit 8: POV – Pack Over Voltage
  if (f1 & ((1<<4)|(1<<5))) sb |= (1<<8);

  // Bit 9: PUV – Pack Under Voltage
  if (f1 & ((1<<6)|(1<<7))) sb |= (1<<9);

  // Bit 10: OCC2 – Over-Current Charge Tier 2
  if (f2 & (1<<4)) sb |= (1<<10);

  // Bit 11: OCD2 – Over-Current Discharge Tier 2
  if (f2 & (1<<5)) sb |= (1<<11);

  // Bit 12: OCC – Over-Current Charge
  if (f2 & (1<<6)) sb |= (1<<12);

  // Bit 13: OCD – Over-Current Discharge
  if (f2 & (1<<7)) sb |= (1<<13);

  // Bit 14: OTC – Over-Temperature Charge
  // Map from Fault1 Byte1 (bits 8–11 = charge temp alarms)
  if (f1 & ((1<<8)|(1<<9)|(1<<10)|(1<<11))) sb |= (1<<14);

  // Bit 15: OTD – Over-Temperature Discharge
  // Map from Fault1 Byte1 (bits 12–15 = discharge temp alarms)
  if (f1 & ((1<<12)|(1<<13)|(1<<14)|(1<<15))) sb |= (1<<15);

  return sb;
}





// ——— Modbus / Bluetooth constants ———
static const uint8_t SLAVE_ID = 0xD2;
static const uint8_t FUNC_READ_HOLDING = 0x03;
static const uint8_t FUNC_WRITE_SINGLE = 0x06;

// ——— Register addresses ———
// Cell voltages (two bytes per cell)
static const uint16_t REG_CELL_V = 0x00;
static const uint16_t REG_CELL_V_END = 0x1F;

// Battery temps (°C, raw–40)
static const uint16_t REG_BAT_TEMP = 0x20;
static const uint16_t REG_BAT_TEMP_END = 0x27;

static const uint16_t REG_TOTAL_VOLTAGE = 0x28;  // 0.1 V
static const uint16_t REG_CURRENT = 0x29;        // 0.1 A, offset 30000
static const uint16_t REG_SOC = 0x2A;            // 0.001 (80% = 800 → 0.8)
static const uint16_t REG_MAX_CELL_V = 0x2B;     // 1 mV
static const uint16_t REG_MIN_CELL_V = 0x2C;     // 1 mV
static const uint16_t REG_MAX_CELL_T = 0x2D;     // raw–40
static const uint16_t REG_MIN_CELL_T = 0x2E;     // raw–40
static const uint16_t REG_CHG_DSG_STAT = 0x2F;   // 0=idle 1=charging 2=discharging

static const uint16_t REG_REMAIN_CAP = 0x30;     // 0.1 Ah
static const uint16_t REG_BAT_QTY = 0x31;        // count
static const uint16_t REG_NUM_TEMP_SENS = 0x32;  // count
static const uint16_t REG_CYCLE_TIMES = 0x33;    // cycles

static const uint16_t REG_BALANCE_STATE = 0x34;  // 0=off 1=on
static const uint16_t REG_CHG_MOS = 0x35;        // 0=off 1=on
static const uint16_t REG_DSG_MOS = 0x36;        // 0=off 1=on

static const uint16_t REG_AVG_VOLTAGE = 0x37;  // 1 mV
static const uint16_t REG_VOLT_DIFF = 0x38;    // 1 mV
static const uint16_t REG_POWER = 0x39;        // 1 W

static const uint16_t REG_FAULT1 = 0x3A;  // bitflags
static const uint16_t REG_FAULT2 = 0x3B;
static const uint16_t REG_FAULT3 = 0x3C;
static const uint16_t REG_FAULT4 = 0x3D;

static const uint16_t REG_RANGE_KM = 0x3E;  // 0.1 km
static const uint16_t REG_RESERVED1 = 0x3F;

// reserved: 0x40–0x46

// Extended cell voltages (if supported)
static const uint16_t REG_CELL_V2 = 0x47;
static const uint16_t REG_CELL_V2_END = 0x56;

// GPS
static const uint16_t REG_GPS_VALID = 0x57;    // ==0xA5 valid
static const uint16_t REG_GPS_LON_DIR = 0x58;  // 'E' or 'W'
static const uint16_t REG_GPS_LON_H = 0x59;    // combined H/L → degrees*1e-6
static const uint16_t REG_GPS_LON_L = 0x5A;
static const uint16_t REG_GPS_LAT_DIR = 0x5B;  // 'N' or 'S'
static const uint16_t REG_GPS_LAT_H = 0x5C;
static const uint16_t REG_GPS_LAT_L = 0x5D;
static const uint16_t REG_GPS_SPEED = 0x5E;  // 0.1 km/h
static const uint16_t REG_GPS_ALT = 0x5F;    // m

// Board & satellite info
static const uint16_t REG_SAT_COUNT = 0x60;
static const uint16_t REG_BOARD_OK = 0x61;  // ==0xA5 OK
static const uint16_t REG_IMEI_H = 0x62;    // 8 regs long (0x62…0x69)
static const uint16_t REG_IMEI_END = 0x69;

// Configuration (R/W)
static const uint16_t REG_RATED_CAP = 0x80;   // 0.1 Ah
static const uint16_t REG_RATED_VOLT = 0x81;  // 1 mV
static const uint16_t REG_NUM_BOARDS = 0x82;
static const uint16_t REG_UNITS_BRD1 = 0x83;
static const uint16_t REG_UNITS_BRD2 = 0x84;
static const uint16_t REG_UNITS_BRD3 = 0x85;
static const uint16_t REG_TEMP_BRD1 = 0x86;
static const uint16_t REG_TEMP_BRD2 = 0x87;
static const uint16_t REG_TEMP_BRD3 = 0x88;
static const uint16_t REG_BATTERY_TYPE = 0x89;    // 0=LiFePO4,1=NiMnCo,2=LTO
static const uint16_t REG_SLEEP_TIME_S = 0x8A;    // seconds
static const uint16_t REG_ALM1_VOLT_HI1 = 0x8B;   // mV
static const uint16_t REG_ALM2_VOLT_HI1 = 0x8C;   // mV
static const uint16_t REG_ALM1_VOLT_LO1 = 0x8D;   // mV
static const uint16_t REG_ALM2_VOLT_LO1 = 0x8E;   // mV
static const uint16_t REG_ALM1_VOLT_HI2 = 0x8F;   // 0.1 V
static const uint16_t REG_ALM2_VOLT_HI2 = 0x90;   // 0.1 V
static const uint16_t REG_ALM1_VOLT_LO2 = 0x91;   // 0.1 V
static const uint16_t REG_ALM2_VOLT_LO2 = 0x92;   // 0.1 V
static const uint16_t REG_ALM1_CHG_CUR = 0x93;    // 0.1 A, offset
static const uint16_t REG_ALM2_CHG_CUR = 0x94;    // 0.1 A, offset
static const uint16_t REG_ALM1_DSG_CUR = 0x95;    // 0.1 A, offset
static const uint16_t REG_ALM2_DSG_CUR = 0x96;    // 0.1 A, offset
static const uint16_t REG_ALM1_CHG_TMP = 0x97;    // raw–40
static const uint16_t REG_ALM2_CHG_TMP = 0x98;    // raw–40
static const uint16_t REG_ALM1_DSG_TMP = 0x99;    // raw–40
static const uint16_t REG_ALM2_DSG_TMP = 0x9A;    // raw–40
static const uint16_t REG_ALM1_DIFF_VOLT = 0x9B;  // mV
static const uint16_t REG_ALM2_DIFF_VOLT = 0x9C;  // mV
static const uint16_t REG_BAL_VOLT = 0xA3;        // mV
static const uint16_t REG_EQ_OPEN_DIFF = 0xA4;    // mV
static const uint16_t REG_CHG_MOS_SW = 0xA5;      // 0/1
static const uint16_t REG_DSG_MOS_SW = 0xA6;      // 0/1
static const uint16_t REG_SOC_SETTING = 0xA7;     // 0.001
static const uint16_t REG_TMP_ALM = 0xA8;         // raw–40

// Version / encoding / password / date / IP
static const uint16_t REG_SW_VER_H = 0xA9;  // 8 regs → software ver
static const uint16_t REG_SW_VER_END = 0xB0;
static const uint16_t REG_HW_VER_H = 0xB1;  // 8 regs → hardware ver
static const uint16_t REG_HW_VER_END = 0xB8;
static const uint16_t REG_MACHINE_ID_H = 0xB9;  // 0xB9…0xC8
static const uint16_t REG_MACHINE_ID_END = 0xC8;
static const uint16_t REG_PSW_H = 0xC9;  // 3 regs → parameter password
static const uint16_t REG_PSW_END = 0xCB;
static const uint16_t REG_PROD_DATE_H = 0xCC;  // 2 regs → YY MM DD 00
static const uint16_t REG_PROD_DATE_END = 0xCD;
static const uint16_t REG_IP_H = 0xCE;  // 0xCE…0xE5 ASCII IP:port
static const uint16_t REG_IP_END = 0xE5;

// Control commands (write only)
static const uint16_t REG_SYS_RST = 0xF0;
static const uint16_t REG_SYS_OFF = 0xF1;
static const uint16_t REG_ZERO_CUR = 0xF2;
static const uint16_t REG_FACTORY_RST = 0xF3;

// ——— CRC16 / helpers ———
uint16_t crc16(const uint8_t *data, size_t len);
bool checkCRC(const uint8_t *data, size_t len);

// ——— Conversions ———
inline float toCellVoltage(uint16_t raw) {
  return raw * 0.001f;
}
inline float toBatteryTemp(uint16_t raw) {
  return raw - 40.0f;
}
inline float toTotalVoltage(uint16_t raw) {
  return raw * 0.1f;
}
inline float toCurrent(uint16_t raw) {
  return 0.1f * (raw - 30000);
}
inline float toSOC(uint16_t raw) {
  return raw * 0.001f;
}
inline float toRemainingCap(uint16_t raw) {
  return raw * 0.1f; /* Ah */
}
inline float toAverageVoltage(uint16_t raw) {
  return raw * 0.001f;
}
inline float toVoltageDiff(uint16_t raw) {
  return raw * 0.001f;
}
inline float toPower(uint16_t raw) {
  return float(raw);
}
inline float toRangeKm(uint16_t raw) {
  return raw * 0.1f;
}
inline bool toChgDsgStatus(uint16_t raw) {
  return raw; /* 0/1/2 */
}
inline bool toFlagValid(uint16_t raw, uint16_t okVal = 0xA5) {
  return raw == okVal;
}
inline float toSpeedKmh(uint16_t raw) {
  return raw * 0.1f;
}
inline uint16_t toAltitude(uint16_t raw) {
  return raw;
}
inline uint16_t toCount(uint16_t raw) {
  return raw;
}
inline float toProductionDateYear(uint16_t regs[2], char *buf, size_t sz);
inline uint16_t toReserved(uint16_t raw) {
  return raw;
}

// combine two regs for lon/lat
inline float toDegrees1e6(uint16_t hi, uint16_t lo) {
  return ((uint32_t)hi << 16 | lo) * 1e-6f;
}
inline char toHemisphere(uint16_t raw) {
  return char(raw & 0xFF);
}

// ASCII‐string parsers
String parseAscii(uint16_t *regs, size_t count);
String toSoftwareVersion(uint16_t *regs);
String toHardwareVersion(uint16_t *regs);
String toMachineEncoding(uint16_t *regs, size_t count);
String toParameterPassword(uint16_t *regs);
String toProductionDate(uint16_t *regs);
String toIPAddress(uint16_t *regs, size_t count);

}  // namespace DalyProtocol

#endif  // DALY_PROTOCOL_H
