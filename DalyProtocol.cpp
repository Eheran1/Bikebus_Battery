// ——————————————————————————————————————————————
// DalyProtocol.cpp
// ——————————————————————————————————————————————
#include "DalyProtocol.h"

namespace DalyProtocol {

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

bool checkCRC(const uint8_t *data, size_t len) {
  if (len < 3) return false;
  uint16_t rec = (uint16_t)data[len - 1] << 8 | data[len - 2];
  return rec == crc16(data, len - 2);
}

String parseAscii(uint16_t *regs, size_t count) {
  String s;
  for (size_t i = 0; i < count; i++) {
    uint8_t hi = regs[i] >> 8;
    uint8_t lo = regs[i] & 0xFF;
    if (hi) s += char(hi);
    if (lo) s += char(lo);
  }
  return s;
}

String toSoftwareVersion(uint16_t *regs) {
  return parseAscii(regs, REG_SW_VER_END - REG_SW_VER_H + 1);
}

String toHardwareVersion(uint16_t *regs) {
  return parseAscii(regs, REG_HW_VER_END - REG_HW_VER_H + 1);
}

String toMachineEncoding(uint16_t *regs, size_t count) {
  return parseAscii(regs, count);
}

String toParameterPassword(uint16_t *regs) {
  return parseAscii(regs, REG_PSW_END - REG_PSW_H + 1);
}

String toProductionDate(uint16_t *regs) {
  uint8_t yy = regs[0] >> 8;
  uint8_t mm = regs[0] & 0xFF;
  uint8_t dd = regs[1] >> 8;
  char buf[16];
  snprintf(buf, sizeof(buf), "20%02u-%02u-%02u", yy, mm, dd);
  return String(buf);
}

String toIPAddress(uint16_t *regs, size_t count) {
  return parseAscii(regs, count);
}



}  // namespace DalyProtocol
