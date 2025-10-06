/* ───────────────────── USB console for simple commands ───────────────────── */

struct RegEntry {
  const char* name;
  uint16_t reg;
  const char* hint;
};

static const RegEntry REG_MAP[] PROGMEM = {
  /* MOS & SOC */
  { "CHG_MOS_SW", DalyProtocol::REG_CHG_MOS_SW, "Charge MOS switch (0/1)" },
  { "DSG_MOS_SW", DalyProtocol::REG_DSG_MOS_SW, "Discharge MOS switch (0/1)" },
  { "SOC_SETTING", DalyProtocol::REG_SOC_SETTING, "SOC *1000 (e.g. 1000 = 100.0 %)" },

  /* Cell voltage alarms (mV) */
  { "ALM1_VOLT_HI1", DalyProtocol::REG_ALM1_VOLT_HI1, "Cell V high L1 (mV)" },
  { "ALM2_VOLT_HI1", DalyProtocol::REG_ALM2_VOLT_HI1, "Cell V high L2 (mV)" },
  { "ALM1_VOLT_LO1", DalyProtocol::REG_ALM1_VOLT_LO1, "Cell V low  L1 (mV)" },
  { "ALM2_VOLT_LO1", DalyProtocol::REG_ALM2_VOLT_LO1, "Cell V low  L2 (mV)" },

  /* Total voltage alarms (0.1 V) */
  { "ALM1_VOLT_HI2", DalyProtocol::REG_ALM1_VOLT_HI2, "Total V high L1 (0.1V)" },
  { "ALM2_VOLT_HI2", DalyProtocol::REG_ALM2_VOLT_HI2, "Total V high L2 (0.1V)" },
  { "ALM1_VOLT_LO2", DalyProtocol::REG_ALM1_VOLT_LO2, "Total V low  L1 (0.1V)" },
  { "ALM2_VOLT_LO2", DalyProtocol::REG_ALM2_VOLT_LO2, "Total V low  L2 (0.1V)" },
};

static bool consoleWriteActive = false;
static uint16_t consoleWriteReg = 0;
static uint16_t consoleWriteVal = 0;




/* --- helpers --------------------------------------------------------------- */
static inline void _trim(String& s) {
  s.trim();
  // collapse multiple spaces
  while (s.indexOf("  ") >= 0) s.replace("  ", " ");
}
static int _tok(String s, String out[], int maxTok) {
  _trim(s);
  int n = 0;
  int start = 0;
  while (n < maxTok) {
    int sp = s.indexOf(' ', start);
    if (sp < 0) {
      out[n++] = s.substring(start);
      break;
    }
    out[n++] = s.substring(start, sp);
    start = sp + 1;
  }
  return n;
}
static bool _parseBool(const String& v, bool& out) {
  String t = v;
  t.toLowerCase();
  if (t == "1" || t == "on" || t == "true" || t == "yes") {
    out = true;
    return true;
  }
  if (t == "0" || t == "off" || t == "false" || t == "no") {
    out = false;
    return true;
  }
  return false;
}
static bool _parseUint16(const String& s, uint16_t& out) {
  String t = s;
  t.trim();
  long base = 10;
  if (t.startsWith("0x") || t.startsWith("0X")) {
    base = 16;
    t = t.substring(2);
  }
  char* endp = nullptr;
  long v = strtol(t.c_str(), &endp, base);
  if (endp == nullptr || *endp != '\0' || v < 0 || v > 0xFFFF) return false;
  out = (uint16_t)v;
  return true;
}
static bool _lookupReg(const String& name, uint16_t& regOut) {
  for (size_t i = 0; i < sizeof(REG_MAP) / sizeof(REG_MAP[0]); ++i) {
    if (name.equalsIgnoreCase(REG_MAP[i].name)) {
      regOut = REG_MAP[i].reg;
      return true;
    }
  }
  return false;
}
static void _printHelp() {
  DebugSerial.println(F("\nUSB Console Commands:"));
  DebugSerial.println(F("  help                       → this help"));
  DebugSerial.println(F("  status                     → show flags & a few values"));
  DebugSerial.println(F("  get  <var>                → var in {print_all, print_any}"));
  DebugSerial.println(F("  set  <var> <0|1|on|off>   → change var"));
  DebugSerial.println(F("  toggle <var>              → flip var"));
  DebugSerial.println(F("  read <regName|0xNNNN>     → read holding register"));
  DebugSerial.println(F("  write <regName|0xNNNN> <val> → non-blocking write (queued)"));
  DebugSerial.println(F("  save                        → persist current flags"));
  DebugSerial.println(F("  load                        → reload flags from flash"));
  DebugSerial.println(F("  factoryreset                → clear saved flags"));
  DebugSerial.println(F("  set autosave <0|1>          → toggle auto-save on set/toggle"));

  DebugSerial.println(F("\nShortcuts:"));
  DebugSerial.println(F("  alm cell_hi1 4220         → write ALM1_VOLT_HI1 = 4220 mV"));
  DebugSerial.println(F("  alm cell_hi2 4250         → write ALM2_VOLT_HI1 = 4250 mV"));
  DebugSerial.println(F("  alm tot_hi1 422           → write ALM1_VOLT_HI2 = 42.2 V"));
  DebugSerial.println(F("  alm tot_hi2 425           → write ALM2_VOLT_HI2 = 42.5 V"));
  DebugSerial.println();
}

/* queued write driver (call every loop) */
static void _driveConsoleWrite() {
  if (!consoleWriteActive) return;
  if (ensureRegWriteNonBlocking(consoleWriteReg, consoleWriteVal)) {
    DebugSerial.printf("[OK] reg 0x%04X := %u\n", consoleWriteReg, consoleWriteVal);
    bmsHoldUntil = millis() + 250;
    consoleWriteActive = false;
  }
}

/* do one READ synchronously but safely (tiny state machine already exists) */
static bool _readRegOnce(uint16_t reg, uint16_t& val) {
  return readSingleRegister(reg, val);
}

/* parse & execute one command line */
static void _processLine(String line) {
  _trim(line);
  if (line.length() == 0) return;
  String tok[4];
  int n = _tok(line, tok, 4);

  /* help */
  if (tok[0].equalsIgnoreCase("help") || tok[0].equalsIgnoreCase("?")) {
    _printHelp();
    return;
  }

  /* status */
  if (tok[0].equalsIgnoreCase("status")) {
    DebugSerial.printf("print_Daly_at_all      = %d\n", (int)print_Daly_at_all);
    DebugSerial.printf("print_all_Daly_details = %d\n", (int)print_all_Daly_details);
    DebugSerial.printf("%.1f V, %+5.2f/%+6.3f A, SOC=%.1f%%,  T=%.1fC",
                       Battery_Voltage, Battery_Current, Battery_Current_Kalman,
                       Battery_SOC, Battery_Temperature);
    // 1) Cell voltages (0x00–0x1F), only 10 present
    DebugSerial.print(", Cells: ");
    for (uint8_t i = 0; i < 10; i++) {
      float v = DalyProtocol::toCellVoltage(rawBlock0[i]);
      DebugSerial.printf(" %.3f ", v);
    }
    DebugSerial.printf(", Bal: %s", rawBlock0[0x34] ? "1" : "0");
    DebugSerial.printf(", ChargeMOS: %s", rawBlock0[0x35] ? "1" : "0");
    DebugSerial.printf(", DischargeMOS: %s", rawBlock0[0x36] ? "1" : "0");


    DebugSerial.printf("\n");
    return;
  }

  /* get / set / toggle variables */
  auto resolveVar = [](const String& v) -> bool* {
    if (v.equalsIgnoreCase("print_any") || v.equalsIgnoreCase("print_daly_at_all"))
      return &print_Daly_at_all;
    if (v.equalsIgnoreCase("print_all") || v.equalsIgnoreCase("print_all_daly_details"))
      return &print_all_Daly_details;
    if (v.equalsIgnoreCase("autosave"))
      return &autosave_console;  // ← add this line
    return (bool*)nullptr;
  };

  if (tok[0].equalsIgnoreCase("get") && n >= 2) {
    bool* p = resolveVar(tok[1]);
    if (!p) {
      DebugSerial.println("[ERR] unknown var");
      return;
    }
    DebugSerial.printf("%s = %d\n", tok[1].c_str(), (int)(*p));
    return;
  }
  if (tok[0].equalsIgnoreCase("set") && n >= 3) {
    bool* p = resolveVar(tok[1]);
    if (!p) {
      DebugSerial.println("[ERR] unknown var");
      return;
    }
    bool v;
    if (!_parseBool(tok[2], v)) {
      DebugSerial.println("[ERR] need 0/1/on/off/true/false");
      return;
    }
    *p = v;
    DebugSerial.printf("[OK] %s := %d\n", tok[1].c_str(), (int)v);
    if (autosave_console) saveSettings();
    return;
  }
  if (tok[0].equalsIgnoreCase("toggle") && n >= 2) {
    bool* p = resolveVar(tok[1]);
    if (!p) {
      DebugSerial.println("[ERR] unknown var");
      return;
    }
    *p = !*p;
    DebugSerial.printf("[OK] %s toggled → %d\n", tok[1].c_str(), (int)(*p));
    if (autosave_console) saveSettings();
    return;
  }

  /* alarm shortcuts: alm <cell_hi1|cell_hi2|tot_hi1|tot_hi2> <value> */
  if (tok[0].equalsIgnoreCase("alm") && n >= 3) {
    uint16_t reg;
    if (tok[1].equalsIgnoreCase("cell_hi1")) reg = DalyProtocol::REG_ALM1_VOLT_HI1;
    else if (tok[1].equalsIgnoreCase("cell_hi2")) reg = DalyProtocol::REG_ALM2_VOLT_HI1;
    else if (tok[1].equalsIgnoreCase("tot_hi1")) reg = DalyProtocol::REG_ALM1_VOLT_HI2;
    else if (tok[1].equalsIgnoreCase("tot_hi2")) reg = DalyProtocol::REG_ALM2_VOLT_HI2;
    else {
      DebugSerial.println("[ERR] unknown alm key");
      return;
    }
    uint16_t val;
    if (!_parseUint16(tok[2], val)) {
      DebugSerial.println("[ERR] bad value");
      return;
    }
    if (consoleWriteActive) {
      DebugSerial.println("[BUSY] write in progress");
      return;
    }
    consoleWriteReg = reg;
    consoleWriteVal = val;
    consoleWriteActive = true;
    DebugSerial.printf("[..] write reg 0x%04X := %u\n", reg, val);
    return;
  }

  /* write <regName|0xNNNN> <val> */
  if (tok[0].equalsIgnoreCase("write") && n >= 3) {
    uint16_t reg, val;
    if (!_lookupReg(tok[1], reg)) {
      if (!_parseUint16(tok[1], reg)) {
        DebugSerial.println("[ERR] bad reg");
        return;
      }
    }
    if (!_parseUint16(tok[2], val)) {
      DebugSerial.println("[ERR] bad value");
      return;
    }
    if (consoleWriteActive) {
      DebugSerial.println("[BUSY] write in progress");
      return;
    }
    consoleWriteReg = reg;
    consoleWriteVal = val;
    consoleWriteActive = true;
    DebugSerial.printf("[..] write reg 0x%04X := %u\n", reg, val);
    return;
  }

  /* read <regName|0xNNNN>  (fires a single-shot read; call again if timed out) */
  if (tok[0].equalsIgnoreCase("read") && n >= 2) {
    uint16_t reg;
    if (!_lookupReg(tok[1], reg)) {
      if (!_parseUint16(tok[1], reg)) {
        DebugSerial.println("[ERR] bad reg");
        return;
      }
    }
    uint16_t v;
    if (_readRegOnce(reg, v)) {
      DebugSerial.printf("[OK] reg 0x%04X = %u (0x%04X)\n", reg, v, v);
    } else {
      DebugSerial.println("[WAIT] not ready yet, try again");
    }
    return;
  }

  if (tok[0].equalsIgnoreCase("save")) {
    saveSettings();
    DebugSerial.println("[OK] saved");
    return;
  }

  if (tok[0].equalsIgnoreCase("load")) {
    loadSettings();
    DebugSerial.println("[OK] loaded");
    return;
  }
  if (tok[0].equalsIgnoreCase("factoryreset")) {
    factoryResetSettings();
    DebugSerial.println("[OK] cleared");
    return;
  }





  DebugSerial.println("[ERR] unknown command; type 'help'");
}

/* line-buffered serial reader for DebugSerial (USB-CDC) */
void handleUsbConsole() {
  static char buf[96];
  static uint8_t idx = 0;

  while (DebugSerial.available()) {
    char c = (char)DebugSerial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = '\0';
      String line(buf);
      idx = 0;
      _processLine(line);
    } else {
      if (idx < sizeof(buf) - 1) buf[idx++] = c;
      else { idx = 0; }  // overflow → reset
    }
  }

  // advance any queued non-blocking console write
  _driveConsoleWrite();
}
