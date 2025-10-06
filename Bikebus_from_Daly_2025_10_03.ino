/*
ESP32C6
 USB CDC On Boot: Enabled
 CPU frequency: 160 MHz
 Debug level: None
 Erase all flash: Disabled
 Events run on core: 1
 Flash Speed: 80MHz
 Flash Mode: QIO
 Flash Size: 4 MByte
 JTAG: Disabled
 Partition Scheme: Default 4MB with spiffs (1.2MB APP/1.5MB SPIFFS)
 Upload Speed: 921600
 Zigbee: Disabled

Tools>Programmer>Esptool
Password: Random, not set

reset all flash:
1. hold "Boot / IO0" button ->
2. python -m esptool --chip esp32 erase_flash
3. let go of button when process starts -> 
4. done

preferences max 15 characters key lenght
*/



// teensy 3.2
//Serial1  on RX 0, TX 1
//Serial2  on RX 9, TX 10
//Serial3  on RX 7, TX 8

// ESP32
//Serial0  on RX 3,  TX 2
//Serial1  defined on RX 18, TX 19
//Serial2  on RX 16, TX 17

// ESP32-C6
//Serial   USB
//Serial0  RX 17, TX 16
//Serial1  RX 9,  TX 10
//Serial2  RX 4,  TX 5 Low power, not usable with Arduino IDE

#include <Arduino.h>
#include <driver/uart.h>
#include "esp_attr.h"
#include "esp_task_wdt.h"  // task-watchdog (TWDT) from ESP-IDF
#include "freertos/semphr.h"
#include <Preferences.h>
Preferences prefs;
bool autosave_console = true;  // if true: persist on each set/toggle


// ─── choose an RTC-capable pin (0-7) ───
static constexpr int BIKEBUS_RX_PIN = 4;   // BikeBus RX  → MCU RX = Yellow = RTC/LP → can wake us
static constexpr int BIKEBUS_TX_PIN = 10;  // BikeBus TX  → MCU TX = Green
static constexpr int BMS_RX_PIN = 18;      // Daly BMS TX → MCU RX = Yellow
static constexpr int BMS_TX_PIN = 19;      // Daly BMS RX → MCU TX = Green

// ESP32-C6: GPIO 0-7 are the only ones that can wake the chip

static constexpr int ATA6661_EN_PIN = 5;                       // ATA6661 Enable -> high = enabled
static constexpr int RGB_LED_PIN = 8;                          // on-board LED
static constexpr int SOC_BUTTON_PIN = 2;                       // button pulls LOW when pressed
static constexpr int BAT_PLUG_PIN = 3;                         // Check if connection from external plug (Rosenberg) is present to run on discharge FET
static constexpr int STATUS_LED_PINS[4] = { 20, 21, 22, 23 };  // active-low LEDs

/* ──────────────── UART wiring / instances ─────────────────────── */
HardwareSerial& BikebusSerial = Serial1;  // UART1  – Bike-Bus (needs wake, only works on default pins)
HardwareSerial& BMSSerial = Serial0;      // UART0  – Daly BMS
auto& DebugSerial = Serial;               // USB-CDC – PC debug console
#define DBG_ON                            // <-- comment this line to disable all debug prints

#ifdef DBG_ON
#define DBG(...) DebugSerial.printf(__VA_ARGS__)
#else
#define DBG(...)
#endif

float Battery_Voltage, Battery_Current, Battery_Current_Kalman, Battery_SOC, Battery_Temperature, Average_Time_to_Empty;
float Total_Battery_capacity = 6., Remaining_Battery_capacity = 0;
bool print_all_Daly_details = 0;
bool print_Daly_at_all = 1;
bool BMS_DEBUG = 0;
bool BMS_DEBUG_CUR = 0;

// SOC LED display and button
static constexpr uint8_t SOC_MARGIN = 7;                  // % above bucket before it counts “full”
static constexpr uint8_t SOC_BLINK_WINDOW = 13;           // % range for blinking the next LED
static constexpr uint8_t SOC_EMPTY_THRESHOLD = 7;         // below this, blink all LEDs
static constexpr unsigned long SOC_DURATION = 5000UL;     // show LEDs for x s
static constexpr unsigned long SOC_BLINK_PERIOD = 400UL;  // blink LEDs every x s
// ——— internal state ———
bool socActive = false;
unsigned long socStartTime = 0;
unsigned long socLastBlink = 0;
bool socBlinkOn = false;
bool socEmptyBlink = false;
uint8_t socFull = 0;
bool socPartial = false;
uint32_t lastNote = 0;
uint64_t wakeMask;

#include "BikeBusProtocol.h"
#define BAUD_RATE 9600
#define MASTER_ADDRESS 0x01
#define BATTERY_ADDRESS 0x20
// Frame‐buffer with timeout
uint8_t busBuf[5];
static const uint32_t FRAME_TIMEOUT = 10;  // ms
uint32_t BikeBus_reply_delay = 4;
// BikeBus timing
uint32_t lastBusTime = 0;
uint16_t busInterval = 4;  // respond x ms after start of master request
size_t currentReqIdx = 0;
// buffer for incoming 1-wire bytes
struct Request {
  uint8_t address, token, lo, hi;
};
extern Request requests[];  // copy from your BikeBus example
extern const size_t numRequests;
uint16_t BikeBus_statusBits;
uint8_t dump;
static uint32_t mosStateStartMs = 0;                    // when we entered current state
static constexpr uint32_t MOS_STATE_TIMEOUT_MS = 3000;  // 3 s watchdog

#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel rgb(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

inline void statusLed(bool on,
                      uint8_t r = 255,
                      uint8_t g = 255,
                      uint8_t b = 255) {
  static bool init = false;
  if (!init) {
    rgb.begin();
    rgb.clear();
    rgb.show();
    init = true;
  }
  if (on) {
    rgb.setPixelColor(0, rgb.Color(r, g, b));
  } else {
    rgb.setPixelColor(0, 0);  // black
  }
  rgb.show();
}




#include "DalyProtocol.h"
void beginWriteRegister(uint16_t reg, uint16_t val);
void serviceWriteRegister();
void parseBlock0(uint8_t* data, size_t len);
void updateSOCDisplay();
void startSOCDisplay();
void initSOCDisplay();
bool ensureMosStateNonBlocking(uint16_t reg, bool turnOn);


// polling reg0 takes ~180 ms
static constexpr long BMS_BAUD = 9600;
static const uint32_t BMS_FULL_POLL_INTERVAL = 60000;   // ms
static const uint32_t BMS_CURRENT_POLL_INTERVAL = 100;  // ms
static const uint32_t BMS_FULL_RX_TIMEOUT_MS = 400;     // full-block frame budget
static const uint32_t BMS_FULL_RETRY_BACKOFF = 200;     // wait before retry after timeout
static const uint32_t BMS_CUR_RX_TIMEOUT_MS = 50;       // single-reg frame budget
static const uint16_t QTY0 = 62;                        // regs 0x0000–0x003D
static const uint16_t QTY1 = 41;                        // regs 0x0080–0x00A8
uint16_t rawBlock0[QTY0];
uint16_t rawBlock1[QTY1];



/* ----------- MOS retry / back-off helpers ---------------- */
static uint32_t nextMosRetryMs = 0;  // when we may try again
static bool wantDsg = false;         /* desired Discharge MOS state */
static bool wantChg = false;         /* desired Charge MOS state */
static uint32_t bmsHoldUntil = 0;
static uint32_t bmsHold_delay = 50;
static uint32_t mosSettle_delay = 1500;  // time it takes to actually set the MOS in the BMS


// Kalman filter
#include "Kalman.h"  // "" for user-defined headers
KalmanFilter Kalman_Current;
float I_Kalman;
float dt_I;
unsigned long lastUpdateTime_I = 0;  // This will store the last update time in milliseconds
float R_I = 10000.;                  // höher = langsamerer Filter / stärkerer Filter,  measurement uncertainty
bool init_I_Kalman = 0;

// ── Plug debounce & cooldown (non-blocking) ─────────────────────
static const uint32_t PLUG_DEBOUNCE_MS = 120;            // level must be stable this long
static const uint32_t PLUG_MIN_EVENT_SPACING_MS = 2000;  // min spacing between accepted events

static int plugLastRawLevel = -1;          // last sampled GPIO level
static int plugLastAcceptedLevel = -1;     // last level we acted on
static uint32_t plugLevelStableSince = 0;  // when current raw level became stable
static uint32_t plugLastEventMs = 0;       // when we last accepted an event
static uint32_t plugCooloffUntil = 0;      // ignore new events until this time



// Call once during boot to pull saved values
void loadSettings() {
  prefs.begin("cfg", /*readOnly=*/true);
  bool print_Daly_at_all_prev = print_Daly_at_all;
  bool print_all_Daly_details_prev = print_all_Daly_details;

  print_Daly_at_all = prefs.getBool("print_any", false);
  print_all_Daly_details = prefs.getBool("print_all", false);
  autosave_console = prefs.getBool("autosave", true);

  DebugSerial.print(F("print_Daly_at_all prev/now: "));
  DebugSerial.print(print_Daly_at_all_prev);
  DebugSerial.print(F("/"));
  DebugSerial.print(print_Daly_at_all);
  DebugSerial.print(F(", print_all_Daly_details prev/now: "));
  DebugSerial.print(print_all_Daly_details_prev);
  DebugSerial.print(F("/"));
  DebugSerial.print(print_all_Daly_details);
  DebugSerial.print("\n");
  prefs.end();
}

// Call when you want to persist current values
void saveSettings() {
  prefs.begin("cfg", /*readOnly=*/false);
  prefs.putBool("print_any", print_Daly_at_all);
  prefs.putBool("print_all", print_all_Daly_details);
  prefs.putBool("autosave", autosave_console);
  prefs.end();
}

// Optional: wipe to defaults
void factoryResetSettings() {
  prefs.begin("cfg", /*readOnly=*/false);
  prefs.clear();
  prefs.end();
  // Re-apply defaults in RAM
  print_Daly_at_all = false;
  print_all_Daly_details = false;
  autosave_console = true;
}



// Phase tracker for the non-blocking MOS sequence
enum class MosChkPhase : uint8_t { Idle,
                                   SendCmd,
                                   WaitEcho,
                                   VerifyRead,
                                   Done };
static MosChkPhase mosChkPhase = MosChkPhase::Idle;
static uint32_t mosChkT0;
static uint16_t mosChkReg;
static bool mosChkTarget;
static bool mosChkResult;



enum MosCtlState {
  Idle,
  StartDisOn,
  WaitDisOn,
  SamplePlug,
  StartDisOff,
  WaitDisOff,
  StartChgOn,   // charge ON
  StartChgOff,  // charge OFF
  WaitChg,      // wait for either ON or OFF
  Done,
  Print
};
static MosCtlState mosState = Idle;
static uint32_t mosTimer = 0;
static uint32_t print_delay_timer = 0;





auto purge_BMS_UART = []() {
  while (BMSSerial.available()) BMSSerial.read();
};


/* ─────────── async write-register helper (NON-BLOCKING) ────────── */
struct WriteRegJob {
  uint16_t reg;           // target register
  uint8_t expect[8];      // copy of the frame we just sent
  uint8_t got[8];         // echo read back from the Daly
  uint8_t idx = 0;        // bytes already received
  uint32_t startMs = 0;   // ↓↓↓  NEW  ↓↓↓
  uint32_t deadline = 0;  // give up after this
  bool busy = false;      // echo still outstanding
  bool ok = false;        // echo complete & CRC good
} wrJob;

static constexpr uint32_t MOS_WRITE_TIMEOUT = 400;

/* ────────── 2.  lock replacement (non‑blocking) ────────── */
static SemaphoreHandle_t bmsMutex;  // binary semaphore protects the UART

// helper: try to take lock for max <ms> – returns true if successful
bool tryLockBms(uint32_t maxWaitMs) {
  return xSemaphoreTake(bmsMutex, pdMS_TO_TICKS(maxWaitMs)) == pdTRUE;
}

inline void unlockBms() {
  xSemaphoreGive(bmsMutex);
}

/* ────────── 3.  wrJob pump – call regularly from loop() ────────── */
void tickWriteEcho() {
  if (!wrJob.busy) return;

  // timeout guard                                                         ↓ 10 ms grace
  if (millis() - wrJob.startMs > MOS_WRITE_TIMEOUT + 10) {
    wrJob.busy = false;
    wrJob.idx = 0;
    return;  // abort
  }
  serviceWriteRegister();  // same function as before
}





// ─────────────────────────────────────────────────────────────────
// Daly BMS manager: non-blocking Read/Write with timeouts
// ─────────────────────────────────────────────────────────────────

class BmsManager {
  // --- fast-current RX scratch (so we can reset between attempts) ---
  uint8_t _curBuf[7] = { 0 };
  uint8_t _curNeed = 0, _curGot = 0;
  // success/latency diagnostics
  uint32_t _curSeq = 0, _fullSeq = 0;
  uint32_t _curT0 = 0, _fullT0 = 0;
  bool _curInFlight = false;
  bool _fullInFlight = false;

public:
  // Call in setup()
  void begin(HardwareSerial& uart, uint32_t baud, uint16_t qty0) {
    _uart = &uart;
    _baud = baud;
    _qty0 = qty0;
    uart.setRxBufferSize(256);
    uart.begin(baud, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
    _state = State::Idle;
    uint32_t now = millis();
    _lastCycle = now;
    headerReceived = false;
    // Start full poll immediately:
    _lastFullMs = now - BMS_FULL_POLL_INTERVAL;  // ← ensures condition is true on first poll
    // Optionally also start fast current immediately:
    _lastCurMs = now - 100;  // if your fast interval is 100 ms
  }

  // Must be called once every loop()
  void poll(uint32_t fastCurIntervalMs = BMS_CURRENT_POLL_INTERVAL, uint32_t fullBlockIntervalMs = BMS_FULL_POLL_INTERVAL) {
    uint32_t now = millis();
    //DBG("[BMS] poll() enter — state=%d lastCycle=%u now=%u busy=%d\n", int(_state), _lastCycle, now, wrJob.busy);

    // Step 1: service any pending Write‐echo
    ::serviceWriteRegister();

    if (now < bmsHoldUntil || wrJob.busy) {
      if (_state == State::WaitCur || _state == State::WaitFull) {
        if (BMS_DEBUG) {
          DBG("[BMS] abort mid-read due to write/hold; flushing RX\n");
        }
        flushRead();
        headerReceived = false;
        _curNeed = _curGot = 0;  // NEW: reset fast scratch
        _curInFlight = false;    // <<< mark aborted
        _fullInFlight = false;   // <<< mark aborted
      }
      _state = State::Idle;
      _lastCycle = now;
      return;
    }

    // Step 2: state machine for Read cycles
    switch (_state) {
      case State::Idle:
        if (now - _lastFullMs >= fullBlockIntervalMs) {
          startReadFull();
          _readStart = now;
          _state = State::WaitFull;
        } else if (now - _lastCurMs >= fastCurIntervalMs) {
          // only start fast current if not within a short “recent full start” window
          startReadCurrent();
          _readStart = now;
          _state = State::WaitCur;
        }
        break;

      case State::WaitFull:
        handleWaitFull(now);
        break;

      case State::WaitCur:
        handleWaitCur(now);
        break;

      case State::DoneFull:
      case State::DoneCur:
        // Hold “Done” one cycle to avoid thrashing
        if (now - _lastCycle >= 1) {
          _state = State::Idle;
        }
        break;
    }
  }



  bool readDone() const {
    return _state == State::DoneFull;
  }  // unchanged users of this keep working
  bool readBusy() const {
    return _state == State::WaitFull || _state == State::WaitCur;
  }
  void clearDone() {
    if (_state == State::DoneFull) {
      _lastCycle = millis();
      _state = State::Idle;
    }
  }

  void writeMOS(uint16_t reg, uint16_t val) {
    if (!wrJob.busy) ::beginWriteRegister(reg, val);
  }



private:
  enum class State { Idle,
                     WaitFull,
                     DoneFull,
                     WaitCur,
                     DoneCur } _state = State::Idle;

  uint32_t _lastCycle = 0, _readStart = 0;
  uint32_t _lastFullMs = 0, _lastCurMs = 0;
  uint16_t _qty0 = 0, _need = 0;
  uint32_t _baud = 0;
  HardwareSerial* _uart = nullptr;
  bool headerReceived = false;
  uint8_t _hdr[3] = { 0, 0, 0 };
  uint8_t _bmsErrCount = 0;
  bool bmsLed = false;

  /* ---- FULL BLOCK ---- */
  void startReadFull() {
    while (_uart->available()) _uart->read();  // optional: start clean
    uint8_t tx[8];
    tx[0] = DalyProtocol::SLAVE_ID;
    tx[1] = DalyProtocol::FUNC_READ_HOLDING;
    tx[2] = 0x00;             // reg hi
    tx[3] = 0x00;             // reg lo (0x0000)
    tx[4] = highByte(_qty0);  // qty hi
    tx[5] = lowByte(_qty0);   // qty lo
    uint16_t crc = DalyProtocol::crc16(tx, 6);
    tx[6] = lowByte(crc);
    tx[7] = highByte(crc);
    _uart->write(tx, 8);
    headerReceived = false;
    _fullT0 = millis();
    _fullInFlight = true;
    ++_fullSeq;
  }
  void handleWaitFull(uint32_t now) {
    if (!headerReceived) {
      if (_uart->available() >= 3) {
        _uart->readBytes(_hdr, 3);
        uint8_t len = _hdr[2];
        if (len < 2 || len > 250) {  // sanity
          flushRead();
          headerReceived = false;
          _fullInFlight = false;
          _need = 0;
          _state = State::Idle;
          return;
        }
        _need = len + 5;  // total bytes
        headerReceived = true;
        if (BMS_DEBUG) {
          DBG("[BMS] full: got header, need=%u\n", (unsigned)_need);
        }
      } else if (now - _readStart > BMS_FULL_RX_TIMEOUT_MS) {
        // header never arrived within overall budget
        if (BMS_DEBUG) {
          DBG("[BMS] full: header timeout after %lums\n", (unsigned long)(now - _readStart));
        }
        ++_bmsErrCount;
        if (_bmsErrCount >= 5) {
          _uart->end();
          delay(10);
          _uart->begin(_baud, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
          delay(500);
          _bmsErrCount = 0;
        }
        flushRead();
        headerReceived = false;
        _fullInFlight = false;
        _need = 0;
        _lastFullMs = now + BMS_FULL_RETRY_BACKOFF - BMS_FULL_POLL_INTERVAL;  // see note below
        _state = State::Idle;
        return;
      } else {
        return;  // keep waiting for header
      }
    }

    // Have header → wait for the rest
    if (_uart->available() >= (_need - 3)) {
      parseFullFrame();
      headerReceived = false;
      _bmsErrCount = 0;
      _state = State::DoneFull;
      _lastCycle = now;
      _lastFullMs = now;  // schedule next full poll from now
      if (BMS_DEBUG) {
        DBG("[BMS] full OK: seq=%lu dt=%lums bytes=%u\n",
            (unsigned long)_fullSeq,
            (unsigned long)(now - _readStart),
            (unsigned)_need);
      }
      _fullInFlight = false;
      _need = 0;
    } else if (now - _readStart > BMS_FULL_RX_TIMEOUT_MS) {
      // body timeout
      if (BMS_DEBUG) {
        DBG("[BMS] full: body timeout after %lums (avail=%u need=%u)\n",
            (unsigned long)(now - _readStart), (unsigned)_uart->available(), (unsigned)(_need - 3));
      }
      ++_bmsErrCount;
      if (_bmsErrCount >= 5) {
        _uart->end();
        delay(10);
        _uart->begin(_baud, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
        delay(500);
        _bmsErrCount = 0;
      }
      flushRead();
      headerReceived = false;
      _fullInFlight = false;
      _need = 0;
      // backoff: don’t immediately requeue a full read on next Idle
      _lastFullMs = now + BMS_FULL_RETRY_BACKOFF - BMS_FULL_POLL_INTERVAL;  // see note
      _state = State::Idle;
    }
  }



  void parseFullFrame() {
    uint8_t buf[260];
    memcpy(buf, _hdr, 3);
    _uart->readBytes(buf + 3, _need - 3);
    if (DalyProtocol::checkCRC(buf, _need) && buf[0] == DalyProtocol::SLAVE_ID && buf[1] == DalyProtocol::FUNC_READ_HOLDING) {
      ::parseBlock0(buf, _need);
    } else {
      flushRead();
    }
  }

  /* ---- FAST CURRENT ---- */
  void startReadCurrent() {
    if (BMS_DEBUG_CUR) {
      DBG("[BMS] fast: startReadCurrent()\n");
    }
    uint8_t tx[8] = {
      DalyProtocol::SLAVE_ID,
      DalyProtocol::FUNC_READ_HOLDING,
      0x00,
      DalyProtocol::REG_CURRENT,
      0x00,
      0x01
    };
    uint16_t crc = DalyProtocol::crc16(tx, 6);
    tx[6] = lowByte(crc);
    tx[7] = highByte(crc);
    _uart->write(tx, 8);
    headerReceived = false;
    _curNeed = 0;
    _curGot = 0;  // ensure fresh read
    _curT0 = millis();
    _curInFlight = true;
    ++_curSeq;
  }

  void handleWaitCur(uint32_t now) {
    if (!headerReceived) {
      if (_uart->available() >= 3) {
        _uart->readBytes(_curBuf, 3);
        _curNeed = _curBuf[2] + 5;
        _curGot = 3;
        headerReceived = true;
        if (BMS_DEBUG_CUR) {
          DBG("[BMS] fast: hdr len=%u need=%u\n", (unsigned)_curBuf[2], (unsigned)_curNeed);
        }
      } else if (now - _readStart > 50) {
        if (BMS_DEBUG_CUR) {
          DBG("[BMS] fast: no header within 50 ms → abort\n");
        }
        headerReceived = false;
        _curNeed = _curGot = 0;  // NEW
        _curInFlight = false;    // <<< mark failed
        _state = State::Idle;
        return;
      } else return;
    }

    while (headerReceived && _uart->available() && _curGot < _curNeed) {
      _curBuf[_curGot++] = _uart->read();
    }

    if (headerReceived && _curGot == _curNeed) {
      bool ok = DalyProtocol::checkCRC(_curBuf, _curNeed) && _curBuf[0] == DalyProtocol::SLAVE_ID && _curBuf[1] == DalyProtocol::FUNC_READ_HOLDING && _curBuf[2] == 2;
      if (!ok) {
        if (BMS_DEBUG_CUR) {
          DBG("[BMS] fast: bad frame (crc/func/len)\n");
        }
        flushRead();
        headerReceived = false;
        _curNeed = _curGot = 0;  // NEW
        _state = State::Idle;
        return;
      }

      uint16_t raw = (uint16_t(_curBuf[3]) << 8) | _curBuf[4];
      rawBlock0[0x29] = raw;  // keep cache
      Battery_Current = DalyProtocol::toCurrent(raw);

      if (init_I_Kalman == 0) {
        Kalman_Current.init(Battery_Current, 1., 1., 1., R_I);
        init_I_Kalman = 1;
        lastUpdateTime_I = millis();
      } else {
        dt_I = computeDeltaT(lastUpdateTime_I);
        Kalman_Current.update(Battery_Current, dt_I);
      }
      Battery_Current_Kalman = Kalman_Current.getState(0);

      float Abs_I = fabs(Battery_Current_Kalman);
      if (Abs_I < 1.0f) Abs_I = 1.0f;
      Average_Time_to_Empty = Remaining_Battery_capacity / Abs_I * 60.0f;

      if (BMS_DEBUG) {
        DBG("[BMS] fast OK: seq=%lu dt=%lums I=%.2fA Kal=%.3fA (got=%u)\n",
            (unsigned long)_curSeq,
            (unsigned long)(now - _curT0),
            Battery_Current, Battery_Current_Kalman, (unsigned)_curGot);
      }
      if (print_Daly_at_all == 1) {
        DebugSerial.printf("[FAST] %+5.2f/%+6.3f A\n", Battery_Current, Battery_Current_Kalman);
      }

      _bmsErrCount = 0;  // NEW: success → healthy
      headerReceived = false;
      _curNeed = _curGot = 0;  // NEW: clear scratch
      _state = State::DoneCur;
      _lastCurMs = now;
      _lastCycle = now;
      return;
    }

    if (headerReceived && (now - _readStart > BMS_CUR_RX_TIMEOUT_MS)) {
      if (BMS_DEBUG_CUR) {
        DBG("[BMS] fast: RX timeout after %lu ms (got=%u need=%u)\n",
            (unsigned long)(now - _readStart), (unsigned)_curGot, (unsigned)_curNeed);
      }
      flushRead();
      headerReceived = false;
      _curNeed = _curGot = 0;  // NEW
      _state = State::Idle;
    }
  }

  void flushRead() {
    while (_uart->available()) _uart->read();
  }
};


BmsManager bms;
static const uint32_t Enable_retry_delay = 10000;

/* ---- task-watchdog configuration ---- */
static const esp_task_wdt_config_t twdt_config = {
  .timeout_ms = 6000,    // 6 s
  .idle_core_mask = 0,   // we feed the dog ourselves
  .trigger_panic = true  // reboot on timeout
};


inline bool mosHwMatches() {
  /* rawBlock0 is updated every Daly poll */
  bool dsg = rawBlock0[DalyProtocol::REG_DSG_MOS] & 0x1;
  bool chg = rawBlock0[DalyProtocol::REG_CHG_MOS] & 0x1;
  return (dsg == wantDsg) && (chg == wantChg);
}






/* ──────────────── power-saving ─────────────────────── */
#include "esp_sleep.h"
static const uint32_t INACTIVITY_MS = 300000UL;  // sleep after inactivity for this long
static const uint32_t SLEEP_LED_GRACE_MS = 40;    // let LEDs fade

volatile bool buttonTapFlag = false;
volatile unsigned long lastBusActivity = 0;
volatile bool plugChangedFlag = false;  // set in ISR, cleared in loop
volatile bool requestWakeFromTap = false;

static volatile bool plugPending = false;

void IRAM_ATTR plugIsr() {
  plugChangedFlag = true;  // still used for LED feedback
}


/* ---------- SOC button edge flag ------------------ */
void IRAM_ATTR socButtonIsr() {  // Without IRAM_ATTR, the interrupt vector might be in flash and gets blocked during flash operations?
  if (!socActive) {
    buttonTapFlag = true;
  }
}



static volatile uint32_t lastActiveMs = 0;  // “awake” marker, tracks user and I/O activity
inline void noteActivity() {                // call from any handler that
  lastActiveMs = millis();                  // counts as "awake" behaviour
}





void beginWriteRegister(uint16_t reg, uint16_t val) {
  if (wrJob.busy) return;  // one job at a time

  /* build the 8-byte frame in wrJob.expect[] */
  uint8_t* tx = wrJob.expect;
  tx[0] = DalyProtocol::SLAVE_ID;
  tx[1] = 0x06;
  tx[2] = highByte(reg);
  tx[3] = lowByte(reg);
  tx[4] = highByte(val);
  tx[5] = lowByte(val);
  uint16_t crc = DalyProtocol::crc16(tx, 6);
  tx[6] = lowByte(crc);
  tx[7] = highByte(crc);

  //DBG("[MOS ] → TX: ");
  //for (int i = 0; i < 8; i++) DBG("%02X ", wrJob.expect[i]);
  //DBG("\n");

  while (BMSSerial.available()) BMSSerial.read();  // purge
  BMSSerial.write(wrJob.expect, 8);                // send
  BMSSerial.flush();                               // ensures bytes are in HW FIFO

  // ——— NEW: mirror the requested bit into our rawBlock0 cache ———
  if (reg == DalyProtocol::REG_DSG_MOS_SW) {
    rawBlock0[DalyProtocol::REG_DSG_MOS] =
      (rawBlock0[DalyProtocol::REG_DSG_MOS] & ~0x1) | (val & 0x1);
  } else if (reg == DalyProtocol::REG_CHG_MOS_SW) {
    rawBlock0[DalyProtocol::REG_CHG_MOS] =
      (rawBlock0[DalyProtocol::REG_CHG_MOS] & ~0x1) | (val & 0x1);
  }
  wantDsg = (reg == DalyProtocol::REG_DSG_MOS_SW) ? (val & 1) : wantDsg;
  wantChg = (reg == DalyProtocol::REG_CHG_MOS_SW) ? (val & 1) : wantChg;

  /* arm the job-tracker  */
  wrJob.reg = reg;
  wrJob.idx = 0;
  wrJob.startMs = millis();              // ↓↓↓  NEW  ↓↓↓
  wrJob.deadline = wrJob.startMs + 200;  // echo must arrive ≤200 ms
  wrJob.busy = true;
  wrJob.ok = false;
  bmsHoldUntil = millis() + bmsHold_delay;  // keep polling off the bus
}


void serviceWriteRegister() {
  if (!wrJob.busy) return;
  if (millis() - wrJob.startMs < 3) return;  // allow some ms for the 8 TX bytes to finish leaving the wire
  uint32_t tSlice = millis();                // start of time slice
  while (BMSSerial.available()) {
    uint8_t c = BMSSerial.read();

    // if we’re at start-of-frame, only accept 0xD2
    if (wrJob.idx == 0 && c != DalyProtocol::SLAVE_ID) {
      continue;
    }
    // second byte must be function=0x06
    if (wrJob.idx == 1 && c != 0x06) {
      wrJob.idx = 0;
      continue;
    }

    // otherwise store it
    wrJob.got[wrJob.idx++] = c;

    // once we have 8 bytes, validate…
    if (wrJob.idx == 8) {
      //DBG("[MOS ] ← RX: ");
      //for (int i = 0; i < 8; i++) DBG("%02X ", wrJob.got[i]);
      //DBG("\n");

      bool crcOK = DalyProtocol::checkCRC(wrJob.got, 8);
      wrJob.ok = crcOK
                 && wrJob.got[0] == wrJob.expect[0]
                 && wrJob.got[1] == 0x06;
      //DBG("[MOS ]   CRC %s, ID match %s\n",
      //    crcOK ? "OK" : "FAIL",
      //    wrJob.ok ? "OK" : "FAIL");

      wrJob.busy = false;
      wrJob.idx = 0;
      break;  // done for this cycle
    }

    /* ---------- yield after 2 ms of tight looping ---------- */
    if (millis() - tSlice > 2) {
      yield();            // let Idle feed the WDT
      tSlice = millis();  // restart slice timer
    }
  }

  // explicit timeout
  if (millis() > wrJob.deadline && wrJob.busy) {
    DBG("[ERR] write echo timeout – reset wrJob\n");
    wrJob.busy = false;
    wrJob.idx = 0;
  }
}






bool readSingleRegister(uint16_t reg, uint16_t& valOut) {
  // very small state machine
  enum Phase { Send,
               WaitHdr,
               WaitRest };
  static Phase phase = Send;
  static uint8_t buf[7];
  static uint8_t need = 0, got = 0;
  static uint32_t t0;

  switch (phase) {
    case Send:
      {
        uint8_t tx[8] = { DalyProtocol::SLAVE_ID, DalyProtocol::FUNC_READ_HOLDING, highByte(reg), lowByte(reg), 0x00, 0x01 };
        uint16_t crc = DalyProtocol::crc16(tx, 6);
        tx[6] = lowByte(crc);
        tx[7] = highByte(crc);
        purge_BMS_UART();
        BMSSerial.write(tx, 8);
        BMSSerial.flush();
        t0 = millis();
        phase = WaitHdr;
        break;
      }

    case WaitHdr:
      if (BMSSerial.available() >= 3) {
        BMSSerial.readBytes(buf, 3);
        need = buf[2] + 5;  // len byte + crc
        got = 3;
        phase = WaitRest;
      }
      break;

    case WaitRest:
      if (BMSSerial.available() && got < need) {
        buf[got++] = BMSSerial.read();
      }
      if (got == need) {
        if (DalyProtocol::checkCRC(buf, need) && buf[0] == DalyProtocol::SLAVE_ID && buf[1] == 0x03 && buf[2] == 2) {
          valOut = (buf[3] << 8) | buf[4];
          phase = Send;  // reset for next time
          return true;   // success!
        } else {
          phase = Send;  // retry caller side
        }
      }
      break;
  }

  // timeout guard (100 ms)
  if (millis() - t0 > 100) phase = Send;
  return false;  // not ready yet
}





// -------- generic non-blocking register writer --------------
bool ensureRegWriteNonBlocking(uint16_t reg, uint16_t val) {
  enum Phase { Idle,
               Send,
               WaitEcho,
               Verify,
               Done };
  static Phase p = Idle;
  static uint32_t t0;
  static bool ok;

  switch (p) {

    case Idle:
      if (!tryLockBms(0)) return false;
      beginWriteRegister(reg, val);
      t0 = millis();
      p = Send;
      return false;

    case Send:
      tickWriteEcho();
      if (!wrJob.busy) {
        t0 = millis();
        p = WaitEcho;
      }
      return false;

    case WaitEcho:
      if (millis() - t0 > MOS_WRITE_TIMEOUT || !wrJob.ok) {
        ok = false;
        p = Done;
      } else {
        t0 = millis();
        p = Verify;
      }
      return false;

    case Verify:
      {
        uint16_t rb;
        if (readSingleRegister(reg, rb) || millis() - t0 > 100) {
          ok = (rb == val);
          p = Done;
        }
        return false;
      }

    case Done:
      unlockBms();
      bool r = ok;
      p = Idle;
      return r;
  }
}






// Call this once from your loop():
void checkAndCalibrateSOC() {
  // —————— tuning parameters ——————
  const float FULL_V_PER_CELL = 4.1f;        // float voltage per cell to call it full
  const float TAPER_FACTOR = 10.0f;          // C/10 taper
  const uint32_t STABLE_MS = 30UL * 1000UL;  // need 30 s of stable full

  // —————— persistent state ——————
  static uint32_t t0 = 0;    // when we first saw full-conditions
  static bool done = false;  // only calibrate once until we drop out
  static bool calibInProgress = false;

  // 2) cell count from BMS
  uint16_t cells = rawBlock0[DalyProtocol::REG_BAT_QTY];

  // 3) compute thresholds
  float Vfull = FULL_V_PER_CELL * cells;                 // 4.1 * 10 = 41 V
  float Itaper = Total_Battery_capacity / TAPER_FACTOR;  // 7 Ah / 10 = 0.7 A

  uint32_t now = millis();

  // 4) detect full-condition: Voltage high enough, current above 0 (charging) but current also low enough to be near the end of the CV charging phase
  if (Battery_Voltage >= Vfull && Battery_Current_Kalman > 0.0f && Battery_Current_Kalman <= Itaper && Battery_SOC <= 98) {

    if (t0 == 0)  // first time we meet the criteria
      t0 = now;

    // after 30 s of stable full begin (or continue) the write
    if (now - t0 >= STABLE_MS && !done) {

      calibInProgress = true;  // flag that we want to run the helper
    }
  } else {
    t0 = 0;
    done = false;
  }

  /* -------- drive the non-blocking writer every loop -------- */
  if (calibInProgress) {
    if (ensureRegWriteNonBlocking(DalyProtocol::REG_SOC_SETTING, 1000)) {
      calibInProgress = false;  // success
      done = true;              // don’t repeat until next charge cycle
      bmsHoldUntil = millis() + bmsHold_delay;
      DebugSerial.println("[SOC] set to 100 % OK");
      startSOCDisplay();
    }
  }

  if (Battery_Current_Kalman > 0.0f) {
    noteActivity();  // prevent sleep while charging
  }
}






// ---------- helper to protect first‑pass dt_I ----------
inline float computeDeltaT(unsigned long& lastTs) {
  unsigned long now = millis();
  if (lastTs == 0) {  // first call ever?
    lastTs = now;     // prime the timestamp
    return 0.4f;      // safe default (s)
  }
  float dt = (now - lastTs) / 1000.0f;
  lastTs = now;
  return dt;
}



// ——— Parse & print block 0 (0x0000…0x003D) ———
// ——— Parse Block 0 (0x0000…0x003D) ———
void parseBlock0(uint8_t* data, size_t len) {



  uint16_t count = data[2] / 2;  // number of regs
  for (uint16_t i = 0; i < count && i < QTY0; i++) {
    rawBlock0[i] = (data[3 + 2 * i] << 8) | data[4 + 2 * i];
  }

  Battery_Voltage = DalyProtocol::toTotalVoltage(rawBlock0[0x28]);
  Battery_Current = DalyProtocol::toCurrent(rawBlock0[0x29]);
  Battery_SOC = DalyProtocol::toSOC(rawBlock0[0x2A]) * 100;
  Remaining_Battery_capacity = rawBlock0[0x30] * 0.1;  // in Ah
  float T_total = 0;
  for (uint8_t t = 0; t < 2; t++) {
    float tmp = DalyProtocol::toBatteryTemp(rawBlock0[32 + t]);
    T_total += tmp;
  }
  Battery_Temperature = T_total / 2.0;

  if (init_I_Kalman == 0) {
    Kalman_Current.init(Battery_Current, 1., 1., 1., R_I);
    init_I_Kalman = 1;
  }

  dt_I = computeDeltaT(lastUpdateTime_I);
  Kalman_Current.update(Battery_Current, dt_I);
  Battery_Current_Kalman = Kalman_Current.getState(0);
  float Abs_Battery_Current_Kalman = fabs(Battery_Current_Kalman);
  if (Abs_Battery_Current_Kalman < 1.0f) {
    Abs_Battery_Current_Kalman = 1.0f;  // do not allow for absurdly large numbers via a minium current
  }
  Average_Time_to_Empty = Remaining_Battery_capacity / Abs_Battery_Current_Kalman * 60.;  // eg. 4 Ah / 2 A = 2 h -> 120 min

  if (print_Daly_at_all == 1) {
    if (print_all_Daly_details == 1) {


      DebugSerial.println(F("\n=== BLOCK 0 (0x0000–0x003D) ==="));

      // 1) Cell voltages (0x00–0x1F)
      // only 10 present
      for (uint8_t i = 0; i < 10; i++) {
        float v = DalyProtocol::toCellVoltage(rawBlock0[i]);
        DebugSerial.printf("Cell %2d Voltage : %.3f V\n", i + 1, v);
      }

      // 2) Battery temperatures (0x20–0x27)
      // only 2 NTCs installed
      for (uint8_t t = 0; t < 2; t++) {
        float tmp = DalyProtocol::toBatteryTemp(rawBlock0[32 + t]);
        DebugSerial.printf("Batt Temp %d     : %.1f °C\n", t, tmp);
      }

      // 3) Total V (0x28), Current (0x29), SOC (0x2A)
      DebugSerial.printf("Total Voltage    : %.2f V\n",
                         DalyProtocol::toTotalVoltage(rawBlock0[0x28]));
      DebugSerial.printf("Current          : %.2f A\n",
                         DalyProtocol::toCurrent(rawBlock0[0x29]));
      DebugSerial.printf("State of Charge  : %.1f %%\n\n",
                         DalyProtocol::toSOC(rawBlock0[0x2A]) * 100);

      // 4) Max/Min cell V & T
      DebugSerial.printf("Max Cell Voltage : %.3f V\n",
                         DalyProtocol::toCellVoltage(rawBlock0[0x2B]));
      DebugSerial.printf("Min Cell Voltage : %.3f V\n",
                         DalyProtocol::toCellVoltage(rawBlock0[0x2C]));
      DebugSerial.printf("Max Cell Temp    : %.1f °C\n",
                         DalyProtocol::toBatteryTemp(rawBlock0[0x2D]));
      DebugSerial.printf("Min Cell Temp    : %.1f °C\n\n",
                         DalyProtocol::toBatteryTemp(rawBlock0[0x2E]));

      // 5) Charge/Discharge status (0x2F)
      {
        uint16_t st = rawBlock0[0x2F];
        const char* m[] = { "Idle", "Charging", "Discharging" };
        DebugSerial.printf("Chg/Dsg Status   : %s (%u)\n\n", m[st], st);
      }

      // 6) Remaining capacity, cell count, temp-sensor count, cycle times
      DebugSerial.printf("Remaining Cap    : %.1f Ah\n",
                         rawBlock0[0x30] * 0.1f);
      DebugSerial.printf("Cell Quantity    : %u\n", rawBlock0[0x31]);
      DebugSerial.printf("Temp Sensor #    : %u\n", rawBlock0[0x32]);
      DebugSerial.printf("Cycle Times      : %u\n\n", rawBlock0[0x33]);

      // 7) Balance, MOS, averages, power
      DebugSerial.printf("Balance State    : %s\n",
                         rawBlock0[0x34] ? "On" : "Off");
      DebugSerial.printf("Charge MOS       : %s\n",
                         rawBlock0[0x35] ? "On" : "Off");
      DebugSerial.printf("Discharge MOS    : %s\n\n",
                         rawBlock0[0x36] ? "On" : "Off");
      DebugSerial.printf("Avg Cell Voltage : %.3f V\n",
                         DalyProtocol::toAverageVoltage(rawBlock0[0x37]));
      DebugSerial.printf("Voltage Diff     : %.3f V\n",
                         DalyProtocol::toVoltageDiff(rawBlock0[0x38]));
      DebugSerial.printf("Power            : %.0f W\n\n",
                         DalyProtocol::toPower(rawBlock0[0x39]));

      // 8) Fault statuses (0x3A–0x3D)
      for (uint8_t f = 0; f < 4; f++) {
        DebugSerial.printf("Fault Status %u   : 0x%04X\n",
                           f + 1, rawBlock0[0x3A + f]);
      }

    } else {  // print only some details

      //DebugSerial.print(", V_bat");
      DebugSerial.printf("%.1f V", Battery_Voltage);

      //DebugSerial.print(", I");
      DebugSerial.printf(", %+5.2f/%+6.3f A", Battery_Current, Battery_Current_Kalman);

      //DebugSerial.print(", SOC");
      DebugSerial.printf(", %5.1f %%", Battery_SOC);
      DebugSerial.printf(", %5.1f min", Average_Time_to_Empty);

      // 1) Cell voltages (0x00–0x1F), only 10 present
      DebugSerial.print(", Cells");
      for (uint8_t i = 0; i < 10; i++) {
        float v = DalyProtocol::toCellVoltage(rawBlock0[i]);
        DebugSerial.printf(", %.3f", v);
      }

      //DebugSerial.print(", Temp");
      for (uint8_t t = 0; t < 2; t++) {
        float tmp = DalyProtocol::toBatteryTemp(rawBlock0[32 + t]);
        DebugSerial.printf(", %.1f", tmp);
      }
      DebugSerial.print(", °C");


      DebugSerial.printf(", Bal, %s", rawBlock0[0x34] ? "1" : "0");
      DebugSerial.printf(", Charge, %s", rawBlock0[0x35] ? "1" : "0");
      DebugSerial.printf(", Discharge, %s", rawBlock0[0x36] ? "1" : "0");

      /*
      DebugSerial.print(F(", Faults"));
      for (uint8_t f = 0; f < 4; f++) {
        uint16_t val = rawBlock0[0x3A + f];
        DebugSerial.printf(", %u = 0x%04X", f + 1, val);
      }
      */

      // 9) Human-readable decode of each
      DebugSerial.print(", Decoded");
      DebugSerial.printf(", %s", DalyProtocol::decodeFault1(rawBlock0[0x3A]).c_str());
      DebugSerial.printf("/%s", DalyProtocol::decodeFault2(rawBlock0[0x3B]).c_str());
      DebugSerial.printf("/%s", DalyProtocol::decodeFault3(rawBlock0[0x3C]).c_str());
      DebugSerial.printf("/%s", DalyProtocol::decodeFault4(rawBlock0[0x3D]).c_str());
      // 10) Battery Status token for BikeBus (0x16)
      BikeBus_statusBits = DalyProtocol::getBatteryStatusBits(rawBlock0);
      DebugSerial.printf(", BikeBus: 0x%04X", BikeBus_statusBits);


      DebugSerial.println("");
    }
  }
}

// ——— Parse Block 1 (0x0080…0x00A8) ———
#define IDX1(r) rawBlock1[(r)-0x80]

void parseBlock1(uint8_t* data, size_t len) {
  uint16_t count = data[2] / 2;
  for (uint16_t i = 0; i < count && i < QTY1; i++)
    rawBlock1[i] = (data[3 + 2 * i] << 8) | data[4 + 2 * i];

  if (print_Daly_at_all == 1) {
    DebugSerial.println(F("\n=== BLOCK 1 (0x0080–0x00A8) ==="));

    DebugSerial.printf("Rated Capacity     : %.1f Ah\n",
                       IDX1(DalyProtocol::REG_RATED_CAP));
    DebugSerial.printf("Cell Ref Voltage   : %.3f V\n",
                       DalyProtocol::toCellVoltage(IDX1(DalyProtocol::REG_RATED_VOLT)));
    DebugSerial.printf("Acq Boards #       : %u\n", IDX1(DalyProtocol::REG_NUM_BOARDS));
    for (int b = 0; b < 3; b++) {
      DebugSerial.printf("Units Board %d     : %u\n",
                         b + 1, IDX1(DalyProtocol::REG_UNITS_BRD1 + b));
    }
    for (int b = 0; b < 3; b++) {
      DebugSerial.printf("Temp Board %d      : %.1f °C\n",
                         b + 1, DalyProtocol::toBatteryTemp(IDX1(DalyProtocol::REG_TEMP_BRD1 + b)));
    }
    {
      const char* types[] = { "LiFePO4", "Ternary", "LTO" };
      uint16_t t = IDX1(DalyProtocol::REG_BATTERY_TYPE);
      DebugSerial.printf("Battery Type       : %s (%u)\n\n",
                         (t < 3 ? types[t] : "Unknown"), t);
    }

    DebugSerial.printf("Sleep Wait Time    : %u s\n\n",
                       IDX1(DalyProtocol::REG_SLEEP_TIME_S));

    // Cell voltage alarms
    DebugSerial.printf("CellV HI Alarm L1  : %.3f V\n",
                       IDX1(DalyProtocol::REG_ALM1_VOLT_HI1) * 0.001f);
    DebugSerial.printf("CellV HI Alarm L2  : %.3f V\n",
                       IDX1(DalyProtocol::REG_ALM2_VOLT_HI1) * 0.001f);
    DebugSerial.printf("CellV LO Alarm L1  : %.3f V\n",
                       IDX1(DalyProtocol::REG_ALM1_VOLT_LO1) * 0.001f);
    DebugSerial.printf("CellV LO Alarm L2  : %.3f V\n\n",
                       IDX1(DalyProtocol::REG_ALM2_VOLT_LO1) * 0.001f);

    // Total voltage alarms
    DebugSerial.printf("TotalV HI Alarm L1 : %.1f V\n",
                       IDX1(DalyProtocol::REG_ALM1_VOLT_HI2) * 0.1f);
    DebugSerial.printf("TotalV HI Alarm L2 : %.1f V\n",
                       IDX1(DalyProtocol::REG_ALM2_VOLT_HI2) * 0.1f);
    DebugSerial.printf("TotalV LO Alarm L1 : %.1f V\n",
                       IDX1(DalyProtocol::REG_ALM1_VOLT_LO2) * 0.1f);
    DebugSerial.printf("TotalV LO Alarm L2 : %.1f V\n\n",
                       IDX1(DalyProtocol::REG_ALM2_VOLT_LO2) * 0.1f);

    // Current alarms
    DebugSerial.printf("ChargeCur HI L1    : %.1f A\n",
                       DalyProtocol::toCurrent(IDX1(DalyProtocol::REG_ALM1_CHG_CUR)));
    DebugSerial.printf("ChargeCur HI L2    : %.1f A\n",
                       DalyProtocol::toCurrent(IDX1(DalyProtocol::REG_ALM2_CHG_CUR)));
    DebugSerial.printf("DischCur HI L1     : %.1f A\n",
                       DalyProtocol::toCurrent(IDX1(DalyProtocol::REG_ALM1_DSG_CUR)));
    DebugSerial.printf("DischCur HI L2     : %.1f A\n\n",
                       DalyProtocol::toCurrent(IDX1(DalyProtocol::REG_ALM2_DSG_CUR)));

    // Temperature alarms
    DebugSerial.printf("ChgTemp HI L1      : %.1f °C\n",
                       DalyProtocol::toBatteryTemp(IDX1(DalyProtocol::REG_ALM1_CHG_TMP)));
    DebugSerial.printf("ChgTemp HI L2      : %.1f °C\n",
                       DalyProtocol::toBatteryTemp(IDX1(DalyProtocol::REG_ALM2_CHG_TMP)));
    DebugSerial.printf("DsgTemp HI L1      : %.1f °C\n",
                       DalyProtocol::toBatteryTemp(IDX1(DalyProtocol::REG_ALM1_DSG_TMP)));
    DebugSerial.printf("DsgTemp HI L2      : %.1f °C\n\n",
                       DalyProtocol::toBatteryTemp(IDX1(DalyProtocol::REG_ALM2_DSG_TMP)));

    // Excess diff alarms
    DebugSerial.printf("Volt Diff Alarm L1 : %u mV\n",
                       IDX1(DalyProtocol::REG_ALM1_DIFF_VOLT));
    DebugSerial.printf("Temp Diff Alarm L1 : %u °C\n\n",
                       IDX1(DalyProtocol::REG_ALM2_DIFF_VOLT));

    // Balancing & MOS
    DebugSerial.printf("Balancing Voltage  : %u mV\n",
                       IDX1(DalyProtocol::REG_BAL_VOLT));
    DebugSerial.printf("Equil Open Diff    : %u mV\n",
                       IDX1(DalyProtocol::REG_EQ_OPEN_DIFF));
    DebugSerial.printf("Charge MOS Switch  : %s\n",
                       IDX1(DalyProtocol::REG_CHG_MOS_SW) ? "ON" : "OFF");
    DebugSerial.printf("Dischg MOS Switch  : %s\n\n",
                       IDX1(DalyProtocol::REG_DSG_MOS_SW) ? "ON" : "OFF");

    DebugSerial.printf("SOC Setting        : %.3f\n",
                       IDX1(DalyProtocol::REG_SOC_SETTING) * 0.001f);
    DebugSerial.printf("MOS Temp Protect   : %.1f °C\n\n",
                       DalyProtocol::toBatteryTemp(IDX1(DalyProtocol::REG_TMP_ALM)));
  }
}



void manageMos() {
  static bool also_turn_on_CHG_MOS = 0;
  // ─── watchdog: reset to Idle if we get stuck ───
  if (mosState != Idle && millis() - mosStateStartMs > MOS_STATE_TIMEOUT_MS) {
    DBG("[MOS ] timeout in state %d (phase=%d, wrJob.busy=%d, readBusy=%d) – forcing Idle\n",
        (int)mosState, (int)mosChkPhase, (int)wrJob.busy, (int)bms.readBusy());
    mosState = Idle;
    mosStateStartMs = millis();
  }

  bool phaseNeedsUart =
    (mosState == StartDisOn) || (mosState == StartDisOff) || (mosState == StartChgOn) || (mosState == StartChgOff);

  if (phaseNeedsUart && bms.readBusy())
    return;  // wait until the Daly read is over


  if (mosState == Idle && requestWakeFromTap) {
    requestWakeFromTap = false;
    mosState = StartDisOn;
    mosStateStartMs = millis();
  }

  // also if battery‐plug changed (plugPending), treat same as a wake
  if (mosState == Idle && plugPending) {
    plugPending = false;
    mosState = StartDisOn;
    mosStateStartMs = millis();
  }

  switch (mosState) {
    case Idle:
      // nothing to do until either flag
      break;

      // ----- step 1: turn Discharge MOS on -----
    case StartDisOn:
      // trigger the non-blocking change
      if (ensureMosStateNonBlocking(DalyProtocol::REG_DSG_MOS_SW, true)) {
        mosState = WaitDisOn;
        mosTimer = millis();
        mosStateStartMs = millis();
      }
      break;


    case WaitDisOn:
      // make sure we give the BMS time to enable MOSFETS etc.
      if (millis() - mosTimer < mosSettle_delay)
        break;

      if (mosChkPhase != MosChkPhase::Idle)  // helper guard
        break;

      if (!mosChkResult) {             // error path
        also_turn_on_CHG_MOS = false;  // do not turn on other MOSFET
      }

      mosState = SamplePlug;  // move on to the next high-level step
      mosStateStartMs = millis();
      break;




    // ----- step 2: sample plug pin -----
    case SamplePlug:
      if (digitalRead(BAT_PLUG_PIN) == LOW) {
        DBG("[PLUG] → keep DSG ON, also enable CHG\n");
        also_turn_on_CHG_MOS = true;  // turn on the charge MOSFET
        mosState = StartChgOn;
        mosStateStartMs = millis();
      } else {
        DBG("[PLUG] no plug → turn all MOS OFF\n");
        also_turn_on_CHG_MOS = false;  // turn off the charge MOSFET
        mosState = StartDisOff;
        mosStateStartMs = millis();
      }
      break;


    // ----- step 3a: if no battery, turn Discharge OFF -----
    case StartDisOff:
      if (ensureMosStateNonBlocking(DalyProtocol::REG_DSG_MOS_SW, false)) {
        mosState = WaitDisOff;
        mosTimer = millis();
        mosStateStartMs = millis();
      }
      break;

    case WaitDisOff:
      if (millis() - mosTimer < mosSettle_delay) break;
      if (mosChkPhase != MosChkPhase::Idle) break;

      if (!also_turn_on_CHG_MOS) {  // do we want to turn off the charge MOSFET?
        mosState = StartChgOff;
        mosStateStartMs = millis();
      } else {
        mosState = Done;
        mosStateStartMs = millis();
        print_delay_timer = millis();
      }

      break;

    // ----- step 3b: if battery present, turn Charge ON -----
    case StartChgOn:
      if (ensureMosStateNonBlocking(DalyProtocol::REG_CHG_MOS_SW, true)) {
        // only when that returns true do we advance to the next state:
        mosState = WaitChg;
        mosTimer = millis();
        mosStateStartMs = millis();
      }
      break;

    case StartChgOff:
      if (ensureMosStateNonBlocking(DalyProtocol::REG_CHG_MOS_SW, false)) {
        // only when that returns true do we advance to the next state:
        mosState = WaitChg;
        mosTimer = millis();
        mosStateStartMs = millis();
      }
      break;

    case WaitChg:
      if (millis() - mosTimer < mosSettle_delay) break;
      if (mosChkPhase != MosChkPhase::Idle) break;

      also_turn_on_CHG_MOS = false;  // reset
      mosState = Done;
      print_delay_timer = millis();
      mosStateStartMs = millis();
      break;

    case Done:
      if (millis() - print_delay_timer < mosSettle_delay) break;
      mosState = Print;
      mosStateStartMs = millis();
      break;


    // ----- final state: we're done until next event -----
    case Print:
      // ensure both MOS match desired
      if (mosHwMatches()) {
        DebugSerial.printf("Balance State    : %s\n", rawBlock0[0x34] ? "On" : "Off");
        DebugSerial.printf("Charge MOS       : %s\n", rawBlock0[0x35] ? "On" : "Off");
        DebugSerial.printf("Discharge MOS    : %s\n", rawBlock0[0x36] ? "On" : "Off");
        mosState = Idle;
        requestWakeFromTap = false;
        mosStateStartMs = millis();
      } else {
        /*
        statusLed(true, 255, 255, 255);  // debug
        delay(500);
        statusLed(true, 50, 50, 50);  // debug
        delay(500);
        statusLed(true, 0, 0, 255);  // debug
        delay(500);
        statusLed(false, 0, 0, 0);  // debug
        delay(500);
        */
        DBG("[PLUG] both MOS do not match\n");
        DebugSerial.printf("Balance State    : %s\n", rawBlock0[0x34] ? "On" : "Off");
        DebugSerial.printf("Charge MOS       : %s\n", rawBlock0[0x35] ? "On" : "Off");
        DebugSerial.printf("Discharge MOS    : %s\n", rawBlock0[0x36] ? "On" : "Off");
        mosState = Idle;
        requestWakeFromTap = false;
        mosStateStartMs = millis();
      }
      break;
  }
}




/* ---------- Bike-Bus slave handler ---------------------------- */
void handleBikeBus() {
  static bool busLed = false;   // debug
  static bool busLed2 = false;  // debug
  /* ---------- 1.  put any pending reply on the wire ---------- */
  static uint8_t replyBuf[5];
  static bool replyPending = false;
  static uint32_t replyReadyAt = 0;  // when we may send

  if (replyPending && millis() >= replyReadyAt) {
    if (!socActive) {
      // debug
      busLed2 = !busLed2;
      digitalWrite(STATUS_LED_PINS[0], busLed2 ? LOW : HIGH);
    }
    //Serial.printf("BikeBus TX: %02X %02X %02X %02X %02X\n",
    //              replyBuf[0], replyBuf[1], replyBuf[2],
    //              replyBuf[3], replyBuf[4]);

    for (uint8_t i = 0; i < 5; ++i) {
      BikebusSerial.write(replyBuf[i]);
      if (i == 1) delayMicroseconds(100);  // timing spec
    }
    BikebusSerial.flush();

    replyPending = false;  // done
    //DBG("[DBG] Bike-Bus activity seen\n");
    noteActivity();
  }



  /* ---------- 2.  receive & parse one complete frame --------- */
  static uint8_t idx = 0;  // local to this function only
  static uint32_t frameStart = 0;
  uint32_t sliceT0 = millis();  //  start of 2-ms slice
  while (BikebusSerial.available()) {
    uint8_t b = BikebusSerial.read();
    uint32_t now = millis();
    //DebugSerial.printf("%d, busIdx %d -> %02X\n", now, idx, b);

    /* --- only 0x20 can start a new frame ------------------ */
    if (idx == 0) {
      if (b != BATTERY_ADDRESS) continue;  // ignore noise / 0x00 etc.
      frameStart = now;
    } else if (now - frameStart > FRAME_TIMEOUT) {
      /* took too long → abort this attempt */
      DebugSerial.println("[WARN] FRAME TIMEOUT");
      idx = 0;
      replyPending = false;  // reset
      continue;
    }

    busBuf[idx++] = b;

    // wait until we have all 5 bytes
    if (idx < 5) {
      /* ---------- give scheduler a chance every 2 ms ---- */
      if (millis() - sliceT0 > 2) {
        yield();             // let Idle task feed TG0 WDT
        sliceT0 = millis();  // restart slice timer
      }
      continue;  // need more bytes
    }

    idx = 0;  // ready for next frame

    /* ---- got a complete, CRC-checked frame --------------- */
    uint8_t addr = busBuf[0], token = busBuf[1];
    uint8_t lo = busBuf[2], hi = busBuf[3];
    uint8_t crc = (addr + token + lo + hi) & 0xFF;

    //DebugSerial.printf("BikeBus RX: %02X %02X %02X %02X %02X\n",
    //              addr, token, lo, hi, busBuf[4]);


    if (addr != BATTERY_ADDRESS) {  // not for us, ignore
      continue;
    }
    if (crc != busBuf[4]) {
      DebugSerial.println("[WARN] bad CRC – dropped");
      continue;
    }

    if (!socActive) {
      // debug
      busLed = !busLed;
      digitalWrite(STATUS_LED_PINS[1], busLed ? LOW : HIGH);
    }

    /* ---- assemble reply, but DON’T transmit yet ---------- */
    uint16_t val = BikeBusProtocol::getValueForToken(
      token, rawBlock0,
      Battery_Current,
      Battery_Current_Kalman,
      Battery_Temperature,
      Average_Time_to_Empty,
      Battery_Voltage,
      Battery_SOC,
      BikeBus_statusBits);

    replyBuf[0] = MASTER_ADDRESS;
    replyBuf[1] = token;
    replyBuf[2] = val & 0xFF;
    replyBuf[3] = (val >> 8) & 0xFF;
    replyBuf[4] = (replyBuf[0] + replyBuf[1] + replyBuf[2] + replyBuf[3]) & 0xFF;


    replyReadyAt = millis() + BikeBus_reply_delay;  // honour silence
    replyPending = true;
    return;  // leave – we’ll continue next loop()
  }
}






void initSOCDisplay() {
  pinMode(BAT_PLUG_PIN, INPUT);
  // LEDs off = drive HIGH
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(STATUS_LED_PINS[i], OUTPUT);
    digitalWrite(STATUS_LED_PINS[i], HIGH);
  }
  // button with pull-up
  pinMode(SOC_BUTTON_PIN, INPUT_PULLUP);

  // Debouncer initialization (do once at boot)
  plugLastRawLevel = digitalRead(BAT_PLUG_PIN);
  plugLastAcceptedLevel = plugLastRawLevel;  // treat current level as already accepted
  plugLevelStableSince = millis();
  plugLastEventMs = millis();
  plugCooloffUntil = 0;
}

// Called when button pressed to begin non-blocking SOC display
void startSOCDisplay() {
  // clamp SOC [0–100]
  int socVal = int(Battery_SOC);
  if (socVal < 0) socVal = 0;
  if (socVal > 100) socVal = 100;

  // decide if "empty" blink mode
  socEmptyBlink = (socVal < SOC_EMPTY_THRESHOLD);

  if (!socEmptyBlink) {
    // compute full LEDs and partial
    socFull = (socVal + SOC_MARGIN) / 25;
    if (socFull > 4) socFull = 4;
    socPartial = false;
    if (socFull < 4) {
      int rem = socVal - socFull * 25;
      socPartial = (rem >= SOC_MARGIN && rem < SOC_MARGIN + SOC_BLINK_WINDOW);
    }
  } else {
    socFull = 0;
    socPartial = false;
  }

  socStartTime = millis();
  socLastBlink = socStartTime;
  socBlinkOn = false;
  socActive = true;
  DebugSerial.printf("SOC: %5.1f %%\n", Battery_SOC);
}



// Must be called each loop to update LEDs
void updateSOCDisplay() {
  if (!socActive) return;

  unsigned long now = millis();

  // end display after duration
  if (now - socStartTime >= SOC_DURATION) {
    // turn all off
    for (uint8_t i = 0; i < 4; ++i) {
      digitalWrite(STATUS_LED_PINS[i], HIGH);
    }
    socActive = false;
    return;
  }

  // — empty-blink mode: blink all 4 LEDs —
  if (socEmptyBlink) {
    if (now - socLastBlink >= SOC_BLINK_PERIOD) {
      socBlinkOn = !socBlinkOn;
      socLastBlink = now;
      for (uint8_t i = 0; i < 4; ++i) {
        digitalWrite(STATUS_LED_PINS[i], socBlinkOn ? LOW : HIGH);
      }
    }
    return;  // skip normal display logic
  }

  // — partial blink for next LED —
  if (socPartial && now - socLastBlink >= SOC_BLINK_PERIOD) {
    socBlinkOn = !socBlinkOn;
    socLastBlink = now;
    digitalWrite(
      STATUS_LED_PINS[socFull],
      socBlinkOn ? LOW : HIGH);
  }

  // — solid-on for full LEDs —

  for (uint8_t i = 0; i < socFull; ++i) {
    digitalWrite(STATUS_LED_PINS[i], LOW);
  }
  // — off for anything beyond full+partial —
  uint8_t offStart = socFull + (socPartial ? 1 : 0);
  for (uint8_t i = offStart; i < 4; ++i) {
    digitalWrite(STATUS_LED_PINS[i], HIGH);
  }
}



/* ---------------  ISR-driven input service  ---------------- */
void serviceInputs() {
  uint32_t now = millis();

  /* ——- SOC button (tap)  ------------------------- */
  bool currBtn = digitalRead(SOC_BUTTON_PIN);  // LOW = pressed
  static uint32_t lastTapMs = 0;               // ≥200 ms repeat block
  static bool prevBtnState = HIGH;

  /* b) ISR flag (for the wake-from-sleep case)  */
  if (buttonTapFlag) {  // set in socButtonIsr()
    buttonTapFlag = false;
    DBG("[DBG] SOC: buttonTapFlag flag seen\n");
    startSOCDisplay();
    requestWakeFromTap = true;  // request Battery connector state check
    noteActivity();
  }




  /* ——- Rosenberg plug debounced handling (non-blocking) —— */
  if (plugChangedFlag) {      // set by ISR on any CHANGE
    plugChangedFlag = false;  // consume asap
    noteActivity();
  }
  int raw = digitalRead(BAT_PLUG_PIN);  // HIGH/LOW

  // Track raw changes and (re)start stability timer
  if (raw != plugLastRawLevel) {
    plugLastRawLevel = raw;
    plugLevelStableSince = now;  // start stability window
  }

  // If MOS sequence running or in cooldown, don't act yet
  if (mosState != Idle || now < plugCooloffUntil) {
    return;
  }

  // Has the raw level been stable long enough?
  bool stable = (now - plugLevelStableSince >= PLUG_DEBOUNCE_MS);

  // Accept exactly one event when:
  //  - level is stable
  //  - level differs from the last accepted level
  //  - min spacing is satisfied
  if (stable && raw != plugLastAcceptedLevel && (now - plugLastEventMs >= PLUG_MIN_EVENT_SPACING_MS)) {

    plugLastAcceptedLevel = raw;  // remember the accepted level
    plugLastEventMs = now;
    plugCooloffUntil = now + 1200;  // brief guard against late wiggles

    // Queue the higher-level action once; manageMos() will do the rest
    plugPending = true;
    startSOCDisplay();
    DBG("[DBG] Rosenberg accepted: level=%d (stable %lu ms)\n",
        raw, (unsigned long)(now - plugLevelStableSince));
  }
}


void restartBikeBusUart() {

  uart_set_pin(UART_NUM_1,      // restore matrix
               BIKEBUS_TX_PIN,  // TX = GPIO10
               BIKEBUS_RX_PIN,  // RX = GPIO4
               UART_PIN_NO_CHANGE,
               UART_PIN_NO_CHANGE);

  uart_flush_input(UART_NUM_1);  // drop partial frame
  //uart_enable_rx_intr(UART_NUM_1);  // make sure RX IRQ is on
}




// Call this once per loop to drive the MOS-change state machine.
// Returns true on success (phase complete), false if still in progress.
bool ensureMosStateNonBlocking(uint16_t reg, bool turnOn) {
  switch (mosChkPhase) {
    case MosChkPhase::Idle:
      // Try to grab the UART mutex without waiting
      if (!tryLockBms(0)) {
        return false;  // another operation is in flight
      }
      // Kick off the write
      mosChkReg = reg;
      mosChkTarget = turnOn;
      beginWriteRegister(reg, turnOn ? 1 : 0);
      mosChkT0 = millis();
      mosChkPhase = MosChkPhase::SendCmd;
      return false;

    case MosChkPhase::SendCmd:
      // Wait for the echo to finish or timeout
      tickWriteEcho();
      if (!wrJob.busy) {
        // Echo done (ok or not)
        mosChkPhase = MosChkPhase::WaitEcho;
        mosChkT0 = millis();
      }
      return false;

    case MosChkPhase::WaitEcho:
      // If echo failed or timed out, bail
      if (millis() - mosChkT0 > MOS_WRITE_TIMEOUT || !wrJob.ok) {
        mosChkResult = false;
        mosChkPhase = MosChkPhase::Done;
      } else {
        // Echo succeeded → read back the register
        mosChkPhase = MosChkPhase::VerifyRead;
        mosChkT0 = millis();
      }
      return false;

    case MosChkPhase::VerifyRead:
      {
        // Attempt a non-blocking read
        uint16_t val;
        if (readSingleRegister(mosChkReg, val)) {
          mosChkResult = ((val & 1) == (mosChkTarget ? 1 : 0));
          mosChkPhase = MosChkPhase::Done;
        } else if (millis() - mosChkT0 > 100) {
          // read timed out
          mosChkResult = false;
          mosChkPhase = MosChkPhase::Done;
        }
        return false;
      }

    case MosChkPhase::Done:
      unlockBms();
      // Optionally log failure:
      if (!mosChkResult) {
        DBG("[MOS] verify failed for reg=0x%04X\n", mosChkReg);
      }
      mosChkPhase = MosChkPhase::Idle;
      return mosChkResult;
  }
  return false;  // should never reach
}









inline void enterSleepIfIdle() {
  const uint32_t now = millis();

  /* a) postpone sleep if… */

  if (now - lastActiveMs < INACTIVITY_MS) {
    //statusLed(true, 50, 50, 50);  // debug
    //DBG("[DBG] sleeping aborted, still active\n");
    return;
  }  // user/bus traffic
  if (wrJob.busy) {
    statusLed(true, 0, 0, 255);  // debug
    //DBG("[DBG] sleeping aborted, busy\n");
    return;
  }  // UART job ongoing
  if (bms.readBusy()) {
    statusLed(true, 255, 0, 0);  // debug
    return;
  }

  if (mosState != Idle) {
    statusLed(true, 100, 100, 0);  // debug
    //DBG("[DBG] sleeping aborted, STEP_IDLE\n");
    return;
  }  // MOS sequence busy
  if (socActive) {
    statusLed(true, 255, 100, 255);  // debug
    //DBG("[DBG] sleeping aborted, socActive\n");
    return;
  }  // stay awake while LEDs are on

  // make sure the line is not high (= master still online)
  // if (digitalRead(BIKEBUS_RX_PIN) == HIGH) {
  //   return;
  // }


  //statusLed(true, 255, 0, 0);  // debug
  /* b) we’re idle – prepare peripherals for low-power        */
  BikebusSerial.flush();
  BMSSerial.flush();
  DebugSerial.flush();
  // make sure all LEDS are off
  statusLed(false);
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(STATUS_LED_PINS[i], HIGH);
  }


  bool lowNow = digitalRead(BAT_PLUG_PIN) == LOW;
  gpio_wakeup_enable(static_cast<gpio_num_t>(BAT_PLUG_PIN), lowNow ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL);  // wake on change

  bool Plug_lowNow = digitalRead(BIKEBUS_RX_PIN) == LOW;
  gpio_wakeup_enable(static_cast<gpio_num_t>(BIKEBUS_RX_PIN), Plug_lowNow ? GPIO_INTR_HIGH_LEVEL : GPIO_INTR_LOW_LEVEL);  // wake on HIGH level, master pulls LIN high when starting
                                                                                                                          // but if not plugged in, it is high
                                                                                                                          // so wake on change to be able to sleep when unplugged

  digitalWrite(ATA6661_EN_PIN, LOW);                      // put ATA6661 into Sleep
  gpio_hold_en(static_cast<gpio_num_t>(ATA6661_EN_PIN));  // keep it low through sleep


  /*  watchdog off while we sleep */
  esp_task_wdt_delete(NULL);  // SW task-WDT off
  //disableCore0WDT();          // HW system WDT off

  delay(1);

  esp_light_sleep_start();

  /* ---- woke up, re-enable watchdog ---- */
  esp_task_wdt_add(NULL);  // SW task-WDT again
  //enableCore0WDT();        // HW system WDT


  gpio_hold_dis(static_cast<gpio_num_t>(ATA6661_EN_PIN));  // let it float again
  digitalWrite(ATA6661_EN_PIN, HIGH);                      // back to Normal as soon as we wake

  gpio_wakeup_disable(static_cast<gpio_num_t>(BAT_PLUG_PIN));
  gpio_wakeup_disable(static_cast<gpio_num_t>(BIKEBUS_RX_PIN));

  //initWakeups();  // re-attach interrupts & re-enable gpio wake levels
  //statusLed(true, 255, 100, 255);

  wakeMask = esp_sleep_get_gpio_wakeup_status();
  //statusLed(true, 255, 255, 255);
  DebugSerial.end();          // tear down the old instance
  DebugSerial.begin(115200);  // bring it back up
  //statusLed(true, 255, 100, 0);

  //while (!DebugSerial);
  //DBG("[DBG] woke; mask=0x%llX button=%d plug=%d\n",
  //    wakeMask, (wakeMask >> SOC_BUTTON_PIN) & 1,
  //    (wakeMask >> BAT_PLUG_PIN) & 1);


  /*
  statusLed(true, 255, 255, 255);
  delay(1000);
  statusLed(true, 0, 100, 100);  // debug
  delay(1000);
  statusLed(true, 100, 0, 100);  // debug
  delay(1000);
  statusLed(true, 255, 0, 0);  // debug
*/

  //uart_disable_rx_intr(UART_NUM_1);  // <-- halt incoming data
  //uart_flush_input(UART_NUM_1);      // empty FIFO & ring buffer
  //BikebusSerial.end();  // now returns immediately
  //BikebusSerial.begin(BAUD_RATE, SERIAL_8N1,
  //                    BIKEBUS_RX_PIN,   // GPIO4
  //                   BIKEBUS_TX_PIN);  // GPIO10

  //statusLed(true, 255, 255, 255);
  //delay(1000);
  //statusLed(true, 0, 0, 255);  // debug
  //delay(1000);
  restartBikeBusUart();

  statusLed(true, 100, 0, 100);  // debug
  //delay(1000);
  //statusLed(true, 100, 100, 0);  // debug
  //delay(1000);
  //statusLed(true, 0, 0, 255);

  /* drain any spurious RX bytes */
  uint32_t purgeT0 = millis();
  while (BikebusSerial.available() && millis() - purgeT0 < 6)  // 5 ms watchdog
  {
    statusLed(true, 255, 0, 255);
    BikebusSerial.read();
  }

  statusLed(false);
  uint32_t wake_hi = wakeMask >> 32;
  uint32_t wake_lo = wakeMask & 0xFFFFFFFF;

  DBG("[DBG] woke; mask=0x%08X%08X "
      "button=%d plug=%d bikebus=%d\n",
      wake_hi, wake_lo,
      (wakeMask >> SOC_BUTTON_PIN) & 1,
      (wakeMask >> BAT_PLUG_PIN) & 1,
      (wakeMask >> BIKEBUS_RX_PIN) & 1);

  DBG("[DBG] sleep -> wake\n");
  noteActivity();  // prevent immediate re-sleep
}


void initWakeups() {
  esp_sleep_enable_gpio_wakeup();  // master switch

  attachInterrupt(digitalPinToInterrupt(SOC_BUTTON_PIN),
                  socButtonIsr,  // use this function
                  FALLING);

  attachInterrupt(digitalPinToInterrupt(BAT_PLUG_PIN),
                  plugIsr,  // use this function
                  CHANGE);

  // SOC button – LOW level wakes
  gpio_wakeup_enable(static_cast<gpio_num_t>(SOC_BUTTON_PIN), GPIO_INTR_LOW_LEVEL);

  // --- Bike-Bus: UART-RX wake - the line is idle HIGH, first start bit pulls it LOW ---
  //gpio_hold_en(static_cast<gpio_num_t>(BIKEBUS_RX_PIN));  // keep pull-up through sleep
  //gpio_wakeup_enable(static_cast<gpio_num_t>(BIKEBUS_RX_PIN),
  //                   GPIO_INTR_HIGH_LEVEL);  // wake on HIGH level, master pulls LIN high when starting
}


void setup() {
  DebugSerial.begin(115200);
  delay(1000);  // let MCU connect

  // create a mutex (starts unlocked by default)
  bmsMutex = xSemaphoreCreateMutex();
  if (!bmsMutex) {
    DBG("[ERR] failed to create bmsMutex\n");
    while (1) delay(1000);
  }

  loadSettings();  // ← restore persisted flags

  initSOCDisplay();
  DBG("[DBG] SOC display init done\n");
  pinMode(ATA6661_EN_PIN, OUTPUT);
  digitalWrite(ATA6661_EN_PIN, HIGH);  // wake ATA6661

  pinMode(BIKEBUS_RX_PIN, INPUT_PULLUP);  // debug
  pinMode(BIKEBUS_TX_PIN, INPUT_PULLUP);  // debug
  //pinMode(BIKEBUS_RX_PIN, OUTPUT);  // debug
  //pinMode(BIKEBUS_TX_PIN, OUTPUT); // debug
  //digitalWrite(BIKEBUS_RX_PIN, HIGH); // debug
  //digitalWrite(BIKEBUS_TX_PIN, HIGH); // debug
  BikebusSerial.begin(BAUD_RATE, SERIAL_8N1, BIKEBUS_RX_PIN, BIKEBUS_TX_PIN);
  //gpio_set_drive_capability(GPIO_NUM_17, GPIO_DRIVE_CAP_3);  // bump drive strength on that pin
  //gpio_pullup_dis(GPIO_NUM_17);                              // disable any internal pulls so it’s driven only by the UART block
  //gpio_pulldown_dis(GPIO_NUM_17);

  initWakeups();



  DBG("[DBG] wake init done\n");

  BMSSerial.setRxBufferSize(256);
  BMSSerial.begin(BMS_BAUD, SERIAL_8N1, BMS_RX_PIN, BMS_TX_PIN);
  bms.begin(BMSSerial, BMS_BAUD, QTY0);

  DBG("[DBG] all UART started\n");
  pinMode(RGB_LED_PIN, OUTPUT);
  statusLed(false);
  statusLed(true);

  delay(200);  // let things settle


  manageMos();

  /*
  bool plugPresent = (digitalRead(BAT_PLUG_PIN) == LOW);
  if (!plugPresent) {       // 2) no plug →
    ensureMosState(false);  //    turn OFF
    DebugSerial.println(F("[INFO] plug absent – MOS off"));
  } else {
    DebugSerial.println(F("[INFO] plug present – MOS stay on"));
  }
  */


  DBG("[DBG] MOS state checked\n");
  bmsHoldUntil = millis() + 50;  // grace period before first poll

  //beginWriteRegister(DalyProtocol::REG_ALM1_VOLT_HI1, 4220);  // cell voltage alarm level 1 at 4220 mV
  //beginWriteRegister(DalyProtocol::REG_ALM2_VOLT_HI1, 4250);  // cell voltage alarm level 2 at 4250 mV
  //beginWriteRegister(DalyProtocol::REG_ALM1_VOLT_HI2, 422);   // total voltage alarm level 1 at 42.2 V
  //beginWriteRegister(DalyProtocol::REG_ALM2_VOLT_HI2, 425);   // total voltage alarm level 2 at 42.5 V


  digitalWrite(STATUS_LED_PINS[0], LOW);
  delay(400);
  digitalWrite(STATUS_LED_PINS[1], LOW);
  delay(400);
  digitalWrite(STATUS_LED_PINS[2], LOW);
  delay(400);
  digitalWrite(STATUS_LED_PINS[3], LOW);
  delay(400);
  for (uint8_t i = 0; i < 4; ++i) {
    digitalWrite(STATUS_LED_PINS[i], HIGH);
  }


  lastBusActivity = 0;  // so the sleep logic triggers immediately
  noteActivity();       // start awake
  statusLed(false);
  DebugSerial.println(F("Bootup done"));
  lastNote = millis();
  /*  watchdog: 6-second timeout, auto-reset on expiry  */
  //esp_task_wdt_init(&twdt_config);  // signature happens automatically before setup() from core
  esp_task_wdt_add(NULL);  // NULL = "this task"
}





void debugPlugBlink() {
  static unsigned long lastBlinkTime = 0;
  static bool blinkOn = false;

  // Only blink when plug is inserted
  if (digitalRead(BAT_PLUG_PIN) == LOW) {
    unsigned long now = millis();
    if (now - lastBlinkTime >= SOC_BLINK_PERIOD && !socActive) {
      // toggle LED (active-low)
      blinkOn = !blinkOn;
      digitalWrite(STATUS_LED_PINS[3], blinkOn ? LOW : HIGH);
      lastBlinkTime = now;
    }
  } else {
    // ensure LED is off when plug not present
    blinkOn = false;
    if (!socActive) {
      digitalWrite(STATUS_LED_PINS[3], HIGH);
    }
    // reset timer so we don't immediately flash on the next insert
    lastBlinkTime = millis();
  }
}



void loop() {
  handleUsbConsole();
  tickWriteEcho();  // pump MOS write echo
  serviceInputs();
  manageMos();

  //DebugSerial.println(F("[INFO] serviceInputs done"));
  serviceWriteRegister();
  handleBikeBus();


  if (millis() >= bmsHoldUntil) {
    bms.poll(BMS_CURRENT_POLL_INTERVAL, BMS_FULL_POLL_INTERVAL);
    if (bms.readDone()) {
      // process data
      bms.clearDone();
    }
  }


  //DebugSerial.println(F("[INFO] pollDalyUart done"));
  checkAndCalibrateSOC();
  updateSOCDisplay();
  //DebugSerial.println(F("[INFO] updateSOCDisplay done"));
  debugPlugBlink();
  enterSleepIfIdle();
  esp_task_wdt_reset();  // kick the watchdog
}
