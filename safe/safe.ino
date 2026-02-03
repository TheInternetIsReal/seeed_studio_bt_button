/*
  Seeed XIAO nRF52840 (non-Sense) — State-Machine HID with Low-Power WFE

  - Button: PRESS -> hold Left Alt; RELEASE -> release (no "wired"/"bt" typing)
  - 8 quick presses in BLE -> OFF
  - USB present -> WIRED has priority (always allowed, regardless of battery)
  - Low-battery guard:
      * Trip at <= 3.20 V: blink red 5x, force OFF (unless USB present)
      * Refuse BLE while <= 3.20 V (blink red 5x)
      * Clear lockout at >= 3.25 V (hysteresis)
  - Low-power: OFF and BLE_ADVERTISING use __WFE() sleep.
    Wakes on button CHANGE interrupt and on regular system tick interrupts.
*/

#include <bluemicro_hid.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>
#include <SeeedNrf52480Battery.h>

// ---------- LED pins (active-LOW) ----------
#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
constexpr int PIN_LED_R = LED_RED;
constexpr int PIN_LED_G = LED_GREEN;
constexpr int PIN_LED_B = LED_BLUE;
#else
constexpr int PIN_LED_R = 11;  // XIAO fallback (active-LOW)
constexpr int PIN_LED_G = 13;
constexpr int PIN_LED_B = 12;
#endif

// ---------- Button ----------
const int SW_PIN = D0;  // to GND (INPUT_PULLUP)

// ---------- Timings ----------
const uint32_t DEBOUNCE_MS        = 35;
const uint32_t MIN_INTER_PRESS_MS = 90;
const uint32_t QUICK_GAP_MS       = 400;
const uint8_t  QUICK_TARGET       = 8;
const uint32_t BLINK_MS           = 500;
const uint32_t BOND_HOLD_MS       = 5000;
const uint32_t ADV_TIMEOUT_MS     = 30000;

// ---------- HID Left Alt ----------
#ifndef HID_KEY_ALT_LEFT
#define HID_KEY_ALT_LEFT 0xE2
#endif
const uint8_t ALT_L = HID_KEY_ALT_LEFT;

// ---------- State machine ----------
enum class RunState : uint8_t { OFF = 0, BLE_ADVERTISING, BLE_CONNECTED, WIRED };
volatile RunState gState = RunState::OFF;

// ---------- BLE connection handle ----------
volatile uint16_t g_bleConn = BLE_CONN_HANDLE_INVALID;
bool g_connUsed = false;

// ---------- OFF tracking ----------
uint32_t g_offEnteredAt = 0;
const uint32_t OFF_ERASE_SUPPRESS_MS = 1500;
const uint32_t OFF_WAKE_SUPPRESS_MS  = 200;
bool g_offRequireRelease = false;
bool g_offPressing = false;
uint32_t g_offPressStartMs = 0;

// ---------- Advertising timeout ----------
bool g_advActive  = false;
uint32_t g_advStartMs = 0;

// ---------- Debounce ----------
bool lastRawHigh = true, stableHigh = true, seenRelease = true;
uint32_t lastEdgeMs = 0;
uint32_t lastPressMs = 0;
uint8_t  quickCount = 0;

// ---------- LED blink ----------
bool blinkState = false;
uint32_t lastBlink = 0;

// ---------- Low-power wake flag (set by button ISR) ----------
volatile bool g_wake = false;

// TinyUSB (optional stronger checks)
extern "C" bool tud_mounted(void);
extern "C" bool tud_connected(void);

// ---------- Helpers ----------
inline bool usb_ready() {
  if (TinyUSBDevice.mounted()) return true;
  if (tud_mounted() || tud_connected()) return true;
  return false;
}
inline bool ble_ready() { return (g_bleConn != BLE_CONN_HANDLE_INVALID) || Bluefruit.connected(); }

// forward-declare the ISR so attachInterrupt can see it
void onButtonChangeISR();

// ---------- LEDs ----------
void ledsOff() {
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, HIGH);
}
void solidBlue() {
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, LOW);
}
void blinkBlueTick(uint32_t now) {
  if (now - lastBlink >= BLINK_MS) {
    lastBlink = now;
    blinkState = !blinkState;
    digitalWrite(PIN_LED_R, HIGH);
    digitalWrite(PIN_LED_G, HIGH);
    digitalWrite(PIN_LED_B, blinkState ? LOW : HIGH);
  }
}
void solidPurple() {
  digitalWrite(PIN_LED_R, LOW);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, LOW);
}
void flashOrange(uint8_t times, uint16_t on_ms = 120, uint16_t off_ms = 120) {
  for (uint8_t i = 0; i < times; ++i) {
    digitalWrite(PIN_LED_R, LOW);
    digitalWrite(PIN_LED_G, LOW);
    digitalWrite(PIN_LED_B, HIGH);
    delay(on_ms);
    ledsOff();
    delay(off_ms);
  }
}

// ---------- HID helpers ----------
inline void alt_down() {
  uint8_t keys[6] = { ALT_L, 0, 0, 0, 0, 0 };
  bluemicro_hid.keyboardReport(0, keys);
}
inline void all_up() {
  bluemicro_hid.keyboardRelease();
}

// ---------- Debouncer (typedef + explicit prototype prevents Arduino auto-proto issues)
typedef enum BtnEvt_t { BTN_NONE = 0, BTN_PRESS, BTN_RELEASE } BtnEvt;
static inline BtnEvt buttonEvent(uint32_t now);  // explicit prototype

// ======================================================================
//                        LOW-VOLTAGE GUARD
// ======================================================================
SeeedNrf52480Battery battery;

constexpr float LV_CUTOFF_V      = 3.20f; // trip at/under this
constexpr float LV_CLEAR_V       = 3.35f; // clear lockout at/over this
constexpr int   ADC_SAMPLES      = 8;
constexpr int   LOW_NEED_N       = 3;     // need N consecutive low readings
constexpr uint32_t SAMPLE_EVERY_MS = 250;

struct Blink5Red {
  bool active = false;
  uint8_t count = 0;
  uint32_t last = 0;
  bool on = false;
  const uint16_t on_ms = 120, off_ms = 120;

  void start() { active = true; count = 0; on = false; last = 0; }
  bool running() const { return active; }

  void step(uint32_t now) {
    if (!active) return;
    if (last == 0) { last = now; on = true; digitalWrite(PIN_LED_R, LOW); return; }
    uint16_t dur = on ? on_ms : off_ms;
    if (now - last >= dur) {
      last = now;
      if (on) { digitalWrite(PIN_LED_R, HIGH); on = false; }
      else {
        count++;
        if (count >= 5) { active = false; ledsOff(); return; }
        on = true; digitalWrite(PIN_LED_R, LOW);
      }
    }
  }
} blink5;

inline float readVoltageAveraged() {
  float sum = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) {
    sum += battery.getVoltage();
    delayMicroseconds(500);
  }
  return sum / ADC_SAMPLES;
}

struct LowVoltGuard {
  bool lockout = false;          // true once we've tripped at <= 3.20V
  uint8_t lowStreak = 0;
  uint32_t lastSampleMs = 0;
  float lastV = 0;

  void tick(uint32_t now) {
    if (now - lastSampleMs < SAMPLE_EVERY_MS) return;
    lastSampleMs = now;

    float v = readVoltageAveraged();
    lastV = v;

    if (!lockout) {
      if (v <= LV_CUTOFF_V) {
        if (++lowStreak >= LOW_NEED_N) {
          lockout = true;
          lowStreak = 0;
          blink5.start();
        }
      } else {
        lowStreak = 0;
      }
    } else {
      if (v >= LV_CLEAR_V) lockout = false;
    }
  }

  bool isLockedOut() const { return lockout; }
  float voltage()   const { return lastV;    }
} lv;

// ---------- Guard helpers ----------
inline bool low_batt_block_ble() {
  if (usb_ready()) return false; // USB present => allow wired regardless
  if (lv.isLockedOut() || lv.voltage() <= LV_CUTOFF_V) {
    if (!blink5.running()) blink5.start();
    return true;
  }
  return false;
}
inline void enforce_low_voltage_policy() {
  if (!usb_ready() && lv.isLockedOut()) {
    if (gState == RunState::BLE_CONNECTED || gState == RunState::BLE_ADVERTISING) {
      // Drop out immediately
      gState = RunState::OFF;
      bleDisconnectAndStopAdv();
      all_up();
      ledsOff();
      g_offEnteredAt = millis();
      g_offRequireRelease = true;
      g_offPressing = false;
    }
  }
}

// ======================================================================

// ---------- BLE helpers ----------
inline void bleStopAdv() {
  Bluefruit.Advertising.stop();
  g_advActive = false;
}
inline void bleDisconnectAndStopAdv() {
  bleStopAdv();
  if (ble_ready()) {
    if (g_bleConn != BLE_CONN_HANDLE_INVALID) Bluefruit.disconnect(g_bleConn);
    else Bluefruit.disconnect(0xFFFF);
    delay(10);
  }
}
inline void bleStartAdvertisingSoft30s() {
  Bluefruit.setName("DougKey");
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(false);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(0);
  Bluefruit.Advertising.start(0);
  g_advStartMs = millis();
  g_advActive  = true;
}

// ---------- State transitions ----------
void goOff() {
  ledsOff();
  all_up();
  bleDisconnectAndStopAdv();
  gState = RunState::OFF;
  g_offEnteredAt = millis();
  g_offPressing = false;

  // Gate only if actually pressed now; prime debouncer so OFF is responsive
  bool rawPressed = (digitalRead(SW_PIN) == LOW);   // pull-up => LOW = pressed
  g_offRequireRelease = rawPressed;

  lastRawHigh = !rawPressed;
  stableHigh  = !rawPressed;
  seenRelease = !rawPressed;
  lastEdgeMs  = millis();
}
void goBleAdvertising() {
  lastBlink = millis();
  all_up();
  bleDisconnectAndStopAdv();
  bleStartAdvertisingSoft30s();
  gState = RunState::BLE_ADVERTISING;
}
void goBleConnected(uint16_t conn_handle) {
  g_bleConn = conn_handle;
  g_connUsed = false;
  all_up();
  solidBlue();
  gState = RunState::BLE_CONNECTED;
}
void leaveBleConnectedToOff() {
  all_up();
  bleDisconnectAndStopAdv();
  g_connUsed = false;
  quickCount = 0;
  lastPressMs = 0;
  seenRelease = true;
  stableHigh = true;
  lastRawHigh = true;
  g_offPressing = false;
  g_offPressStartMs = 0;
  gState = RunState::OFF;
  ledsOff();
  g_offEnteredAt = millis();
  g_offRequireRelease = true;
}

// ---------- BLE callbacks ----------
void prph_connect_callback(uint16_t conn_handle) { goBleConnected(conn_handle); }
void prph_disconnect_callback(uint16_t, uint8_t) {
  g_bleConn = BLE_CONN_HANDLE_INVALID;
  if (gState == RunState::WIRED || usb_ready()) return;
  if (gState == RunState::BLE_CONNECTED) {
    if (g_connUsed) goOff(); else goBleAdvertising();
  } else if (gState == RunState::BLE_ADVERTISING) {
    // ignore; timeout will handle
  } else goOff();
}

// ---------- Button ISR ----------
void onButtonChangeISR() {
  g_wake = true;
  __SEV();   // wake from WFE
}

// ---------- Setup ----------
void setup() {
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  ledsOff();

  // Button CHANGE interrupt for WFE wake
  attachInterrupt(digitalPinToInterrupt(SW_PIN), onButtonChangeISR, CHANGE);

  // Battery config (library already patched to enable VBAT correctly)
  // Ensure VBAT sense is enabled in the library (it drives VBAT_ENABLE low)
  // and set your calibration window.
  battery.setMinVoltage(3.10f);
  battery.setMaxVoltage(4.20f);
  battery.enableVoltageReading();

  bluemicro_hid.begin();
  Bluefruit.setTxPower(8);
  Bluefruit.setName("DougKey");
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  if (usb_ready()) {
    gState = RunState::WIRED;
  } else {
    goOff();
  }
}

// ---------- Debouncer impl ----------
static inline BtnEvt buttonEvent(uint32_t now) {
  bool rawHigh = (digitalRead(SW_PIN) == HIGH); // pull-up: LOW = pressed
  if (rawHigh != lastRawHigh) {
    lastRawHigh = rawHigh;
    lastEdgeMs = now;
  }
  if ((now - lastEdgeMs) >= DEBOUNCE_MS && stableHigh != rawHigh) {
    stableHigh = rawHigh;
    if (!stableHigh && seenRelease) {
      seenRelease = false;
      return BTN_PRESS;
    }
    if (stableHigh && !seenRelease) {
      seenRelease = true;
      return BTN_RELEASE;
    }
  }
  return BTN_NONE;
}

// ---------- Main loop ----------
void loop() {
  const uint32_t now = millis();

  // 1) Strong idempotent USB watcher (forces state & LEDs every pass)
  if (usb_ready()) {
    if (gState != RunState::WIRED) {
      gState = RunState::WIRED;
      all_up();
      bleDisconnectAndStopAdv();
    }
    solidPurple();
  } else {
    if (gState == RunState::WIRED) {
      // Hard drop to OFF and clear LED now
      gState = RunState::OFF;
      bleDisconnectAndStopAdv();
      all_up();
      ledsOff();
      g_offEnteredAt = now;
      g_offRequireRelease = true;
      g_offPressing = false;
    }
  }

  // 2) Supervisors: low-voltage & non-blocking blinks
  lv.tick(now);
  blink5.step(now);
  enforce_low_voltage_policy();

  // 3) State machine
  if (gState == RunState::OFF) {
    // Sleep until an event (button edge or systick), then run OFF logic
    __WFE(); g_wake = false;

    bool rawPressed = (digitalRead(SW_PIN) == LOW);
    if (g_offRequireRelease) {
      if (!rawPressed && (now - g_offEnteredAt >= OFF_WAKE_SUPPRESS_MS))
        g_offRequireRelease = false;
    } else {
      if (rawPressed) {
        if (!g_offPressing) {
          g_offPressing = true;
          g_offPressStartMs = now;
        }
        if (g_offPressing &&
            (now - g_offPressStartMs >= BOND_HOLD_MS) &&
            (now - g_offEnteredAt >= OFF_ERASE_SUPPRESS_MS)) {
          Bluefruit.Periph.clearBonds();
          flashOrange(5);
          g_offPressing = false;
          g_offRequireRelease = true;
          g_offEnteredAt = now;
        }
      } else if (g_offPressing) {
        uint32_t held = now - g_offPressStartMs;
        g_offPressing = false;
        if (held >= DEBOUNCE_MS && held < BOND_HOLD_MS) {
          // Try to enter BLE unless battery is too low
          if (low_batt_block_ble()) {
            g_offRequireRelease = true;
          } else {
            goBleAdvertising();
            // prime debouncer so first release inside ADV isn't misread
            seenRelease = false;
            lastRawHigh = false;
            stableHigh  = false;
            lastEdgeMs  = now;
            g_offRequireRelease = true;
          }
        }
      }
    }
  }

  else if (gState == RunState::BLE_ADVERTISING) {
    // Sleep between advertising events; wake on button or tick
    blinkBlueTick(now);
    if (low_batt_block_ble()) {
      goOff();
    } else if (g_advActive && !ble_ready() && (now - g_advStartMs >= ADV_TIMEOUT_MS)) {
      bleStopAdv();
      goOff();
    }
    __WFE(); g_wake = false;
  }

  else if (gState == RunState::BLE_CONNECTED) {
    // We don't sleep here; keep HID snappy
    if (low_batt_block_ble()) {
      leaveBleConnectedToOff();
    } else {
      switch (buttonEvent(now)) {
        case BTN_PRESS: {
          bool accept = true;
          if (lastPressMs) {
            uint32_t dt = now - lastPressMs;
            if (dt < MIN_INTER_PRESS_MS) accept = false;
            else if (dt <= QUICK_GAP_MS) quickCount++;
            else quickCount = 1;
          } else quickCount = 1;
          lastPressMs = now;
          if (accept) {
            if (quickCount >= QUICK_TARGET) {
              quickCount = 0;
              leaveBleConnectedToOff();
            } else {
              g_connUsed = true;
              alt_down();
            }
          }
        } break;
        case BTN_RELEASE:
          all_up();
          break;
        default: break;
      }
      solidBlue();
    }
  }

  else if (gState == RunState::WIRED) {
    // Always allow wired HID regardless of battery level
    switch (buttonEvent(now)) {
      case BTN_PRESS: {
        bool accept = true;
        if (lastPressMs && (now - lastPressMs) < MIN_INTER_PRESS_MS) accept = false;
        lastPressMs = now;
        if (accept) alt_down();
      } break;
      case BTN_RELEASE:
        all_up();
        break;
      default: break;
    }
    // (LED already painted by USB watcher)
    // Light sleep is optional here; TinyUSB tasks may need CPU regularly
    // delay(1);
  }

  // 4) Process HID queues
  switch (gState) {
    case RunState::BLE_CONNECTED:
      bluemicro_hid.processQueues(CONNECTION_MODE_BLE_ONLY);
      break;
    case RunState::WIRED:
      bluemicro_hid.processQueues(CONNECTION_MODE_USB_ONLY);
      break;
    default:
      bluemicro_hid.processQueues(CONNECTION_MODE_AUTO);
      break;
  }
}
