/*
  Seeed XIAO nRF52840 (non-Sense) — State-Machine HID

  Change summary for power in OFF:
    - No HID processQueues in OFF/ADVERTISING
    - Float LED pins in OFF (high-Z) to prevent leakage
    - Flash red once when entering OFF
    - Preserve all prior behavior (8-press OFF in BLE, 5s bond-erase in OFF, USB priority)

  Button behavior:
    PRESS/HOLD -> Left Alt down
    RELEASE    -> Left Alt up
*/

#include <bluemicro_hid.h>
#include <Adafruit_TinyUSB.h>
#include <bluefruit.h>

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

// ---------- Helpers ----------
inline bool usb_ready() { return TinyUSBDevice.mounted(); }
inline bool ble_ready() { return (g_bleConn != BLE_CONN_HANDLE_INVALID) || Bluefruit.connected(); }

// ---------- LED helpers (active-LOW) ----------
void ledsOff() {
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, HIGH);
}
void ledsOutputs() {
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
}
void ledsHighZ() {   // float LEDs in OFF to eliminate leakage
  pinMode(PIN_LED_R, INPUT);
  pinMode(PIN_LED_G, INPUT);
  pinMode(PIN_LED_B, INPUT);
}
void flashRedOnce() {           // brief red flash on OFF entry
  ledsOutputs();                // ensure pins are outputs for the flash
  digitalWrite(PIN_LED_R, LOW); // red ON (active-LOW)
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, HIGH);
  delay(120);
  ledsOff();                    // all off
  // caller will immediately high-Z after this
}
void solidBlue() { // BLE connected
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, LOW);
}
void blinkBlueTick(uint32_t now) { // BLE advertising
  if (now - lastBlink >= BLINK_MS) {
    lastBlink = now;
    blinkState = !blinkState;
    digitalWrite(PIN_LED_R, HIGH);
    digitalWrite(PIN_LED_G, HIGH);
    digitalWrite(PIN_LED_B, blinkState ? LOW : HIGH);
  }
}
void solidPurple() { // WIRED
  digitalWrite(PIN_LED_R, LOW);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, LOW);
}
void flashOrange(uint8_t times, uint16_t on_ms = 120, uint16_t off_ms = 120) {
  ledsOutputs();
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
inline void all_up() { bluemicro_hid.keyboardRelease(); }

// ---------- Debouncer ----------
enum BtnEvt { BTN_NONE = 0, BTN_PRESS, BTN_RELEASE };
static inline BtnEvt buttonEvent(uint32_t now);

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
  // Release any key and shut radios
  all_up();
  bleDisconnectAndStopAdv();
  gState = RunState::OFF;

  // One red flash, then float pins to save power
  flashRedOnce();
  ledsHighZ();

  // Reset OFF guards/flags
  g_offEnteredAt = millis();
  g_offRequireRelease = true;
  g_offPressing = false;
}

void goBleAdvertising() {
  all_up();
  bleDisconnectAndStopAdv();
  bleStartAdvertisingSoft30s();
  gState = RunState::BLE_ADVERTISING;

  ledsOutputs();               // restore LED drive
  lastBlink = millis();
  // Initial blink state will be handled by blinkBlueTick()
}

void goBleConnected(uint16_t conn_handle) {
  g_bleConn = conn_handle;
  g_connUsed = false;
  all_up();
  gState = RunState::BLE_CONNECTED;

  ledsOutputs();
  solidBlue();
}

void leaveBleConnectedToOff() {
  all_up();
  bleDisconnectAndStopAdv();

  // Reset press tracking so next press is clean
  g_connUsed   = false;
  quickCount   = 0;
  lastPressMs  = 0;
  seenRelease  = true;
  stableHigh   = true;
  lastRawHigh  = true;
  g_offPressing = false;
  g_offPressStartMs = 0;

  gState = RunState::OFF;

  // Red flash + float pins
  flashRedOnce();
  ledsHighZ();

  g_offEnteredAt = millis();
  g_offRequireRelease = true;
}

// ---------- BLE callbacks ----------
void prph_connect_callback(uint16_t conn_handle) { goBleConnected(conn_handle); }
void prph_disconnect_callback(uint16_t, uint8_t) {
  g_bleConn = BLE_CONN_HANDLE_INVALID;

  // If USB takeover is in progress, ignore BLE disconnect.
  if (gState == RunState::WIRED || usb_ready()) return;

  // If connection was used, go OFF; else resume Advertising (fresh 30s)
  if (gState == RunState::BLE_CONNECTED) {
    if (g_connUsed) goOff(); else goBleAdvertising();
  } else if (gState == RunState::BLE_ADVERTISING) {
    // ignore; software timeout handles it
  } else {
    goOff();
  }
}

// ---------- Setup ----------
void setup() {
  pinMode(SW_PIN, INPUT_PULLUP);
  ledsOutputs();   // set as outputs first
  ledsOff();

  bluemicro_hid.begin();
  Bluefruit.setTxPower(8);
  Bluefruit.setName("DougKey");
  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  if (usb_ready()) {
    solidPurple();
    gState = RunState::WIRED;
  } else {
    goOff();  // will flash red and float pins
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

  // USB watcher — USB wins on appearance; OFF on removal from WIRED
  static bool lastUsb = usb_ready();
  bool usbNow = usb_ready();
  if (usbNow && !lastUsb) {
    // USB TAKEOVER
    gState = RunState::WIRED;     // mark first (BLE callbacks will ignore)
    all_up();
    bleDisconnectAndStopAdv();
    ledsOutputs();
    solidPurple();
  } else if (!usbNow && lastUsb) {
    if (gState == RunState::WIRED) {
      goOff(); // immediate OFF on unplug from WIRED (red flash + float)
    }
  }
  lastUsb = usbNow;

  // --- OFF ---
  if (gState == RunState::OFF) {
    // Raw button state (active-low)
    bool rawPressed = (digitalRead(SW_PIN) == LOW);

    // Require a release after entering OFF and a small quiet window
    if (g_offRequireRelease) {
      if (!rawPressed && (now - g_offEnteredAt >= OFF_WAKE_SUPPRESS_MS)) {
        g_offRequireRelease = false;
      }
    } else {
      // Press-and-hold handling in OFF:
      if (rawPressed) {
        if (!g_offPressing) {
          g_offPressing = true;
          g_offPressStartMs = now;
        }
        // Erase bonds (held >= 5s) after suppression window
        if (g_offPressing &&
            (now - g_offPressStartMs >= BOND_HOLD_MS) &&
            (now - g_offEnteredAt >= OFF_ERASE_SUPPRESS_MS)) {
          // Need outputs to flash orange; temporarily restore, then float again
          Bluefruit.Periph.clearBonds();
          flashOrange(5);
          ledsHighZ();

          // Stay in OFF; require a release before any further action
          g_offPressing = false;
          g_offRequireRelease = true;
          g_offEnteredAt = now;  // restart suppression window
        }
      } else if (g_offPressing) {
        // Released: short press -> start Advertising (30s software window)
        uint32_t held = now - g_offPressStartMs;
        g_offPressing = false;
        if (held >= DEBOUNCE_MS && held < BOND_HOLD_MS) {
          goBleAdvertising(); // software 30s adv window

          // consume this press so it won't appear later as an edge
          seenRelease = false;
          lastRawHigh = false;
          stableHigh  = false;
          lastEdgeMs  = now;
          g_offRequireRelease = true; // require release before next OFF wake
        }
      }
    }

    __WFE(); __SEV(); __WFE();
  }
  // --- BLE ADVERTISING ---
  else if (gState == RunState::BLE_ADVERTISING) {
    // LED blink blue
    blinkBlueTick(now);

    // Software 30s timeout: if nobody connects within 30s, go OFF
    if (g_advActive && !ble_ready() && (now - g_advStartMs >= ADV_TIMEOUT_MS)) {
      bleStopAdv();
      goOff();
    }

    __WFE(); __SEV(); __WFE();
  }
  // --- BLE CONNECTED ---
  else if (gState == RunState::BLE_CONNECTED) {
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
            leaveBleConnectedToOff();  // OFF with red flash + float
          } else {
            g_connUsed = true;         // mark this connection as "used"
            alt_down();                // Left Alt down on press
          }
        }
      } break;

      case BTN_RELEASE:
        all_up();
        break;

      default: break;
    }

    // LED: solid blue
    solidBlue();

    __WFE(); __SEV(); __WFE();
  }
  // --- WIRED ---
  else if (gState == RunState::WIRED) {
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

    // LED: solid purple
    solidPurple();

    __WFE(); __SEV(); __WFE();
  }

  // Process HID queues ONLY when actually connected
  switch (gState) {
    case RunState::BLE_CONNECTED:
      bluemicro_hid.processQueues(CONNECTION_MODE_BLE_ONLY);
      break;
    case RunState::WIRED:
      bluemicro_hid.processQueues(CONNECTION_MODE_USB_ONLY);
      break;
    default:
      // OFF and BLE_ADVERTISING: do not pump queues
      break;
  }
}
