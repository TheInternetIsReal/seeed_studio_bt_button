#include <bluemicro_hid.h>
#include <Adafruit_TinyUSB.h>  // TinyUSBDevice.detach()/attach(), mounted()
#include <bluefruit.h>         // Bluefruit.connected(), Advertising.*, disconnect()

// ---------- LED pins (active-LOW) ----------
#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
constexpr int PIN_LED_R = LED_RED;
constexpr int PIN_LED_G = LED_GREEN;
constexpr int PIN_LED_B = LED_BLUE;
#else
// Fallback mapping often used on XIAO nRF52840 (active-LOW)
constexpr int PIN_LED_R = 11;
constexpr int PIN_LED_G = 13;
constexpr int PIN_LED_B = 12;
#endif

// ---------- Button ----------
const int SW_PIN = D0;  // button to GND (INPUT_PULLUP)

// ---------- Timings ----------
const uint32_t DEBOUNCE_MS        = 35;
const uint32_t MIN_INTER_PRESS_MS = 90;   // reject too-fast bounce
const uint32_t RAPID_MAX_MS       = 300;  // next press must occur within this to count toward the 10-fast streak
const uint8_t  PRESS_TARGET       = 10;   // 10 fast presses in a row
const uint32_t BLINK_MS           = 500;

// ---- Type engine timings (non-blocking) ----
const uint32_t KEY_DOWN_MS = 6;   // how long a key is held
const uint32_t CHAR_GAP_MS = 14;  // time between characters

// ---------- Mode/state ----------
enum ConnMode : uint8_t { MODE_USB, MODE_BLE, MODE_AUTO };
ConnMode mode = MODE_USB;

// Button debouncer
bool lastRawHigh = true, stableHigh = true, seenRelease = true;
uint32_t lastEdgeMs = 0;

// Rapid-10 detector
uint8_t rapidCount = 0;
uint32_t lastPressMs = 0;

// LED blink
bool blinkState = false;
uint32_t lastBlink = 0;

// BLE connection tracking + adv flag
volatile uint16_t ble_conn = BLE_CONN_HANDLE_INVALID;
volatile bool advRunning = false;

// ---------- Simple non-blocking “type engine” ----------
static const uint8_t TQ_CAP = 64;
uint8_t tq_buf[TQ_CAP];
uint8_t tq_head = 0, tq_tail = 0;

enum TQ_Phase : uint8_t { TQ_IDLE, TQ_KEYDOWN, TQ_KEYUP, TQ_GAP };
TQ_Phase tq_phase = TQ_IDLE;
uint32_t tq_phaseStart = 0;
uint8_t tq_currentKey = 0;

inline bool tq_empty() { return tq_head == tq_tail; }
inline bool tq_full()  { return uint8_t((tq_tail + 1) % TQ_CAP) == tq_head; }
void tq_clear()        { tq_head = tq_tail = 0; tq_phase = TQ_IDLE; }

// --- Transport readiness gates (added back) ---
inline bool usb_ready() { return TinyUSBDevice.mounted(); }
inline bool ble_ready() { return (ble_conn != BLE_CONN_HANDLE_INVALID) || Bluefruit.connected(); }
inline bool can_send_now() {
  if (mode == MODE_USB) return usb_ready();
  if (mode == MODE_BLE) return ble_ready();
  return false;
}

// ---------- Type Queue ----------
void tq_enqueue_char(char c) {
  // force lowercase
  if (c >= 'A' && c <= 'Z') c = char(c - 'A' + 'a');
  uint8_t keycode = 0;
  if (c >= 'a' && c <= 'z') keycode = HID_KEY_A + (c - 'a');
  else if (c == ' ') keycode = HID_KEY_SPACE;
  else return;  // ignore other chars

  if (tq_full()) return;
  tq_buf[tq_tail] = keycode;
  tq_tail = (tq_tail + 1) % TQ_CAP;
}

void tq_enqueue_word(const char* s) {
  while (*s && !tq_full()) tq_enqueue_char(*s++);
}

void tq_tick(uint32_t now) {
  switch (tq_phase) {
    case TQ_IDLE:
      if (!tq_empty() && can_send_now()) {
        tq_currentKey = tq_buf[tq_head];
        tq_head = (tq_head + 1) % TQ_CAP;
        uint8_t keys[6] = { tq_currentKey, 0, 0, 0, 0, 0 };
        bluemicro_hid.keyboardReport(0, keys);
        tq_phase = TQ_KEYDOWN;
        tq_phaseStart = now;
      }
      break;

    case TQ_KEYDOWN:
      if (now - tq_phaseStart >= KEY_DOWN_MS) {
        bluemicro_hid.keyboardRelease();
        tq_phase = TQ_KEYUP;
        tq_phaseStart = now;
      }
      break;

    case TQ_KEYUP:
      if (now - tq_phaseStart >= 1) {
        tq_phase = TQ_GAP;
        tq_phaseStart = now;
      }
      break;

    case TQ_GAP:
      if (now - tq_phaseStart >= CHAR_GAP_MS) {
        tq_phase = TQ_IDLE;
      }
      break;
  }
}

// ---------- LEDs (active-LOW) ----------
void ledsOff() {
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, HIGH);
}

void setBlueSolid() {
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, HIGH);
  digitalWrite(PIN_LED_B, LOW);
}

void setGreenSolid() {
  digitalWrite(PIN_LED_R, HIGH);
  digitalWrite(PIN_LED_G, LOW);
  digitalWrite(PIN_LED_B, HIGH);
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

// ---------- Mode helpers ----------
void queueModeWord() {
  if (mode == MODE_USB) tq_enqueue_word("wired");
  else if (mode == MODE_BLE) tq_enqueue_word("bt");
}

// ---------- BLE callbacks ----------
void prph_connect_callback(uint16_t conn_handle) {
  ble_conn = conn_handle;
}

void prph_disconnect_callback(uint16_t /*conn_handle*/, uint8_t /*reason*/) {
  ble_conn = BLE_CONN_HANDLE_INVALID;
  if (mode == MODE_BLE && !Bluefruit.Advertising.isRunning()) {
    Bluefruit.Advertising.start(0);
    advRunning = true;
  } else {
    Bluefruit.Advertising.stop();
    advRunning = false;
  }
}

// --- Transport teardown/bring-up helpers ---
inline void usbDetach() { TinyUSBDevice.detach(); delay(50); }
inline void usbAttach() { TinyUSBDevice.attach(); delay(50); }
inline void bleStopAdv() { Bluefruit.Advertising.stop(); advRunning = false; }

inline void bleDisconnectAndStopAdv() {
  bleStopAdv();
  if (ble_conn != BLE_CONN_HANDLE_INVALID || Bluefruit.connected()) {
    if (ble_conn != BLE_CONN_HANDLE_INVALID) {
      Bluefruit.disconnect(ble_conn);
    } else {
      Bluefruit.disconnect(0xFFFF);
    }
    delay(10);
  }
}

inline void bleStartAdvertising() {
  Bluefruit.Advertising.start(0);
  advRunning = true;
}

inline void bleQuietAtBoot() {
  bleStopAdv();
  if (Bluefruit.connected()) {
    Bluefruit.disconnect(0xFFFF);
    delay(10);
  }
}

// ---------- Safe mode toggle ----------
void safeToggleMode() {
  bluemicro_hid.keyboardRelease();
  tq_clear();

  bluemicro_hid.processQueues(mode == MODE_BLE ? CONNECTION_MODE_BLE_ONLY : CONNECTION_MODE_USB_ONLY);

  if (mode == MODE_BLE) bleDisconnectAndStopAdv();
  else if (mode == MODE_USB) usbDetach();

  if (mode == MODE_BLE) {
    mode = MODE_USB;
  } else {
    mode = MODE_BLE;
    lastBlink = millis();
  }

  bluemicro_hid.keyboardRelease();
  bluemicro_hid.processQueues(mode == MODE_BLE ? CONNECTION_MODE_BLE_ONLY : CONNECTION_MODE_USB_ONLY);

  if (mode == MODE_USB) {
    usbAttach();
    bleStopAdv();
    setGreenSolid();
  } else {
    bleStartAdvertising();
  }
}

// ---------- Setup ----------
void setup() {
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  ledsOff();

  bluemicro_hid.begin();

  Bluefruit.Periph.setConnectCallback(prph_connect_callback);
  Bluefruit.Periph.setDisconnectCallback(prph_disconnect_callback);

  bleQuietAtBoot();
  setGreenSolid();
}

// ---------- Debounced PRESS-edge ----------
bool pressedEdge(uint32_t now) {
  bool rawHigh = (digitalRead(SW_PIN) == HIGH);
  if (rawHigh != lastRawHigh) {
    lastRawHigh = rawHigh;
    lastEdgeMs = now;
  }
  if ((now - lastEdgeMs) >= DEBOUNCE_MS && stableHigh != rawHigh) {
    stableHigh = rawHigh;
    if (!stableHigh && seenRelease) {
      seenRelease = false;
      return true;
    }
    if (stableHigh) seenRelease = true;
  }
  return false;
}

// ---------- Loop ----------
void loop() {
  const uint32_t now = millis();

  // 1) Press detection (non-blocking)
  if (pressedEdge(now)) {
    if (!lastPressMs || (now - lastPressMs) >= MIN_INTER_PRESS_MS) {
      if (!lastPressMs) rapidCount = 1;
      else {
        uint32_t dt = now - lastPressMs;
        rapidCount = (dt <= RAPID_MAX_MS && dt >= MIN_INTER_PRESS_MS) ? (rapidCount + 1) : 1;
      }
      lastPressMs = now;
      bool toggled = false;
      if (rapidCount >= PRESS_TARGET) {
        safeToggleMode();
        rapidCount = 0;
        toggled = true;
      }
      if (!toggled) queueModeWord();
    }
  }

  // 2) Run the non-blocking type engine
  tq_tick(now);

  // 3) LED states
  if (mode == MODE_BLE) {
    if (ble_ready()) setBlueSolid();
    else blinkBlueTick(now);
  } else if (mode == MODE_USB) {
    setGreenSolid();
  } else {
    ledsOff();
  }

  // 4) Keep advertising alive
  if (mode == MODE_BLE && !ble_ready()) {
    if (!Bluefruit.Advertising.isRunning()) bleStartAdvertising();
  }

  // 5) Process HID queues
  switch (mode) {
    case MODE_BLE: bluemicro_hid.processQueues(CONNECTION_MODE_BLE_ONLY); break;
    case MODE_USB: bluemicro_hid.processQueues(CONNECTION_MODE_USB_ONLY); break;
    default:       bluemicro_hid.processQueues(CONNECTION_MODE_AUTO); break;
  }

  // 6) Light sleep
  __WFE();
  __SEV();
  __WFE();
}
