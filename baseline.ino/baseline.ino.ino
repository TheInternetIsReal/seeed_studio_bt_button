#include <bluemicro_hid.h>

// --------- LED pin mapping (active-LOW) ----------
#if defined(LED_RED) && defined(LED_GREEN) && defined(LED_BLUE)
constexpr int PIN_LED_R = LED_RED;
constexpr int PIN_LED_G = LED_GREEN;
constexpr int PIN_LED_B = LED_BLUE;
#else
constexpr int PIN_LED_R = 11;
constexpr int PIN_LED_G = 13;
constexpr int PIN_LED_B = 12;
#endif

// --------- Button pin ----------
const int SW_PIN = D0;   // button to GND (INPUT_PULLUP)

// --------- Timings ----------
const uint32_t DEBOUNCE_MS        = 35;
const uint32_t MIN_INTER_PRESS_MS = 90;
const uint32_t RAPID_MAX_MS       = 200;
const uint8_t  PRESS_TARGET       = 10;
const uint32_t BLINK_MS           = 500;
const uint32_t PAIRING_WINDOW_MS  = 30000;

// ---- Type engine timings ----
const uint32_t KEY_DOWN_MS  = 6;
const uint32_t CHAR_GAP_MS  = 14;

// --------- State ----------
enum ConnMode : uint8_t { MODE_USB, MODE_BLE, MODE_AUTO };
ConnMode mode = MODE_USB;

bool pairingActive = false;
uint32_t pairingStart = 0;

bool lastRawHigh   = true;
bool stableHigh    = true;
uint32_t lastEdgeMs = 0;
bool seenRelease   = true;

uint8_t  rapidCount  = 0;
uint32_t lastPressMs = 0;

bool blinkState = false;
uint32_t lastBlink = 0;

// --------- Non-blocking type engine ---------
struct TypeEngine {
  static constexpr uint8_t CAP = 64;
  uint8_t buf[CAP];
  uint8_t head = 0, tail = 0;

  enum Phase : uint8_t { IDLE, KEYDOWN, KEYUP, GAP } phase = IDLE;
  uint32_t phaseStart = 0;
  uint8_t  currentKey = 0;

  bool isFull123() const  { return uint8_t((tail + 1) % CAP) == head; }
  bool isEmpty() const { return head == tail; }

  void clear() { head = tail = 0; phase = IDLE; }

  void enqueueChar(char c) {
    uint8_t keycode = 0, mod = 0;
    if (c >= 'a' && c <= 'z') keycode = HID_KEY_A + (c - 'a');
    else if (c >= 'A' && c <= 'Z') { keycode = HID_KEY_A + (c - 'A'); mod = KEYBOARD_MODIFIER_LEFTSHIFT; }
    else if (c == ' ') keycode = HID_KEY_SPACE;
    if (!keycode || isFull123()) return;

    if (mod == KEYBOARD_MODIFIER_LEFTSHIFT) {
      uint8_t sentinel = 0xF0 | (keycode & 0x0F);
      buf[tail] = sentinel; tail = (tail + 1) % CAP;
      buf[tail] = keycode;  tail = (tail + 1) % CAP;
    } else {
      buf[tail] = keycode;  tail = (tail + 1) % CAP;
    }
  }

  void enqueueWord(const char* s) {
    while (*s && !isFull123()) enqueueChar(*s++);
  }

  void tick(uint32_t now) {
    switch (phase) {
      case IDLE:
        if (!isEmpty()) {
          currentKey = buf[head]; head = (head + 1) % CAP;
          if ((currentKey & 0xF0) == 0xF0) {
            if (isEmpty()) { phase = IDLE; break; }
            uint8_t nextKey = buf[head]; head = (head + 1) % CAP;
            uint8_t keys[6] = { nextKey, 0, 0, 0, 0, 0 };
            bluemicro_hid.keyboardReport(0, keys, KEYBOARD_MODIFIER_LEFTSHIFT);
            phase = KEYDOWN;
            phaseStart = now;
            currentKey = nextKey;
          } else {
            uint8_t keys[6] = { currentKey, 0, 0, 0, 0, 0 };
            bluemicro_hid.keyboardReport(0, keys);
            phase = KEYDOWN;
            phaseStart = now;
          }
        }
        break;
      case KEYDOWN:
        if (now - phaseStart >= KEY_DOWN_MS) {
          bluemicro_hid.keyboardRelease();
          phase = KEYUP;
          phaseStart = now;
        }
        break;
      case KEYUP:
        if (now - phaseStart >= 1) {
          phase = GAP;
          phaseStart = now;
        }
        break;
      case GAP:
        if (now - phaseStart >= CHAR_GAP_MS) {
          phase = IDLE;
        }
        break;
    }
  }
} typer;

// --------- LED helpers ---------
void ledsOff() { digitalWrite(PIN_LED_R,HIGH); digitalWrite(PIN_LED_G,HIGH); digitalWrite(PIN_LED_B,HIGH); }
void setBlueSolid() { digitalWrite(PIN_LED_R,HIGH); digitalWrite(PIN_LED_G,HIGH); digitalWrite(PIN_LED_B,LOW); }
void setGreenSolid() { digitalWrite(PIN_LED_R,HIGH); digitalWrite(PIN_LED_G,LOW); digitalWrite(PIN_LED_B,HIGH); }
void blinkBlueTick(uint32_t now) {
  if (now - lastBlink >= BLINK_MS) {
    lastBlink = now; blinkState = !blinkState;
    digitalWrite(PIN_LED_R,HIGH); digitalWrite(PIN_LED_G,HIGH);
    digitalWrite(PIN_LED_B, blinkState ? LOW : HIGH);
  }
}

// --------- Mode helpers ---------
void queueModeWord() {
  if (mode == MODE_USB) typer.enqueueWord("wired");
  else if (mode == MODE_BLE) typer.enqueueWord("bt");
}
void safeToggleMode() {
  bluemicro_hid.keyboardRelease();
  typer.clear();
  bluemicro_hid.processQueues(mode == MODE_BLE ? CONNECTION_MODE_BLE_ONLY : CONNECTION_MODE_USB_ONLY);
  if (mode == MODE_BLE) { mode = MODE_USB; pairingActive = false; }
  else { mode = MODE_BLE; pairingActive = true; pairingStart = millis(); }
  bluemicro_hid.keyboardRelease();
  bluemicro_hid.processQueues(mode == MODE_BLE ? CONNECTION_MODE_BLE_ONLY : CONNECTION_MODE_USB_ONLY);
}

// --------- Setup ---------
void setup() {
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  ledsOff();
  bluemicro_hid.begin();
}

// --------- Press detector ---------
bool pressedEdge(uint32_t now) {
  bool rawHigh = (digitalRead(SW_PIN) == HIGH);
  if (rawHigh != lastRawHigh) { lastRawHigh = rawHigh; lastEdgeMs = now; }
  if ((now - lastEdgeMs) >= DEBOUNCE_MS && stableHigh != rawHigh) {
    stableHigh = rawHigh;
    if (!stableHigh && seenRelease) { seenRelease = false; return true; }
    if (stableHigh) seenRelease = true;
  }
  return false;
}

// --------- Loop ---------
void loop() {
  const uint32_t now = millis();
  if (pressedEdge(now)) {
    if (!lastPressMs || (now - lastPressMs) >= MIN_INTER_PRESS_MS) {
      if (!lastPressMs) rapidCount = 1;
      else {
        uint32_t dt = now - lastPressMs;
        if (dt <= RAPID_MAX_MS && dt >= MIN_INTER_PRESS_MS) rapidCount++;
        else rapidCount = 1;
      }
      lastPressMs = now;

      bool toggled = false;
      if (rapidCount >= PRESS_TARGET) { safeToggleMode(); rapidCount = 0; toggled = true; }
      if (!toggled) queueModeWord();
    }
  }

  typer.tick(now);

  if (mode == MODE_BLE) {
    if (pairingActive && (now - pairingStart) <= PAIRING_WINDOW_MS) blinkBlueTick(now);
    else { pairingActive = false; setBlueSolid(); }
  } else if (mode == MODE_USB) setGreenSolid();
  else ledsOff();

  switch (mode) {
    case MODE_BLE: bluemicro_hid.processQueues(CONNECTION_MODE_BLE_ONLY); break;
    case MODE_USB: bluemicro_hid.processQueues(CONNECTION_MODE_USB_ONLY); break;
    default:       bluemicro_hid.processQueues(CONNECTION_MODE_AUTO);     break;
  }

  __WFE(); __SEV(); __WFE();
}
