#include <Arduino.h>
#include <string.h>
#include <math.h>

// =====================================================
// USER PINOUT (Arduino Uno)
// =====================================================
// Decoder
const uint8_t PIN_TRIG = 2;   // D2 (INT0) rising edge = start of packet
const uint8_t PIN_PKT  = A0;  // analog packet (0..~4.5V)

// LED + door
const uint8_t PIN_DOOR = 8;   // door switch to GND, use INPUT_PULLUP

// -------------------------------
// CHANGED: move LEDs to Timer1 pins for high-frequency PWM
// OLD:
// const uint8_t PIN_LED1 = 5;   // PWM
// const uint8_t PIN_LED2 = 6;   // PWM
// NEW:
const uint8_t PIN_LED1 = 9;   // PWM (Timer1)
const uint8_t PIN_LED2 = 10;  // PWM (Timer1)
// -------------------------------

const uint8_t PIN_POT1 = A2;  // pot 1 wiper
const uint8_t PIN_POT2 = A4;  // pot 2 wiper

// =====================================================
// PACKET / SAMPLING PARAMETERS
// =====================================================
const uint8_t  NUM_DIGITS = 8;        // 8 hex digits: 4 trial + 4 stim
const uint32_t DIGIT_US   = 9500UL;  // 10 ms per digit
const uint8_t  SAMPLES_PER_DIGIT = 5; // sample only in the middle band (robust)

// sample only in the "middle" portion of each digit window:
const uint32_t MID_WINDOW_US = 5000UL;   // 5 ms wide sampling region
const uint32_t MID_OFFSET_US = 0000UL;   // start 2.5 ms into the digit window

const uint32_t POST_TRIGGER_DELAY_US = 000UL;

// =====================================================
// ADC / SCALING
// =====================================================
const int   ADC_MAX = 1023;
const float MAX_EXPECTED_VOLTAGE = 4.5; // your packet top (approx)
const float ADC_MAX_FOR_4V5 = ADC_MAX * (MAX_EXPECTED_VOLTAGE / 5.0); // ~921.6

// =====================================================
// LED CONTROL PARAMETERS
// =====================================================
const unsigned long DEBOUNCE_MS = 20;
const uint8_t RAMP_STEP = 3;      // set to 0 for immediate response
const unsigned long LOOP_MS = 5;  // LED update loop delay

// =====================================================
// STATE
// =====================================================
volatile bool triggerDetected = false;

// =====================================================
// ISR
// =====================================================
void onTriggerRise() {
  triggerDetected = true;
}

// =====================================================
// NEW: High-frequency PWM setup on Timer1 (~31.25 kHz on pins 9 & 10)
// - Uses Fast PWM 8-bit (so analogWrite still effectively works: 0..255)
// - Sets prescaler to 1 and WGM=5 (Fast PWM 8-bit)
// - This does NOT break millis()/delay() because Timer0 is untouched.
static void setupHighFreqPWM_Timer1() {
  // Clear control registers
  TCCR1A = 0;
  TCCR1B = 0;

  // Fast PWM 8-bit: WGM10=1, WGM11=0, WGM12=1, WGM13=0  => WGM=5
  TCCR1A |= (1 << WGM10);
  TCCR1B |= (1 << WGM12);

  // Non-inverting mode on OC1A (D9) and OC1B (D10)
  TCCR1A |= (1 << COM1A1);
  TCCR1A |= (1 << COM1B1);

  // Prescaler = 1  => ~16MHz / 256 = 62.5 kHz PWM base in Fast PWM 8-bit
  // BUT because of Fast PWM specifics, effective frequency is ~62.5 kHz.
  // (Either way: way above camera flicker regime.)
  TCCR1B |= (1 << CS10);

  // Initialize duty cycles to 0
  OCR1A = 0;
  OCR1B = 0;
}
// =====================================================

// =====================================================
// HELPERS
// =====================================================
bool readDoorOpenDebounced() {
  static bool lastRaw = true;
  static bool stable = true;
  static unsigned long lastChange = 0;

  bool raw = (digitalRead(PIN_DOOR) == HIGH);
  unsigned long now = millis();

  if (raw != lastRaw) {
    lastRaw = raw;
    lastChange = now;
  }
  if (now - lastChange > DEBOUNCE_MS) {
    stable = raw;
  }
  return stable; // true = door open
}

uint8_t readPotAsLevel(uint8_t pinAnalog) {
  int v = analogRead(pinAnalog);           // 0..1023
  int lvl = map(v, 0, 1023, 0, 255);       // 0..255
  return (uint8_t)constrain(lvl, 0, 255);
}

uint8_t rampToward(uint8_t current, uint8_t target) {
  if (RAMP_STEP == 0) return target;

  if (current < target) {
    uint16_t v = current + RAMP_STEP;
    return (v > target) ? target : (uint8_t)v;
  }
  if (current > target) {
    int v = (int)current - (int)RAMP_STEP;
    return (v < (int)target) ? target : (uint8_t)v;
  }
  return current;
}

char toHexChar(int val) {
  if (val < 10) return (char)('0' + val);
  return (char)('A' + (val - 10));
}

int hexCharToInt(char c) {
  if (c >= '0' && c <= '9') return (c - '0');
  if (c >= 'A' && c <= 'F') return (c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (c - 'a' + 10);
  return 0;
}

void insertionSort(int arr[], int n) {
  for (int i = 1; i < n; i++) {
    int key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

inline void waitUntil(uint32_t targetUs) {
  while ((int32_t)(micros() - targetUs) < 0) {
    // busy wait
  }
}

// =====================================================
// DECODER
// =====================================================
void decodeAndPrintOnce() {
  if (POST_TRIGGER_DELAY_US > 0) delayMicroseconds(POST_TRIGGER_DELAY_US);

  (void)analogRead(PIN_PKT);
  delayMicroseconds(50);
  (void)analogRead(PIN_PKT);

  const uint32_t t0 = micros();

  char hexDigits[NUM_DIGITS + 1];
  hexDigits[NUM_DIGITS] = '\0';

  for (uint8_t d = 0; d < NUM_DIGITS; d++) {
    int samples[SAMPLES_PER_DIGIT];

    const uint32_t digitStart = t0 + (uint32_t)d * DIGIT_US;
    const uint32_t midStart   = digitStart + MID_OFFSET_US;

    for (uint8_t k = 0; k < SAMPLES_PER_DIGIT; k++) {
      const uint32_t target =
        midStart + (uint32_t)(((uint32_t)k * MID_WINDOW_US + (MID_WINDOW_US / 2)) / SAMPLES_PER_DIGIT);

      waitUntil(target);
      samples[k] = analogRead(PIN_PKT);
    }

    insertionSort(samples, SAMPLES_PER_DIGIT);
    const int med = samples[SAMPLES_PER_DIGIT / 2];

    int hexVal = (int)lround(15.0f * (float)med / (float)ADC_MAX_FOR_4V5);
    hexVal = constrain(hexVal, 0, 15);
    hexDigits[d] = toHexChar(hexVal);
  }

  char trialHex[5], stimHex[5];
  memcpy(trialHex, hexDigits, 4);        trialHex[4] = '\0';
  memcpy(stimHex,  hexDigits + 4, 4);    stimHex[4]  = '\0';

  long trialNum = strtol(trialHex, NULL, 16);
  long stimNum  = strtol(stimHex,  NULL, 16);

  Serial.print("TRIAL:");
  Serial.print(trialNum);
  Serial.print(",STIM:");
  Serial.println(stimNum);
}

// =====================================================
// SETUP / LOOP
// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_TRIG, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_TRIG), onTriggerRise, RISING);

  pinMode(PIN_DOOR, INPUT_PULLUP);

  // CHANGED: set pins 9/10 as outputs (still fine)
  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);

  // NEW: configure Timer1 for high-frequency PWM on D9/D10
  setupHighFreqPWM_Timer1();

  // Start off
  analogWrite(PIN_LED1, 0);
  analogWrite(PIN_LED2, 0);

  Serial.println("Uno: packet decoder + dual LED dimmer + door interlock ready.");
}

void loop() {
  if (triggerDetected) {
    triggerDetected = false;
    decodeAndPrintOnce();
    return;
  }

  bool doorOpen = readDoorOpenDebounced();

  uint8_t pot1 = readPotAsLevel(PIN_POT1);
  uint8_t pot2 = readPotAsLevel(PIN_POT2);

  uint8_t target1 = doorOpen ? 0 : pot1;
  uint8_t target2 = doorOpen ? 0 : pot2;

  static uint8_t out1 = 0, out2 = 0;
  out1 = rampToward(out1, target1);
  out2 = rampToward(out2, target2);

  // CHANGED: still use analogWrite; Timer1 is now high-frequency
  analogWrite(PIN_LED1, out1);
  analogWrite(PIN_LED2, out2);

  delay(LOOP_MS);
}
