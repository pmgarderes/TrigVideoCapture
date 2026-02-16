// Pin definitions
const int digitalTriggerPin = 2;  // Rising edge detection
const int analogSignalPin = A0;   // Carries the encoded signal

// Constants
const int samplesPerWindow = 10;       // Sample 10x per 10ms window = 1kHz
const int numWindows = 8;              // 8 hex digits
const int windowDurationMs = 9;       // 10 ms per hex digit
const int packetDurationMs = 80;       // Full 8-digit message duration
const float maxVoltage = 4.8;          // Arduino analog ref voltage
const int adcMax = 1023;               // 10-bit ADC

const float maxExpectedVoltage = 4.5;  // Your real-world signal max
const float adcMaxFor4_5V = adcMax * (maxExpectedVoltage / 5.0);  // ~921.6

// State
volatile bool triggerDetected = false;

void setup() {
  Serial.begin(115200);
  pinMode(digitalTriggerPin, INPUT);

  // Attach interrupt on RISING edge of digital trigger line
  attachInterrupt(digitalPinToInterrupt(digitalTriggerPin), onTriggerRise, RISING);
}

void loop() {
  if (triggerDetected) {
    triggerDetected = false;

    // Record analog signal for 80ms into 8 bins
    int readings[numWindows][samplesPerWindow];
    unsigned long startTime = millis();

      // Optional: Wait briefly after trigger to let analog signal stabilize
    delayMicroseconds(200);  // ~0.2ms delay (tweak as needed)
    
    // Begin precise sampling loop
    unsigned long startMicros = micros();
    
    for (int w = 0; w < numWindows; w++) {
      for (int s = 0; s < samplesPerWindow; s++) {
        int index = w * samplesPerWindow + s;
        readings[w][s] = analogRead(analogSignalPin);
    
        // Wait until next 1ms tick (1000 us per sample)
        while (micros() - startMicros < 1000 * (index + 1)) {
          // Busy-wait for accurate timing
        }
      }
    }

    
    Serial.print("Medians: ");
    for (int w = 0; w < numWindows; w++) {
      int sorted[samplesPerWindow];
      memcpy(sorted, readings[w], sizeof(sorted));
      insertionSort(sorted, samplesPerWindow);
      int median = sorted[samplesPerWindow / 2];
      Serial.print(median);
      Serial.print(" ");
    }
    Serial.println();

    
  // Decode each window to a hex digit
  char hexDigits[numWindows + 1];
  hexDigits[numWindows] = '\0';
  
  for (int w = 0; w < numWindows; w++) {
    int sorted[samplesPerWindow];
    memcpy(sorted, readings[w], sizeof(sorted));
    insertionSort(sorted, samplesPerWindow);
    int median = sorted[samplesPerWindow / 2];
  
    // Use 4.5V scaling instead of full 5V range
    int hexVal = round(15.0 * median / adcMaxFor4_5V);
    hexVal = constrain(hexVal, 0, 15);
    hexDigits[w] = toHexChar(hexVal);
  }
    // Parse to integers
  char trialHex[5], stimHex[5];
  memcpy(trialHex, hexDigits, 4); trialHex[4] = '\0';
  memcpy(stimHex, hexDigits + 4, 4); stimHex[4] = '\0';

  int trialNum = strtol(trialHex, NULL, 16);
  int stimNum = strtol(stimHex, NULL, 16);
  

  // Send to Python
  Serial.print("TRIAL:");
  Serial.print(trialNum);
  Serial.print(",STIM:");
  Serial.println(stimNum);
 
  }
}

// Interrupt handler
void onTriggerRise() {
  triggerDetected = true;
}

// Helper: Insertion sort for median
void insertionSort(int arr[], int size) {
  for (int i = 1; i < size; i++) {
    int key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }
}

// Helper: Convert int 0–15 to hex char
char toHexChar(int val) {
  if (val < 10) return '0' + val;
  else return 'A' + (val - 10);
}
