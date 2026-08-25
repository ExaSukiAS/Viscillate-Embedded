#include <Arduino.h>

// GPIO pin decleration
#define ADC_PIN              1
#define MED_MODE_SWITCH_PIN  10
#define LOW_MODE_SWITCH_PIN  0

// Gains and Offsets calculated through external passive summing network
const double gain[3] = {0.020504, 0.053263, 0.123724};
const double offset[3] = {1384.0694, 1337.7798, 1238.2160};

// Volatile variables modified inside GPIO ISRs (triggered by switch movement) for switching between voltage range modes
volatile uint8_t mode = 0; // current mode (0 = +-40v, 1 = +-17v, 2 = +-7.5v)
volatile uint32_t lastInterruptTime = 0; 
const uint32_t debounceDelay = 50; // ms

// read GPIO states safely inside ISR
void IRAM_ATTR handleSwitchISR() {
    uint32_t now = millis();
    if (now - lastInterruptTime > debounceDelay) {
        if (digitalRead(MED_MODE_SWITCH_PIN) == LOW) {
            mode = 1;
        } else if (digitalRead(LOW_MODE_SWITCH_PIN) == LOW) {
            mode = 2;
        } else {
            mode = 0;
        }
        lastInterruptTime = now;
    }
}

void setup() {
    Serial.begin(2000000);

    pinMode(MED_MODE_SWITCH_PIN, INPUT_PULLUP);
    pinMode(LOW_MODE_SWITCH_PIN, INPUT_PULLUP);

    if (digitalRead(MED_MODE_SWITCH_PIN) == LOW) mode = 1;
    else if (digitalRead(LOW_MODE_SWITCH_PIN) == LOW) mode = 2;
    else mode = 0;

    attachInterrupt(digitalPinToInterrupt(MED_MODE_SWITCH_PIN), handleSwitchISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(LOW_MODE_SWITCH_PIN), handleSwitchISR, CHANGE);

    while (!Serial && millis() < 3000) { delay(10); }
}

void loop() {
    uint16_t sig_mV = analogReadMilliVolts(ADC_PIN); // Read output voltage of the passive summing network
    int32_t mV = (sig_mV - offset[mode]) / gain[mode]; // Vin of the passive summing networ
    
    // send a sync header (0xAA, 0xBB)
    const uint8_t header[2] = {0xAA, 0xBB};
    Serial.write(header, 2);

    // send the voltage data
    Serial.write((uint8_t*)&mV, 4);

    vTaskDelay(pdMS_TO_TICKS(1)); 
}