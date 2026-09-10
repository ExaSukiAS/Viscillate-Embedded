#include <Arduino.h>
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define ADC_PIN              1 
#define MED_MODE_SWITCH_PIN  10
#define LOW_MODE_SWITCH_PIN  0

const float invGain[3] = { 48.7709715f, 18.7747592f, 8.0825062f }; // gain is inverted (1/original gain) to avoid floating point division in the main loop
const float offset[3] = { 1384.0694f, 1337.7798f, 1238.2160f };

// mode switching variables (modes are switched via external switches connected to GPIO 10 and GPIO 0)
volatile uint8_t mode = 0; 
volatile uint32_t lastInterruptTime = 0; 
const uint32_t debounceDelay = 50; 

// Interrupt Service Routine (ISR) for handling mode switch changes
void IRAM_ATTR handleSwitchISR() {
    uint32_t now = millis();
    if (now - lastInterruptTime > debounceDelay) {
        if (digitalRead(MED_MODE_SWITCH_PIN) == LOW) mode = 1;
        else if (digitalRead(LOW_MODE_SWITCH_PIN) == LOW) mode = 2;
        else mode = 0;
        lastInterruptTime = now;
    }
}

// ---------------- ADC DMA CONFIGURATION ---------------- //
#define SAMPLE_FREQ_HZ  10000  // Stable 10 kHz target sampling frequency for ADC continuous mode
#define READ_LEN        1024   // Chunk size for reading ADC data in bytes (must be a multiple of 4 for 32-bit alignment)

adc_continuous_handle_t adcHandle = NULL;
adc_cali_handle_t adcCaliHandle = NULL;

// 14-byte packet: Header(2) + Float Voltage(4) + Timestamp(8)
uint8_t txBuf[ (READ_LEN / 4) * 14 ]; 

void initADCcontinuous() {
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = ADC_UNIT_1,
        .chan = ADC_CHANNEL_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&cali_config, &adcCaliHandle));

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = READ_LEN * 4,
        .conv_frame_size = READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &adcHandle));

    adc_digi_pattern_config_t adc_pattern = {
        .atten = ADC_ATTEN_DB_12,
        .channel = ADC_CHANNEL_1,
        .unit = ADC_UNIT_1,
        .bit_width = ADC_BITWIDTH_12
    };

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = &adc_pattern,
        .sample_freq_hz = SAMPLE_FREQ_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2, 
    };
    ESP_ERROR_CHECK(adc_continuous_config(adcHandle, &dig_cfg));
    ESP_ERROR_CHECK(adc_continuous_start(adcHandle));
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
    initADCcontinuous();
}

void loop() {
    uint8_t resultBuffer[READ_LEN] __attribute__((aligned(4))); // __attribute__((aligned(4))) ensures the byte array aligns cleanly in memory to avoid garbage pointer casts
    uint32_t outLength = 0;
    
    esp_err_t ret = adc_continuous_read(adcHandle, resultBuffer, READ_LEN, &outLength, 10);
    
    if (ret == ESP_OK && outLength > 0) {
        int txIdx = 0; // Index for the txBuf to keep track of where to write the next packet
        int sampleCount = outLength / sizeof(adc_digi_output_data_t);
        
        static uint64_t nextTimestamp = 0;
        uint64_t currentTime = esp_timer_get_time();
        
        if (nextTimestamp == 0 || currentTime > nextTimestamp + 1000 || nextTimestamp > currentTime + 1000) {
            nextTimestamp = currentTime - (sampleCount * 100); 
        }
        
        uint8_t currentMode = mode; 
        
        for (int i = 0; i < outLength; i += sizeof(adc_digi_output_data_t)) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&resultBuffer[i];
            
            if (p->type2.channel == ADC_CHANNEL_1) {
                int mV = 0;
                adc_cali_raw_to_voltage(adcCaliHandle, p->type2.data, &mV);
                
                // Keep data as a pure floating point decimal so it NEVER overflows at ±50V
                float finaal_mV = (mV - offset[currentMode]) * invGain[currentMode];
                float finalVolt = finaal_mV / 1000.0f; 
                
                txBuf[txIdx++] = 0xAA;
                txBuf[txIdx++] = 0xBB;
                
                memcpy(&txBuf[txIdx], &finalVolt, 4); // 4-byte float Volts
                txIdx += 4;
                
                uint64_t ts = nextTimestamp;
                memcpy(&txBuf[txIdx], &ts, 8); // 8-byte LE Timestamp
                txIdx += 8;
            }
            
            nextTimestamp += 100; // Exactly +100µs per sample for 10 kHz
        }
        
        Serial.write(txBuf, txIdx);
    }
}