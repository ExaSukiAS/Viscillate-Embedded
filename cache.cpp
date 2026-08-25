#include <Arduino.h>
#include <soc/soc_caps.h>
#include "esp_adc/adc_continuous.h"

// GPIO pin decleration
#define ADC_CHANNEL          ADC_CHANNEL_1
#define MED_MODE_SWITCH_PIN  10
#define LOW_MODE_SWITCH_PIN  0

// Volatile variables modified inside GPIO ISRs (triggered by switch movement) for switching between voltage range modes
volatile uint8_t mode = 0; // current mode (0 = +-40v, 1 = +-17v, 2 = +-7.5v)
volatile bool modeChanged = false; // flag to tell the main loop to send an update whenever mode changes 
volatile uint32_t lastInterruptTime = 0; 
const uint32_t debounceDelay = 50; // ms

#define SAMPLES_PER_FRAME   1024
#define FRAME_SIZE_BYTES    (SAMPLES_PER_FRAME * SOC_ADC_DIGI_RESULT_BYTES) // 4byte per sample = 4KB/frame
#define SAMPLE_RATE_HZ      80000 // 8KHz sampling rate

adc_continuous_handle_t adcHandle = NULL;
TaskHandle_t processingTaskHandle = NULL;

uint16_t rawValues[SAMPLES_PER_FRAME]; // stores raw adc values for a single frame
bool dataSendEnabled = false; // flag to stop or start voltage data sending throug serial

// DMA Interrupt Callback
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(processingTaskHandle, &mustYield);
    return (mustYield == pdTRUE);
}

// initializes ADC DMA
void init_adc_dma() {
    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = FRAME_SIZE_BYTES * 8,
        .conv_frame_size = FRAME_SIZE_BYTES,
    };
    adc_continuous_new_handle(&adc_config, &adcHandle);

    adc_continuous_config_t config = {
        .pattern_num = 1,
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2, 
    };

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_DB_12, // allows us to measure 0 - 2.5v (linear curve)
        .channel = ADC_CHANNEL,
        .unit = ADC_UNIT_1,
        .bit_width = SOC_ADC_DIGI_MAX_BITWIDTH,
    };
    
    config.adc_pattern = &pattern;
    adc_continuous_config(adcHandle, &config);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    adc_continuous_register_event_callbacks(adcHandle, &cbs, NULL);
    adc_continuous_start(adcHandle);
}

// Data Processing Task
void scopeProcessTask(void *pvParameters) {
    uint8_t rawBuffer[FRAME_SIZE_BYTES];
    uint32_t retNum = 0;
    uint32_t frameCount = 0;

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        esp_err_t ret = adc_continuous_read(adcHandle, rawBuffer, FRAME_SIZE_BYTES, &retNum, 0);
        if (ret == ESP_OK && retNum == FRAME_SIZE_BYTES) {
            
            // extract values to keep buffer clear
            for (int i = 0; i < retNum; i += SOC_ADC_DIGI_RESULT_BYTES) {
                adc_digi_output_data_t *p = (adc_digi_output_data_t *)&rawBuffer[i];
                rawValues[i / SOC_ADC_DIGI_RESULT_BYTES] = p->type2.data; 
            }
            
            // only transmit if enabled by the desktop app
            if (dataSendEnabled) {
                uint8_t header[2] = {0xAA, 0xBB}; // data header for voltage
                Serial.write(header, 2);
                Serial.write((uint8_t*)&frameCount, 4);
                Serial.write((uint8_t*)rawValues, SAMPLES_PER_FRAME * sizeof(uint16_t));
                frameCount++;
            }
        }
    }
}

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
        
        modeChanged = true; // Signal the loop() to send the serial message
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

    xTaskCreate(scopeProcessTask, "ScopeTask", 8192, NULL, 10, &processingTaskHandle);
    init_adc_dma();
}

void loop() {
    // handle ISR mode changes and notify desktop app
    if (modeChanged) {
        modeChanged = false;
        uint8_t currentMode = mode; // copy volatile to local
        uint8_t header[2] = {0xCC, 0xDD}; // data header for mode
        Serial.write(header, 2);
        Serial.write(&currentMode, 1);
    }

    // handle incoming Serial Commands
    if(Serial.available() > 0){
        String msg = Serial.readStringUntil('\n');
        msg.trim(); // Remove whitespace/returns
        
        if(msg == "getMode"){
            uint8_t currentMode = mode;
            uint8_t header[2] = {0xCC, 0xDD}; 
            Serial.write(header, 2);
            Serial.write(&currentMode, 1);
        }
        else if (msg == "start") {
            dataSendEnabled = true;
        }
        else if (msg == "stop") {
            dataSendEnabled = false;
        }
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
}