#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>

#define I2S_BCLK    5
#define I2S_LRC     6
#define I2S_DIN     4
#define I2S_NUM     I2S_NUM_0
#define SAMPLE_RATE 44100

#define WAVETABLE_SIZE 256
#define PHASE_ACCUMULATOR_MAX 0xFFFFFFFF
#define MAX_VOICES 8

int16_t sine_table[WAVETABLE_SIZE];
int16_t square_table[WAVETABLE_SIZE];
int16_t sawtooth_table[WAVETABLE_SIZE];
int16_t triangle_table[WAVETABLE_SIZE];

uint32_t phase_accumulator = 0;
uint32_t phase_increment = 0;

struct Voice {
    uint32_t phase_accumulator; 
    uint32_t phase_increment;
    float frequency;
    bool active;
    char key;
};

Voice voices[MAX_VOICES];

int searchFreeVoice() {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (!voices[i].active) return i;
    }
    return -1;
}

void StartNote(float frequency, char key) {
    int voice = searchFreeVoice();
    if (voice >= 0) {
        voices[voice].frequency = frequency;
        voices[voice].phase_increment = (uint32_t)((frequency * (1ULL << 32)) / SAMPLE_RATE);
        voices[voice].phase_accumulator = 0;
        voices[voice].active = true;
        voices[voice].key = key;
    } else {
        Serial.println("No voices freed");
    }
}

void StopNote(char key) {
    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].active && voices[i].key == key) {
            voices[i].active = false;
            Serial.printf("Freed voice %d\n", i);
            break;
        }
    }
}

void generateTables() {
    Serial.println("Generating tables...");

    for(int i = 0; i < WAVETABLE_SIZE; i++) {
        float base_rms = 0.25;
        float angle = (2.0 * PI * i) / WAVETABLE_SIZE;

        // multipler is amplitude / volume of output; messing with values to try and get a consistent sounding volume

        // rms
        float sine_mult = base_rms / 0.707f;
        float square_mult = base_rms / 0.577f;
        float sawtooth_mult = base_rms / 4.0f;
        float triangle_mult = base_rms / 0.577f;

        sine_table[i] = (int16_t)(sin(angle) * 32767 * sine_mult);

        square_table[i] = (angle < PI) ? 32767 * square_mult : -32767 * square_mult;

        sawtooth_table[i] = (int16_t)((2.0 * i / WAVETABLE_SIZE - 1.0) * 32767 * sawtooth_mult);

        if (i < WAVETABLE_SIZE / 2) {
            triangle_table[i] = (int16_t)((4.0 * i / WAVETABLE_SIZE - 1.0) * 32767 * triangle_mult);
        } else {
            triangle_table[i] = (int16_t)((3.0 - 4.0 * i / WAVETABLE_SIZE) * 32767 * triangle_mult);
        }
    }
    Serial.println("Wavetables generated.");
};

void setFrequency(float frequency) {
    phase_increment = (uint32_t)((frequency * (1ULL << 32)) / SAMPLE_RATE);
    Serial.printf("Set frequency: %.2f Hz\n", frequency);
}

int16_t getNextSample(int16_t* wavetable) {
    uint32_t table_index = phase_accumulator >> 24;
    int16_t sample = wavetable[table_index];

    phase_accumulator += phase_increment;

    return sample;
}

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("SynthTest");

    generateTables();
    
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 1024,
        .use_apll = false,
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };
    
    esp_err_t err = i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
    Serial.printf("I2S install: %s\n", err == ESP_OK ? "SUCCESS" : "FAILED");
    
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_LRC,
        .data_out_num = I2S_DIN,
        .data_in_num = I2S_PIN_NO_CHANGE
    };
    
    err = i2s_set_pin(I2S_NUM, &pin_config);
    Serial.printf("I2S pins: %s\n", err == ESP_OK ? "SUCCESS" : "FAILED");
    
    Serial.println("Synth init completed");
}

void loop() {
    int16_t* waveforms[] = {sine_table, square_table, sawtooth_table, triangle_table};
    const char* names[] = {"Sine", "Square", "Sawtooth", "TRiangle"};
    float frequencies[] = {440.0, 660.0, 880.0};

    for (int wave = 0; wave < 4; wave++) {
        for(int freq = 0; freq < 3; freq++) {
            Serial.printf("Playing %s wave @ %.0f Hz\n", names[wave], frequencies[freq]);
            setFrequency(frequencies[freq]);

            for(int i = 0; i < SAMPLE_RATE * 3; i++) {
                int16_t sample = getNextSample(waveforms[wave]);
                uint32_t stereo_sample = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;

                size_t bytes_written;
                i2s_write(I2S_NUM, &stereo_sample, 4, &bytes_written, portMAX_DELAY);
            }
        }
    }
    delay(2000);
}