#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>
#include "esp_system.h"

#define I2S_BCLK    5
#define I2S_LRC     6
#define I2S_DIN     4
#define I2S_NUM     I2S_NUM_0
#define SAMPLE_RATE 44100
#define MASTER_GAIN 2.0f

#define WAVETABLE_SIZE 512
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
        voices[voice].phase_accumulator = esp_random();
        voices[voice].active = true;
        voices[voice].key = key;
        Serial.printf("Started note %c at %.2f Hz, voice %d\n", key, frequency, voice);
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
        float angle = (2.0 * PI * i) / WAVETABLE_SIZE;

        sine_table[i] = (int16_t)(sin(angle) * 16383);

        square_table[i] = (angle < PI) ? 16383 : -16383;

        sawtooth_table[i] = (int16_t)((2.0 * i / WAVETABLE_SIZE - 1.0) * 16383);

        if (i < WAVETABLE_SIZE / 2) {
            triangle_table[i] = (int16_t)((4.0 * i / WAVETABLE_SIZE - 1.0) * 16383);
        } else {
            triangle_table[i] = (int16_t)((3.0 - 4.0 * i / WAVETABLE_SIZE) * 16383);
        }
    }
    Serial.println("Wavetables generated.");
}

int16_t getIndividualSample(Voice &voice, int16_t* wavetable) {
    uint32_t table_index = voice.phase_accumulator >> (32 - 9);
    int16_t sample = wavetable[table_index];
    voice.phase_accumulator += voice.phase_increment;
    return sample;
}

int16_t getNextSample(int16_t* wavetable) {
    int32_t mixed_sample = 0;
    int active_count = 0;

    for (int i = 0; i < MAX_VOICES; i++) {
        if (voices[i].active) {
            mixed_sample += getIndividualSample(voices[i], wavetable);
            active_count++;
        }
    }

    if (active_count > 0) {
        float out = (float)mixed_sample / (float)active_count;
        out *= MASTER_GAIN;
        int32_t s = (int32_t)out;
        if (s > 32767) s = 32767;
        if (s < -32768) s = -32768;
        return (int16_t)s;
    } else {
        return 0;
    }
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
    static unsigned long start_time = millis();
    unsigned long elapsed = millis() - start_time;

    if (elapsed < 2000) {
        StopNote('B');
        bool a_playing = false;
        for(int i = 0; i < MAX_VOICES; i++) {
            if(voices[i].active && voices[i].key == 'A') {
                a_playing = true;
                break;
            }
        }
        if(!a_playing) StartNote(440.0, 'A');
    }
    else if (elapsed < 4000) {
        bool b_playing = false;
        for(int i = 0; i < MAX_VOICES; i++) {
            if(voices[i].active && voices[i].key == 'B') {
                b_playing = true;
                break;
            }
        }
        if(!b_playing) StartNote(880.0, 'B');
    }
    else if (elapsed < 6000) {
        StopNote('A');
    }
    else {
        start_time = millis();
    }

    const int BUFFER_SIZE = 1024;
    uint32_t stereo_samples[BUFFER_SIZE];
    
    int16_t* waveform = sine_table;
    
    for(int i = 0; i < BUFFER_SIZE; i++) {
        int16_t sample = getNextSample(waveform);
        stereo_samples[i] = ((uint32_t)(uint16_t)sample << 16) | (uint16_t)sample;
    }
    
    size_t bytes_written;
    i2s_write(I2S_NUM, stereo_samples, BUFFER_SIZE * 4, &bytes_written, portMAX_DELAY);
}