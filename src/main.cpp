#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>
#include "esp_system.h"

// Pin-outs
#define I2S_BCLK 5
#define I2S_LRC 6
#define I2S_DIN 4
#define GAIN_PIN 7

#define I2S_NUM I2S_NUM_0

// Pin-outs (bridge to ground to get specified wave)
#define PIN_SINE 9
#define PIN_SQUARE 10
#define PIN_SAW 11
#define PIN_TRIANGLE 12

// Synth conf
#define SAMPLE_RATE 44100
#define MASTER_GAIN 1.0f
#define WAVETABLE_SIZE 2048
#define PHASE_ACCUMULATOR_MAX 0xFFFFFFFF
#define MAX_VOICES 8

// Global variables
int16_t*      selectedWaveTable;
float         compressorGain    = 1.0f;
float         compressorAttack  = 0.001f;
float         compressorRelease = 0.1f;
double        samplesPerTick;
int           songIndex         = 0;
int           eventCount        = 0;
int           eventIndex        = 0;

int16_t sine_table[WAVETABLE_SIZE];
int16_t square_table[WAVETABLE_SIZE];
int16_t sawtooth_table[WAVETABLE_SIZE];
int16_t triangle_table[WAVETABLE_SIZE];

// Note lengths
#define WHOLE 4.0f
#define HALF 2.0f
#define QUARTER 1.0f
#define EIGHTH 0.5f
#define SIXTEENTH 0.25f

#define PPQ 480

// ADSR
enum ADSRState {
  Inactive,
  Attack,
  Decay,
  Sustain,
  Release,
};

struct ADSR {
  float attack;
  float decay;
  float sustain;
  float release;

  ADSRState state;
};

enum EventType {
  NoteOn,
  NoteOff,
  TempoChange,
};

// LPF
struct LowPassFilter {
  float cutoff;
  float resonance;
  float a0, a1, a2, b0, b1, b2;
  float x1, x2, y1, y2;
};

LowPassFilter globalFilter = {
  6000.0f,
  1.0f,
  0, 0, 0, 0, 0, 0,
  0, 0
};

// Events (duh)
struct NoteEvent {
  const char* note;
  float startBeat;
  float duration;
  char key;
};

// Load song - goal is to make it easy to load different ones on boot
#define SONG_LEGEND 1
#define CURRENT_SONG SONG_LEGEND

#if CURRENT_SONG == SONG_LEGEND
  #include "songs/theLegend.h"
  const NoteEvent* song = theLegend;
#endif

struct InternalEvent {
  uint64_t sampleTime;
  float frequency;
  char key;
  bool noteOn;
  EventType type;
  float newBPM;
};

InternalEvent events[2048];

struct TempoEvent {
  float beat; 
  float bpm;
};

// TODO - add this to the song file instead of in the main script
TempoEvent tempoMap[] = {
  {0.0f, 110},
  {164.0f, 165},
  {0, 0}
};

struct Voice {
  uint32_t phase_accumulator;
  uint32_t phase_increment;
  float frequency;
  bool active;
  char key;
  
  ADSRState envState;
  float envLevel;
  float envTime;
};

Voice voices[MAX_VOICES];

uint32_t voicePhaseCounter = 0;

float updateADSR(Voice *voice, ADSR *adsr, float sampleTime) {
  voice -> envTime += sampleTime;

  switch (voice -> envState) {
    case Attack:
      voice->envLevel = voice->envTime / adsr->attack;
      
      if (voice->envLevel >= 1.0f) {
        voice->envLevel = 1.0f;
        voice->envState = Decay;
        voice->envTime = 0.0f;
      }
      break;

    case Decay:
      voice->envLevel = 1.0f - (1.0f - adsr->sustain) * (voice->envTime / adsr->decay);
      if (voice->envTime >= adsr->decay) {
        voice->envLevel = adsr->sustain;
        voice->envState = Sustain;
        voice->envTime = 0.0f;
      }
      break;

    case Sustain:
    voice->envLevel = adsr->sustain;
    break;

    case Release:
      voice->envLevel = voice->envLevel * (1.0f - (voice->envTime / adsr->release));
      if (voice->envTime >= adsr->release) {
        voice->envLevel = 0.0f;
        voice->envState = Inactive;
        voice->active = false;
      }
      break;

    case Inactive:
    default:
      voice->envLevel = 0.0f;
      break;
  }

  return voice->envLevel;
}

// low pass filter stuff
void updateFilterCoefficients(LowPassFilter* filter) {
  float freq = filter->cutoff;
  if (freq > SAMPLE_RATE * 0.45f) freq = SAMPLE_RATE * 0.45f;
  if (freq < 20.0f) freq = 20.0f;
  
  float res = filter->resonance;
  if (res < 0.5f) res = 0.5f;
  if (res > 10.0f) res = 10.0f;
  
  float omega = 2.0f * PI * freq / SAMPLE_RATE;
  float sin_omega = sin(omega);
  float cos_omega = cos(omega);
  float alpha = sin_omega / (2.0f * res);
  
  float norm = 1.0f / (1.0f + alpha);

  filter->b0 = ((1.0f - cos_omega) * 0.5f) * norm;
  filter->b1 = (1.0f - cos_omega) * norm;
  filter->b2 = filter->b0;
  filter->a0 = 1.0f;
  filter->a1 = (-2.0f * cos_omega) * norm;
  filter->a2 = (1.0f - alpha) * norm;
}

float processFilter(LowPassFilter* filter, float input) {
  float output = filter->b0 * input + filter->b1 * filter->x1 + filter->b2 * filter->x2
                - filter->a1 * filter->y1 - filter->a2 * filter->y2;

  if (isnan(output) || isinf(output)) {
    filter->x1 = filter->x2 = 0;
    filter->y1 = filter->y2 = 0;
    output = input * 0.5f; 
  }

  filter->x2 = filter->x1;
  filter->x1 = input;
  filter->y2 = filter->y1;
  filter->y1 = output;
  
  return output;
}


int searchFreeVoice() {
  for (int i = 0; i < MAX_VOICES; i++) {
    if (!voices[i].active) return i;
  }
  return -1;
}

void clearVoices(){
  for(int i=0;i<MAX_VOICES;i++){
    voices[i].active = false;
    voices[i].phase_accumulator = 0;
    voices[i].phase_increment = 0;
    voices[i].frequency = 0.0f;
  }
}

void StartNote(float frequency, char key) {
  int voice = searchFreeVoice();
  if (voice >= 0) {
      voices[voice].frequency = frequency;
      voices[voice].phase_increment = (uint32_t)((frequency * (1ULL << 32)) / SAMPLE_RATE);
      voices[voice].phase_accumulator = esp_random();
      voices[voice].active = true;
      voices[voice].key = key;

      voices[voice].envState = Attack;
      voices[voice].envLevel = 0.0f;
      voices[voice].envTime = 0.0f;
  }
}

void StopNote(char key) {
  for (int i = 0; i < MAX_VOICES; i++) {
    if (voices[i].active && voices[i].key == key) {
      if (voices[i].envState != Release && voices[i].envState != Inactive) {
        voices[i].envState = Release;
        voices[i].envTime = 0.0f;
      }
      break;
    }
  }
}

void generateTables() {

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

}

// best function name i've ever made
float noteToFrequency(const char* noteName) {
  
  static const char* names[] = {
    "C","C#","D","D#","E","F","F#","G","G#","A","A#","B"
  };
  
  int semitone=-1;
  int octave=-1;
  char base[3]={0};
  base[0]=noteName[0];
  int idx=1;
  
  if(noteName[1]=='#'||noteName[1]=='b') { 
    base[1]=noteName[1]; idx++; 
  }
  
  octave = atoi(&noteName[idx]);
  
  for(int i=0;i<12;i++) { 
    if(strcmp(base,names[i])==0){ 
      semitone=i; break; 
    } 
  }
  
  if(semitone<0||octave<0) {
    return 440.0f;
  }

  int noteNumber = semitone + ((octave + 1) * 12);
  return 440.0f * pow(2.0f,(noteNumber-69)/12.0f);
}

void buildEvents() {
  eventCount = 0;
  eventIndex = 0;
  
  int tempoIndex = 0;
  float currentBPM = tempoMap[tempoIndex].bpm;

  double cumulativeSamples = 0.0;
  double lastTempoBeat = 0.0;
  
  for (int i = 0; song[i].note != nullptr; i++) {
    while (tempoMap[tempoIndex + 1].bpm > 0 &&
           song[i].startBeat >= tempoMap[tempoIndex + 1].beat) {
      
      double beatsInSegment = tempoMap[tempoIndex + 1].beat - lastTempoBeat;
      double samplesPerBeat = (SAMPLE_RATE * 60.0) / currentBPM;
      cumulativeSamples += beatsInSegment * samplesPerBeat;
      
      events[eventCount++] = {(uint64_t)cumulativeSamples, 0.0f, 0, false, TempoChange, tempoMap[tempoIndex + 1].bpm};
      
      lastTempoBeat = tempoMap[tempoIndex + 1].beat;
      tempoIndex++;
      currentBPM = tempoMap[tempoIndex].bpm;
    }
    
    double samplesPerBeat = (SAMPLE_RATE * 60.0) / currentBPM;
    

    double beatsFromLastTempo = song[i].startBeat - lastTempoBeat;
    uint64_t start = (uint64_t)(cumulativeSamples + beatsFromLastTempo * samplesPerBeat);
    
    double endBeat = song[i].startBeat + song[i].duration;
    
    uint64_t end;
    if (tempoMap[tempoIndex + 1].bpm > 0 && endBeat > tempoMap[tempoIndex + 1].beat) {

      double beatsToTempoChange = tempoMap[tempoIndex + 1].beat - song[i].startBeat;
      double beatsAfterTempoChange = endBeat - tempoMap[tempoIndex + 1].beat;
      
      double samplesBeforeChange = beatsToTempoChange * samplesPerBeat;
      double newSamplesPerBeat = (SAMPLE_RATE * 60.0) / tempoMap[tempoIndex + 1].bpm;
      double samplesAfterChange = beatsAfterTempoChange * newSamplesPerBeat;
      
      end = start + (uint64_t)(samplesBeforeChange + samplesAfterChange);
    } else {
      end = start + (uint64_t)(song[i].duration * samplesPerBeat);
    }
    
    float freq = noteToFrequency(song[i].note);
    events[eventCount++] = {start, freq, song[i].key, true, NoteOn, 0.0f};
    events[eventCount++] = {end, freq, song[i].key, false, NoteOff, 0.0f};
  }
  
  for(int a = 0; a < eventCount; a++){
    for(int b = a + 1; b < eventCount; b++){
      if(events[b].sampleTime < events[a].sampleTime){
        InternalEvent tmp = events[a]; 
        events[a] = events[b]; 
        events[b] = tmp;
      }
    }
  }
}

void processEvents(uint64_t startSample, uint64_t endSample){
  while(eventIndex < eventCount){
    uint64_t t = events[eventIndex].sampleTime;
    if(t < endSample){
      if(events[eventIndex].noteOn) StartNote(events[eventIndex].frequency, events[eventIndex].key);
      else StopNote(events[eventIndex].key);
      eventIndex++;
    } else break;
  }
}

ADSR adsr = {
  0.01f,
  0.1f,
  0.7f,
  0.3f
};

inline int16_t getIndividualSample(Voice &voice, int16_t *wavetable){
  uint32_t i = voice.phase_accumulator >> (32 - 11);
  int16_t s = wavetable[i];
  voice.phase_accumulator += voice.phase_increment;
  return s;
}

int16_t getNextSample(int16_t *wavetable) {
  float mix = 0.0f;
  int activeCount = 0;
  float sampleTime = 1.0f / SAMPLE_RATE;

  const float voiceAmplitude = 0.4f;

  for (int i = 0; i < MAX_VOICES; i++) {
    if (!voices[i].active) continue;

    float envLevel = updateADSR(&voices[i], &adsr, sampleTime);

    int16_t raw = getIndividualSample(voices[i], wavetable);
    float normalized = raw / 32767.0f;
    mix += normalized * envLevel * voiceAmplitude;
    activeCount++;
  }

  mix *= MASTER_GAIN;
  mix = processFilter(&globalFilter, mix);

  float targetGain = 1.0f;
  float absMix = fabs(mix);

  if (absMix > 0.8f) {
    targetGain = 0.8f / absMix;
  }

  if (targetGain < compressorGain) {
    compressorGain += (targetGain - compressorGain) * (sampleTime / compressorAttack);
  } else {
    compressorGain += (targetGain - compressorGain) * (sampleTime / compressorRelease);
  }

  mix *= compressorGain;

  if (mix > 1.0f) mix = 1.0f;
  if (mix < -1.0f) mix = -1.0f;

  return (int16_t)(mix * 32767.0f);
}

uint64_t playheadSamples = 0;

void setup(){
  
  setCpuFrequencyMhz(240);
  updateFilterCoefficients(&globalFilter);

  voicePhaseCounter = 0x12345678;
  
  clearVoices();
  generateTables();
  buildEvents();
  
  pinMode(PIN_SINE, INPUT_PULLUP);
  pinMode(PIN_SQUARE, INPUT_PULLUP);
  pinMode(PIN_SAW, INPUT_PULLUP);
  pinMode(PIN_TRIANGLE, INPUT_PULLUP);

  delay(10);

  if (digitalRead(PIN_SINE) == LOW) {
    selectedWaveTable = sine_table;
  } else if (digitalRead(PIN_SQUARE) == LOW) {
    selectedWaveTable = square_table;
  } else if (digitalRead(PIN_SAW) == LOW) {
    selectedWaveTable = sawtooth_table;
  } else if (digitalRead(PIN_TRIANGLE) == LOW) {
    selectedWaveTable = triangle_table;
  } else {
    selectedWaveTable = square_table;
  }

  pinMode(GAIN_PIN, OUTPUT);

  digitalWrite(GAIN_PIN, HIGH);
  delay(200);
  digitalWrite(GAIN_PIN, LOW);
  delay(200);
  digitalWrite(GAIN_PIN, HIGH); // amp is wonky, try to ensure it's in 15db boost mode
  delay(200);

  i2s_config_t cfg = {
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
  
  i2s_driver_install(I2S_NUM, &cfg, 0, nullptr);

  i2s_pin_config_t pins = {
      .bck_io_num = I2S_BCLK,
      .ws_io_num = I2S_LRC,
      .data_out_num = I2S_DIN,
      .data_in_num = I2S_PIN_NO_CHANGE
  };
  i2s_set_pin(I2S_NUM, &pins);

}

void loop(){
  const int BUF = 1024;
  uint32_t stereo[BUF];

  for(int i = 0; i < BUF; i++){
    int16_t s = getNextSample(selectedWaveTable);
    stereo[i] = ((uint32_t)(uint16_t)s << 16) | (uint16_t)s;
  }

  size_t written;
  i2s_write(I2S_NUM, stereo, BUF * 4, &written, portMAX_DELAY);

  uint64_t prev = playheadSamples;
  playheadSamples += BUF;
  processEvents(prev, playheadSamples);
}