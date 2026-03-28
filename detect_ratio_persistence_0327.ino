#include <Arduino.h>
#include <driver/i2s.h>
#include "FFT.h"

#define POWER_RATIO_THRESHOLD 0.3
#define TARGET_TO_LOWER_RATIO_THRESHOLD 8.00

#define I2S_READ_LEN 128
#define REQUIRED_CONSECUTIVE_FRAMES 3

i2s_port_t I2S_PORT = I2S_NUM_0;

int PIN_I2S_BCLK = 25;
int PIN_I2S_WS   = 26;
int PIN_I2S_SD   = 22;

int32_t sampleBuffer[I2S_READ_LEN];

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];

// This buffer keeps raw audio safe because FFT overwrites vReal
static double rawAudio[NUM_SAMPLES]; 

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, NUM_SAMPLES, SAMPLING_FREQ);

void fftsetup();
void fftloop();

void setup() {
  fftsetup();
}

void loop() {
  fftloop();
}

void fftsetup() {
  Serial.begin(115200);
  delay(1000);

  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLING_FREQ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = I2S_READ_LEN,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

void fftloop() {
  static unsigned long windowStart = 0;
  static double ratioSum = 0.0;
  static double ratioMax = 0.0;
  static double ratioTargetLowerSum = 0.0;
  static double ratioTargetLowerMax = 0.0; 
  static double globalFreqSum = 0.0;
  static double targetFreqSum = 0.0;
  static int frameCount = 0;
  static int hitCount = 0;
  static int consecutiveHitFrames = 0;

  // OVERLAPPING LOGIC
  int halfSamples = NUM_SAMPLES / 2;

  // 1. Shift the second half of the OLD raw audio to the first half
  for (int i = 0; i < halfSamples; i++) {
    rawAudio[i] = rawAudio[i + halfSamples];
  }

  // 2. Read only enough NEW samples to fill the second half
  int sampleIndex = halfSamples; 
  while (sampleIndex < NUM_SAMPLES) {
    size_t bytesRead = 0;
    i2s_read(I2S_PORT, sampleBuffer, I2S_READ_LEN * sizeof(int32_t), &bytesRead, portMAX_DELAY);
    int samplesRead = bytesRead / sizeof(int32_t);

    for (int i = 0; i < samplesRead && sampleIndex < NUM_SAMPLES; i++) {
      rawAudio[sampleIndex] = sampleBuffer[i] >> 8;
      sampleIndex++;
    }
  }

  // 3. Copy the raw audio into vReal for the FFT to process
  for (int i = 0; i < NUM_SAMPLES; i++) {
    vReal[i] = rawAudio[i];
    vImag[i] = 0;
  }

  // FFT
  FFT.dcRemoval();
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double total_energy = 0;
  double target_band_energy = 0;
  double lower_band_energy = 0; 

  double global_max_mag = 0;
  int global_max_index = 0;
  double target_max_mag = 0;
  int target_max_index = 0;

  // target band
  int start_index = (600 * NUM_SAMPLES) / SAMPLING_FREQ;
  int end_index   = (2000 * NUM_SAMPLES) / SAMPLING_FREQ;

  // lower band
  int start_index_lower = (100 * NUM_SAMPLES) / SAMPLING_FREQ;
  int end_index_lower   = (500 * NUM_SAMPLES) / SAMPLING_FREQ;
  if (start_index_lower < 1) start_index_lower = 1;
  if (end_index_lower >= start_index) end_index_lower = start_index - 1;

  for (int i = 0; i < (NUM_SAMPLES / 2); i++) {
    double mag = vReal[i];
    double power = mag * mag;
    total_energy += power;

    if (mag > global_max_mag) {
      global_max_mag = mag;
      global_max_index = i;
    }

    if (i >= start_index && i <= end_index) {
      target_band_energy += power;
      if (mag > target_max_mag) {
        target_max_mag = mag;
        target_max_index = i;
      }
    }

    if (i >= start_index_lower && i <= end_index_lower) {
      lower_band_energy += power;
    }
  }

  double ratio = (total_energy > 0) ? (target_band_energy / total_energy) : 0.0;
  double ratio_target_to_lower = (lower_band_energy > 0) ? (target_band_energy / lower_band_energy) : 9999.0;

  double global_freq = (global_max_index * 1.0 * SAMPLING_FREQ) / NUM_SAMPLES;
  double target_freq = (target_max_index * 1.0 * SAMPLING_FREQ) / NUM_SAMPLES;

  if (windowStart == 0) windowStart = millis();

  ratioSum += ratio;
  if (ratio > ratioMax) ratioMax = ratio;
  ratioTargetLowerSum += ratio_target_to_lower;
  if (ratio_target_to_lower > ratioTargetLowerMax) ratioTargetLowerMax = ratio_target_to_lower;
  globalFreqSum += global_freq;
  targetFreqSum += target_freq;
  frameCount++;

  // PERSISTENCE LOGIC
  if (ratio > POWER_RATIO_THRESHOLD && ratio_target_to_lower > TARGET_TO_LOWER_RATIO_THRESHOLD) {
    consecutiveHitFrames++;
  } else {
    consecutiveHitFrames = 0;
  }

  if (consecutiveHitFrames >= REQUIRED_CONSECUTIVE_FRAMES) {
    hitCount++;
    if (consecutiveHitFrames == REQUIRED_CONSECUTIVE_FRAMES) {
       Serial.println(" >>> SIREN/HORN DETECTED! <<< ");
    }
  }

  // DEBUG OUTPUT
  if (millis() - windowStart >= 5000) {
    Serial.print("avg_ratio:"); Serial.print(ratioSum / frameCount); Serial.print(" ");
    Serial.print("max_ratio:"); Serial.print(ratioMax); Serial.print(" ");
    Serial.print("avg_tgt_low_ratio:"); Serial.print(ratioTargetLowerSum / frameCount); Serial.print(" ");
    Serial.print("max_tgt_low_ratio:"); Serial.print(ratioTargetLowerMax); Serial.print(" ");
    Serial.print("avg_global_freq:"); Serial.print(globalFreqSum / frameCount); Serial.print(" ");
    Serial.print("avg_target_freq:"); Serial.print(targetFreqSum / frameCount); Serial.print(" ");
    Serial.print("hits:"); Serial.println(hitCount);

    windowStart = millis();
    ratioSum = 0.0; ratioMax = 0.0;
    ratioTargetLowerSum = 0.0; ratioTargetLowerMax = 0.0;
    globalFreqSum = 0.0; targetFreqSum = 0.0;
    frameCount = 0; hitCount = 0; consecutiveHitFrames = 0;
  }
}
