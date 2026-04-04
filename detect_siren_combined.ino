#include <Arduino.h>
#include <driver/i2s.h>
#include "FFT.h"

#define POWER_RATIO_THRESHOLD 0.3
#define TARGET_TO_LOWER_RATIO_THRESHOLD 15.00

#define I2S_READ_LEN 128
#define REQUIRED_CONSECUTIVE_FRAMES 5
#define REQUIRED_CONSECUTIVE_FRAMES_2 20

// Method 2 Parameters
#define MY_MIN_MAG        3000000  // min magnitude for a peak to count, tune this
#define MY_PEAK_FACTOR      2   // peak must be this many times the band average, tune this
#define MY_MIN_PEAK_SPACING ((int)(300.0 * NUM_SAMPLES / SAMPLING_FREQ)) // 250 Hz spacing in bins, tune this

i2s_port_t I2S_PORT = I2S_NUM_0;

int PIN_I2S_BCLK = 25;
int PIN_I2S_WS   = 26;
int PIN_I2S_SD   = 22;

int32_t sampleBuffer[I2S_READ_LEN];

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];

// Flag to be accessed by feedback client unit
bool sirenDetected  = false;
bool sirenDetected2 = false; 

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
  static int hitCount_2 = 0;
  static int consecutiveHitFrames = 0;
  static int consecutiveHitFrames2 = 0;

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

  // Method 1
  double total_energy = 0;
  double target_band_energy = 0;
  double lower_band_energy = 0; 

  double global_max_mag = 0;
  int global_max_index = 0;
  double target_max_mag = 0;
  int target_max_index = 0;

  // target band
  int start_index = (900 * NUM_SAMPLES) / SAMPLING_FREQ;
  int end_index   = (1600 * NUM_SAMPLES) / SAMPLING_FREQ;

  // lower band
  int start_index_lower = (100 * NUM_SAMPLES) / SAMPLING_FREQ;
  int end_index_lower   = (899 * NUM_SAMPLES) / SAMPLING_FREQ;
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
    sirenDetected = false;
    //Serial.println(sirenDetected);
  }

  if (consecutiveHitFrames >= REQUIRED_CONSECUTIVE_FRAMES) {
    hitCount++;
    sirenDetected = true;
    Serial.println(String("Power ratio: ") + sirenDetected);
  }

  // DEBUG OUTPUT
  /*
  if (millis() - windowStart >= 500) {
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
    sirenDetected = false;
  }
  */

  // Method 2
  // Reuses vReal[] magnitudes alr computed above.

  {
    int my_start = (600  * NUM_SAMPLES) / SAMPLING_FREQ;
    int my_end   = (3000 * NUM_SAMPLES) / SAMPLING_FREQ;
    if (my_end >= NUM_SAMPLES / 2) my_end = (NUM_SAMPLES / 2) - 1;

    // Compute band average magnitude
    double bandSum = 0.0;
    for (int i = my_start; i <= my_end; i++) {
      bandSum += vReal[i];
    }
    double bandAvg = bandSum / (my_end - my_start + 1);

    // Find up to 2 tallest peaks that pass spacing + threshold rules
    double peak1_mag = 0.0, peak2_mag = 0.0;
    int    peak1_idx = -1,  peak2_idx = -1;
    int    lastAcceptedIdx = -1000000;

    for (int k = my_start + 1; k <= my_end - 1; k++) {
      // local maximum?
      if (vReal[k] <= vReal[k-1] || vReal[k] <= vReal[k+1]) continue;
      // above absolute floor?
      if (vReal[k] < MY_MIN_MAG) continue;
      // above band average threshold?
      if (vReal[k] < MY_PEAK_FACTOR * bandAvg) continue;
      // far enough from last accepted peak?
      if (k - lastAcceptedIdx < MY_MIN_PEAK_SPACING) continue;

      // keep top 2 by magnitude
      if (vReal[k] > peak1_mag) {
        peak2_mag = peak1_mag; peak2_idx = peak1_idx;
        peak1_mag = vReal[k]; peak1_idx = k;
      } else if (vReal[k] > peak2_mag) {
        peak2_mag = vReal[k]; peak2_idx = k;
      }
      lastAcceptedIdx = k;
    }

    //debugging method 2
    Serial.print("bandAvg: "); Serial.println(bandAvg);
    Serial.print("peak1_mag: "); Serial.println(peak1_mag);

    // second peak was found -> siren 
    sirenDetected2 = (peak1_idx != -1);

    // PERSISTENCE LOGIC
    if (sirenDetected2) {
      consecutiveHitFrames2++;
    } else {
      consecutiveHitFrames2 = 0;
      sirenDetected2 = false;
    }

    if (consecutiveHitFrames >= REQUIRED_CONSECUTIVE_FRAMES_2) {
      hitCount_2++;
      sirenDetected2 = true;
    }
    Serial.println(String("Peak detection: ") + sirenDetected2);
  }
}
