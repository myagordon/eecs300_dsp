#include <Arduino.h>
#include <driver/i2s.h>
#include "FFT.h"

#define POWER_RATIO_THRESHOLD 0.30
#define I2S_READ_LEN 128

i2s_port_t I2S_PORT = I2S_NUM_0;

int PIN_I2S_BCLK = 25;
int PIN_I2S_WS   = 26;
int PIN_I2S_SD   = 22;

int32_t sampleBuffer[I2S_READ_LEN];

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];

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
  int sampleIndex = 0;

  while (sampleIndex < NUM_SAMPLES) {
    size_t bytesRead = 0;

    i2s_read(
      I2S_PORT,
      sampleBuffer,
      I2S_READ_LEN * sizeof(int32_t),
      &bytesRead,
      portMAX_DELAY
    );

    int samplesRead = bytesRead / sizeof(int32_t);

    for (int i = 0; i < samplesRead && sampleIndex < NUM_SAMPLES; i++) {
      vReal[sampleIndex] = sampleBuffer[i] >> 8;
      vImag[sampleIndex] = 0;
      sampleIndex++;
    }
  }

  FFT.dcRemoval();
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double total_energy = 0;
  double target_band_energy = 0;
  double global_max_mag = 0;
  int global_max_index = 0;
  double target_max_mag = 0;
  int target_max_index = 0;

  int start_index = (600 * NUM_SAMPLES) / SAMPLING_FREQ;
  int end_index   = (1500 * NUM_SAMPLES) / SAMPLING_FREQ;

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
  }

  double ratio = 0.0;
  if (total_energy > 0) {
    ratio = target_band_energy / total_energy;
  }

  double global_freq = (global_max_index * 1.0 * SAMPLING_FREQ) / NUM_SAMPLES;
  double target_freq = (target_max_index * 1.0 * SAMPLING_FREQ) / NUM_SAMPLES;

//scale freq data by 1000 to display on serial moniter
  Serial.print("ratio_x1000:");
  Serial.print(ratio * 1000);
  Serial.print(" ");
  Serial.print("threshold_x1000:");
  Serial.print(POWER_RATIO_THRESHOLD * 1000);
  Serial.print(" ");
  Serial.print("global_freq:");
  Serial.print(global_freq);
  Serial.print(" ");
  Serial.print("target_freq:");
  Serial.println(target_freq);
}
