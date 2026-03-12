#include <Arduino.h>
#include <driver/i2s.h>
#include "FFT.h"

#define POWER_RATIO_THRESHOLD 0.30

i2s_port_t I2S_PORT = I2S_NUM_0;

int PIN_I2S_BCLK = 25; //bit clock line: ESP32 outputs this clock to the mic
int PIN_I2S_WS   = 26; //word select LRCLK tells I2S device whether the current time slot is L or R channel
int PIN_I2S_SD   = 22; //serial data line

int SAMPLE_RATE = 16000;

int32_t sampleBuffer[128];

double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, NUM_SAMPLES, SAMPLING_FREQ);

void setup() {
  fftsetup();
}

void loop() {
  fftloop();
}

//now configures digital mic instead of analog
void fftsetup() {
  Serial.begin(115200);
  delay(1000);

  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 128,
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

  //config i2s peripheral
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

  size_t bytesRead = 0;

  i2s_read(
    I2S_PORT,
    sampleBuffer,
    128 * sizeof(int32_t),
    &bytesRead,
    portMAX_DELAY
  );

  if (bytesRead != 128 * sizeof(int32_t)) {
    return;
  }

  for (int i = 0; i < NUM_SAMPLES; i++) {
    vReal[i] = sampleBuffer[i] >> 8;
    vImag[i] = 0;
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
 
  //changed band to cover 600-1500hz
  int start_index = (600 * NUM_SAMPLES) / SAMPLING_FREQ;
  int end_index = (1500 * NUM_SAMPLES) / SAMPLING_FREQ;

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

  if (ratio > POWER_RATIO_THRESHOLD) {
    Serial.print(target_freq);
    Serial.println(" siren/horn detected");
  } else {
    Serial.println(global_freq);
  }
}
