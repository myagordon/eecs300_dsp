#include <Arduino.h>
#include <driver/i2s.h>

i2s_port_t I2S_PORT = I2S_NUM_0;

// Same pins as before
int PIN_I2S_BCLK = 25;
int PIN_I2S_WS   = 26;
int PIN_I2S_SD   = 22;
// LED pins
const int LED_GREEN = 5;
const int LED_YELLOW = 19;
const int LED_RED = 18;

int SAMPLE_RATE = 16000;

// Read 128 stereo frames at a time from I2S
const int FRAMES_PER_READ = 128;

// Use a short block for correlation
const int BLOCK_SIZE = 256;

// Only search small lags near zero
const int MAX_LAG = 12;

// Interleaved raw stereo buffer from I2S
int32_t sampleBuffer[FRAMES_PER_READ * 2];

// Separate left/right sample buffers for one short block
int32_t leftBlock[BLOCK_SIZE];
int32_t rightBlock[BLOCK_SIZE];

int blockIndex = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,   // stereo
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = FRAMES_PER_READ,
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

void loop() {
  size_t bytesRead = 0;

  i2s_read(
    I2S_PORT,
    sampleBuffer,
    sizeof(sampleBuffer),
    &bytesRead,
    portMAX_DELAY
  );

  if (bytesRead != sizeof(sampleBuffer)) {
    return;
  }

  // De-interleave stereo samples and fill one correlation block
  for (int i = 0; i < FRAMES_PER_READ; i++) {
    // Some ESP32/I2S setups swap these two slots.
    // If left/right prints backward in testing, swap these assignments.
    int32_t rightSample = sampleBuffer[2 * i]     >> 8;
    int32_t leftSample  = sampleBuffer[2 * i + 1] >> 8;

    leftBlock[blockIndex] = leftSample;
    rightBlock[blockIndex] = rightSample;
    blockIndex++;

    if (blockIndex >= BLOCK_SIZE) {
      estimateDirection();
      blockIndex = 0;
    }
  }
}

void estimateDirection() {
  // Remove simple DC offset
  int64_t sumL = 0;
  int64_t sumR = 0;
  for (int i = 0; i < BLOCK_SIZE; i++) {
    sumL += leftBlock[i];
    sumR += rightBlock[i];
  }

  int32_t meanL = sumL / BLOCK_SIZE;
  int32_t meanR = sumR / BLOCK_SIZE;

  int32_t bestLag = 0;
  int64_t bestCorr = 0;

  // Cross-correlation over small lag window only
  for (int lag = -MAX_LAG; lag <= MAX_LAG; lag++) {
    int64_t corr = 0;

    for (int n = 0; n < BLOCK_SIZE; n++) {
      int m = n - lag;
      if (m >= 0 && m < BLOCK_SIZE) {
        int32_t x = leftBlock[n] - meanL;
        int32_t y = rightBlock[m] - meanR;
        corr += (int64_t)x * (int64_t)y;
      }
    }

    int64_t absCorr = (corr >= 0) ? corr : -corr;
    if (absCorr > bestCorr) {
      bestCorr = absCorr;
      bestLag = lag;
    }
  }

  //Serial.print("Best lag: ");
  //Serial.print(bestLag);
  //Serial.print("   ");

  if (bestLag > 0) {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, HIGH);
    Serial.println("RIGHT");
  } else if (bestLag < 0) {
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_RED, LOW);
    Serial.println("LEFT");
  } else {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_YELLOW, HIGH);
    digitalWrite(LED_RED, LOW);
    Serial.println("Other");
  }
}