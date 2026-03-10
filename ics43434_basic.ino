#include <Arduino.h>
#include <driver/i2s.h> //I2S driver functions

i2s_port_t I2S_PORT = I2S_NUM_0; //use I2S peripheral 0

int PIN_I2S_BCLK = 25; //bit clock line: ESP32 outputs this clock to the mic
int PIN_I2S_WS   = 26; //word select LRCLK tells I2S device whether the current time slot is L or R channel
int PIN_I2S_SD   = 22; //serial data line

int SAMPLE_RATE = 16000; //consider increasing this down the line for sound direction estimation

int32_t sampleBuffer[128]; //ESP32 I2S driver is commonly set up to read into 32-bit

void setup() {
  Serial.begin(115200);
  delay(1000); 

  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), //ESP32 should be master + receiver
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT, //tells I2S driver to store each sample as 32-bit word
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, //leave SEL/LR floating for left channel
    .communication_format = I2S_COMM_FORMAT_I2S, 
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, //sets interrupt behavior, use default setting
    .dma_buf_count = 4, //Creates 4 DMA buffers
    .dma_buf_len = 128, //Sets each DMA buffer len to 128 samples
    .use_apll = false, //not necessary at 16kHz
    .tx_desc_auto_clear = false, //default
    .fixed_mclk = 0 //No fixed master clock is being used
  };

  //assign pins
  const i2s_pin_config_t pin_config = {
    .bck_io_num = PIN_I2S_BCLK,
    .ws_io_num = PIN_I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = PIN_I2S_SD
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL); //creates + starts I2S driver
  i2s_set_pin(I2S_PORT, &pin_config); //tells I2S peripheral which GPIO pins to use for BCLK, WS, + data in
  i2s_zero_dma_buffer(I2S_PORT); //Fills DMA buffer with zeros initially
}

void loop() {
  size_t bytesRead = 0; //variable stores how many bytes i2s_read() actually received

  //read in audio
  i2s_read(
    I2S_PORT,
    sampleBuffer, //store incoming samples into this array
    128 * sizeof(int32_t), //there are 128 samples, each is 4 bytes
    &bytesRead, //write # of bytes received into this variable
    portMAX_DELAY //wait until info is available
  );

  //only want to process data if buffer is filled
  if (bytesRead == 128 * sizeof(int32_t)) {
    int64_t sum = 0;

    //loop throgh 128 samples, avg them, print to get volume estimate
    for (int i = 0; i < 128; i++) {
      int32_t s = sampleBuffer[i] >> 8; //mic’s meaningful data is 24-bit, but it is stored in 32-bit integer, shift 8???
      if (s < 0) s = -s;
      sum += s;
    }
    Serial.print("volume: ");
    Serial.println(sum / 128);
  }
}
