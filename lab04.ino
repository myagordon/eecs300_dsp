#include "FFT.h"
#define POWER_RATIO_THRESHOLD 0.30 

volatile unsigned int sampling_period_us;
int bins[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
double vReal[NUM_SAMPLES];
double vImag[NUM_SAMPLES];
unsigned long newTime;

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, NUM_SAMPLES, SAMPLING_FREQ);


void setup() {
  fftsetup(); 
}

void loop() {
  fftloop();
}


void fftsetup() {
  Serial.begin(115200);
  sampling_period_us = round(1000000.0 * (1.0 / (double)SAMPLING_FREQ));
}

void fftloop() {
  // Reset bins[]
  for (int i = 0; i<16; i++){
    bins[i] = 0;
  }

  // Sample
  //We must sample at a fixed frequency
  for (int i = 0; i < NUM_SAMPLES; i++) {
    newTime = micros();
    
    vReal[i] = analogRead(INPUT_PIN);//analogRead() takes about 42 microseconds to run for the ESP32
    #ifdef ENABLE_FIR
      vReal[i] = fir.processReading(vReal[i]);
    #endif
    vImag[i] = 0;
    while ((micros() - newTime) < sampling_period_us) {}
  }

  // Compute FFT
  FFT.dcRemoval();
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();
  
  /* place FFT results into 16 bins - chose 16 because it is readable (a bigger number takes up more horizontal screen space)
  For an index i in vReal, FFT.Compute() will store the strength of the frequency i * SAMPLING_FREQ / NUM_SAMPLES
  E.g. if SAMPLING_FREQ is twice of NUM_SAMPLES,
  then, vReal[10] ( after calling FFT.Compute() ) will contain the strength of the frequency 20,
  and vReal[15] will contain the strength of the frequency 30
  we only care about the first half of the output */


  double total_energy = 0;
  double target_band_energy = 0;
  double global_max_mag = 0;
  int global_max_index = 0;
  double target_max_mag = 0;
  int target_max_index = 0;

  // Calculate Bin Indices for 300Hz and 500Hz
  int start_index = (300 * NUM_SAMPLES) / SAMPLING_FREQ;
  int end_index = (500 * NUM_SAMPLES) / SAMPLING_FREQ;

  // Iterate through FFT bins
  for (int i = 0; i < (NUM_SAMPLES / 2); i++) {
    double mag = vReal[i];
    double power = mag * mag; 
    total_energy += power;
    if (mag > global_max_mag) {
        global_max_mag = mag;
        global_max_index = i;
    }

    // Check if this bin is inside 300Hz - 500Hz
    if (i >= start_index && i <= end_index) {
        target_band_energy += power;
        if (mag > target_max_mag) {
            target_max_mag = mag;
            target_max_index = i;
        }
    }
  }

  // 4. CALCULATE RATIO
  double ratio = 0.0;
  if (total_energy > 0) {
    ratio = target_band_energy / total_energy;
  }

  double global_freq = (global_max_index * 1.0 * SAMPLING_FREQ) / NUM_SAMPLES;
  double target_freq = (target_max_index * 1.0 * SAMPLING_FREQ) / NUM_SAMPLES;

  // Serial.print("Ratio: "); Serial.println(ratio); 

  if (ratio > POWER_RATIO_THRESHOLD) {
    // SIREN DETECTED
    Serial.print(target_freq);
    Serial.println(" siren/horn detected");
  } 
  else {
    // NO SIREN
    Serial.println(global_freq);
  }
}
