/*
 * VL53L4CX Multi-Object Speed Detector + Microphone Siren Detector for ESP32
 * ---------------------------------------------------------------------------
 * Detects approaching objects using a VL53L4CX ToF sensor AND detects sirens
 * using an I2S microphone + FFT analysis. Both run concurrently in the main
 * loop without blocking each other.
 *
 * Key design decisions:
 *   - Sensor poll is NON-BLOCKING: checked once per loop(), skipped if not ready.
 *   - fftloop() fills the audio buffer INCREMENTALLY (I2S_READ_LEN samples per
 *     call) so it never stalls. FFT only executes when all FFT_NUM_SAMPLES are
 *     ready, matching the sensor's 50 ms timing budget.
 *   - Because ticks_to_wait=0 is passed to i2s_read(), it returns immediately
 *     with whatever is in the DMA buffer and never blocks the loop.
 *
 * Wiring (ESP32):
 *   VL53L4CX GND    → GND          Mic BCLK → GPIO 27
 *   VL53L4CX VDD    → 3.3 V        Mic WS   → GPIO 26
 *   VL53L4CX SCL    → GPIO 22      Mic SD   → GPIO 32
 *   VL53L4CX SDA    → GPIO 21
 *   VL53L4CX XSHUT  → GPIO 23
 *   LED             → GPIO 2 (built-in)
 */

/* ── Includes ────────────────────────────────────────────────────────────── */
#include <Wire.h>
#include <vl53l4cx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <driver/i2s.h>
#include "FFT.h"

// WiFi library from lab — must come before WirelessCommunication.h
#include "sharedVariable.h"
#include "WirelessCommunication.h"

/* ── Pin Definitions ─────────────────────────────────────────────────────── */
#define SerialPort Serial
#define SDA_PIN 21
#define SCL_PIN 22
#define SENSOR_1 23            // give senor a ID just in case there is multiple sensors, it also is the Xshut pin
#define TRIGGER_PIN 33
#define LED_RED 18
#define LED_YELLOW 19
#define LED_GREEN 5

#ifndef LED_BUILTIN
  #define LED_BUILTIN 2
#endif
#define LedPin LED_BUILTIN

/* ── Sensor Setup ────────────────────────────────────────────────────────── */
// Components.
// use library function to set up VL53L4CX, component name is sensor_vl53l4cx_sat
#define DEV_I2C Wire          // define name for I2C bus
VL53L4CX sensor_vl53l4cx_sat(&DEV_I2C, SENSOR_1);

/*--------------------WIFI--------------------------------*/
volatile shared_uint32 x;
volatile bool reset_flag = false;
static const uint32_t ALERT_COOLDOWN_MS  = 50;
static uint32_t       alert_cooldown_end = 0;

/* ── Tunable Constants: Distance Sensor ─────────────────────────────────── */
// Detection thresholds
static const float    DIST_THRESHOLD_MM    = 5000.0f;  // Only care about objects within 5 m
static const float    SPEED_THRESHOLD_MS   = 0.6f;     // Alert threshold: 0.6 m/s
static const float    MATCH_GATE_MM        = 400.0f;   // Max jump allowed between frames (mm)

// Debounce / hit window
static const int      REQUIRED_HITS        = 3;        // Consecutive fast frames needed
static const uint32_t HIT_WINDOW_MS        = 600;      // All hits must land within 600 ms

// Moving average
static const int      MA_WINDOW_SIZE          = 5;        // Window size for smoothing

// Tracker patience
static const int      MAX_MISSED_FRAMES    = 3;        // Frames to wait before dropping an object
static const uint32_t TRACKER_TIMEOUT_MS = 500; // Wipe if unseen for 500 ms (~10 frames)

// LED hold
static const uint32_t LED_HOLD_MS          = 200;     // Keep LED on for 0.2 s after trigger

// Sensor timing budget (µs) — 50 ms = ~20 Hz
static const uint32_t TIMING_BUDGET_US     = 50000;

// Maximum number of objects we track simultaneously (sensor hardware limit is 4)
static const int      MAX_OBJECTS          = 4;

/* ── Tracker State ───────────────────────────────────────────────────────── */
struct ObjectTracker {
  // Moving average
  float    history[MA_WINDOW_SIZE]; // Circular buffer of raw distances
  float    dist_sum;             // Running sum for the average
  int      idx;                  // Next write position in the circular buffer
  int      count;                // How many valid samples are in the buffer
  float    smoothed;             // Current smoothed distance
  float    prev_smoothed;        // Previous smoothed distance (for speed)

  // Debounce
  int      hit_counter;
  uint32_t first_hit_time;

  // Patience
  int      missed_frames;
  uint32_t last_seen_ms;
};

static ObjectTracker trackers[MAX_OBJECTS];

/* ── Timing ──────────────────────────────────────────────────────────────── */
static uint32_t previous_time_us = 0;
static uint32_t current_time_us  = 0;

/* ── LED Hold ────────────────────────────────────────────────────────────── */
static uint32_t led_off_time_ms = 0;

/* ── Misc ────────────────────────────────────────────────────────────────── */
static bool first_frame = true;
static int  sensor_status = 0; // Used only where we need to propagate status

/* -------------------------------------------------- Microphone --------------------------------------------- */

#define FFT_NUM_SAMPLES   512   // Must match FFT.h NUM_SAMPLES
// SAMPLING_FREQ is used directly from FFT.h

#define POWER_RATIO_THRESHOLD            0.3
#define TARGET_TO_LOWER_RATIO_THRESHOLD 15.00

#define I2S_READ_LEN                128   // Samples read from DMA per fftloop() call
#define REQUIRED_CONSECUTIVE_FRAMES   5   // Method 1: consecutive detections to confirm
#define REQUIRED_CONSECUTIVE_FRAMES_2 7  // Method 2: consecutive detections to confirm

// Method 2 peak-detection parameters
#define MY_MIN_MAG         3000000   // Minimum FFT magnitude for a peak to count
#define MY_PEAK_FACTOR           2   // Peak must be >= this multiple of the band average
#define MY_MIN_PEAK_SPACING  ((int)(300.0 * FFT_NUM_SAMPLES / SAMPLING_FREQ))  // ~300 Hz in bins

// Method 1: Target Band (900-1600 Hz)
#define M1_TARGET_LO  29
#define M1_TARGET_HI  51
// Method 1: Lower Band (100-899 Hz)
#define M1_LOWER_LO   3
#define M1_LOWER_HI   28

// Method 2: Siren frequency range (600-3000 Hz)
#define SIREN_FFT_BIN_LO  19
#define SIREN_FFT_BIN_HI  96

/* ── Microphone / FFT Variables ──────────────────────────────────────────── */
i2s_port_t I2S_PORT = I2S_NUM_0;

int PIN_I2S_BCLK = 27;
int PIN_I2S_WS   = 26;
int PIN_I2S_SD   = 32;

int32_t  sampleBuffer[I2S_READ_LEN];
double   vReal[FFT_NUM_SAMPLES];
double   vImag[FFT_NUM_SAMPLES];

// rawAudio keeps a safe copy because FFT.compute() overwrites vReal in-place
static double rawAudio[FFT_NUM_SAMPLES];

// Detection results — read by WiFi task or other modules
bool sirenDetected  = false;   // Method 1 result
bool sirenDetected2 = false;   // Method 2 result

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_NUM_SAMPLES, SAMPLING_FREQ);

// Internal state: how many samples have been collected in the current window
static int fft_sample_count = 0;

// Consecutive-frame counters for both detection methods
static int method1_consec = 0;
static int method2_consec = 0;
/* ── Forward Declarations ────────────────────────────────────────────────── */
void reset_tracker(int slot);
void push_sample(int slot, float raw_d);
bool process_objects(int object_count, float raw_distances[], int raw_statuses[]);
void fire_combined_alert(bool dist_alert);
void fftsetup();
void fftloop();

/* Setup ---------------------------------------------------------------------*/

void setup()
{
  // Led.
  pinMode(LedPin, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);

  // Initialize serial for output.
  SerialPort.begin(115200);

  while (!Serial);

  INIT_SHARED_VARIABLE(x, 0);   // Start trigger counter at 0
  init_wifi_task();

  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Initialise all tracker slots
  for (int i = 0; i < MAX_OBJECTS; i++) reset_tracker(i);

  // ── Sensor Initialisation ──
  // Configure VL53L4CX satellite component.
  sensor_vl53l4cx_sat.begin();

  // IMPORTANT: Explicitly call Off then On to try and reset the internal state
  sensor_vl53l4cx_sat.VL53L4CX_Off();

  //Initialize VL53L4CX satellite component.
  int status = sensor_vl53l4cx_sat.InitSensor(0x52);
  if (status != 0) {
    Serial.print("[ERROR] InitSensor failed, status=");
    Serial.println(status);
    while (true) { delay(1000); } // Halt — check wiring / I2C address
  }

  // Set the actual time the laser is "on" (Timing Budget)
  // Must be at least 8ms max 200 ms
  status = sensor_vl53l4cx_sat.VL53L4CX_SetMeasurementTimingBudgetMicroSeconds(TIMING_BUDGET_US);
  if (status != 0) {
    Serial.print("[ERROR] SetTimingBudget failed, status=");
    Serial.println(status);
    while (true) { delay(1000); }
  }

  status = sensor_vl53l4cx_sat.VL53L4CX_StartMeasurement();
  SerialPort.print("Start Status: ");
  SerialPort.println(status);
  if (status != 0) {
    while (true) {
      SerialPort.println("Init failed! Check wiring/address.");
      delay(1000);
    }
  }

  // set up for the microphone
  fftsetup();
}

void loop()
{
  // ── Handle reboot request from server ──
  if (reset_flag) {
    reset_flag = false;
    Serial.println("[WiFi] Reboot requested by server — restarting...");
    delay(100);
    ESP.restart();
  }

  uint8_t data_ready = 0;
  sensor_status = sensor_vl53l4cx_sat.VL53L4CX_GetMeasurementDataReady(&data_ready);
  if (sensor_status != 0) {
    Serial.println("[WARN] GetMeasurementDataReady failed — will retry next iteration");
  }

  if (data_ready) {
    previous_time_us = current_time_us;
    current_time_us  = micros();

    VL53L4CX_MultiRangingData_t ranging_data;
    sensor_status = sensor_vl53l4cx_sat.VL53L4CX_GetMultiRangingData(&ranging_data);
    if (sensor_status != 0) {
      Serial.print("[ERROR] GetMultiRangingData failed, status=");
      Serial.println(sensor_status);
      // Clear the interrupt so the sensor keeps running, then fall through to fftloop()
      sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    } else {
      int   object_count = ranging_data.NumberOfObjectsFound;
      int   capped_count = min(object_count, MAX_OBJECTS);
      float raw_dist[MAX_OBJECTS]   = {};
      int   raw_status[MAX_OBJECTS] = {};

      Serial.printf("Frame %u | Objects=%d\n", ranging_data.StreamCount, object_count);

      for (int j = 0; j < capped_count; j++) {
        raw_status[j] = ranging_data.RangeData[j].RangeStatus;
        raw_dist[j]   = ranging_data.RangeData[j].RangeMilliMeter;
        Serial.printf(
          "  [%d] status=%d  dist=%.0f mm  signal=%.2f Mcps  ambient=%.2f Mcps\n",
          j,
          raw_status[j],
          raw_dist[j],
          (float)ranging_data.RangeData[j].SignalRateRtnMegaCps  / 65536.0f,
          (float)ranging_data.RangeData[j].AmbientRateRtnMegaCps / 65536.0f
        );
      }

      if (!first_frame && capped_count > 0) {
        bool dist_alert = process_objects(capped_count, raw_dist, raw_status);
        fire_combined_alert(dist_alert);
      }

      if (first_frame) {
        for (int j = 0; j < capped_count; j++) {
          if (raw_status[j] == 0) {
            reset_tracker(j);
            trackers[j].history[0]    = raw_dist[j];
            trackers[j].dist_sum      = raw_dist[j];
            trackers[j].count         = 1;
            trackers[j].idx           = 1;
            trackers[j].smoothed      = raw_dist[j];
            trackers[j].prev_smoothed = raw_dist[j];
          }
        }
        first_frame = false;
      }

      bool  obj0_valid = (capped_count > 0 && raw_status[0] == 0);
      float obj0_dist  = obj0_valid ? raw_dist[0] : 99999.0f;
      update_led(obj0_valid, obj0_dist);

      sensor_vl53l4cx_sat.VL53L4CX_ClearInterruptAndStartMeasurement();
    }
  }
  // microphone processing
  fftloop();

  // ── Combined siren alert (checked every loop iteration after FFT updates flags) ──
  if (sirenDetected || sirenDetected2) {
    fire_combined_alert(false);   // dist_alert=false; siren flags carry the trigger
  }
}

/* ══════════════════════════════════════════════════════════════════════════
   HELPERS
   ══════════════════════════════════════════════════════════════════════════ */

// set up for microphone
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

void fftloop()
{
  // ── Pull one DMA chunk (non-blocking) ──
  size_t bytes_read = 0;
  i2s_read(I2S_PORT,
           sampleBuffer,
           sizeof(sampleBuffer),
           &bytes_read,
           0);                    // ticks_to_wait = 0  →  non-blocking

  int samples_read = (int)(bytes_read / sizeof(int32_t));

  // ── Append to accumulation buffer ──
  for (int i = 0; i < samples_read && fft_sample_count < FFT_NUM_SAMPLES; i++) {
    // INMP441 outputs 24-bit data left-aligned in a 32-bit word; shift right to normalise.
    rawAudio[fft_sample_count++] = (double)(sampleBuffer[i] >> 8);
  }

  // Buffer not yet full — return early, no computation this iteration
  if (fft_sample_count < FFT_NUM_SAMPLES) return;

  // ── Full window ready: reset counter for next collection cycle ──
  fft_sample_count = 0;

  // ── Copy rawAudio → FFT working arrays ──
  // (rawAudio is kept intact; FFT.compute() will overwrite vReal in-place)
  for (int i = 0; i < FFT_NUM_SAMPLES; i++) {
    vReal[i] = rawAudio[i];
    vImag[i] = 0.0;
  }

  // ── Compute FFT ──
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();   // Results land in vReal[0 .. FFT_NUM_SAMPLES/2]

  // ── Build 16 coarse bins ──
  // Each bin spans (SAMPLING_FREQ / 32) = 256 Hz.
  // In FFT buckets: (FFT_NUM_SAMPLES/2) / 16 = 16 buckets per bin.
  static const int BIN_WIDTH = (FFT_NUM_SAMPLES / 2) / 16;   // = 16
  int bins[16] = {};
  for (int b = 0; b < 16; b++) {
    double sum = 0.0;
    for (int k = 0; k < BIN_WIDTH; k++) {
      sum += vReal[b * BIN_WIDTH + k];
    }
    bins[b] = (int)sum;
  }

  /* ── METHOD 1: Power-ratio detection ──────────────────────────────────── */
  double target_energy = 0.0, lower_energy = 0.0, total_energy = 0.0;

  for (int i = 1; i < (FFT_NUM_SAMPLES / 2); i++) {
    double power = vReal[i] * vReal[i];
    total_energy += power;

    if (i >= M1_TARGET_LO && i <= M1_TARGET_HI) {
      target_energy += power;
    } else if (i >= M1_LOWER_LO && i <= M1_LOWER_HI) {
      lower_energy += power;
    }
  }

  double ratio_total = (total_energy > 0.0) ? (target_energy / total_energy) : 0.0;
  double ratio_lower = (lower_energy > 0.0) ? (target_energy / lower_energy) : 0.0;

  if (ratio_total > POWER_RATIO_THRESHOLD && ratio_lower > TARGET_TO_LOWER_RATIO_THRESHOLD) {
    method1_consec++;
  } else {
    method1_consec = 0;
  }
  sirenDetected = (method1_consec >= REQUIRED_CONSECUTIVE_FRAMES);

  /* ── METHOD 2: Prominent-peak detection ───────────────────────────────── */
  // Compute average magnitude in the siren frequency range for the prominence check
  double band_sum = 0.0;
  int    band_cnt = 0;
  for (int k = SIREN_FFT_BIN_LO; k <= SIREN_FFT_BIN_HI; k++) {
    band_sum += vReal[k];
    band_cnt++;
  }
  double band_avg = (band_cnt > 0) ? (band_sum / band_cnt) : 1.0;

  int peak_count    = 0;
  int last_peak_bin = SIREN_FFT_BIN_LO - MY_MIN_PEAK_SPACING - 1;

  for (int k = SIREN_FFT_BIN_LO + 1; k < SIREN_FFT_BIN_HI; k++) {
    bool is_local_max = (vReal[k] > vReal[k - 1]) && (vReal[k] > vReal[k + 1]);
    bool above_min    = (vReal[k] > (double)MY_MIN_MAG);
    bool prominent    = (vReal[k] > (double)MY_PEAK_FACTOR * band_avg);
    bool well_spaced  = ((k - last_peak_bin) >= MY_MIN_PEAK_SPACING);

    if (is_local_max && above_min && prominent && well_spaced) {
      peak_count++;
      last_peak_bin = k;
    }
  }

  if (peak_count >= 2) {
    method2_consec++;
  } else {
    method2_consec = 0;
  }
  sirenDetected2 = (method2_consec >= REQUIRED_CONSECUTIVE_FRAMES_2);

  // ── Debug output (uncomment printBins for full table) ──
  // printBins(bins);
  Serial.printf("[FFT] ratio_total=%.2f ratio_lower=%.2f consec1=%d | peaks=%d consec2=%d | siren1=%d siren2=%d\n",
                ratio_total, ratio_lower, method1_consec, peak_count, method2_consec,
                (int)sirenDetected, (int)sirenDetected2);
}
/* ═══════════════════════════════════════════════════════════════════════════
   HELPERS — Distance Tracker
   ═══════════════════════════════════════════════════════════════════════════ */

   /* Wipe a tracker slot back to its default state */
void reset_tracker(int slot)
{
  ObjectTracker& t = trackers[slot];
  memset(t.history, 0, sizeof(t.history));
  t.dist_sum      = 0.0f;
  t.idx           = 0;
  t.count         = 0;
  t.smoothed      = 0.0f;
  t.prev_smoothed = 0.0f;
  t.hit_counter   = 0;
  t.first_hit_time = 0;
  t.missed_frames = 0;
  t.last_seen_ms   = 0;
}

/* Update a tracker's circular moving average with a new raw distance */
void push_sample(int slot, float raw_d)
{
  ObjectTracker& t = trackers[slot];

  // If the buffer is full, subtract the oldest value before overwriting it
  if (t.count == MA_WINDOW_SIZE) {
    t.dist_sum -= t.history[t.idx];
  } else {
    t.count++;
  }

  t.history[t.idx] = raw_d;
  t.dist_sum      += raw_d;
  t.idx            = (t.idx + 1) % MA_WINDOW_SIZE;

  t.prev_smoothed = t.smoothed;
  t.smoothed      = t.dist_sum / (float)t.count;
}

/* ══════════════════════════════════════════════════════════════════════════
   CORE TRACKING + SPEED LOGIC
   ══════════════════════════════════════════════════════════════════════════ */
bool process_objects(int object_count, float raw_dist[], int raw_statuses[])
{
  // ── Delta time (seconds) ──
  float delta_t = (float)(current_time_us - previous_time_us) / 1e6f;
  if (delta_t <= 0.0f || delta_t > 1.0f) {
    // Implausibly large gap — sensor stalled or restarted; skip this frame
    Serial.println("[WARN] Implausible delta_t, skipping frame.");
    return false;
  }

  bool trigger_alert = false;

  // Track which incoming readings have been matched to a history slot
  bool reading_claimed[MAX_OBJECTS] = {};

  // ──────────────────────────────────────────────────────────────────────────
  // PHASE 1 — MATCH existing trackers to the nearest new reading
  // ──────────────────────────────────────────────────────────────────────────
  for (int t = 0; t < MAX_OBJECTS; t++) {
    if (trackers[t].count == 0) continue; // Slot is empty — nothing to match

    float best_diff = MATCH_GATE_MM;
    int   best_r    = -1;

    for (int r = 0; r < object_count; r++) {
      if (reading_claimed[r])     continue; // Already assigned
      if (raw_statuses[r] != 0)   continue; // Not a valid reading

      float diff = fabsf(trackers[t].smoothed - raw_dist[r]);
      if (diff < best_diff) {
        best_diff = diff;
        best_r    = r;
      }
    }

    if (best_r != -1) {
      // ── Matched — update this tracker ──
      reading_claimed[best_r]     = true;
      trackers[t].missed_frames   = 0;

      push_sample(t, raw_dist[best_r]);

      // Speed (positive = object approaching)
      float speed = 0.0f;
      if (trackers[t].prev_smoothed > 0.0f && trackers[t].count > 1) {
        speed = (trackers[t].prev_smoothed - trackers[t].smoothed) / 1000.0f / delta_t;
      }

      Serial.printf("  Tracker[%d] smoothed=%.0f mm  speed=%.3f m/s\n",
                    t, trackers[t].smoothed, speed);

      // ── Debounce / hit-window logic ──
      if (speed > SPEED_THRESHOLD_MS && trackers[t].smoothed < DIST_THRESHOLD_MM) {

        uint32_t now = millis();

        if (trackers[t].hit_counter == 0) {
          // First hit — start the window
          trackers[t].first_hit_time = now;
          trackers[t].hit_counter    = 1;
        } else if (now - trackers[t].first_hit_time <= HIT_WINDOW_MS) {
          // Within window — accumulate hit
          trackers[t].hit_counter++;

          if (trackers[t].hit_counter >= REQUIRED_HITS) {
            Serial.printf("  *** ALERT: Tracker[%d] approaching at %.3f m/s ***\n", t, speed);
            trigger_alert            = true;
            trackers[t].hit_counter  = 0; // Reset so a sustained trigger can re-fire
          }
        } else {
          // Window expired — restart
          trackers[t].first_hit_time = now;
          trackers[t].hit_counter    = 1;
        }

      } else {
        // Speed dropped below threshold — reset debounce cleanly
        trackers[t].hit_counter = 0;
      }

    } else {
      // ── No match found — increment missed-frame counter ──
      trackers[t].missed_frames++;
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // PHASE 2 — NEWCOMERS: assign unclaimed readings to empty slots
  // ──────────────────────────────────────────────────────────────────────────
  for (int r = 0; r < object_count; r++) {
    if (reading_claimed[r])   continue;
    if (raw_statuses[r] != 0) continue; // Ignore invalid readings

    for (int t = 0; t < MAX_OBJECTS; t++) {
      if (trackers[t].count == 0) {
        // Seed new tracker
        reset_tracker(t);
        trackers[t].history[0]    = raw_dist[r];
        trackers[t].dist_sum      = raw_dist[r];
        trackers[t].count         = 1;
        trackers[t].idx           = 1;
        trackers[t].smoothed      = raw_dist[r];
        trackers[t].prev_smoothed = raw_dist[r];

        reading_claimed[r] = true;
        Serial.printf("  [New object → Tracker[%d] at %.0f mm]\n", t, raw_dist[r]);
        break;
      }
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // PHASE 3 — REAPER: drop trackers that have been missing too long
  // ──────────────────────────────────────────────────────────────────────────
  for (int t = 0; t < MAX_OBJECTS; t++) {
    if (trackers[t].count == 0) continue;

    bool timed_out    = (trackers[t].last_seen_ms > 0) &&
                        (millis() - trackers[t].last_seen_ms > TRACKER_TIMEOUT_MS);
    bool missed_too_long = (trackers[t].missed_frames >= MAX_MISSED_FRAMES);

    if (timed_out || missed_too_long) {    // ← Either condition wipes it
      Serial.printf("  [Tracker[%d] expired — %s]\n", t,
                    timed_out ? "timeout" : "missed frames");
      reset_tracker(t);
    }
  }

  // Return trigger flag
  return trigger_alert;
}
// LED trigger plus WIFI
// Triggered by any of:  fast-approaching object  |  siren method 1  |  siren method 2
void fire_combined_alert(bool dist_alert)
{
  bool any_alert = dist_alert || sirenDetected || sirenDetected2;
  if (!any_alert) return;

  uint32_t now = millis();

  // ── Local LED — extend hold window ──
  led_off_time_ms = now + LED_HOLD_MS;
  digitalWrite(LedPin, HIGH);

  // ── WiFi — rate-limited so the channel isn't flooded ──
  if (now >= alert_cooldown_end) {
    alert_cooldown_end = now + ALERT_COOLDOWN_MS;
    LOCK_SHARED_VARIABLE(x);
    x.value++;
    UNLOCK_SHARED_VARIABLE(x);
  }

  // Debug
  Serial.printf("[ALERT] dist=%d  siren1=%d  siren2=%d  → LED HIGH + WiFi sent\n",
                (int)dist_alert, (int)sirenDetected, (int)sirenDetected2);
}


/* Drive the main alert LED and the traffic-light distance LEDs */
void update_led(bool object_visible, float closest_dist)
{
  // Speed alert LED — unchanged
  if (millis() < led_off_time_ms) {
    digitalWrite(LedPin, HIGH);
  } else {
    digitalWrite(LedPin, LOW);
  }

  // Distance zone LEDs — driven directly from object[0]
  if (object_visible) {
    if (closest_dist < 1000.0f) {
      digitalWrite(LED_GREEN,  LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_RED,    HIGH);
    } else if (closest_dist < 2500.0f) {
      digitalWrite(LED_GREEN,  LOW);
      digitalWrite(LED_RED,    LOW);
      digitalWrite(LED_YELLOW, HIGH);
    } else {
      digitalWrite(LED_RED,    LOW);
      digitalWrite(LED_YELLOW, LOW);
      digitalWrite(LED_GREEN,  HIGH);
    }
  } else {
    // No valid object[0] — all clear
    digitalWrite(LED_RED,    LOW);
    digitalWrite(LED_YELLOW, LOW);
    digitalWrite(LED_GREEN,  HIGH);
  }
}
