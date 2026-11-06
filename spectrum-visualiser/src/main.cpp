/*
  Audio-Reactive Spectrum Visualiser – final firmware
  University of Liverpool EEE, Mini-Challenge 2

  Hardware
    ESP32-S3 (Freenove FNK0085)
    Electret mic -> Q1 (S8050 common-emitter, ~x35) -> 2-stage RC filter (100 kOhm / 100 pF, fc ~6 kHz)
                 -> Q2 (S8050 emitter follower) -> GPIO 4 (audio ADC)
    Mic node -> 10 kOhm / 100 nF -> GPIO 5 (mic health monitor)
    4-in-1 MAX7219 32x8 matrix on hardware SPI (CS 10, MOSI 11, SCK 12)
    I2C LCD1602 (SDA 8, SCL 9)
    Mode button on GPIO 7 to GND (internal pull-up)

  How it works
    Core 0: a dedicated task samples GPIO 4 at exactly 20 kHz into a ring buffer, without gaps.
    Core 1: every 512 new samples (25.6 ms) the main loop takes the latest 512 or 1024 samples,
            removes DC, applies a Hann window, runs the FFT, maps bins to 32 bands, converts to dB
            against a per-column noise floor measured at start-up, smooths, and draws the display.
    Because sampling never stops, slow tasks (LCD over I2C) cannot cause gaps in the audio.

  Modes (button cycles; LOW-FLASH is the power-on default for photosensitive safety)
    LOW-FLASH  80 Hz-4.5 kHz, N=512, fixed 6 dB/row, slow decay (<3 flashes/s), lowest brightness
    ANALYSE    80 Hz-4.5 kHz, N=512, fixed 6 dB/row, fast decay, summed band energy
    MUSIC      30 Hz-4 kHz,   N=1024 with 50% overlap, bass-weighted, auto-ranging, peak-hold dots

  Libraries: MD_MAX72XX (majicDesigns), arduinoFFT v2 (Enrique Condes), LiquidCrystal I2C (Frank de Brabander)
*/

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <MD_MAX72xx.h>
#include <arduinoFFT.h>
#include <LiquidCrystal_I2C.h>
#include "esp_arduino_version.h"

// Arduino-ESP32 core 3.x (current Arduino IDE): sample with the ADC's DMA continuous mode, so timing is set by
// hardware and is perfectly regular. Older core 2.x falls back to a software-timed sampler.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define USE_DMA_ADC 1
  #include "esp_adc/adc_continuous.h"
#else
  #define USE_DMA_ADC 0
#endif

// =====================================================================================
//  SETTINGS TO CHECK AFTER YOUR OWN MEASUREMENTS
// =====================================================================================

// Display orientation (use the values that passed the matrix diagnostic test)
#define HARDWARE_TYPE   MD_MAX72XX::FC16_HW
#define FLIP_COLUMNS    false        // true if low frequencies appear on the right
#define FLIP_ROWS       true         // true if bars grow downwards (your matrix: true, it showed "WIC")

// Audio bias window on GPIO 4 (set to your measured column-20 voltage +/- 300 mV)
const uint16_t AUDIO_BIAS_MIN_MV = 600;
const uint16_t AUDIO_BIAS_MAX_MV = 2200;

// Mic health window on GPIO 5 (healthy mic node is roughly 2.3-2.9 V)
const uint16_t MIC_OK_MIN_MV = 500;    // below this: mic shorted
const uint16_t MIC_OK_MAX_MV = 3090;   // at/above this the ADC is saturated (~3.1 V max): mic node at 3.3 V = disconnected
                                       // (any reading below saturation means the mic is drawing current)
const bool     MIC_CHECK_ENABLED = true;   // set false temporarily to test the audio path without the mic monitor

// SENSITIVITY TUNING (the two settings to adjust by ear)
//   NOISE_GATE_DB: how far above the measured silence a sound must be before a bar lights.
//                  Raise it (e.g. 4-6) if bars flicker in a quiet room; lower it (e.g. 1-2) if quiet sounds are missed.
//   DB_PER_ROW_SETTING: dB per lit row. Raise it (e.g. 6) if bars hit the top too easily; lower it (e.g. 3) for taller bars.
const float NOISE_GATE_DB      = 3.0f;
const float DB_PER_ROW_SETTING = 4.5f;

// Brightness (0-15). Keep BRIGHT_INTENSITY at the level your current measurement showed was safe.
const uint8_t LOWFLASH_INTENSITY = 1;
const uint8_t BRIGHT_INTENSITY      = 3;

// =====================================================================================
//  PINS
// =====================================================================================
const uint8_t AUDIO_PIN  = 4;    // ADC1 channel 3: audio from Q2 emitter
const uint8_t MON_PIN    = 5;    // ADC1 channel 4: mic health monitor
const uint8_t BUTTON_PIN = 7;    // to GND, internal pull-up
const uint8_t SDA_PIN    = 8;
const uint8_t SCL_PIN    = 9;
const uint8_t CS_PIN     = 10;   // MAX7219 CS; MOSI = 11, SCK = 12 (ESP32-S3 default hardware SPI)
const uint8_t MAX_DEVICES = 4;

// =====================================================================================
//  SIGNAL PROCESSING CONSTANTS
// =====================================================================================
const float    FS        = 20000.0f;        // requested sample rate (Hz); Nyquist 10 kHz, guard band 4.5-10 kHz
const uint32_t PERIOD_US = 50;              // 1 / FS
const uint16_t HOP       = 512;             // new samples per display update (25.6 ms)
const uint16_t N_MAX     = 1024;
const uint16_t COLS      = MAX_DEVICES * 8; // 32 display columns
const uint8_t  ROWS      = 8;

const float DB_PER_ROW       = DB_PER_ROW_SETTING;
const float FLOOR_MARGIN_DB  = NOISE_GATE_DB;          // margin above measured silence
const float FLOOR_MARGIN_MAX_DB = NOISE_GATE_DB + 1.0f; // max-bin mode margin
const float DEFAULT_FLOOR_DB = 12.0f;  // used if start-up calibration is too noisy
const float NOISY_RMS_COUNTS = 400.0f; // calibration only rejected if the room is extremely loud (e.g. music playing);
                                       // constant electrical interference is exactly what the per-column floor should absorb
const float DOMINANT_DB      = 6.0f;   // a peak must stand one row above the median column
const float SILENCE_DB       = 4.0f;   // loudest column below floor + this = "Listening..."
const float GUARD_DB         = 15.0f;  // guard-band content this far above its noise floor counts as a real sound

const uint16_t CLIP_LOW  = 5;          // raw ADC counts treated as clipping
const uint16_t CLIP_HIGH = 4090;
const float    FS_TOLERANCE  = 0.01f;  // timing warning if measured fs is off by more than 1 %
const uint32_t MAX_LATE_WARN_US = 10;  // a sample later than this counts as late
const uint32_t MAX_LATE_PER_SEC = 20;  // timing warning only if more than 0.1 % of samples are late

// =====================================================================================
//  MODES
// =====================================================================================
struct ModeConfig {
  const char *lcdName;        // shown on LCD line 1
  const char *matrixName;     // shown on matrix when selected (max 5 chars)
  uint16_t fftN;              // 512 or 1024 (1024 = 50 % overlap because hop is 512)
  float lowHz, highHz;        // display range
  uint8_t bassCols;           // >0: this many columns for lowHz..bassTopHz, rest log-spaced above
  float bassTopHz;
  bool sumEnergy;             // true = summed band energy, false = max bin
  bool autoRange;             // true = scale to recent loudest, false = fixed 6 dB/row
  float attackRowsPerSec;     // how fast bars rise (0 = instant)
  float decayRowsPerSec;      // how fast bars fall
  float hysteresisRows;       // a lit row only changes once the level moves this far past it (stops shimmer)
  bool peakHold;              // peak-hold dots
  uint8_t intensity;          // matrix brightness
  float boostDb;              // extra sensitivity for this mode (dB added to every column)
};

enum ModeId { MODE_LOWFLASH = 0, MODE_ANALYSE = 1, MODE_MUSIC = 2, MODE_COUNT = 3 };

const ModeConfig MODES[MODE_COUNT] = {
  // name         matrix   N     low   high    bass top    sum    auto   attack decay  hyst   peak   bright  boost
  // LOW-FLASH: rise AND fall limited to ~16 rows/s, so a full bar needs ~1 s for one on-off cycle (well under 3 flashes/s),
  // and strong hysteresis stops single pixels flickering at the frame rate.
  // (Attack 24 rows/s + decay 16 rows/s = a full on-off cycle takes ~0.83 s, still well under 3 flashes/s.)
  { "LOW-FLASH",  "LOWFL", 512,  80,  4500,   0,   0,     true,  false, 24.0f, 16.0f, 0.60f, false, LOWFLASH_INTENSITY, 3.0f },
  { "ANALYSE",    "ANLYS", 512,  80,  4500,   0,   0,     true,  false, 0.0f,  50.0f, 0.30f, false, BRIGHT_INTENSITY,   1.5f },
  { "MUSIC",      "MUSIC", 1024, 30,  4000,  12, 250,     false, true,  0.0f,  33.0f, 0.30f, true,  BRIGHT_INTENSITY,   1.5f },
};

// =====================================================================================
//  GLOBALS
// =====================================================================================
MD_MAX72XX mx(HARDWARE_TYPE, CS_PIN, MAX_DEVICES);   // hardware SPI
LiquidCrystal_I2C *lcd = nullptr;
bool lcdPresent = false;

// --- Sampler (core 0) shared state ---
const uint32_t RING_SIZE = 4096;                 // 205 ms of audio
const uint32_t RING_MASK = RING_SIZE - 1;
volatile uint16_t ring[RING_SIZE];
volatile uint32_t writeIndex = 0;                // total samples written
volatile uint32_t maxLateUs  = 0;                // worst lateness since last reset
volatile uint32_t overruns   = 0;                // times the sampler fell >1 ms behind
volatile uint32_t lateCount  = 0;                // samples later than MAX_LATE_WARN_US since last reset
volatile uint16_t monMv      = 0;                // latest mic monitor reading

// --- FFT ---
float vReal[N_MAX];
float vImag[N_MAX];
float amp[N_MAX / 2 + 1];                        // amplitude per bin (ADC counts, window-corrected)
ArduinoFFT<float> fft512(vReal, vImag, 512, FS);
ArduinoFFT<float> fft1024(vReal, vImag, 1024, FS);

// --- Measured sample rate (the ADC hardware runs slightly off the requested rate; bins use the real value) ---
float fsActual = FS;

// --- Bands and calibration ---
struct BandTable { uint16_t lo[COLS]; uint16_t hi[COLS]; uint16_t firstBin; uint16_t lastBin; float binWidth; };
BandTable bands[MODE_COUNT];
float colFloorDb[MODE_COUNT][COLS];
float noisePow512[257];
float noisePow1024[513];
bool  calibratedOk = false;

// --- Display state ---
uint8_t mode = MODE_LOWFLASH;
float level[COLS];                // smoothed bar height (rows)
uint8_t shownH[COLS];             // height actually drawn (with hysteresis)
float peakLevel[COLS];
uint32_t peakHoldUntil[COLS];
float runningTopDb = 12.0f;       // auto-range ceiling (dB above floor)
uint32_t showNameUntil = 0;
uint32_t lastFrameMs = 0;

// --- Health state ---
uint32_t clipUntil = 0, timingWarnUntil = 0;
uint8_t badMicCount = 0, badBiasCount = 0;
bool micFault = false, biasFault = false;
float lastBiasMv = 0, measuredFs = FS;
uint32_t lastHealthMs = 0, lastTimingMs = 0, lastTimingIndex = 0;

// --- Peak / status for LCD and log ---
char statusText[17] = "";
int peakLoHz = -1, peakHiHz = -1;

// --- Per-frame statistics (defined here, before any function, so Arduino's auto-prototypes compile) ---
struct FrameStats { bool clipped; float meanCounts; float acRms; };

// --- Frame bookkeeping ---
uint32_t lastProcessedEnd = 0;

// =====================================================================================
//  SAMPLER TASK (core 0)
// =====================================================================================
#if USE_DMA_ADC
// ---------------- DMA sampler (core 3.x) ----------------
// The ADC hardware alternates between the audio pin (GPIO 4 = ADC1 channel 3) and the mic monitor
// (GPIO 5 = ADC1 channel 4) at 40 kHz in total, giving exactly 20 kHz per pin with no software timing.
adc_continuous_handle_t adcHandle = nullptr;
const adc_channel_t AUDIO_ADC_CH = ADC_CHANNEL_3;   // GPIO 4
const adc_channel_t MON_ADC_CH   = ADC_CHANNEL_4;   // GPIO 5
const uint32_t ADC_FRAME_BYTES = 256;

bool IRAM_ATTR onAdcOverflow(adc_continuous_handle_t, const adc_continuous_evt_data_t *, void *) {
  overruns++;               // the DMA buffer filled before the task read it: samples were lost
  return false;
}

bool startDmaAdc() {
  adc_continuous_handle_cfg_t handleCfg = {};
  handleCfg.max_store_buf_size = 4096;
  handleCfg.conv_frame_size = ADC_FRAME_BYTES;
  if (adc_continuous_new_handle(&handleCfg, &adcHandle) != ESP_OK) return false;

  static adc_digi_pattern_config_t pattern[2] = {};
  pattern[0].atten = ADC_ATTEN_DB_12; pattern[0].channel = AUDIO_ADC_CH; pattern[0].unit = ADC_UNIT_1;
  pattern[0].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;
  pattern[1].atten = ADC_ATTEN_DB_12; pattern[1].channel = MON_ADC_CH; pattern[1].unit = ADC_UNIT_1;
  pattern[1].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

  adc_continuous_config_t cfg = {};
  cfg.pattern_num = 2;
  cfg.adc_pattern = pattern;
  cfg.sample_freq_hz = (uint32_t)(FS * 2);      // two pins share the ADC
  cfg.conv_mode = ADC_CONV_SINGLE_UNIT_1;
  cfg.format = ADC_DIGI_OUTPUT_FORMAT_TYPE2;
  if (adc_continuous_config(adcHandle, &cfg) != ESP_OK) return false;

  adc_continuous_evt_cbs_t cbs = {};
  cbs.on_pool_ovf = onAdcOverflow;
  adc_continuous_register_event_callbacks(adcHandle, &cbs, nullptr);
  return adc_continuous_start(adcHandle) == ESP_OK;
}

void samplerTask(void *) {
  static uint8_t buf[ADC_FRAME_BYTES];
  uint32_t localIndex = 0;
  for (;;) {
    uint32_t got = 0;
    if (adc_continuous_read(adcHandle, buf, ADC_FRAME_BYTES, &got, 100) != ESP_OK) continue;
    for (uint32_t i = 0; i + SOC_ADC_DIGI_RESULT_BYTES <= got; i += SOC_ADC_DIGI_RESULT_BYTES) {
      adc_digi_output_data_t *p = (adc_digi_output_data_t *)&buf[i];
      uint32_t ch = p->type2.channel;
      uint32_t v = p->type2.data;
      if (ch == AUDIO_ADC_CH) {
        ring[localIndex & RING_MASK] = (uint16_t)v;
        localIndex++;
        writeIndex = localIndex;
      } else if (ch == MON_ADC_CH) {
        monMv = (uint16_t)(v * 3100UL / 4095UL);
      }
    }
  }
}
#else
// ---------------- Software-timed sampler (core 2.x fallback) ----------------
void samplerTask(void *) {
  uint32_t next = micros();
  uint32_t localIndex = 0;
  for (;;) {
    while ((int32_t)(micros() - next) < 0) { }          // wait for the exact sample time
    uint32_t late = micros() - next;
    uint16_t v = analogRead(AUDIO_PIN);
    ring[localIndex & RING_MASK] = v;
    localIndex++;
    writeIndex = localIndex;
    if (late > maxLateUs) maxLateUs = late;
    if (late > MAX_LATE_WARN_US) lateCount++;
    next += PERIOD_US;
    if ((int32_t)(micros() - next) > 1000) {            // badly behind: resynchronise and count it
      next = micros() + PERIOD_US;
      overruns++;
    }
    if ((localIndex & 2047) == 0) {                     // ~10 times per second, in the spare time
      // A fast raw read (not analogReadMilliVolts, which is slow enough to delay the next audio sample).
      // All ADC access stays on this one task.
      monMv = (uint16_t)(analogRead(MON_PIN) * 3100UL / 4095UL);
    }
  }
}

#endif

// =====================================================================================
//  SMALL HELPERS
// =====================================================================================
uint16_t physCol(uint16_t c) { return FLIP_COLUMNS ? (COLS - 1 - c) : c; }

uint8_t topRow() { return FLIP_ROWS ? (ROWS - 1) : 0; }

// Bitmask for a bar of height h growing from the bottom of the display
uint8_t barBits(uint8_t h) {
  if (h == 0) return 0;
  if (h > ROWS) h = ROWS;
  uint8_t fromTop = (uint8_t)((1u << h) - 1);
  uint8_t fromBottom = (uint8_t)(fromTop << (ROWS - h));
  return FLIP_ROWS ? fromTop : fromBottom;
}

// Row index for a single dot at height h (1 = bottom row, 8 = top row)
uint8_t rowForLevel(uint8_t h) {
  if (h < 1) h = 1;
  if (h > ROWS) h = ROWS;
  return FLIP_ROWS ? (h - 1) : (ROWS - h);
}

uint8_t reverseBits(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// Draws centred text on the matrix using the library font, respecting the orientation flags
void drawText(const char *txt) {
  uint8_t buf[8];
  uint16_t total = 0;
  for (const char *p = txt; *p; p++) total += mx.getChar(*p, sizeof(buf), buf) + 1;
  if (total > 0) total--;
  int16_t x = (COLS > total) ? (COLS - total) / 2 : 0;
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  mx.clear();
  for (const char *p = txt; *p; p++) {
    uint8_t w = mx.getChar(*p, sizeof(buf), buf);
    for (uint8_t i = 0; i < w && x < COLS; i++, x++) {
      uint8_t bits = FLIP_ROWS ? reverseBits(buf[i]) : buf[i];
      if (x >= 0) mx.setColumn(physCol(x), bits);
    }
    x++;  // 1-column gap
  }
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

// LCD writes are slow over I2C, so only write when text changes, at most 4 times per second
void lcdShow(const char *l1, const char *l2, bool force = false) {
  static char last1[17] = "", last2[17] = "";
  static uint32_t lastWrite = 0;
  if (!lcdPresent) return;
  char a[17], b[17];
  snprintf(a, sizeof(a), "%-16.16s", l1);
  snprintf(b, sizeof(b), "%-16.16s", l2);
  bool changed = strcmp(a, last1) != 0 || strcmp(b, last2) != 0;
  if (!changed) return;
  if (!force && millis() - lastWrite < 250) return;
  lcd->setCursor(0, 0); lcd->print(a);
  lcd->setCursor(0, 1); lcd->print(b);
  strcpy(last1, a); strcpy(last2, b);
  lastWrite = millis();
}

bool i2cDevicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// =====================================================================================
//  FFT FRAME
// =====================================================================================
// Copies the latest N samples ending at 'end', removes DC, windows, FFTs and fills amp[]
FrameStats computeSpectrum(uint16_t N, uint32_t end) {
  FrameStats st = { false, 0, 0 };
  uint32_t start = end - N;
  double sum = 0;
  for (uint16_t i = 0; i < N; i++) {
    uint16_t v = ring[(start + i) & RING_MASK];
    if (v <= CLIP_LOW || v >= CLIP_HIGH) st.clipped = true;
    vReal[i] = v;
    vImag[i] = 0;
    sum += v;
  }
  st.meanCounts = sum / N;
  double sq = 0;
  for (uint16_t i = 0; i < N; i++) {
    vReal[i] -= st.meanCounts;               // remove the DC bias
    sq += (double)vReal[i] * vReal[i];
  }
  st.acRms = sqrt(sq / N);

  ArduinoFFT<float> &fft = (N == 1024) ? fft1024 : fft512;
  fft.windowing(FFTWindow::Hann, FFTDirection::Forward);
  fft.compute(FFTDirection::Forward);
  fft.complexToMagnitude();

  const float norm = N / 4.0f;               // a sine of amplitude A counts gives magnitude A*N/4 with Hann
  for (uint16_t k = 0; k <= N / 2; k++) amp[k] = vReal[k] / norm;
  return st;
}

float toDb(float a) { return 20.0f * log10f(a + 1e-3f); }

// Band value (amplitude, counts) for column c of a mode, from an amplitude-squared array
float bandValueFromPow(const ModeConfig &m, const BandTable &bt, uint16_t c, const float *powArr) {
  float acc = 0;
  for (uint16_t k = bt.lo[c]; k <= bt.hi[c]; k++) {
    if (m.sumEnergy) acc += powArr[k];
    else if (powArr[k] > acc) acc = powArr[k];
  }
  return sqrtf(acc);
}

float bandValueFromAmp(const ModeConfig &m, const BandTable &bt, uint16_t c) {
  float acc = 0;
  for (uint16_t k = bt.lo[c]; k <= bt.hi[c]; k++) {
    if (m.sumEnergy) acc += amp[k] * amp[k];
    else if (amp[k] > acc) acc = amp[k];
  }
  return m.sumEnergy ? sqrtf(acc) : acc;
}

// =====================================================================================
//  BAND TABLES (hybrid log: logarithmic spacing, but every column gets at least one bin)
// =====================================================================================
void buildBands(uint8_t mi) {
  const ModeConfig &m = MODES[mi];
  BandTable &bt = bands[mi];
  bt.binWidth = fsActual / m.fftN;
  int first = max(1, (int)lroundf(m.lowHz / bt.binWidth));
  int last  = min((int)m.fftN / 2 - 1, (int)lroundf(m.highHz / bt.binWidth));
  bt.firstBin = first;
  bt.lastBin = last;
  int prevHi = first - 1;
  for (uint16_t c = 0; c < COLS; c++) {
    float fEdge;
    if (m.bassCols > 0 && c < m.bassCols) {
      fEdge = m.lowHz * powf(m.bassTopHz / m.lowHz, (float)(c + 1) / m.bassCols);
    } else if (m.bassCols > 0) {
      uint16_t n = COLS - m.bassCols;
      fEdge = m.bassTopHz * powf(m.highHz / m.bassTopHz, (float)(c - m.bassCols + 1) / n);
    } else {
      fEdge = m.lowHz * powf(m.highHz / m.lowHz, (float)(c + 1) / COLS);
    }
    int lo = prevHi + 1;
    int hi = (int)floorf(fEdge / bt.binWidth - 0.5f);   // last bin whose centre is below the edge
    if (c == COLS - 1) hi = last;
    if (hi < lo) hi = lo;
    int maxHi = last - (COLS - 1 - c);                  // leave at least one bin per remaining column
    if (hi > maxHi) hi = maxHi;
    if (hi < lo) hi = lo;
    bt.lo[c] = lo;
    bt.hi[c] = hi;
    prevHi = hi;
  }
}

void printBands(uint8_t mi) {
  const BandTable &bt = bands[mi];
  Serial.printf("BAND_TABLE,%s,fs=%.0f,N=%u,bin_width=%.2f\n", MODES[mi].lcdName, fsActual, MODES[mi].fftN, bt.binWidth);
  Serial.println("BAND,column,low_Hz,high_Hz,first_bin,last_bin,floor_dB");
  for (uint16_t c = 0; c < COLS; c++) {
    Serial.printf("BAND,%u,%.0f,%.0f,%u,%u,%.1f\n", c, (bt.lo[c] - 0.5f) * bt.binWidth,
                  (bt.hi[c] + 0.5f) * bt.binWidth, bt.lo[c], bt.hi[c], colFloorDb[mi][c]);
  }
}

// =====================================================================================
//  START-UP: SELF-TEST AND SILENCE CALIBRATION
// =====================================================================================
void selfTest() {
  lcdShow("Self-test", "Watch every col", true);
  mx.control(MD_MAX72XX::INTENSITY, LOWFLASH_INTENSITY);
  for (uint16_t c = 0; c < COLS; c++) {            // one column at a time: slow, no full-display flash
    mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
    mx.clear();
    mx.setColumn(physCol(c), 0xFF);
    mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
    delay(40);
  }
  mx.clear();
}

void waitForSamples(uint32_t target) {
  while ((int32_t)(writeIndex - target) < 0) delay(1);
}

void calibrate() {
  lcdShow("Calibrating", "Stay quiet 2s", true);
  drawText("QUIET");
  memset(noisePow512, 0, sizeof(noisePow512));
  memset(noisePow1024, 0, sizeof(noisePow1024));
  uint16_t frames = 0;
  double rmsSum = 0;

  waitForSamples(writeIndex + N_MAX);              // make sure 1024 fresh samples exist
  uint32_t end = writeIndex;
  uint32_t stopAt = millis() + 2000;
  while ((int32_t)(millis() - stopAt) < 0) {
    end += HOP;
    waitForSamples(end);
    FrameStats st = computeSpectrum(512, end);
    for (uint16_t k = 0; k <= 256; k++) noisePow512[k] = max(noisePow512[k], amp[k] * amp[k]);   // peak-hold
    computeSpectrum(1024, end);
    for (uint16_t k = 0; k <= 512; k++) noisePow1024[k] = max(noisePow1024[k], amp[k] * amp[k]); // peak-hold
    rmsSum += st.acRms;
    frames++;
  }
  // Floors use the LOUDEST level each bin reached during the 2 s of silence (not the average), so fluctuating
  // interference such as the MAX7219's ~730 Hz scan noise stays below the floor instead of flickering above it.
  float meanRms = rmsSum / frames;
  calibratedOk = meanRms <= NOISY_RMS_COUNTS;
  Serial.printf("CAL,frames=%u,ac_rms_counts=%.2f,%s\n", frames, meanRms, calibratedOk ? "ok" : "TOO_NOISY");

  for (uint8_t mi = 0; mi < MODE_COUNT; mi++) {
    const float *powArr = (MODES[mi].fftN == 1024) ? noisePow1024 : noisePow512;
    for (uint16_t c = 0; c < COLS; c++) {
      colFloorDb[mi][c] = calibratedOk
        ? toDb(bandValueFromPow(MODES[mi], bands[mi], c, powArr)) + (MODES[mi].sumEnergy ? FLOOR_MARGIN_DB : FLOOR_MARGIN_MAX_DB)
        : DEFAULT_FLOOR_DB;
    }
  }
  if (!calibratedOk) {
    lcdShow("Too noisy to cal", "Using defaults", true);
    delay(2000);
  }
  lastProcessedEnd = writeIndex;
}

// =====================================================================================
//  MODE CHANGE AND BUTTON
// =====================================================================================
void enterMode(uint8_t m) {
  mode = m;
  for (uint16_t c = 0; c < COLS; c++) { level[c] = 0; shownH[c] = 0; peakLevel[c] = 0; peakHoldUntil[c] = 0; }
  runningTopDb = 12.0f;
  mx.control(MD_MAX72XX::INTENSITY, MODES[m].intensity);
  drawText(MODES[m].matrixName);
  showNameUntil = millis() + 800;
  lcdShow(MODES[m].lcdName, "", true);
  Serial.printf("MODE,%s\n", MODES[m].lcdName);
  printBands(m);
}

// Polled about 1000 times per second; a press counts once the pin has been stable LOW for 30 ms
void handleButton() {
  static bool stableState = HIGH, lastRead = HIGH;
  static uint32_t lastChange = 0;
  bool r = digitalRead(BUTTON_PIN);
  if (r != lastRead) { lastRead = r; lastChange = millis(); }
  if (millis() - lastChange >= 30 && r != stableState) {
    stableState = r;
    if (stableState == LOW) enterMode((mode + 1) % MODE_COUNT);
  }
}

// =====================================================================================
//  HEALTH CHECKS
// =====================================================================================
void updateHealth(const FrameStats &st) {
  uint32_t now = millis();
  if (st.clipped) clipUntil = now + 500;
  lastBiasMv = st.meanCounts * 3100.0f / 4095.0f;     // approximate: 11 dB attenuation ~ 0-3.1 V

  if (now - lastHealthMs >= 500) {
    lastHealthMs = now;
    uint16_t m = monMv;
    bool micBad = MIC_CHECK_ENABLED && ((m < MIC_OK_MIN_MV) || (m > MIC_OK_MAX_MV));
    bool biasBad = (lastBiasMv < AUDIO_BIAS_MIN_MV) || (lastBiasMv > AUDIO_BIAS_MAX_MV);
    badMicCount  = micBad  ? min(badMicCount + 1, 3)  : 0;   // two bad readings in a row = fault
    badBiasCount = biasBad ? min(badBiasCount + 1, 3) : 0;
    micFault  = badMicCount  >= 2;
    biasFault = badBiasCount >= 2;
  }

  if (now - lastTimingMs >= 1000) {
    uint32_t idx = writeIndex;
    if (lastTimingMs != 0) measuredFs = (idx - lastTimingIndex) * 1000.0f / (now - lastTimingMs);
    lastTimingMs = now;
    lastTimingIndex = idx;
    uint32_t late = maxLateUs; maxLateUs = 0;
    uint32_t ov = overruns;    overruns = 0;
    uint32_t lc = lateCount;   lateCount = 0;
    bool bad = fabsf(measuredFs - fsActual) > fsActual * FS_TOLERANCE || lc > MAX_LATE_PER_SEC || ov > 0;
    if (bad) timingWarnUntil = now + 1500;
    Serial.printf("TIMING,fs=%.0f,max_late_us=%lu,late_samples=%lu,overruns=%lu,mic_mV=%u,bias_mV=%.0f\n",
                  measuredFs, (unsigned long)late, (unsigned long)lc, (unsigned long)ov, monMv, lastBiasMv);
  }
}

// =====================================================================================
//  ANALYSIS, SMOOTHING AND DRAWING
// =====================================================================================
void processFrame(uint32_t end) {
  const ModeConfig &m = MODES[mode];
  const BandTable &bt = bands[mode];
  uint32_t now = millis();
  float dt = (lastFrameMs == 0) ? 0.0256f : (now - lastFrameMs) / 1000.0f;
  lastFrameMs = now;

  FrameStats st = computeSpectrum(m.fftN, end);
  updateHealth(st);

  // ---- Fault: stop the spectrum and say why ----
  if (micFault || biasFault) {
    if (now >= showNameUntil) drawText(micFault ? "MIC" : "BIAS");
    if (micFault) lcdShow("MIC FAULT", monMv > MIC_OK_MAX_MV ? "Mic disconnected" : "Mic shorted");
    else          lcdShow("AUDIO BIAS FAULT", "Check Q1 / Q2");
    strcpy(statusText, micFault ? "MIC_FAULT" : "BIAS_FAULT");
    peakLoHz = peakHiHz = -1;
    return;
  }

  // ---- Per-column level in dB above the calibrated floor ----
  float excess[COLS];
  float frameMax = 0;
  for (uint16_t c = 0; c < COLS; c++) {
    excess[c] = toDb(bandValueFromAmp(m, bt, c)) - colFloorDb[mode][c] + m.boostDb;
    if (excess[c] > frameMax) frameMax = excess[c];
  }

  // ---- Target heights ----
  if (m.autoRange) {
    runningTopDb = max(frameMax, runningTopDb - 15.0f * dt);  // jumps up instantly, relaxes 15 dB/s (recovers in ~1 s)
    if (runningTopDb < 12.0f) runningTopDb = 12.0f;
  }
  for (uint16_t c = 0; c < COLS; c++) {
    float target = m.autoRange ? (excess[c] / runningTopDb) * ROWS : excess[c] / DB_PER_ROW;
    target = constrain(target, 0.0f, (float)ROWS);
    // Rise and fall rates (LOW-FLASH limits both, which is what bounds the flash rate)
    if (target >= level[c]) level[c] = (m.attackRowsPerSec > 0) ? min(target, level[c] + m.attackRowsPerSec * dt) : target;
    else level[c] = max(target, level[c] - m.decayRowsPerSec * dt);
    // Hysteresis: only change the drawn height once the level is clearly past the next row
    if (level[c] > shownH[c] + 0.5f + m.hysteresisRows || level[c] < shownH[c] - 0.5f - m.hysteresisRows)
      shownH[c] = (uint8_t)constrain((int)lroundf(level[c]), 0, (int)ROWS);
    if (level[c] < 0.25f) shownH[c] = 0;
    if (m.peakHold) {
      if (level[c] >= peakLevel[c]) { peakLevel[c] = level[c]; peakHoldUntil[c] = now + 400; }
      else if (now > peakHoldUntil[c]) peakLevel[c] = max(level[c], peakLevel[c] - 10.0f * dt);
    }
  }

  // ---- Dominant peak (honest: reported as the range of one FFT bin) ----
  float sorted[COLS];
  memcpy(sorted, excess, sizeof(sorted));
  for (uint16_t i = 1; i < COLS; i++) {                 // insertion sort, 32 values
    float v = sorted[i]; int j = i - 1;
    while (j >= 0 && sorted[j] > v) { sorted[j + 1] = sorted[j]; j--; }
    sorted[j + 1] = v;
  }
  float median = 0.5f * (sorted[COLS / 2 - 1] + sorted[COLS / 2]);
  uint16_t bestCol = 0;
  for (uint16_t c = 1; c < COLS; c++) if (excess[c] > excess[bestCol]) bestCol = c;

  // Strongest content in the guard band (above the display range, below Nyquist)
  const float *noisePow = (m.fftN == 1024) ? noisePow1024 : noisePow512;
  float guardMax = 0;
  for (uint16_t k = bt.lastBin + 1; k < m.fftN / 2 - 2; k++) {
    float e = toDb(amp[k]) - (calibratedOk ? 10.0f * log10f(noisePow[k] + 1e-6f) : DEFAULT_FLOOR_DB);
    if (e > guardMax) guardMax = e;
  }

  char line2[17];
  peakLoHz = peakHiHz = -1;
  if (frameMax < SILENCE_DB && guardMax < GUARD_DB) {
    strcpy(statusText, "LISTENING");
    strcpy(line2, "Listening...");
  } else if (guardMax >= GUARD_DB && guardMax > frameMax + 3.0f) {
    strcpy(statusText, "ABOVE_RANGE");
    snprintf(line2, sizeof(line2), "Peak >%.1fkHz", m.highHz / 1000.0f);
  } else if (frameMax - median >= DOMINANT_DB) {
    uint16_t kBest = bt.lo[bestCol];
    for (uint16_t k = bt.lo[bestCol]; k <= bt.hi[bestCol]; k++) if (amp[k] > amp[kBest]) kBest = k;
    peakLoHz = (int)lroundf((kBest - 0.5f) * bt.binWidth);
    peakHiHz = (int)lroundf((kBest + 0.5f) * bt.binWidth);
    strcpy(statusText, "PEAK");
    snprintf(line2, sizeof(line2), "Peak %d-%dHz", peakLoHz, peakHiHz);
  } else {
    strcpy(statusText, "BROAD");
    strcpy(line2, "Peak: broad");
  }

  // ---- LCD line 1: mode + warning ----
  char line1[17];
  const char *warn = (now < clipUntil) ? "CLIP" : (now < timingWarnUntil) ? "TIME" : "";
  snprintf(line1, sizeof(line1), "%-12.12s%4s", m.lcdName, warn);
  lcdShow(line1, line2);

  // ---- Matrix ----
  if (now < showNameUntil) return;                     // mode name still showing
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::OFF);
  mx.clear();
  for (uint16_t c = 0; c < COLS; c++) {
    uint8_t h = shownH[c];
    uint8_t bits = barBits(h);
    if (m.peakHold) {
      uint8_t p = (uint8_t)(peakLevel[c] + 0.5f);
      if (p > h && p >= 1) bits |= (uint8_t)(1u << rowForLevel(p));
    }
    mx.setColumn(physCol(c), bits);
  }
  if (now < clipUntil) mx.setPoint(topRow(), physCol(COLS - 1), true);   // steady clip indicator
  mx.control(MD_MAX72XX::UPDATE, MD_MAX72XX::ON);
}

// Re-send the MAX7219 configuration every 2 s. If electrical noise corrupts a write (3.3 V logic driving a
// 5 V chip has little margin), a module can get stuck in 'display test' (all LEDs on) or shut down.
// Refreshing the registers recovers automatically; the next frame redraws the content anyway.
void refreshMatrixRegisters() {
  static uint32_t last = 0;
  if (millis() - last < 2000) return;
  last = millis();
  mx.control(MD_MAX72XX::TEST, MD_MAX72XX::OFF);
  mx.control(MD_MAX72XX::DECODE, MD_MAX72XX::OFF);
  mx.control(MD_MAX72XX::SCANLIMIT, ROWS - 1);
  mx.control(MD_MAX72XX::SHUTDOWN, MD_MAX72XX::OFF);
  mx.control(MD_MAX72XX::INTENSITY, MODES[mode].intensity);
}

// One CSV line four times per second, for test evidence
void logLine() {
  static uint32_t lastLog = 0;
  if (millis() - lastLog < 250) return;
  lastLog = millis();
  Serial.printf("SPEC,%s,%s,%d,%d,", MODES[mode].lcdName, statusText, peakLoHz, peakHiHz);
  for (uint16_t c = 0; c < COLS; c++) Serial.print(shownH[c]);
  Serial.println();
}

// =====================================================================================
//  ARDUINO ENTRY POINTS
// =====================================================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Spectrum visualiser ===");

  pinMode(BUTTON_PIN, INPUT_PULLUP);
#if !USE_DMA_ADC
  analogReadResolution(12);
  analogSetPinAttenuation(AUDIO_PIN, ADC_11db);
  analogSetPinAttenuation(MON_PIN, ADC_11db);
#endif

  mx.begin();
  mx.control(MD_MAX72XX::INTENSITY, LOWFLASH_INTENSITY);
  mx.clear();

  Wire.begin(SDA_PIN, SCL_PIN);
  uint8_t addr = i2cDevicePresent(0x27) ? 0x27 : (i2cDevicePresent(0x3F) ? 0x3F : 0);
  if (addr) {
    lcd = new LiquidCrystal_I2C(addr, 16, 2);
    lcd->init();
    lcd->backlight();
    lcdPresent = true;
    Serial.printf("LCD found at 0x%02X\n", addr);
  } else {
    Serial.println("LCD not found: running matrix-only");
  }

#if USE_DMA_ADC
  if (startDmaAdc()) Serial.println("Sampling: ADC DMA continuous mode (hardware-timed)");
  else               Serial.println("ERROR: could not start ADC DMA mode");
  xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, nullptr, 10, nullptr, 0);
#else
  // The sampler gets core 0 to itself. It never sleeps, so the idle watchdog on core 0 is disabled.
  Serial.println("Sampling: software-timed (core 2.x)");
  disableCore0WDT();
  xTaskCreatePinnedToCore(samplerTask, "sampler", 4096, nullptr, 10, nullptr, 0);
#endif

  // Measure the real sample rate over 1 s, then build the band tables from it
  delay(200);
  uint32_t i0 = writeIndex, t0 = micros();
  delay(1000);
  uint32_t i1 = writeIndex, t1 = micros();
  fsActual = (i1 - i0) * 1e6f / (float)(t1 - t0);
  if (fsActual < FS * 0.9f || fsActual > FS * 1.1f) fsActual = FS;   // implausible: keep nominal
  Serial.printf("FS_MEASURED,%.1f\n", fsActual);
  for (uint8_t mi = 0; mi < MODE_COUNT; mi++) buildBands(mi);

  selfTest();
  calibrate();
  enterMode(MODE_LOWFLASH);
}

void loop() {
  handleButton();
  uint32_t w = writeIndex;
  if (w - lastProcessedEnd < HOP) { delay(1); return; }   // wait for the next 512 samples
  uint32_t end = w;            // always use the latest complete window (skips ahead if processing fell behind)
  lastProcessedEnd = end;
  processFrame(end);
  refreshMatrixRegisters();
  logLine();
}
