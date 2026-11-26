#include <SPI.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "soc/gpio_reg.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// =====================================================================
// Shrike-Fi optical 2x2 matrix monitor
// Photodiode readout: ESP32-S3 charge/discharge capacitance method
// Channels: 0=A11 (IO4), 1=A12 (IO47), 2=A21 (IO2 - detached), 3=A22 (IO5)
//
// NOTE: IO6/IO7 are the Qwiic I2C pins (pull-ups on board) -> do NOT use.
// Also avoid: IO8..IO13 (FPGA link), IO21 (user LED), IO0/3/46 (strapping),
// IO19/20 (USB), IO43/44 (UART0), and the display/button pins below.
// Wiring: BPW34 cathode -> sense pin, anode -> GND.
// =====================================================================

// ---------------------------------------------------------------------
// Display pinout (HSPI) - ST7789 screen er pins
// ---------------------------------------------------------------------
#define TFT_SCLK 16
#define TFT_MOSI 15
#define TFT_RST  17
#define TFT_DC   18
#define TFT_BLK  45

// Set to a real GPIO when the display CS is wired. -1 means CS is tied LOW.
#ifndef TFT_CS
#define TFT_CS -1
#endif

// Lower starting clock is more reliable with jumper wires and breadboard layouts.
#ifndef TFT_SPI_HZ
#define TFT_SPI_HZ 20000000
#endif

// Active-LOW buttons (to GND) - chap dile LOW hoy
const int HOLD_BUTTON_PIN = 37;
const int VIEW_BUTTON_PIN = 38;
const int ZERO_BUTTON_PIN = 14;

// ---------------------------------------------------------------------
// Photodiode charge/discharge pins (IO 4, 47, 2, 5) - 4ta channel er pin
// ---------------------------------------------------------------------
const int SENSE_PINS[4] = { 4, 47, 2, 5 };

// Detach A21 (index 2, IO2) from physical sensor and drive it proportionately
// in the exact same direction as the average of A11, A12, and A22
const bool  A21_DETACHED          = true;
const float A21_PROPORTION_FACTOR = 0.98f;  // 1.00f = exact 1:1 average of other 3 sensors

// Nijer hardware capacitance ar light level onujayi ei value gulo tune koro
const uint32_t CHARGE_US      = 30;     // pin HIGH kore charge korar somoy (microseconds)
const uint32_t TIMEOUT_US     = 20000;  // dark/slow hole max wait limit (= 20 ms)
const uint32_t DARK_CUTOFF_US = 1500;   // er cheye slow discharge hole dark leakage dhore 0 korbe
const uint32_t FULL_US        = 2;      // khub bright light e discharge time (us)
const uint32_t FAST_WINDOW_US = 500;    // ISR-protected fast discharge window (us)

// 5 sub-samples spaced 4 ms apart = 20 ms (50 Hz mains cycle) median filter
const int      BURST_SAMPLES     = 5;
const uint32_t BURST_INTERVAL_US = 4000;

// Output stabilization tuning
const int   MAX_STEP_PER_SAMPLE = 60;     // ek sample e max koto count change hote parbe
const float EMA_ALPHA           = 0.35f;  // smoothing factor (0.1 = beshi smooth, 0.5 = fast)
const int   OUTPUT_DEADBAND     = 3;      // +/-3 count er choto flicker ignore korbe

// Prottek sample e Serial e discharge time ar scaled value print korbe
#define DEBUG_SERIAL 1

// Channel gulo balance korar jonno per-channel gain (IO47 1.8V domain compensate kora)
float channelGain[4] = { 1.0f, 0.65f, 1.0f, 1.0f };

const int VAL_MAX = 4095;  // display full scale value

// ---------------------------------------------------------------------
// RGB565 Colors
// ---------------------------------------------------------------------
#define BLACK   0x0000
#define WHITE   0xFFFF
#define CYAN    0x07FF
#define GREEN   0x07E0
#define YELLOW  0xFFE0
#define ORANGE  0xFD20
#define MAGENTA 0xF81F
#define GRAY    0x4208

SPIClass tftSPI;
portMUX_TYPE sampleMux = portMUX_INITIALIZER_UNLOCKED;

// ---------------------------------------------------------------------
// 5x7 Bitmap Glyphs - screen e lekhar font data
// ---------------------------------------------------------------------
const uint8_t DIGITS_5X7[10][5] = {
  {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
  {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
  {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
  {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
  {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
  {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
  {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
  {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
  {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
  {0x06, 0x49, 0x49, 0x29, 0x1E}  // 9
};

const uint8_t GLYPH_A[5]     = {0x7E, 0x11, 0x11, 0x11, 0x7E};
const uint8_t GLYPH_O[5]     = {0x3E, 0x41, 0x41, 0x41, 0x3E};
const uint8_t GLYPH_U[5]     = {0x3F, 0x40, 0x40, 0x40, 0x3F};
const uint8_t GLYPH_T[5]     = {0x01, 0x01, 0x7F, 0x01, 0x01};
const uint8_t GLYPH_P[5]     = {0x7F, 0x09, 0x09, 0x09, 0x06};
const uint8_t GLYPH_COLON[5] = {0x00, 0x00, 0x24, 0x00, 0x00};
const uint8_t GLYPH_MINUS[5] = {0x08, 0x08, 0x08, 0x08, 0x08};

const char* ELEMENT_NAMES[4] = { "A11", "A12", "A21", "A22" };

// ---------------------------------------------------------------------
// Plot geometry (240x240 display) - graph er size ar position
// ---------------------------------------------------------------------
const uint16_t PLOT_X = 24;
const uint16_t PLOT_Y = 44;
const uint16_t PLOT_W = 200;
const uint16_t PLOT_H = 176;
const int HIST_STEP_X = 4;
const int HIST_LEN    = PLOT_W / HIST_STEP_X; // 50 samples history

// ---------------------------------------------------------------------
// Measurement / display state - current reading ar UI state gulo
// ---------------------------------------------------------------------
uint32_t lastDischargeUs[4] = { 0, 0, 0, 0 };
bool     chTimeout[4]       = { false, false, false, false };
int      rawVals[4]         = { 0, 0, 0, 0 };
int      zeroOffsets[4]     = { 0, 0, 0, 0 };
float    emaVals[4]         = { 0.0f, 0.0f, 0.0f, 0.0f };
bool     emaReady[4]        = { false, false, false, false };
int      currentVals[4]     = { 0, 0, 0, 0 };
int      history[4][HIST_LEN];
bool     historyValid[4][HIST_LEN];
static uint8_t plotBuffer[PLOT_W * PLOT_H * 2];

int      prevCellVals[4]   = { 99999, 99999, 99999, 99999 };
uint16_t prevCellColors[4] = { 0, 0, 0, 0 };
bool     matrixDrawnOnce   = false;

int  lastHeaderVal     = 99999;
bool lastHeaderTimeout = false;

// View Mode: 0 = 2x2 Matrix, 1 = A11, 2 = A12, 3 = A21, 4 = A22
int  viewMode = 0;
bool needsFullRedraw = true;
bool isFrozen = false;

// Button debounce structure
struct DebouncedButton {
  int pin;
  int lastRaw;
  int stableState;
  unsigned long lastChangeMs;
};

DebouncedButton holdBtn = { HOLD_BUTTON_PIN, HIGH, HIGH, 0 };
DebouncedButton viewBtn = { VIEW_BUTTON_PIN, HIGH, HIGH, 0 };
DebouncedButton zeroBtn = { ZERO_BUTTON_PIN, HIGH, HIGH, 0 };

const unsigned long DEBOUNCE_MS = 35;   // button bounce komanor somoy
const unsigned long SAMPLE_MS   = 200;  // koto ms por por newa sample
unsigned long lastSampleMs = 0;

const int ZERO_AVG = 4;  // zero korar somoy koto bar average nebe
bool zeroCalibrationActive = false;
int zeroSamplesTaken = 0;
long zeroAccumulator[4] = { 0, 0, 0, 0 };
int zeroValidCount[4] = { 0, 0, 0, 0 };

// =====================================================================
// Capacitance discharge readout (IO 4, 47, 2, 5)
// =====================================================================

// Fast GPIO level read
static inline bool readPinFast(int pin) {
  return gpio_get_level((gpio_num_t)pin) != 0;
}

// Sense pin ke internal pull chara clean floating input e set kore
static inline void setSensePinFloatingInput(int pin) {
  gpio_num_t g = (gpio_num_t)pin;
  gpio_pullup_dis(g);
  gpio_pulldown_dis(g);
  gpio_set_direction(g, GPIO_MODE_INPUT);
}

static inline void releaseAllSensePins() {
  REG_CLR_BIT(GPIO_ENABLE_REG, (1U << 4) | (1U << 5));
  if (!A21_DETACHED) {
    REG_CLR_BIT(GPIO_ENABLE_REG, (1U << 2));
  }
  REG_CLR_BIT(GPIO_ENABLE1_REG, 1U << (47 - 32));
}

// Boot-time check: pin LOW kore chere dao, kono external pull-up HIGH tane kina dekho.
// Shudhu photodiode thakle node HIGH hobe na. Jodi HIGH hoy, tahole
// oi pin e external pull-up ache (I2C, FPGA, header resistor).
void checkSensePinsForPullups() {
  for (int i = 0; i < 4; i++) {
    if (A21_DETACHED && i == 2) {
      Serial.println("Pin check A21 (IO2): DETACHED (proportional to A11, A12, A22)");
      continue;
    }

    int pin = SENSE_PINS[i];
    gpio_num_t g = (gpio_num_t)pin;

    gpio_pullup_dis(g);
    gpio_pulldown_dis(g);
    gpio_set_direction(g, GPIO_MODE_INPUT_OUTPUT);
    gpio_set_level(g, 0);
    delayMicroseconds(500);          // node fully discharge howar jonno

    setSensePinFloatingInput(pin);
    delay(2);                        // pull-up thakle kaj korar somoy dao

    bool high = readPinFast(pin);
    Serial.print("Pin check ");
    Serial.print(ELEMENT_NAMES[i]);
    Serial.print(" (IO");
    Serial.print(pin);
    Serial.print("): ");
    Serial.println(high ? "WARNING - pulled HIGH (external pull-up?)" : "OK (floats low)");
  }
}

// Light beshi hole discharge taratari hoy -> dischargeUs kom hoy.
// Photocurrent ~ 1 / dischargeUs hisebe 0..VAL_MAX e scale kora hoy.
int dischargeUsToScaled(int ch, uint32_t dischargeUs, bool timeout) {
  if (timeout || dischargeUs >= DARK_CUTOFF_US) return 0;
  if (dischargeUs < FULL_US) dischargeUs = FULL_US;

  float rate     = 1.0f / (float)dischargeUs;
  float rateDark = 1.0f / (float)DARK_CUTOFF_US;
  float rateFull = 1.0f / (float)FULL_US;

  float norm = (rate - rateDark) / (rateFull - rateDark);
  norm = constrain(norm, 0.0f, 1.0f);

  float scaled = norm * channelGain[ch] * (float)VAL_MAX;
  return (int)constrain(scaled + 0.5f, 0.0f, (float)VAL_MAX);
}

// Single raw charge/discharge snapshot (ISR-protected fast window)
void measureOnceRaw(uint32_t outUs[4]) {
  bool waiting[4] = { false, false, false, false };
  int pending = 0;

  for (int i = 0; i < 4; i++) {
    outUs[i] = TIMEOUT_US;
    if (A21_DETACHED && i == 2) continue;

    gpio_num_t g = (gpio_num_t)SENSE_PINS[i];
    gpio_pullup_dis(g);
    gpio_pulldown_dis(g);
    gpio_set_direction(g, GPIO_MODE_INPUT_OUTPUT);
    waiting[i] = true;
    pending++;
  }

  portENTER_CRITICAL(&sampleMux);
  for (int i = 0; i < 4; i++) {
    if (!waiting[i]) continue;
    gpio_set_level((gpio_num_t)SENSE_PINS[i], 1);
  }
  delayMicroseconds(CHARGE_US);

  int64_t releaseTime = esp_timer_get_time();
  releaseAllSensePins();

  while (pending > 0) {
    const int64_t now = esp_timer_get_time();
    const uint32_t dt = (uint32_t)(now - releaseTime);

    for (int i = 0; i < 4; i++) {
      if (!waiting[i]) continue;

      if (!readPinFast(SENSE_PINS[i])) {
        outUs[i] = (dt == 0) ? 1 : dt;
        waiting[i] = false;
        pending--;
      } else if (dt >= TIMEOUT_US) {
        waiting[i] = false;
        pending--;
      }
    }
    if (dt >= FAST_WINDOW_US) break;
  }
  portEXIT_CRITICAL(&sampleMux);

  while (pending > 0) {
    const int64_t now = esp_timer_get_time();
    const uint32_t dt = (uint32_t)(now - releaseTime);

    for (int i = 0; i < 4; i++) {
      if (!waiting[i]) continue;

      if (!readPinFast(SENSE_PINS[i])) {
        outUs[i] = dt;
        waiting[i] = false;
        pending--;
      } else if (dt >= TIMEOUT_US) {
        waiting[i] = false;
        pending--;
      }
    }
  }
}

// 5-sample median burst niye spike ar 50 Hz flicker bad diye discharge observation nao
void sampleAllRaw() {
  uint32_t burstUs[4][BURST_SAMPLES];

  for (int s = 0; s < BURST_SAMPLES; s++) {
    int64_t subStart = esp_timer_get_time();
    uint32_t singleUs[4];
    measureOnceRaw(singleUs);

    for (int i = 0; i < 4; i++) {
      burstUs[i][s] = singleUs[i];
    }

    if (s + 1 < BURST_SAMPLES) {
      while ((esp_timer_get_time() - subStart) < (int64_t)BURST_INTERVAL_US) {
        delayMicroseconds(200);
      }
    }
  }

  for (int i = 0; i < 4; i++) {
    if (A21_DETACHED && i == 2) continue;

    // Insertion sort 5 readings to pick median discharge time
    for (int a = 1; a < BURST_SAMPLES; a++) {
      uint32_t key = burstUs[i][a];
      int b = a - 1;
      while (b >= 0 && burstUs[i][b] > key) {
        burstUs[i][b + 1] = burstUs[i][b];
        b--;
      }
      burstUs[i][b + 1] = key;
    }

    lastDischargeUs[i] = burstUs[i][BURST_SAMPLES / 2];
    chTimeout[i] = lastDischargeUs[i] >= DARK_CUTOFF_US;
    rawVals[i] = dischargeUsToScaled(i, lastDischargeUs[i], chTimeout[i]);
  }

  if (A21_DETACHED) {
    float avgRaw3 = (rawVals[0] + rawVals[1] + rawVals[3]) / 3.0f;
    rawVals[2] = (int)constrain(roundf(avgRaw3 * A21_PROPORTION_FACTOR), 0.0f, (float)VAL_MAX);
    chTimeout[2] = chTimeout[0] && chTimeout[1] && chTimeout[3];
    lastDischargeUs[2] = (lastDischargeUs[0] + lastDischargeUs[1] + lastDischargeUs[3]) / 3;
  }
}

// Zero offset bad diye final value ber koro
int applyZeroOffset(int idx, int rawVal) {
  int diff = rawVal - zeroOffsets[idx];
  return (int)constrain(diff, -VAL_MAX, VAL_MAX);
}

// =====================================================================
// ST7789 low-level driver
// =====================================================================

void sendCmd(uint8_t cmd) {
  digitalWrite(TFT_DC, LOW);
  tftSPI.transfer(cmd);
}

void sendData(uint8_t data) {
  digitalWrite(TFT_DC, HIGH);
  tftSPI.transfer(data);
}

// ST7789 display initialize kora hocche
void initST7789() {
  pinMode(TFT_DC, OUTPUT);
  pinMode(TFT_RST, OUTPUT);
  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);

  digitalWrite(TFT_RST, HIGH);
  delay(50);
  digitalWrite(TFT_RST, LOW);
  delay(50);
  digitalWrite(TFT_RST, HIGH);
  delay(150);

  tftSPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  tftSPI.beginTransaction(SPISettings(TFT_SPI_HZ, MSBFIRST, SPI_MODE3));

  sendCmd(0x01); delay(150);
  sendCmd(0x11); delay(120);
  sendCmd(0x3A); sendData(0x55);
  sendCmd(0x36); sendData(0x00);
  sendCmd(0x21);
  sendCmd(0x13); delay(10);
  sendCmd(0x29); delay(50);
}

void setWriteWindow(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  sendCmd(0x2A);
  sendData(x >> 8); sendData(x & 0xFF);
  sendData((x + w - 1) >> 8); sendData((x + w - 1) & 0xFF);

  sendCmd(0x2B);
  sendData(y >> 8); sendData(y & 0xFF);
  sendData((y + h - 1) >> 8); sendData((y + h - 1) & 0xFF);

  sendCmd(0x2C);
  digitalWrite(TFT_DC, HIGH);
}

void writeRgb565Buffer(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint8_t *buffer) {
  setWriteWindow(x, y, w, h);
  tftSPI.writeBytes(buffer, (size_t)w * h * 2);
}

// Screen e ekta ronger box/rectangle anka
void fillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
  if (w == 0 || h == 0 || x >= 240 || y >= 240) return;
  if (x + w > 240) w = 240 - x;
  if (y + h > 240) h = 240 - y;

  setWriteWindow(x, y, w, h);

  uint8_t lineBuf[480];
  uint8_t hi = color >> 8;
  uint8_t lo = color & 0xFF;
  uint16_t bytesPerRow = w * 2;
  if (bytesPerRow > sizeof(lineBuf)) bytesPerRow = sizeof(lineBuf);

  for (uint16_t i = 0; i + 1 < bytesPerRow; i += 2) {
    lineBuf[i] = hi;
    lineBuf[i + 1] = lo;
  }
  for (uint16_t row = 0; row < h; row++) {
    tftSPI.writeBytes(lineBuf, bytesPerRow);
  }
}

void drawGlyph5x7(uint16_t x, uint16_t y, const uint8_t glyph[5], uint8_t scale, uint16_t fg, uint16_t bg) {
  if (scale == 0 || scale > 3) return;

  const uint16_t width = 5 * scale;
  const uint16_t height = 7 * scale;
  uint8_t glyphBuffer[5 * 3 * 7 * 3 * 2];

  for (uint16_t row = 0; row < height; row++) {
    uint16_t sourceRow = row / scale;
    for (uint16_t col = 0; col < width; col++) {
      uint16_t sourceCol = col / scale;
      uint16_t color = (glyph[sourceCol] & (1 << sourceRow)) ? fg : bg;
      size_t byteOffset = ((size_t)row * width + col) * 2;
      glyphBuffer[byteOffset] = color >> 8;
      glyphBuffer[byteOffset + 1] = color & 0xFF;
    }
  }

  writeRgb565Buffer(x, y, width, height, glyphBuffer);
}

void drawChar5x7(uint16_t x, uint16_t y, char ch, uint8_t scale, uint16_t fg, uint16_t bg) {
  if (ch >= '0' && ch <= '9') {
    drawGlyph5x7(x, y, DIGITS_5X7[ch - '0'], scale, fg, bg);
  } else if (ch == '-') {
    drawGlyph5x7(x, y, GLYPH_MINUS, scale, fg, bg);
  } else if (ch == 'A') {
    drawGlyph5x7(x, y, GLYPH_A, scale, fg, bg);
  } else if (ch == 'O') {
    drawGlyph5x7(x, y, GLYPH_O, scale, fg, bg);
  } else if (ch == 'U') {
    drawGlyph5x7(x, y, GLYPH_U, scale, fg, bg);
  } else if (ch == 'T') {
    drawGlyph5x7(x, y, GLYPH_T, scale, fg, bg);
  } else if (ch == 'P') {
    drawGlyph5x7(x, y, GLYPH_P, scale, fg, bg);
  } else if (ch == ':') {
    drawGlyph5x7(x, y, GLYPH_COLON, scale, fg, bg);
  }
}

// =====================================================================
// Rendering - screen e matrix ar graph draw korar function gulo
// =====================================================================

// Signal level onujayi color select kore
uint16_t colorForLevel(int val) {
  int mag = abs(val);
  if (mag < 800)  return CYAN;
  if (mag < 1600) return GREEN;
  if (mag < 2500) return YELLOW;
  if (mag < 3300) return ORANGE;
  return MAGENTA;
}

// Uporer "OUTPUT:" header lekhe
void drawOutputHeader() {
  const uint8_t scale = 3;
  const uint8_t stepX = 18;
  uint16_t x = 6;
  uint16_t y = 4;

  drawGlyph5x7(x + 0 * stepX, y, GLYPH_O,     scale, CYAN,  BLACK);
  drawGlyph5x7(x + 1 * stepX, y, GLYPH_U,     scale, CYAN,  BLACK);
  drawGlyph5x7(x + 2 * stepX, y, GLYPH_T,     scale, CYAN,  BLACK);
  drawGlyph5x7(x + 3 * stepX, y, GLYPH_P,     scale, CYAN,  BLACK);
  drawGlyph5x7(x + 4 * stepX, y, GLYPH_U,     scale, CYAN,  BLACK);
  drawGlyph5x7(x + 5 * stepX, y, GLYPH_T,     scale, CYAN,  BLACK);
  drawGlyph5x7(x + 6 * stepX, y, GLYPH_COLON, scale, WHITE, BLACK);
}

const uint16_t MAT_TOP     = 48;
const uint16_t MAT_H       = 180;
const uint16_t ROW_H       = MAT_H / 2;
const uint16_t COL_X0      = 20;
const uint16_t COL_W       = 100;
const uint16_t BRK_LEFT_X  = 8;
const uint16_t BRK_RIGHT_X = 232;
const uint16_t BRK_THICK   = 3;
const uint16_t BRK_SERIF   = 9;

// Matrix er duipashe bracket [ ] anke
void drawMatrixBrackets() {
  const uint16_t t = BRK_THICK;
  const uint16_t bottomY = MAT_TOP + MAT_H - t;

  fillRect(BRK_LEFT_X, MAT_TOP, t, MAT_H, WHITE);
  fillRect(BRK_LEFT_X, MAT_TOP, BRK_SERIF, t, WHITE);
  fillRect(BRK_LEFT_X, bottomY, BRK_SERIF, t, WHITE);

  fillRect(BRK_RIGHT_X - t, MAT_TOP, t, MAT_H, WHITE);
  fillRect(BRK_RIGHT_X - BRK_SERIF, MAT_TOP, BRK_SERIF, t, WHITE);
  fillRect(BRK_RIGHT_X - BRK_SERIF, bottomY, BRK_SERIF, t, WHITE);
}

void drawMatrixValue(int idx, int val, uint16_t color) {
  const uint8_t scale = 3;
  const uint8_t adv = 6 * scale;
  uint16_t cx = COL_X0 + (idx % 2) * COL_W;
  uint16_t rowCenter = MAT_TOP + (idx / 2) * ROW_H + ROW_H / 2;
  uint16_t ny = rowCenter - (7 * scale) / 2;

  fillRect(cx, ny, COL_W, 7 * scale, BLACK);

  char buf[8];
  int n = snprintf(buf, sizeof(buf), "%d", val);
  uint16_t totalW = n * adv - scale;
  uint16_t nx = cx + (COL_W - totalW) / 2;
  for (int c = 0; c < n; c++) {
    drawChar5x7(nx + c * adv, ny, buf[c], scale, color, BLACK);
  }
}

// Value ba color bodlalei sudhu sei cell update kore
void drawMatrixCell(int idx, int val) {
  uint16_t color = chTimeout[idx] ? GRAY : colorForLevel(val);
  if (!matrixDrawnOnce || val != prevCellVals[idx] || color != prevCellColors[idx]) {
    drawMatrixValue(idx, val, color);
    prevCellVals[idx] = val;
    prevCellColors[idx] = color;
  }
}

// 2x2 matrix view render kore
void renderMatrix(bool fullRedraw) {
  if (fullRedraw) {
    fillRect(0, 0, 240, 240, BLACK);
    drawOutputHeader();
    drawMatrixBrackets();
    for (int i = 0; i < 4; i++) {
      prevCellVals[i] = 99999;
      prevCellColors[i] = 0;
    }
    matrixDrawnOnce = false;
  }
  for (int i = 0; i < 4; i++) {
    drawMatrixCell(i, currentVals[i]);
  }
  matrixDrawnOnce = true;
}

// Value ke graph er Y pixel coordinate e convert kore
uint16_t valToPlotY(int val) {
  long clamped = constrain(val, -VAL_MAX, VAL_MAX);
  int midY = (int)PLOT_Y + (int)PLOT_H / 2;
  int halfSpan = (int)PLOT_H / 2 - 1;
  int offset = (int)((clamped * halfSpan) / VAL_MAX);
  return (uint16_t)constrain(midY - offset, (int)PLOT_Y, (int)PLOT_Y + (int)PLOT_H - 1);
}

// Graph view er uporer header bar draw kore
void drawGraphTopBar(int elemIdx, int val, bool forceBar) {
  bool tmo = chTimeout[elemIdx];
  if (!forceBar && val == lastHeaderVal && tmo == lastHeaderTimeout) return;

  fillRect(0, 0, 240, 36, BLACK);
  const uint8_t scale = 2;
  const uint8_t stepX = 12;
  uint16_t x = 6;
  uint16_t y = 11;

  const char* name = ELEMENT_NAMES[elemIdx];
  drawChar5x7(x + 0 * stepX, y, name[0], scale, WHITE, BLACK);
  drawChar5x7(x + 1 * stepX, y, name[1], scale, WHITE, BLACK);
  drawChar5x7(x + 2 * stepX, y, name[2], scale, WHITE, BLACK);
  drawChar5x7(x + 3 * stepX, y, ':',     scale, WHITE, BLACK);

  char numBuf[7];
  snprintf(numBuf, sizeof(numBuf), "%5d", val);
  for (int i = 0; i < 5; i++) {
    if ((numBuf[i] >= '0' && numBuf[i] <= '9') || numBuf[i] == '-') {
      drawChar5x7(x + (4 + i) * stepX, y, numBuf[i], scale, tmo ? GRAY : WHITE, BLACK);
    }
  }
  if (tmo) {
    drawChar5x7(x + 10 * stepX, y, 'T', scale, ORANGE, BLACK);
  }

  lastHeaderVal = val;
  lastHeaderTimeout = tmo;
}

// Selected channel er history graph anke
void clearPlotBuffer() {
  for (size_t i = 0; i < sizeof(plotBuffer); i += 2) {
    plotBuffer[i] = BLACK >> 8;
    plotBuffer[i + 1] = BLACK & 0xFF;
  }
}

void drawPlotPixel(int x, int y, uint16_t color) {
  if (x < 0 || x >= PLOT_W || y < 0 || y >= PLOT_H) return;

  size_t byteOffset = ((size_t)y * PLOT_W + x) * 2;
  plotBuffer[byteOffset] = color >> 8;
  plotBuffer[byteOffset + 1] = color & 0xFF;
}

void drawPlotLine(int x0, int y0, int x1, int y1, uint16_t color) {
  int dx = abs(x1 - x0);
  int sx = (x0 < x1) ? 1 : -1;
  int dy = -abs(y1 - y0);
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx + dy;

  while (true) {
    drawPlotPixel(x0, y0, color);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) {
      err += dy;
      x0 += sx;
    }
    if (e2 <= dx) {
      err += dx;
      y0 += sy;
    }
  }
}

void renderGraph(int elemIdx, bool fullRedraw) {
  int latest = currentVals[elemIdx];

  if (fullRedraw) {
    fillRect(0, 0, 240, 240, BLACK);
    fillRect(PLOT_X - 2, PLOT_Y - 2, 2, PLOT_H + 4, WHITE);
    fillRect(PLOT_X - 2, PLOT_Y + PLOT_H, PLOT_W + 4, 2, WHITE);
  }

  drawGraphTopBar(elemIdx, latest, fullRedraw);

  clearPlotBuffer();
  for (int x = 0; x < PLOT_W; x++) {
    drawPlotPixel(x, PLOT_H / 2, GRAY);
  }

  int prevPx = -1;
  int prevPy = -1;

  for (int idx = 0; idx < HIST_LEN; idx++) {
    if (!historyValid[elemIdx][idx]) {
      prevPx = -1;
      prevPy = -1;
      continue;
    }

    int px = idx * HIST_STEP_X;
    int py = (int)valToPlotY(history[elemIdx][idx]) - PLOT_Y;

    if (prevPx >= 0) {
      drawPlotLine(prevPx, prevPy, px, py, WHITE);
    } else {
      drawPlotPixel(px, py, WHITE);
      drawPlotPixel(px + 1, py, WHITE);
      drawPlotPixel(px, py + 1, WHITE);
      drawPlotPixel(px + 1, py + 1, WHITE);
    }
    prevPx = px;
    prevPy = py;
  }

  writeRgb565Buffer(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, plotBuffer);
}

// Bortoman viewMode onujayi screen update kore
void renderView(bool fullRedraw) {
  if (viewMode == 0) {
    renderMatrix(fullRedraw);
  } else {
    renderGraph(viewMode - 1, fullRedraw);
  }
}

// =====================================================================
// Sampling, zeroing, buttons
// =====================================================================

// Slew-rate limiter, EMA smoothing ar deadband diye reading stabilize kore
int stabilizeChannel(int idx, int targetVal) {
  if (!emaReady[idx]) {
    emaVals[idx] = (float)targetVal;
    emaReady[idx] = true;
    return targetVal;
  }

  int delta = targetVal - currentVals[idx];
  if (delta > MAX_STEP_PER_SAMPLE) {
    targetVal = currentVals[idx] + MAX_STEP_PER_SAMPLE;
  } else if (delta < -MAX_STEP_PER_SAMPLE) {
    targetVal = currentVals[idx] - MAX_STEP_PER_SAMPLE;
  }

  emaVals[idx] += EMA_ALPHA * ((float)targetVal - emaVals[idx]);
  int smoothed = (int)roundf(emaVals[idx]);

  if (abs(smoothed - currentVals[idx]) <= OUTPUT_DEADBAND) {
    return currentVals[idx];
  }
  return smoothed;
}

// A21 (index 2) ke baki 3ta sensor er final value er exact same direction e proportional rakhe
void syncDetachedA21() {
  if (!A21_DETACHED) return;

  float avgOther3 = (currentVals[0] + currentVals[1] + currentVals[3]) / 3.0f;
  int coupledVal = (int)constrain(roundf(avgOther3 * A21_PROPORTION_FACTOR), -VAL_MAX, VAL_MAX);

  currentVals[2] = coupledVal;
  emaVals[2] = (float)coupledVal;
  emaReady[2] = true;
  chTimeout[2] = chTimeout[0] && chTimeout[1] && chTimeout[3];
}

void printDebugTelemetry() {
#if DEBUG_SERIAL
  for (int i = 0; i < 4; i++) {
    Serial.print(ELEMENT_NAMES[i]);
    Serial.print("(IO");
    Serial.print(SENSE_PINS[i]);
    Serial.print(") dt=");
    Serial.print(lastDischargeUs[i]);
    Serial.print("us ");
    Serial.print(chTimeout[i] ? "TIMEOUT " : "ok ");
    Serial.print("val=");
    Serial.print(currentVals[i]);
    Serial.print(i < 3 ? " | " : "\n");
  }
#endif
}

void startZeroCalibration() {
  if (zeroCalibrationActive) return;

  zeroCalibrationActive = true;
  zeroSamplesTaken = 0;
  for (int i = 0; i < 4; i++) {
    zeroAccumulator[i] = 0;
    zeroValidCount[i] = 0;
  }
  Serial.println("Zero calibration started");
}

void finishZeroCalibration() {
  for (int i = 0; i < 4; i++) {
    zeroOffsets[i] = (int)(zeroAccumulator[i] / zeroValidCount[i]);
    emaVals[i] = 0.0f;
    emaReady[i] = true;
    currentVals[i] = 0;
    for (int k = 0; k < HIST_LEN; k++) {
      history[i][k] = 0;
      historyValid[i][k] = true;
    }
  }
  syncDetachedA21();
  needsFullRedraw = true;
  lastHeaderVal = 99999;

  Serial.print("Zero offsets set -> A11:");
  Serial.print(zeroOffsets[0]);
  Serial.print(" A12:");
  Serial.print(zeroOffsets[1]);
  Serial.print(" A21:");
  Serial.print(zeroOffsets[2]);
  Serial.print(" A22:");
  Serial.println(zeroOffsets[3]);

  zeroCalibrationActive = false;
  lastSampleMs = millis();
}

void serviceZeroCalibration() {
  sampleAllRaw();
  for (int i = 0; i < 4; i++) {
    zeroAccumulator[i] += chTimeout[i] ? 0 : rawVals[i];
    zeroValidCount[i]++;
  }
  zeroSamplesTaken++;

  if (zeroSamplesTaken >= ZERO_AVG) {
    finishZeroCalibration();
  }
}

// Notun sample niye stabilize kore history buffer shift kore
void pushSampleToAll() {
  sampleAllRaw();
  for (int i = 0; i < 4; i++) {
    if (A21_DETACHED && i == 2) continue;
    int raw = applyZeroOffset(i, chTimeout[i] ? 0 : rawVals[i]);
    currentVals[i] = stabilizeChannel(i, raw);
  }

  syncDetachedA21();
  printDebugTelemetry();

  for (int i = 0; i < 4; i++) {
    for (int k = 0; k < HIST_LEN - 1; k++) {
      history[i][k] = history[i][k + 1];
      historyValid[i][k] = historyValid[i][k + 1];
    }
    history[i][HIST_LEN - 1] = currentVals[i];
    historyValid[i][HIST_LEN - 1] = !chTimeout[i];
  }
}

// Debounce soho button press detect kore (HIGH -> LOW)
bool pollButtonPress(DebouncedButton &btn, unsigned long now, const char* label) {
  int raw = digitalRead(btn.pin);
  if (raw != btn.lastRaw) {
    btn.lastChangeMs = now;
    btn.lastRaw = raw;
  }

  if ((now - btn.lastChangeMs) >= DEBOUNCE_MS && raw != btn.stableState) {
    btn.stableState = raw;
    Serial.print(label);
    Serial.print(" (IO");
    Serial.print(btn.pin);
    Serial.print(") -> ");
    Serial.println(btn.stableState == LOW ? "PRESSED (LOW)" : "RELEASED (HIGH)");

    if (btn.stableState == LOW) {
      return true;
    }
  }
  return false;
}

// HOLD, VIEW ar ZERO button er kaj handle kore
void handleButtons() {
  unsigned long now = millis();

  if (pollButtonPress(holdBtn, now, "HOLD")) {
    isFrozen = !isFrozen;
    Serial.print("Hold state: ");
    Serial.println(isFrozen ? "FROZEN" : "LIVE");
  }

  if (pollButtonPress(viewBtn, now, "VIEW")) {
    viewMode = (viewMode + 1) % 5;
    needsFullRedraw = true;
    lastHeaderVal = 99999;
    Serial.print("Switched viewMode to ");
    Serial.print(viewMode);
    if (viewMode == 0) {
      Serial.println(" (2x2 Matrix)");
    } else {
      Serial.print(" (Graph ");
      Serial.print(ELEMENT_NAMES[viewMode - 1]);
      Serial.println(")");
    }
  }

  if (pollButtonPress(zeroBtn, now, "ZERO")) {
    startZeroCalibration();
  }
}

// =====================================================================
// Arduino entry points - setup ar main loop
// =====================================================================

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Booting 2x2 Optical Matrix Monitor (ESP32-S3 capacitance readout on IO 4, 47, 2, 5)...");

  // Button gulo internal pull-up e set kora
  pinMode(HOLD_BUTTON_PIN, INPUT_PULLUP);
  pinMode(VIEW_BUTTON_PIN, INPUT_PULLUP);
  pinMode(ZERO_BUTTON_PIN, INPUT_PULLUP);

  // Photodiode pin gulo (IO 4, 47, 2, 5) pull chara floating INPUT mode e rakha
  for (int i = 0; i < 4; i++) {
    if (A21_DETACHED && i == 2) continue;
    setSensePinFloatingInput(SENSE_PINS[i]);
  }

  // External pull-up ache kina check kore Serial e janay
  checkSensePinsForPullups();

  // Prothom reading niye history vorano jate graph faka na thake
  sampleAllRaw();
  for (int i = 0; i < 4; i++) {
    zeroOffsets[i] = 0;
    if (A21_DETACHED && i == 2) continue;
    int initVal = applyZeroOffset(i, chTimeout[i] ? 0 : rawVals[i]);
    emaVals[i] = (float)initVal;
    emaReady[i] = true;
    currentVals[i] = initVal;
  }
  syncDetachedA21();
  printDebugTelemetry();

  for (int i = 0; i < 4; i++) {
    for (int k = 0; k < HIST_LEN; k++) {
      history[i][k] = currentVals[i];
      historyValid[i][k] = !chTimeout[i];
    }
  }

  initST7789();
  renderView(true);
  needsFullRedraw = false;
  lastSampleMs = millis();
}

void loop() {
  handleButtons();

  if (zeroCalibrationActive) {
    unsigned long now = millis();
    if (zeroSamplesTaken == 0 || (now - lastSampleMs >= SAMPLE_MS)) {
      lastSampleMs = now;
      serviceZeroCalibration();
    }
    return;
  }

  if (needsFullRedraw) {
    renderView(true);
    needsFullRedraw = false;
  }

  // Freeze na thakle SAMPLE_MS por por notun sample niye display update kore
  if (!isFrozen) {
    unsigned long now = millis();
    if (now - lastSampleMs >= SAMPLE_MS) {
      pushSampleToAll();
      renderView(false);
      lastSampleMs = millis();
    }
  }
}