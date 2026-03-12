/* Edge Impulse + 3 Phase Motor Resistance Tester */

#include <motor-project_inferencing.h>
#include <U8g2lib.h>
#include <SPI.h>

/* ADC Pins */
#define ADC_U_PIN 4
#define ADC_V_PIN 5
#define ADC_W_PIN 6

#define BUTTON_PIN 16

#define KNOWN_RESISTOR 270.0
#define VREF 3.3

/* LCD */
U8G2_ST7920_128X64_F_SW_SPI lcd(U8G2_R0, 12, 11, 10, 13);

/* Calibration */
#define CAL_R1_ACTUAL 10.0
#define CAL_R2_ACTUAL 82.0

#define CAL_R1_MEASURED 7.03
#define CAL_R2_MEASURED 74.53

float scale;
float offset;

float lastU = 0;
float lastV = 0;
float lastW = 0;

/* Edge Impulse sensor structure */
typedef struct {
    const char *name;
    float *value;
    uint8_t (*poll_sensor)(void);
    bool (*init_sensor)(void);
    int8_t status;
} eiSensors;

#define N_SENSORS 3

static float data[3];
static int8_t fusion_sensors[N_SENSORS];
static int fusion_ix = 0;

static const bool debug_nn = false;

/* Forward declarations */
bool init_ADC(void);
uint8_t poll_ADC(void);

/* Sensor list (U V W resistance) */
eiSensors sensors[] = {
    {"phaseU", &data[0], &poll_ADC, &init_ADC, -1},
    {"phaseV", &data[1], &poll_ADC, &init_ADC, -1},
    {"phaseW", &data[2], &poll_ADC, &init_ADC, -1},
};

/* Resistance Reading */
float readResistance(int pin) {

  int adcValue = 0;

  for (int i = 0; i < 10; i++) {
    adcValue += analogRead(pin);
  }

  adcValue /= 10;

  float voltage = adcValue * (VREF / 4095.0);

  if (voltage >= VREF - 0.01) return -1;

  float resistance = (voltage * KNOWN_RESISTOR) / (VREF - voltage);

  return resistance;
}

float calibrate(float measured) {
  return (measured * scale) + offset;
}

/* LCD Screens */

void drawCentered(int y, const char *text) {
  int16_t x = (128 - lcd.getStrWidth(text)) / 2;
  lcd.drawStr(x, y, text);
}

void showReady() {

  lcd.clearBuffer();
  lcd.setFont(u8g2_font_ncenB08_tr);

  drawCentered(20,"Motor Tester");
  drawCentered(40,"Press Button");
  drawCentered(55,"To Start");

  lcd.sendBuffer();
}

void showSampling() {

  lcd.clearBuffer();
  lcd.setFont(u8g2_font_ncenB08_tr);

  lcd.drawStr(25,30,"Sampling...");

  lcd.sendBuffer();
}

void showProcessing() {

  lcd.clearBuffer();
  lcd.setFont(u8g2_font_ncenB08_tr);

  lcd.drawStr(20,30,"AI Processing");

  lcd.sendBuffer();
}

void showResult(String label) {

  lcd.clearBuffer();
  lcd.setFont(u8g2_font_ncenB08_tr);

  drawCentered(10,"Result:");

  drawCentered(22,label.c_str());

  char buf[20];

  sprintf(buf,"U: %.2f", lastU);
  drawCentered(38,buf);

  sprintf(buf,"V: %.2f", lastV);
  drawCentered(50,buf);

  sprintf(buf,"W: %.2f", lastW);
  drawCentered(62,buf);

  lcd.sendBuffer();
}

/* ADC Init */

bool init_ADC(void) {
  static bool init_status = false;

  if (!init_status) {
    init_status = true;
  }

  return init_status;
}

/* Poll ADC -> Read U V W */

uint8_t poll_ADC(void) {

  float u = calibrate(readResistance(ADC_U_PIN));
  float v = calibrate(readResistance(ADC_V_PIN));
  float w = calibrate(readResistance(ADC_W_PIN));

  data[0] = u;
  data[1] = v;
  data[2] = w;

  lastU = u;
  lastV = v;
  lastW = w;

  // ===== SERIAL PRINT =====
  Serial.print("U: ");
  Serial.print(u, 2);
  Serial.print("  V: ");
  Serial.print(v, 2);
  Serial.print("  W: ");
  Serial.println(w, 2);
  // ========================

  return 0;
}

/* Setup */

void setup() {

  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  lcd.begin();

  scale = (CAL_R2_ACTUAL - CAL_R1_ACTUAL) /
          (CAL_R2_MEASURED - CAL_R1_MEASURED);

  offset = CAL_R1_ACTUAL - (scale * CAL_R1_MEASURED);

  showReady();

  Serial.println("Motor AI Tester Ready");
}

/* Main Loop */

void loop() {

  if (digitalRead(BUTTON_PIN) == LOW) {

    delay(50);

    if (digitalRead(BUTTON_PIN) == LOW) {

      showSampling();

      /* Allocate Edge Impulse buffer */
      float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

      for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {

        poll_ADC();

        buffer[ix + 0] = data[0];
        buffer[ix + 1] = data[1];
        buffer[ix + 2] = data[2];

        delay(EI_CLASSIFIER_INTERVAL_MS);
      }

      showProcessing();

      signal_t signal;

      int err = numpy::signal_from_buffer(buffer,
                                           EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
                                           &signal);

      if (err != 0) {
        Serial.println("Signal error");
        return;
      }

      ei_impulse_result_t result = {0};

      err = run_classifier(&signal, &result, debug_nn);

      if (err != EI_IMPULSE_OK) {
        Serial.println("Classifier error");
        return;
      }

      Serial.println("Predictions:");

      float best = 0;
      String bestLabel = "";

      for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {

        Serial.print(result.classification[ix].label);
        Serial.print(": ");
        Serial.println(result.classification[ix].value);

        if (result.classification[ix].value > best) {

          best = result.classification[ix].value;
          bestLabel = result.classification[ix].label;
        }
      }

      showResult(bestLabel);

      delay(5000);

      showReady();
    }
  }
}