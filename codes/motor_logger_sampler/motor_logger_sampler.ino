/* Edge Impulse + 3 Phase Motor Resistance Tester
   Hybrid Decision: Machine Learning + Resistive Imbalance
*/

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

static float data[3];
static const bool debug_nn = false;


/* ---------------- Resistance Reading ---------------- */

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


/* ---------------- Motor Detection ---------------- */

bool motorConnected(float u, float v, float w) {

  float threshold = 0.10;

  if (abs(u - 1.43) < threshold &&
      abs(v - 1.43) < threshold &&
      abs(w - 1.43) < threshold)
      return false;

  if (u <= 0 || v <= 0 || w <= 0)
      return false;

  return true;
}


/* ---------------- Resistive Imbalance ---------------- */

float computeImbalance(float u, float v, float w) {

  float rave = (u + v + w) / 3.0;

  float du = abs(u - rave);
  float dv = abs(v - rave);
  float dw = abs(w - rave);

  float rmax = max(du, max(dv, dw));

  float imbalance = (rmax / rave) * 100.0;

  return imbalance;
}


/* ---------------- LCD Utilities ---------------- */

void drawCentered(int y, const char *text) {
  int16_t x = (128 - lcd.getStrWidth(text)) / 2;
  lcd.drawStr(x, y, text);
}

void showReady() {

  lcd.clearBuffer();
  lcd.setFont(u8g2_font_ncenB08_tr);

  drawCentered(20,"3PH Motor Tester");
  drawCentered(40,"Press Button");
  drawCentered(55,"To Start");

  lcd.sendBuffer();
}

void showSampling()
{

  lcd.clearBuffer();

  lcd.setFont(u8g2_font_6x10_tr);
  drawCentered(10,"3PH MOTOR TESTER");

  lcd.setFont(u8g2_font_ncenB08_tr);
  drawCentered(35,"MEASURING");

  lcd.setFont(u8g2_font_6x12_tr);
  drawCentered(55,"PLEASE WAIT");

  lcd.sendBuffer();
}

void showProcessing()
{

  lcd.clearBuffer();

  lcd.setFont(u8g2_font_6x10_tr);
  drawCentered(10,"3PH MOTOR TESTER");

  lcd.setFont(u8g2_font_ncenB08_tr);
  drawCentered(35,"AI ANALYZING");

  lcd.setFont(u8g2_font_6x12_tr);
  drawCentered(55,"RUNNING MODEL");

  lcd.sendBuffer();
}

void showNoMotor()
{

  lcd.clearBuffer();

  lcd.setFont(u8g2_font_6x10_tr);
  drawCentered(10,"3PH MOTOR TESTER");

  lcd.setFont(u8g2_font_ncenB08_tr);
  drawCentered(30,"SYSTEM ERROR");

  lcd.setFont(u8g2_font_6x12_tr);
  drawCentered(50,"NO MOTOR CONNECTED");

  lcd.sendBuffer();
}

void showResult(String label, float imbalance)
{

  lcd.clearBuffer();

  /* TITLE */
  lcd.setFont(u8g2_font_6x10_tr);
  drawCentered(10,"3PH MOTOR TESTER");

  /* STATUS */
  lcd.setFont(u8g2_font_6x12_tr);

  String status = "STATUS: " + label;
  drawCentered(24,status.c_str());

  /* VALUES */

  lcd.setFont(u8g2_font_6x12_tr);

  char buf[20];

  sprintf(buf,"U: %.3f", lastU);
  lcd.drawStr(2,40,buf);

  sprintf(buf,"V: %.3f", lastV);
  lcd.drawStr(66,40,buf);

  sprintf(buf,"W: %.3f", lastW);
  lcd.drawStr(2,52,buf);

  sprintf(buf,"I: %.2f%%", imbalance);
  lcd.drawStr(66,52,buf);

  /* IMBALANCE BAR */

  int barWidth = map(imbalance,0,10,0,120);

  if(barWidth > 120) barWidth = 120;

  lcd.drawFrame(4,56,120,6);
  lcd.drawBox(4,56,barWidth,6);

  lcd.sendBuffer();
}


/* ---------------- ADC Poll ---------------- */

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

  Serial.print("U: ");
  Serial.print(u,3);
  Serial.print("  V: ");
  Serial.print(v,3);
  Serial.print("  W: ");
  Serial.println(w,3);

  return 0;
}


/* ---------------- Setup ---------------- */

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


/* ---------------- Main Loop ---------------- */

void loop() {

  if (digitalRead(BUTTON_PIN) == LOW) {

    delay(50);

    if (digitalRead(BUTTON_PIN) == LOW) {

      showSampling();

      float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};

      for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {

        poll_ADC();

        buffer[ix + 0] = data[0];
        buffer[ix + 1] = data[1];
        buffer[ix + 2] = data[2];

        delay(EI_CLASSIFIER_INTERVAL_MS);
      }

      /* Motor connection check */

      if (!motorConnected(lastU,lastV,lastW)) {

        Serial.println("ERROR: No Motor Found");

        showNoMotor();
        delay(4000);
        showReady();

        return;
      }

      showProcessing();

      signal_t signal;

      int err = numpy::signal_from_buffer(
        buffer,
        EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE,
        &signal
      );

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

      /* -------- Machine Learning Prediction -------- */

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

      /* -------- Resistive Imbalance -------- */

      float imbalance = computeImbalance(lastU,lastV,lastW);

      Serial.print("Imbalance %: ");
      Serial.println(imbalance);

      /* -------- Hybrid Decision -------- */

      String health;

      if (bestLabel == "bad") {
        health = "MOTOR BAD";
      }
      else {

        if (imbalance < 3)
          health = "MOTOR GOOD";
        else if (imbalance < 5)
          health = "WARNING";
        else
          health = "MOTOR BAD";
      }

      showResult(health, imbalance);

      delay(5000);

      showReady();
    }
  }
}