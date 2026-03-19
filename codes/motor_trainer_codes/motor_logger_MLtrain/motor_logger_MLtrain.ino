#define ADC_U_PIN 4
#define ADC_V_PIN 5
#define ADC_W_PIN 6

#define BUTTON_PIN 16

#define KNOWN_RESISTOR 270.0
#define VREF 3.3

#define SAMPLE_INTERVAL 10
#define RECORD_TIME 5000

// ===== 2 POINT CALIBRATION =====
// Actual resistors used
#define CAL_R1_ACTUAL 10.0
#define CAL_R2_ACTUAL 82.0

// Measured values from ESP32 (CHANGE THESE after measuring)
#define CAL_R1_MEASURED 7.03
#define CAL_R2_MEASURED 74.53
// ===============================

float scale;
float offset;

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

void setup() {

  Serial.begin(115200);

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Compute calibration parameters
  scale = (CAL_R2_ACTUAL - CAL_R1_ACTUAL) /
          (CAL_R2_MEASURED - CAL_R1_MEASURED);

  offset = CAL_R1_ACTUAL - (scale * CAL_R1_MEASURED);

  Serial.println("Press IO16 button to start recording");
}

void loop() {

  if (digitalRead(BUTTON_PIN) == LOW) {

    delay(50);

    if (digitalRead(BUTTON_PIN) == LOW) {

      Serial.println("timestamp,U1,V1,W1");

      unsigned long startTime = millis();
      unsigned long currentTime = 0;

      while (currentTime <= RECORD_TIME) {

        float u = readResistance(ADC_U_PIN);
        float v = readResistance(ADC_V_PIN);
        float w = readResistance(ADC_W_PIN);

        // apply calibration
        u = calibrate(u);
        v = calibrate(v);
        w = calibrate(w);

        Serial.print(currentTime);
        Serial.print(",");
        Serial.print(u, 2);
        Serial.print(",");
        Serial.print(v, 2);
        Serial.print(",");
        Serial.println(w, 2);

        delay(SAMPLE_INTERVAL);

        currentTime = millis() - startTime;
      }

      Serial.println("Done");

      while (digitalRead(BUTTON_PIN) == LOW) {
        delay(10);
      }

      Serial.println("Press button again to record");
    }
  }
}